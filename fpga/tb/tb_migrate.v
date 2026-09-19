// tb_migrate.v - Kirchhoff migration engine vs float golden on a small grid
// (N_IFFT=64 rows, N_POS=16 cols, max_bin=12): shape comparison after
// normalisation (fixed-point vs double), correlation and relative RMS.
`timescale 1ns / 1ps
module tb_migrate;
    localparam NI = 64, NP = 16, LB = 6, MB = 12;
    reg clk = 0, rst = 1, go = 0;
    reg s_tvalid = 0, s_tlast = 0, s_frame_last = 0;
    reg [31:0] s_tdata; reg [15:0] s_tuser; reg [7:0] s_tpos;
    wire s_tready, m_tvalid, m_tlast, m_frame_last, busy;
    wire [15:0] m_tdata, m_tuser; wire [7:0] m_tpos;

    gpr_migrate #(
        .N_IFFT(NI), .N_POS(NP), .LB(LB), .NAP_MAX(8),
        .NAP_FILE("mig_nap.hex")
    ) dut (
        .clk(clk), .rst(rst),
        .s_tvalid(s_tvalid), .s_tready(s_tready), .s_tdata(s_tdata),
        .s_tuser(s_tuser), .s_tuser_pos(s_tpos), .s_tlast(s_tlast),
        .s_frame_last(s_frame_last),
        .inv_dr_q20(32'd243289095), .roff_q20(32'd0),
        .dx_q20(32'd10486), .dr_q20(32'd4519),
        .kbeta_q16(32'd381485), .alpha2_q8(16'sd276),
        .max_bin(MB), .out_scale_q8(16'd256), .go(go),
        .m_tvalid(m_tvalid), .m_tready(1'b1), .m_tdata(m_tdata),
        .m_tuser(m_tuser), .m_tuser_pos(m_tpos), .m_tlast(m_tlast),
        .m_frame_last(m_frame_last), .busy(busy));

    always #2 clk = ~clk;

    reg [31:0] inv [0:NI*NP-1];     // column-major (pos outer, bin inner)
    integer goldm [0:MB*NP-1];      // row-major (iz outer), milli-units
    real gold [0:MB*NP-1];
    real got  [0:MB*NP-1];
    integer i, j, idx, gotn = 0, errors = 0, gfd;
    real line, mxg, mxr, num, dg, dr2, err, rel;

    initial begin
        $readmemh("mig_in.hex", inv);
        gfd = $fopen("mig_gold.hex", "r");
        idx = 0;
        while (!$feof(gfd) && idx < MB*NP) begin
            if ($fscanf(gfd, "%d\n", goldm[idx]) == 1) begin
                gold[idx] = $itor(goldm[idx]) / 1000.0;
                idx = idx + 1;
            end
        end
        $fclose(gfd);
        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);
        go <= 1;
        idx = 0;
        for (j = 0; j < NP; j = j + 1) begin
            for (i = 0; i < NI; i = i + 1) begin
                s_tvalid <= 1;
                s_tdata  <= inv[idx];
                s_tuser  <= i;
                s_tpos   <= j;
                s_tlast  <= (i == NI-1);
                s_frame_last <= (j == NP-1 && i == NI-1);
                @(posedge clk);
                while (!s_tready) @(posedge clk);
                idx = idx + 1;
            end
        end
        s_tvalid <= 0; s_frame_last <= 0;
    end

    always @(posedge clk) begin
        if (!rst && m_tvalid) begin
            got[m_tuser*NP + m_tpos] = $itor(m_tdata);
            gotn = gotn + 1;
        end
    end

    initial begin
        #20000000;
        if (gotn != NI*NP) begin
            $display("MIG fail: %0d pixels emitted (want %0d)", gotn, NI*NP);
            errors = errors + 1;
        end
        // normalise both to unit max over rows 1..MB-1, then compare
        mxg = 0; mxr = 0;
        for (i = 1; i < MB; i = i + 1)
            for (j = 0; j < NP; j = j + 1) begin
                if (gold[i*NP+j] > mxg) mxg = gold[i*NP+j];
                if (got [i*NP+j] > mxr) mxr = got [i*NP+j];
            end
        num = 0; dg = 0; dr2 = 0; rel = 0;
        for (i = 1; i < MB; i = i + 1)
            for (j = 0; j < NP; j = j + 1) begin
                num = num + (gold[i*NP+j]/mxg) * (got[i*NP+j]/mxr);
                dg  = dg  + (gold[i*NP+j]/mxg) * (gold[i*NP+j]/mxg);
                dr2 = dr2 + (got [i*NP+j]/mxr) * (got [i*NP+j]/mxr);
                err = (gold[i*NP+j]/mxg) - (got[i*NP+j]/mxr);
                rel = rel + err*err;
            end
        rel = $sqrt(rel / ((MB-1)*NP));
        num = num / ($sqrt(dg)*$sqrt(dr2));
        $display("MIG: correlation=%.4f  rel_rms=%.4f  peak_got=%0d", num, rel, $rtoi(mxr));
        for (i = 1; i < 4; i = i + 1) begin
            for (j = 0; j < NP; j = j + 2)
                $display("  iz=%0d ix=%0d got=%8.1f gold=%8.1f", i, j,
                         got[i*NP+j], gold[i*NP+j]);
        end
        if (num < 0.95) begin $display("MIG correlation low"); errors = errors+1; end
        if (rel > 0.12) begin $display("MIG rel_rms high");   errors = errors+1; end
        if (errors == 0) $display("TB_MIGRATE PASS");
        else             $display("TB_MIGRATE FAIL (%0d)", errors);
        $finish;
    end
endmodule
