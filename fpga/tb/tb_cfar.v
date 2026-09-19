// tb_cfar.v - 2-D CA-CFAR (mean estimator) vs exact integer golden on a
// 64x64 patch, GUARD=4 TRAIN=8
`timescale 1ns / 1ps
module tb_cfar;
    localparam NI = 64, NP = 64, LB = 6, GUARD = 4, TRAIN = 8;
    reg clk = 0, rst = 1, go = 0;
    reg s_tvalid = 0, s_frame_last = 0;
    reg [15:0] s_tdata; reg [15:0] s_tuser; reg [7:0] s_tpos;
    wire s_tready, m_tvalid, m_tdet, m_pass2, m_tlast, m_frame_last, busy, f1d;
    wire [15:0] m_tdata, m_tuser; wire [7:0] m_tpos;

    gpr_cfar #(.N_IFFT(NI), .N_POS(NP), .GUARD(GUARD), .TRAIN(TRAIN), .LB(LB))
    dut (
        .clk(clk), .rst(rst),
        .s_tvalid(s_tvalid), .s_tready(s_tready), .s_tdata(s_tdata),
        .s_tuser(s_tuser), .s_tuser_pos(s_tpos), .s_frame_last(s_frame_last),
        .go(go),
        .b_min0(16'd4), .b_max0(16'd59), .cfar_mode(1'b0),
        .alpha_q12(16'd57313), .log2alpha_q26(32'sd0), .gamma2_q26(32'sd0),
        .pfloor_k(16'd2874), .pfloor_sh(5'd28),
        .m_tvalid(m_tvalid), .m_tready(1'b1), .m_tdata(m_tdata),
        .m_tdet(m_tdet), .m_pass2(m_pass2), .m_tuser(m_tuser),
        .m_tuser_pos(m_tpos), .m_tlast(m_tlast), .m_frame_last(m_frame_last),
        .busy(busy), .frame1_done(f1d));

    always #2 clk = ~clk;

    reg [15:0] img [0:NI*NP-1];   // row-major (iz outer)
    integer gold [0:NI*NP-1];
    integer i, j, idx, errors = 0, got1 = 0, got2 = 0, ndet = 0;
    integer gfd;
    reg [63:0] line;

    initial begin
        $readmemh("cfar_in.hex", img);
        gfd = $fopen("cfar_gold.hex", "r");
        idx = 0;
        while (!$feof(gfd)) begin
            if ($fscanf(gfd, "%d\n", line) == 1) begin
                gold[idx] = line;
                idx = idx + 1;
            end
        end
        $fclose(gfd);
        if (idx != NI*NP) begin
            $display("CFAR tb: golden has %0d entries", idx);
        end
        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);
        go <= 1;
        // feed frame row-major, respecting ready
        idx = 0;
        for (i = 0; i < NI; i = i + 1) begin
            for (j = 0; j < NP; j = j + 1) begin
                s_tvalid <= 1;
                s_tdata  <= img[idx];
                s_tuser  <= i;
                s_tpos   <= j[7:0];
                s_frame_last <= (idx == NI*NP-1);
                @(posedge clk);
                while (!s_tready) @(posedge clk);
                idx = idx + 1;
            end
        end
        s_tvalid <= 0; s_frame_last <= 0;
    end

    always @(posedge clk) begin
        if (!rst && m_tvalid) begin
            if (!m_pass2) begin
                if (got1 < NI*NP) begin
                    if (m_tuser*NP + m_tpos !== got1) begin
                        $display("CFAR order fail: beat %0d at (%0d,%0d)",
                                 got1, m_tuser, m_tpos);
                        errors = errors + 1;
                    end
                    if ((m_tdet ? 1 : 0) !== gold[got1]) begin
                        if (errors < 8)
                            $display("CFAR det mismatch @(%0d,%0d): got %b want %0d",
                                     m_tuser, m_tpos, m_tdet, gold[got1]);
                        errors = errors + 1;
                    end
                    if (m_tdet) ndet = ndet + 1;
                end
                got1 = got1 + 1;
            end else
                got2 = got2 + 1;
        end
    end

    initial begin
        #40000000;
        if (got1 != NI*NP) begin
            $display("CFAR fail: pass1 beats %0d", got1); errors = errors + 1;
        end
        if (got2 != NI*NP) begin
            $display("CFAR fail: pass2 beats %0d", got2); errors = errors + 1;
        end
        $display("CFAR detections: %0d (golden %0d)", ndet, 41);
        if (errors == 0) $display("TB_CFAR PASS");
        else             $display("TB_CFAR FAIL (%0d)", errors);
        $finish;
    end
endmodule
