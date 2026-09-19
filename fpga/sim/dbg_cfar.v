`timescale 1ns / 1ps
module dbg_cfar;
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
    reg [15:0] img [0:NI*NP-1];
    integer i, j, idx, shown = 0;
    initial begin
        $readmemh("cfar_in.hex", img);
        repeat (3) @(posedge clk); rst = 0;
        @(posedge clk); go <= 1;
        idx = 0;
        for (i = 0; i < NI; i = i + 1)
            for (j = 0; j < NP; j = j + 1) begin
                s_tvalid <= 1; s_tdata <= img[idx]; s_tuser <= i; s_tpos <= j;
                s_frame_last <= (idx == NI*NP-1);
                @(posedge clk);
                while (!s_tready) @(posedge clk);
                idx = idx + 1;
            end
        s_tvalid <= 0;
    end
    always @(posedge clk) begin
        if (!rst && dut.st == 4'd9 && dut.rp < 20)   // C_NROW
            $display("NROW rp=%0d colbig[5]=%0d colgA[5]=%0d colgB[5]=%0d",
                     dut.rp, dut.colbig[5], dut.colgA[5], dut.colgB[5]);
        // V-phase writes during row pass rp=17
        if (!rst && dut.st == 4'd5 && dut.rp == 13'd17 && (dut.cj == 9'd5 || dut.cj == 9'd6))
            $display("VN rp=17 cj=%0d newpm=%0d slot_b=%0d", dut.cj, dut.newpm, dut.slot_b);
        // H-phase tap reads during row pass rp=17
        if (!rst && dut.st == 4'd6 && dut.rp == 13'd17 && (dut.cj == 9'd17 || dut.cj == 9'd18))
            $display("H1 rp=17 cj=%0d jo_s=%0d slot_mid=%0d colgA=%0d colgB=%0d hb=%0d hgA=%0d hgB=%0d",
                     dut.cj, dut.jo_s, dut.slot_mid_b,
                     dut.colgA[dut.cj < 64 ? dut.cj : 0],
                     dut.colgB[dut.cj < 64 ? dut.cj : 0],
                     dut.hb, dut.hgA, dut.hgB);
        if (!rst && m_tvalid && !m_pass2 && shown < 14) begin
            if ({m_tuser, m_tpos} == 16'h0701 || {m_tuser, m_tpos} == 16'h0600 ||
                {m_tuser, m_tpos} == 16'h0505 || {m_tuser, m_tpos} == 16'h0A0A ||
                {m_tuser, m_tpos} == 16'h1414 || {m_tuser, m_tpos} == 16'h1E1E) begin
                $display("pix(%0d,%0d) mig=%0d det=%b tap_pm=%0d sum=%0d gsum=%0d cnt=%0d vrows=%0d vcols=%0d vrowsg=%0d vcolsg=%0d p_floor=%0d",
                         m_tuser, m_tpos, m_tdata, m_tdet, dut.tap_pm,
                         dut.h2_sum, dut.h2_gsum, dut.h2_cnt,
                         dut.vrows, dut.vcols, dut.vrowsg, dut.vcolsg, dut.p_floor);
                shown = shown + 1;
            end
        end
    end
    initial begin #80000000; $finish; end
endmodule
