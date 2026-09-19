`timescale 1ns / 1ps
module dbg_bg;
    localparam NI = 64, NP = 8, LB = 6;
    reg clk = 0, rst = 1;
    reg s_tvalid = 0, s_tlast = 0;
    reg [31:0] s_tdata; reg [15:0] s_tuser;
    wire m_tvalid, m_tlast, m_frame_last, frame_ready, overrun;
    wire [31:0] m_tdata; wire [15:0] m_tuser; wire [7:0] m_pos;
    wire [7:0] frame_count;
    gpr_background #(.N_IFFT(NI), .N_POS(NP), .LB(LB)) dut (
        .clk(clk), .rst(rst), .nbg(4'd1),
        .s_tvalid(s_tvalid), .s_tdata(s_tdata), .s_tuser(s_tuser), .s_tlast(s_tlast),
        .m_tvalid(m_tvalid), .m_tready(1'b1), .m_tdata(m_tdata),
        .m_tuser(m_tuser), .m_tuser_pos(m_pos), .m_tlast(m_tlast),
        .m_frame_last(m_frame_last), .frame_ready(frame_ready),
        .frame_count(frame_count), .overrun(overrun));
    always #2 clk = ~clk;
    reg [31:0] inv [0:NI*NP-1];
    integer i, j, idx, shown = 0;
    initial begin
        $readmemh("bg_in.hex", inv);
        repeat (3) @(posedge clk); rst = 0;
        @(posedge clk);
        idx = 0;
        for (j = 0; j < NP; j = j + 1)
            for (i = 0; i < NI; i = i + 1) begin
                s_tvalid <= 1; s_tdata <= inv[idx]; s_tuser <= i;
                s_tlast <= (i == NI-1);
                idx = idx + 1;
                @(posedge clk);
            end
        s_tvalid <= 0; s_tlast <= 0;
    end
    // dump sort arrays when entering S_MED for the first 4 bins
    always @(posedge clk) begin
        if (dut.st == 4 && shown < 4) begin   // S_MED
            $display("S_MED bin=%0d sre=%0d %0d %0d %0d %0d %0d %0d %0d | bgm_next=(%0d,%0d)",
                     dut.d_idx, dut.sre[0], dut.sre[1], dut.sre[2], dut.sre[3],
                     dut.sre[4], dut.sre[5], dut.sre[6], dut.sre[7],
                     (dut.sre[3]+dut.sre[4])>>>1, (dut.sim[3]+dut.sim[4])>>>1);
            shown = shown + 1;
        end
    end
    initial begin #3000000; $finish; end
endmodule
