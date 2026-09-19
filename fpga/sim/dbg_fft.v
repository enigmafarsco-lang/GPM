`timescale 1ns / 1ps
module dbg_fft;
    localparam N = 64;
    reg clk = 0, rst = 1, load_begin = 0, s_tvalid = 0, start_fft = 0;
    reg [31:0] s_tdata; reg [7:0] s_tuser;
    wire s_tready, m_tvalid, m_tlast, busy, done;
    wire [31:0] m_tdata; wire [15:0] m_tuser; wire idle;
    gpr_fft #(.N(N), .LOG2N(6), .LOG2T(8), .TWID_FILE("twid64.hex")) dut (
        .clk(clk), .rst(rst), .inv(1'b1), .gain_q10(16'd1024),
        .load_begin(load_begin),
        .s_tvalid(s_tvalid), .s_tready(s_tready), .s_tdata(s_tdata),
        .s_tuser_tone(s_tuser), .start_fft(start_fft),
        .m_tvalid(m_tvalid), .m_tready(1'b1), .m_tdata(m_tdata),
        .m_tuser(m_tuser), .m_tlast(m_tlast), .busy(busy), .done(done),
        .idle(idle));
    always #2 clk = ~clk;
    integer n;
    reg signed [15:0] rr, ri;
    initial begin
        repeat (3) @(posedge clk); rst = 0;
        @(posedge clk);
        load_begin <= 1;
        @(posedge clk);
        load_begin <= 0;
        @(posedge clk);
        while (!s_tready) @(posedge clk);
        s_tvalid <= 1; s_tdata <= {16'sd32767, 16'sd0}; s_tuser <= 8'd1;
        @(posedge clk);
        s_tvalid <= 0;
        repeat (4) @(posedge clk);
        start_fft <= 1;
        @(posedge clk);
        start_fft <= 0;
        n = 0;
        while (n < N) begin
            @(posedge clk);
            if (m_tvalid) begin
                rr = $signed(m_tdata[31:16]); ri = $signed(m_tdata[15:0]);
                if (n < 20 || n == 32 || n == 48)
                    $display("out[%0d] = (%0d, %0d)", n, rr, ri);
                n = n + 1;
            end
        end
        $finish;
    end
    integer trc = 0;
    always @(posedge clk) begin
        if (dut.st == 4'd6 && trc < 5000) begin   // ST_WA
            $display("WA stg=%0d grp=%0d jj=%0d i0=%0d i1=%0d a=(%0d,%0d) wb=(%0d,%0d) sum=(%0d,%0d)",
                     dut.s_stg, dut.grp, dut.jj, dut.i0, dut.i1,
                     dut.a_re, dut.a_im, dut.wbr, dut.wbi,
                     dut.sum_re, dut.sum_im);
            trc = trc + 1;
        end
    end
    initial begin #2000000; $display("DBG TIMEOUT"); $finish; end
endmodule
