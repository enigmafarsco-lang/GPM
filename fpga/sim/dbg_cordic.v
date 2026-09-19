`timescale 1ns / 1ps
module dbg_cordic;
    reg clk = 0, rst = 1, start = 0, mode = 0;
    reg signed [25:0] x0 = 0, y0 = 0;
    reg [31:0] z0 = 0;
    wire signed [25:0] xo, yo;
    wire [31:0] zo;
    wire busy, done;
    cordic dut (.clk(clk), .rst(rst), .start(start), .mode(mode),
                .x0(x0), .y0(y0), .z0(z0), .xo(xo), .yo(yo), .zo(zo),
                .busy(busy), .done(done));
    always #2 clk = ~clk;
    initial begin
        repeat (3) @(posedge clk); rst = 0;
        @(posedge clk);
        // rotation of a=0.96875 turns, amp = 2^24
        mode <= 0; x0 <= 26'sd16777216; y0 <= 0;
        z0 <= 32'hF8000000;
        @(posedge clk);
        start <= 1;
        @(posedge clk);
        start <= 0;
        repeat (25) begin
            @(posedge clk);
            $display("i=%0d tail=%b d=%b x=%0d y=%0d z=%0d",
                     dut.i, dut.tail, dut.d, dut.x, dut.y, dut.z);
        end
        $display("xo=%0d yo=%0d (want %0d, %0d)", xo, yo,
                 $rtoi(16777216.0*$cos(0.96875*6.283185307179586)),
                 $rtoi(16777216.0*$sin(0.96875*6.283185307179586)));
        // simple sanity: a=0 should give (A, 0)
        @(posedge clk);
        z0 <= 32'h00000000;
        @(posedge clk); start <= 1; @(posedge clk); start <= 0;
        @(posedge done); @(posedge clk);
        $display("a=0: xo=%0d yo=%0d (want 16777216, 0)", xo, yo);
        // a = 0.125 (45 deg)
        @(posedge clk);
        z0 <= 32'h20000000;
        @(posedge clk); start <= 1; @(posedge clk); start <= 0;
        @(posedge done); @(posedge clk);
        $display("a=45deg: xo=%0d yo=%0d (want %0d, %0d)", xo, yo,
                 $rtoi(16777216.0*0.7071067811865476),
                 $rtoi(16777216.0*0.7071067811865476));
        $finish;
    end
endmodule
