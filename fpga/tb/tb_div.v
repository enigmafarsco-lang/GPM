// tb_div.v - randomised divider check
`timescale 1ns / 1ps
module tb_div;
    reg clk = 0, rst = 1, start = 0;
    reg [31:0] num; reg [15:0] den;
    wire [31:0] quo; wire [15:0] rem; wire busy, done;
    integer errors = 0, t;
    reg [31:0] seed = 32'h1234_5678;
    reg [31:0] r2;

    gpr_div dut (.clk(clk), .rst(rst), .start(start), .num(num), .den(den),
                 .quo(quo), .rem(rem), .busy(busy), .done(done));
    always #2 clk = ~clk;


    initial begin
        repeat (3) @(posedge clk); rst = 0;
        for (t = 0; t < 300; t = t + 1) begin
            seed = seed * 32'd1664525 + 32'd1013904223;
            num = seed >> (t % 8);           // spread magnitudes
            seed = seed * 32'd1664525 + 32'd1013904223;
            r2  = seed;
            den = r2[15:0];
            if (den == 0) den = 1;
            @(posedge clk);
            start <= 1;
            @(posedge clk);
            start <= 0;
            @(posedge done);
            @(posedge clk);
            if (quo !== num / den || rem !== num % den) begin
                $display("DIV fail: %0u/%0u -> %0u r %0u (want %0u r %0u)",
                         num, den, quo, rem, num/den, num%den);
                errors = errors + 1;
            end
        end
        if (errors == 0) $display("TB_DIV PASS");
        else             $display("TB_DIV FAIL (%0d)", errors);
        $finish;
    end
    initial begin #2000000; $display("TB_DIV TIMEOUT"); $finish; end
endmodule
