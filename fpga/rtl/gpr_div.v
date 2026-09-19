// ---------------------------------------------------------------------------
// gpr_div.v - restoring integer divider: quo = floor(num/den), rem = num%den
// num: unsigned 32b, den: unsigned 16b (0 treated as 1), 33 cycles.
// start -> busy; done is a 1-cycle pulse with quo/rem valid.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_div (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [31:0] num,
    input  wire [15:0] den,
    output reg  [31:0] quo,
    output reg  [15:0] rem,
    output reg         busy,
    output reg         done
);
    reg [5:0]  i;
    reg [16:0] acc;
    reg [31:0] q;
    reg [15:0] d;

    wire [16:0] acc17 = {acc[15:0], (i < 32) ? num[i] : 1'b0};
    wire        sub   = (acc17 >= {1'b0, d});
    wire [16:0] accn  = sub ? (acc17 - {1'b0, d}) : acc17;

    always @(posedge clk) begin
        if (rst) begin
            busy <= 0; done <= 0; i <= 0; acc <= 0; q <= 0; d <= 0;
            quo <= 0; rem <= 0;
        end else begin
            done <= 0;
            if (start && !busy) begin
                busy <= 1; i <= 31; acc <= 17'd0; q <= 32'd0;
                d <= (den == 0) ? 16'd1 : den;
            end else if (busy) begin
                acc <= accn;
                if (sub) q <= q | (32'd1 << i);
                if (i == 0) begin
                    busy <= 0; done <= 1;
                    quo  <= sub ? (q | 32'd1) : q;
                    rem  <= accn[15:0];
                end else
                    i <= i - 1;
            end
        end
    end
endmodule
