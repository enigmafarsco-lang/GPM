// tb_cordic.v - self-checking CORDIC verification against $cos/$sin/$atan2
`timescale 1ns / 1ps
module tb_cordic;
    reg clk = 0, rst = 1, start = 0, mode = 0;
    reg signed [25:0] x0 = 0, y0 = 0;
    reg [31:0] z0 = 0;
    wire signed [25:0] xo, yo;
    wire [31:0] zo;
    wire busy, done;
    integer errors = 0;
    integer k;
    real a, ex, ey, got;
    reg [31:0] adiff;
    reg [32:0] aerr;

    cordic #(.ITER(18), .DW(26)) dut (
        .clk(clk), .rst(rst), .start(start), .mode(mode),
        .x0(x0), .y0(y0), .z0(z0), .xo(xo), .yo(yo), .zo(zo),
        .busy(busy), .done(done));

    always #2 clk = ~clk;

    task run(input m, input signed [25:0] xx, input signed [25:0] yy, input [31:0] zz);
        begin
            @(posedge clk);
            mode <= m; x0 <= xx; y0 <= yy; z0 <= zz; start <= 1;
            @(posedge clk);
            start <= 0;
            @(posedge done);
            @(posedge clk);
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        rst = 0;
        repeat (2) @(posedge clk);

        // ---- rotation: (A,0) at angle a turns -> (A cos, A sin)
        for (k = 0; k < 32; k = k + 1) begin
            a = k / 32.0;
            run(1'b0, 26'sd16777216, 0, $rtoi(a * 4294967296.0));
            ex = 16777216.0 * $cos(2*3.14159265358979*a);
            ey = 16777216.0 * $sin(2*3.14159265358979*a);
            got = $itor(xo) - ex;
            if (got < 0) got = -got;
            if (got > 400) begin
                $display("ROT cos fail a=%f: got %0d want %0d", a, xo, $rtoi(ex));
                errors = errors + 1;
            end
            got = $itor(yo) - ey;
            if (got < 0) got = -got;
            if (got > 400) begin
                $display("ROT sin fail a=%f: got %0d want %0d", a, yo, $rtoi(ey));
                errors = errors + 1;
            end
        end

        // ---- vectoring: magnitude + angle
        run(1'b1, 26'sd1000000, 26'sd1000000, 0);
        ex = 1000000.0 * $sqrt(2.0);
        got = $itor(xo) - ex;
        if (got < 0) got = -got;
        if (got/ex > 0.002) begin
            $display("VEC mag fail: got %0d want %0d", xo, $rtoi(ex));
            errors = errors + 1;
        end
        adiff = zo - 32'd536870912;        // 0.125 turns
        aerr  = adiff[31] ? (33'h100000000 - {1'b0, adiff}) : {1'b0, adiff};
        if (aerr > 33'd200000) begin
            $display("VEC ang fail: got %0d want 536870912 err %0d", zo, aerr);
            errors = errors + 1;
        end
        run(1'b1, -26'sd1000000, 26'sd1000000, 0);   // 2nd quadrant: 0.375
        adiff = zo - 32'd1610612736;       // 0.375 turns
        aerr  = adiff[31] ? (33'h100000000 - {1'b0, adiff}) : {1'b0, adiff};
        if (aerr > 33'd200000) begin
            $display("VEC ang2 fail: got %0d want 1610612736 err %0d", zo, aerr);
            errors = errors + 1;
        end
        run(1'b1, -26'sd1000000, -26'sd1000000, 0);  // 3rd quadrant: 0.625
        adiff = zo - 32'd2684354560;       // 0.625 turns
        aerr  = adiff[31] ? (33'h100000000 - {1'b0, adiff}) : {1'b0, adiff};
        if (aerr > 33'd200000) begin
            $display("VEC ang3 fail: got %0d want 2684354560 err %0d", zo, aerr);
            errors = errors + 1;
        end
        run(1'b1, 26'sd0, -26'sd1000000, 0);         // -y axis: 0.75
        adiff = zo - 32'd3221225472;       // 0.75 turns
        aerr  = adiff[31] ? (33'h100000000 - {1'b0, adiff}) : {1'b0, adiff};
        if (aerr > 33'd200000) begin
            $display("VEC ang4 fail: got %0d want 3221225472 err %0d", zo, aerr);
            errors = errors + 1;
        end

        if (errors == 0) $display("TB_CORDIC PASS");
        else             $display("TB_CORDIC FAIL (%0d errors)", errors);
        $finish;
    end
    initial begin
        #200000;
        $display("TB_CORDIC TIMEOUT");
        $finish;
    end
endmodule
