// ---------------------------------------------------------------------------
// cordic.v - shared iterative CORDIC coprocessor (rotation + vectoring)
// Part of the GPM RFSoC ZCU208 package.  Verilog-2001, synthesizable.
//
//   rotation  (mode=0): z0 = angle, unsigned Q0.32 fraction of a full turn.
//     xo,yo = x0*(cos 2*pi*z0, sin 2*pi*z0).  (y0 ignored, drive 0.)
//   vectoring (mode=1): xo = sqrt(x0^2+y0^2), zo = atan2(y0,x0) as an
//     unsigned Q0.32 turn fraction.  Quadrant pre-rotation included, so the
//     full circle is covered in both modes.
//
//   The internal CORDIC gain K=0.60725 is compensated (1/K on the rotation
//   input, 1/K on the vectoring magnitude output).  start -> busy, done is a
//   1-cycle pulse ITER+2 cycles later.  Outputs are registered.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module cordic #(
    parameter ITER = 18,
    parameter DW   = 26
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 start,
    input  wire                 mode,
    input  wire signed [DW-1:0] x0,
    input  wire signed [DW-1:0] y0,
    input  wire        [31:0]   z0,
    output reg  signed [DW-1:0] xo,
    output reg  signed [DW-1:0] yo,
    output reg         [31:0]   zo,
    output reg                  busy,
    output reg                  done
);
    // atan(2^-i)/(2*pi) in Q0.32 turn fractions
    reg [31:0] tab [0:23];
    integer ti;
    initial begin
        tab[0]=32'd536870912;  tab[1]=32'd316933406;  tab[2]=32'd167458907;
        tab[3]=32'd85004756;   tab[4]=32'd42667331;   tab[5]=32'd21354465;
        tab[6]=32'd10679838;   tab[7]=32'd5340245;    tab[8]=32'd2670163;
        tab[9]=32'd1335087;    tab[10]=32'd667544;    tab[11]=32'd333772;
        tab[12]=32'd166886;    tab[13]=32'd83443;     tab[14]=32'd41722;
        tab[15]=32'd20861;     tab[16]=32'd10430;     tab[17]=32'd5215;
        tab[18]=32'd2608;      tab[19]=32'd1304;      tab[20]=32'd652;
        tab[21]=32'd326;       tab[22]=32'd163;       tab[23]=32'd81;
    end
    localparam signed [24:0] K_Q24 = 25'd10188014;      // 0.60725 * 2^24
    // NOTE: the CORDIC micro-rotations inflate the magnitude by 1/K = 1.6468.
    // Rotation therefore pre-scales the amplitude by K, and vectoring
    // post-scales x by K.  Inputs must satisfy |v| < 2^(DW-1)*K to avoid
    // intermediate saturation (DW=26 -> |v| < ~2.2e7).

    reg [4:0]              i;
    reg signed [DW-1:0]    x, y;
    reg signed [32:0]      z;        // signed turn fraction, Q1.32
    reg                    tail;     // final scale-compensation cycle

    // one CORDIC micro-rotation, d = +1 (CCW) or -1 (CW)
    wire                 d    = mode ? y[DW-1] : ~z[32];  // vector: y<0 -> CCW; rot: z>=0 -> CCW
    wire signed [DW-1:0] xs   = x >>> i;
    wire signed [DW-1:0] ys   = y >>> i;
    wire signed [DW:0]   xn   = d ? (x - ys) : (x + ys);
    wire signed [DW:0]   yn   = d ? (y + xs) : (y - xs);
    wire signed [33:0]   zn   = d ? ({z[32],z} - $signed({2'b00,tab[i]}))
                                  : ({z[32],z} + $signed({2'b00,tab[i]}));

    function signed [DW-1:0] sat(input signed [DW:0] v);
        begin
            if      (v >  (1 <<< (DW-1)) - 1) sat =  (1 <<< (DW-1)) - 1;
            else if (v < -(1 <<< (DW-1)))     sat = -(1 <<< (DW-1));
            else                              sat = v[DW-1:0];
        end
    endfunction

    wire signed [2*DW:0] xk = x * K_Q24;      // undo the CORDIC gain
    // rotation amplitude pre-scaled by 1/K
    wire signed [DW-1:0] sA = sat(({{(DW+1){x0[DW-1]}}, x0} * K_Q24) >>> 24);

    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; done <= 1'b0; i <= 5'd0; tail <= 1'b0;
            x <= 0; y <= 0; z <= 0; xo <= 0; yo <= 0; zo <= 0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                busy <= 1'b1;
                i    <= 5'd0;
                tail <= 1'b0;
                zo   <= 32'd0;
                if (mode) begin
                    // vectoring: pre-rotate left half-plane into the right
                    if (x0[DW-1]) begin                 // x0 < 0
                        if (!y0[DW-1]) begin            // y >= 0: rotate by -90 deg
                            x <=  y0;  y <= -x0;
                            z <=  33'sd1073741824;      // add the 0.25 turn back
                        end else begin                  // y < 0: rotate by +90 deg
                            x <= -y0;  y <=  x0;
                            z <= -33'sd1073741824;
                        end
                    end else begin
                        x <= x0; y <= y0; z <= 33'sd0;
                    end
                end else begin
                    // rotation: pre-scale by 1/K, fold quadrant of z0 into (x,y)
                    // q=0:(A,0) q=1:(0,A) q=2:(-A,0) q=3:(0,-A)
                    case (z0[31:30])
                        2'd0: begin x <=  sA; y <=  0;  end
                        2'd1: begin x <=  0;  y <=  sA; end
                        2'd2: begin x <= -sA; y <=  0;  end
                        2'd3: begin x <=  0;  y <= -sA; end
                    endcase
                    z <= $signed({1'b0, z0[29:0]});   // residual angle, Q1.32 turns  // residual in [0,0.25) turn, Q1.32
                end
            end else if (busy) begin
                if (tail) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    if (mode) begin
                        xo <= sat(xk >>> 24);   // |v| with 1/K applied
                        yo <= 0;
                        zo <= z[31:0];          // accumulated angle, Q0.32 turns
                    end else begin
                        xo <= x; yo <= y;       // (A cos, A sin)
                    end
                end else begin
                    x <= sat(xn);
                    y <= sat(yn);
                    z <= zn[32:0];
                    if (i == ITER-1) tail <= 1'b1;
                    else             i <= i + 5'd1;
                end
            end
        end
    end
endmodule
