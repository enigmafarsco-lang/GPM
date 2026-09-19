// ---------------------------------------------------------------------------
// gpr_fft.v - B10_RangeProc second half: windowed zero-padded IFFT along the
// tone axis.  In-place radix-2 DIT engine over one block RAM:
//   ZERO  : clear the N-point buffer
//   LOAD  : stream of N_TONES windowed samples -> bit-reversed addresses
//   RUN   : LOG2N stages x N/2 butterflies (5 cycles each), >>1 per stage
//           with rounding = exact 1/N for power-of-two N, matching the
//           MATLAB ifft normalisation
//   OUT   : natural-order read-out, baked gain 18.4298 (Q10 register)
// inv=1 conjugates the twiddles (IFFT).  Twiddle ROM = exp(-j*2*pi*m/N) Q15
// from make_golden.m.  N=2048 @ 245.76 MHz: ~0.25 ms per transform, three
// orders of magnitude inside the 12.8 ms per-position budget of mode A.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_fft #(
    parameter N         = 2048,
    parameter LOG2N     = 11,
    parameter LOG2T     = 8,           // tone index width
    parameter TWID_FILE = "twid2048.hex"
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        inv,            // 1 = IFFT
    input  wire [15:0] gain_q10,
    // load stream (windowed tone vector of one position)
    input  wire        load_begin,     // pulse: zero buffer, then accept
    input  wire        s_tvalid,
    output wire        s_tready,
    input  wire [31:0] s_tdata,
    input  wire [LOG2T-1:0] s_tuser_tone,
    input  wire        start_fft,      // pulse: transform
    // result stream: N bins in natural order
    output reg         m_tvalid,
    input  wire        m_tready,
    output reg  [31:0] m_tdata,
    output reg  [15:0] m_tuser,        // depth bin index
    output reg         m_tlast,
    output reg         busy,
    output reg         done
,
    output wire idle
);
    reg [31:0] mem   [0:N-1];          // {re[31:16], im[15:0]} Q15
    reg [31:0] twrom [0:N/2-1];
    initial $readmemh(TWID_FILE, twrom);

    localparam [3:0] ST_IDLE = 0, ST_ZERO = 1, ST_LOAD = 2,
                     ST_RDA  = 3, ST_RDB  = 4, ST_CALC = 5,
                     ST_WA   = 6, ST_WB   = 7,
                     ST_OUTA = 8, ST_OUTB = 9;
    reg [3:0] st;
    reg [11:0] cnt;
    reg [LOG2N-1:0]  s_stg;
    reg [LOG2N-2:0]  grp, jj;
    reg [LOG2N-1:0]  i0, i1;
    reg signed [15:0] a_re, a_im, b_re, b_im;
    reg [31:0] rd, tw;

    assign s_tready = (st == ST_LOAD);
    assign idle     = (st == ST_IDLE);

    wire [LOG2N-1:0] tone_x = s_tuser_tone;  // truncate/zero-extend to log2(N)

    function [LOG2N-1:0] bitrev(input [LOG2N-1:0] a);
        integer bb;
        begin
            for (bb = 0; bb < LOG2N; bb = bb + 1)
                bitrev[bb] = a[LOG2N-1-bb];
        end
    endfunction
    function signed [15:0] sat16(input signed [33:0] v);
        begin
            if      (v >  34'sd32767) sat16 =  16'sd32767;
            else if (v < -34'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    // twiddle (conjugated for IFFT)
    wire signed [15:0] w_re = $signed(tw[31:16]);
    wire signed [15:0] w_im = inv ? -$signed(tw[15:0]) : $signed(tw[15:0]);

    // wb = w*b, Q15 with rounding
    wire signed [31:0] wbr = (w_re*b_re - w_im*b_im + 32'sd16384) >>> 15;
    wire signed [31:0] wbi = (w_re*b_im + w_im*b_re + 32'sd16384) >>> 15;

    // butterfly outputs with per-stage /2 rounding
    // NOTE: all operands must be *declared signed* - concatenations are
    // unsigned in Verilog and would turn >>> into a logical shift.
    wire signed [33:0] a34r = a_re, a34i = a_im;
    wire signed [33:0] w34r = wbr,  w34i = wbi;
    wire signed [33:0] sum_re = (a34r + w34r + 34'sd1) >>> 1;
    wire signed [33:0] sum_im = (a34i + w34i + 34'sd1) >>> 1;
    wire signed [33:0] dif_re = (a34r - w34r + 34'sd1) >>> 1;
    wire signed [33:0] dif_im = (a34i - w34i + 34'sd1) >>> 1;

    wire [LOG2N-1:0] half = {{(LOG2N-1){1'b0}}, 1'b1} << s_stg;   // 2^s
    wire [LOG2N-2:0] m_idx = jj << (LOG2N-1-s_stg);

    // output gain
    wire signed [15:0] g15 = $signed(gain_q10[15:0]);
    wire signed [32:0] gre = ($signed(rd[31:16]) * g15 + 33'sd512) >>> 10;
    wire signed [32:0] gim = ($signed(rd[15:0])  * g15 + 33'sd512) >>> 10;

    always @(posedge clk) begin
        if (rst) begin
            st <= ST_IDLE; cnt <= 0; busy <= 0; done <= 0;
            m_tvalid <= 0; m_tdata <= 0; m_tuser <= 0; m_tlast <= 0;
            s_stg <= 0; grp <= 0; jj <= 0; i0 <= 0; i1 <= 0;
            a_re <= 0; a_im <= 0; b_re <= 0; b_im <= 0; rd <= 0; tw <= 0;
        end else begin
            done <= 1'b0;
            if (st == ST_RDA || st == ST_RDB)
                tw <= twrom[m_idx];
            case (st)
            ST_IDLE: begin
                // hold the final output beat until it is accepted (ST_OUTB
                // enters IDLE with m_tvalid still set on the last beat)
                if (m_tvalid && m_tready)
                    m_tvalid <= 1'b0;
                if ((!m_tvalid || m_tready) && load_begin) begin
                    st <= ST_ZERO; cnt <= 0; busy <= 1'b1;
                end
            end
            ST_ZERO: begin
                mem[cnt[LOG2N-1:0]] <= 32'd0;
                if (cnt == N-1) begin
                    st <= ST_LOAD; busy <= 1'b0; cnt <= 0;
                end else
                    cnt <= cnt + 1;
            end
            ST_LOAD: begin
                if (s_tvalid)
                    mem[bitrev(tone_x)] <= s_tdata;
                if (start_fft) begin
                    st <= ST_RDA; busy <= 1'b1;
                    s_stg <= 0; grp <= 0; jj <= 0;
                    i0 <= 0;    i1 <= {{(LOG2N-1){1'b0}}, 1'b1};
                end
            end
            ST_RDA: begin
                rd <= mem[i0];
                st <= ST_RDB;
            end
            ST_RDB: begin
                a_re <= $signed(rd[31:16]); a_im <= $signed(rd[15:0]);
                rd   <= mem[i1];
                st   <= ST_CALC;
            end
            ST_CALC: begin
                b_re <= $signed(rd[31:16]); b_im <= $signed(rd[15:0]);
                st   <= ST_WA;
            end
            ST_WA: begin
                mem[i0] <= {sat16(sum_re), sat16(sum_im)};
                st <= ST_WB;
            end
            ST_WB: begin
                mem[i1] <= {sat16(dif_re), sat16(dif_im)};
                if (jj != half - 1) begin                 // next butterfly in group
                    jj <= jj + 1;
                    i0 <= i0 + 1;
                    i1 <= i1 + 1;
                    st <= ST_RDA;
                end else if (grp != (N >> (s_stg + 1)) - 1) begin   // next group
                    grp <= grp + 1;
                    jj  <= 0;
                    i0  <= i0 + half + 1;
                    i1  <= i1 + half + 1;
                    st  <= ST_RDA;
                end else if (s_stg != LOG2N-1) begin      // next stage
                    s_stg <= s_stg + 1;
                    grp   <= 0;
                    jj    <= 0;
                    i0    <= 0;
                    i1    <= half << 1;
                    st    <= ST_RDA;
                end else begin                            // transform done
                    st   <= ST_OUTA;
                    cnt  <= 0;
                    busy <= 1'b0;
                end
            end
            ST_OUTA: begin
                if (m_tvalid && m_tready)
                    m_tvalid <= 1'b0;   // previous beat consumed
                rd <= mem[cnt[LOG2N-1:0]];
                st <= ST_OUTB;
            end
            ST_OUTB: begin
                if (!m_tvalid || m_tready) begin
                    m_tdata  <= {sat16({{1{gre[32]}}, gre}), sat16({{1{gim[32]}}, gim})};
                    m_tuser  <= cnt;
                    m_tlast  <= (cnt == N-1);
                    m_tvalid <= 1'b1;
                    if (cnt == N-1) begin
                        st   <= ST_IDLE;
                        cnt  <= 0;
                        done <= 1'b1;
                    end else begin
                        cnt <= cnt + 1;
                        st  <= ST_OUTA;
                    end
                end
            end
            default: st <= ST_IDLE;
            endcase
        end
    end
endmodule
