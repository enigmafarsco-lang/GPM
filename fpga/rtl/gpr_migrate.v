// ---------------------------------------------------------------------------
// gpr_migrate.v - B12_Migration: coherent Kirchhoff migration of the
// background-subtracted frame.  Mirrors code_migration.m in structure and
// weight law (aperture plan via the nap ROM from make_golden.m, linear
// depth interpolation, obliquity/spreading/attenuation/taper weights,
// coherent phase exp(+j*2*pi*kbeta*R), |acc|/sqrt(nc) output) in fixed
// point:
//
//   geometry   Q20 metres (1 um LSB): z, xs, R = sqrt(z^2+xs^2) via CORDIC
//   fb         Q16 bins: fb = 1 + (R + r_off)/dr, 64-bit product
//   weights    ratio=z/R Q15 (divider), 1/sqrt(R) Q13 (Newton sqrt + div),
//              exp(-2a(R-z)) Q15 (1024-entry LUT, u in 1/128 steps),
//              taper 0.5(1+cos(pi s/(nap+1))) Q15 (divider + CORDIC)
//   W[s]       w * exp(j*2*pi*kbeta*R), complex Q25
//   pixels     acc (64-bit) += W[s] * interp(dat), mag via CORDIC,
//              /sqrt(nc) via LUT, x out_scale_q8, saturating unsigned
//
// The bs frame is collected into a local buffer; the engine auto-starts on
// s_frame_last and back-pressures the stream until it is done (frame period
// 2.56 s >> ~25 ms engine time at mode-A sizes, so the stall is invisible).
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_migrate #(
    parameter N_IFFT   = 2048,
    parameter N_POS    = 200,
    parameter LB       = 11,
    parameter NAP_MAX  = 64,
    parameter NAP_FILE = "nap_a.hex",
    parameter EXP_FILE = "exp_lut.hex",
    parameter INS_FILE = "invsqrt_nc.hex"
) (
    input  wire        clk,
    input  wire        rst,
    // bs frame stream in (column-major from gpr_background)
    input  wire        s_tvalid,
    output wire        s_tready,
    input  wire [31:0] s_tdata,
    input  wire [15:0] s_tuser,          // depth bin
    input  wire [7:0]  s_tuser_pos,
    input  wire        s_tlast,
    input  wire        s_frame_last,
    // physical constants (make_golden / AXI-Lite registers)
    input  wire [31:0] inv_dr_q20,
    input  wire [31:0] roff_q20,
    input  wire [31:0] dx_q20,
    input  wire [31:0] dr_q20,
    input  wire [31:0] kbeta_q16,
    input  wire signed [15:0] alpha2_q8,
    input  wire [15:0] max_bin,
    input  wire [15:0] out_scale_q8,
    input  wire        go,
    // mig frame stream out (iz outer, ix inner), unsigned 16
    output reg         m_tvalid,
    input  wire        m_tready,
    output reg  [15:0] m_tdata,
    output reg  [15:0] m_tuser,          // iz (0-based)
    output reg  [7:0]  m_tuser_pos,      // ix (0-based)
    output reg         m_tlast,          // end of row
    output reg         m_frame_last,
    output reg         busy
);
    localparam DEPTH = N_IFFT * N_POS;
    reg [31:0] dat [0:DEPTH-1];          // bs frame: addr = (pos<<LB)|bin
    reg [7:0]  nap_rom [0:2047];
    reg [15:0] exp_rom [0:1023];
    reg [15:0] ins_rom [0:63];
    initial begin
        $readmemh(NAP_FILE, nap_rom);
        $readmemh(EXP_FILE, exp_rom);
        $readmemh(INS_FILE, ins_rom);
    end

    // W table for the current depth row
    reg signed [31:0] w_re [0:2*NAP_MAX];
    reg signed [31:0] w_im [0:2*NAP_MAX];
    reg        [15:0] b0_ram [0:2*NAP_MAX];   // 0-based bin, 0x7FFF = invalid
    reg        [15:0] fr_ram [0:2*NAP_MAX];   // Q16

    // ------------------------------------------------------------- helpers
    reg         c_start, c_mode;
    reg signed [25:0] c_x0, c_y0;
    reg  [31:0] c_z0;
    wire signed [25:0] c_xo, c_yo;
    wire [31:0] c_zo;
    wire c_busy, c_done;
    cordic #(.ITER(18), .DW(26)) u_cordic (
        .clk(clk), .rst(rst), .start(c_start), .mode(c_mode),
        .x0(c_x0), .y0(c_y0), .z0(c_z0),
        .xo(c_xo), .yo(c_yo), .zo(c_zo), .busy(c_busy), .done(c_done));

    reg  [31:0] d_num;
    reg  [15:0] d_den;
    reg         d_start;
    wire [31:0] d_quo;
    wire        d_busy, d_done;
    gpr_div u_div (
        .clk(clk), .rst(rst), .start(d_start), .num(d_num), .den(d_den),
        .quo(d_quo), .rem(), .busy(d_busy), .done(d_done));

    // ------------------------------------------------------------- states
    localparam [4:0]
        M_IDLE=0,  M_IZ=1,   M_IZ2=2,   M_WS=3,   M_WR=4,
        M_FB=5,    M_FB2=6,  M_SQ0=7,   M_SQ1=8,  M_SQ2=9,
        M_RATIO=10,M_IR=11,  M_TAP=13,  M_PH=14,
        M_WMUL=15, M_PIX0=16,M_PTAP=17, M_PRD0=18,M_PRD1=19,
        M_PRD2=24, M_PACC=25, M_PMAG=20, M_PMAGw=21, M_POUT=22, M_DONE=23;
    reg [4:0] st;

    reg [15:0] iz0, ix, sp;
    reg signed [31:0] z_q20, xs_q20, r_q20;
    reg signed [47:0] fb_q16s;
    reg [31:0] recip_r;                  // 2^31/(nap+1)
    reg [31:0] xsq_x;
    reg signed [31:0] g;
    reg [3:0]  nit;
    reg signed [23:0] ratio_q15, isqr_q13, expv_q15, tapq_q15, w_q13;
    reg signed [63:0] acc_re, acc_im;
    reg [8:0]  nc;
    reg [7:0]  nap;
    reg        zero_row;                 // iz outside [1, max_bin): emit zeros
    reg [31:0] rdreg, rdreg2;
    reg signed [17:0] d0_re, d0_im;
    reg signed [17:0] val_re, val_im;
    reg [15:0] outpix;
    reg        frame_have;

    assign s_tready = !frame_have && (st == M_IDLE);

    // tap geometry (combinational)
    wire signed [15:0] ssigned = $signed({8'd0, sp[7:0]}) - $signed({8'd0, nap});
    wire signed [47:0] xs_c    = ssigned * $signed({16'd0, dx_q20[31:0]});
    wire signed [15:0] it_c    = $signed({8'd0, ix[7:0]}) + ssigned;

    // fb = 1 + (R + r_off)/dr, Q16  (64-bit product >> 24)
    wire signed [63:0] fbprod = (r_q20 + $signed({16'd0, roff_q20})) *
                                $signed({16'd0, inv_dr_q20});

    // weight chain (combinational, registered at the end of the tap)
    wire signed [63:0] wprod1 = ratio_q15 * isqr_q13;          // Q28
    wire signed [31:0] wa     = wprod1 >>> 18;                 // Q10
    wire signed [63:0] wprod2 = wa * expv_q15;                 // Q25
    wire signed [31:0] wb     = wprod2 >>> 15;                 // Q10
    wire signed [63:0] wprod3 = wb * tapq_q15;                 // Q25
    wire signed [31:0] wc     = wprod3 >>> 15;                 // Q10
    // W = wc (Q10) * phase vector (Q24) -> Q34 product, stored Q25 (>>>9)
    wire signed [63:0] wmul_re = wc * c_xo;
    wire signed [63:0] wmul_im = wc * c_yo;
    // phase turns fraction = kbeta_q16 * R_q20 * 2^-4 (Q32, wraps)
    wire [63:0] phprod = kbeta_q16 * r_q20[31:0];
    wire [31:0] ph_q32 = phprod[35:4];      // Q36 -> Q32 turn fraction
    // taper angle fraction = |s| * recip_r  (Q32 turns)
    wire [15:0] abs_s   = ssigned[15] ? (~ssigned + 16'd1) : ssigned;
    wire [31:0] tap_ang = abs_s * recip_r;      // < 2^31 by construction

    // interpolation
    wire signed [17:0] diff_re = $signed({{2{rdreg2[31]}}, rdreg2[31:16]}) - d0_re;
    wire signed [17:0] diff_im = $signed({{2{rdreg2[15]}}, rdreg2[15:0]})  - d0_im;
    wire signed [39:0] fmul_re = $signed({24'd0, fr_ram[sp[7:0]]}) * diff_re;
    wire signed [39:0] fmul_im = $signed({24'd0, fr_ram[sp[7:0]]}) * diff_im;
    // NOTE: sign-extension concatenations must be wrapped in $signed() before
    // arithmetic that feeds a signed function input - a bare concat makes the
    // whole expression unsigned (and iverilog mangles the width).
    wire signed [39:0] valx_re = $signed({{22{d0_re[17]}}, d0_re}) + (fmul_re >>> 16);
    wire signed [39:0] valx_im = $signed({{22{d0_im[17]}}, d0_im}) + (fmul_im >>> 16);

    // exp LUT index from u = alpha2*(R-z) (Q8*Q12=Q20 -> >>11 = 1/128 steps)
    wire signed [31:0] diff12 = (r_q20 - z_q20) >>> 8;
    wire signed [47:0] u_q20  = alpha2_q8 * diff12;
    wire [31:0]        exp_idx_raw = (u_q20 < 0) ? 32'd0 : u_q20[31:13];  // idx = u*128 (LUT: e^-i/128)
    wire [31:0]        exp_idx = (exp_idx_raw > 32'd1023) ? 32'd1023 : exp_idx_raw;

    function signed [25:0] sat26(input signed [63:0] v);
        begin
            if      (v >  64'sd33554431) sat26 =  26'sd33554431;
            else if (v < -64'sd33554432) sat26 = -26'sd33554432;
            else                         sat26 = v[25:0];
        end
    endfunction
    function [15:0] sat16u(input signed [47:0] v);
        begin
            if      (v < 0)           sat16u = 16'd0;
            else if (v > 48'sd65535)  sat16u = 16'd65535;
            else                      sat16u = v[15:0];
        end
    endfunction
    function signed [15:0] sat16s(input signed [39:0] v);
        begin
            if      (v >  40'sd32767) sat16s =  16'sd32767;
            else if (v < -40'sd32768) sat16s = -16'sd32768;
            else                      sat16s = v[15:0];
        end
    endfunction

    wire tap_valid = (b0_ram[sp[7:0]] != 16'h7FFF);
    wire signed [26:0] taper_sum = 27'sd16777216 + $signed({1'b0, c_xo});
    wire signed [26:0] taper_sh  = taper_sum >>> 10;
    wire [LB+7:0] rdaddr0 = {it_c[7:0], b0_ram[sp[7:0]][LB-1:0]};
    wire [LB+7:0] rdaddr1 = {it_c[7:0], b0_ram[sp[7:0]][LB-1:0] + 1'b1};

    always @(posedge clk) begin
        if (rst) begin
            st <= M_IDLE; busy <= 0; frame_have <= 0;
            iz0 <= 0; ix <= 0; sp <= 0; nc <= 0; nap <= 0; nit <= 0; g <= 0;
            acc_re <= 0; acc_im <= 0; recip_r <= 0; xsq_x <= 0; outpix <= 0;
            c_start <= 0; c_mode <= 0; c_x0 <= 0; c_y0 <= 0; c_z0 <= 0;
            d_start <= 0; d_num <= 0; d_den <= 0;
            m_tvalid <= 0; m_tdata <= 0; m_tuser <= 0; m_tuser_pos <= 0;
            m_tlast <= 0; m_frame_last <= 0;
            z_q20 <= 0; xs_q20 <= 0; r_q20 <= 0; fb_q16s <= 0; zero_row <= 0;
            ratio_q15 <= 0; isqr_q13 <= 0; expv_q15 <= 0; tapq_q15 <= 0; w_q13 <= 0;
            rdreg <= 0; rdreg2 <= 0; d0_re <= 0; d0_im <= 0; val_re <= 0; val_im <= 0;
        end else begin
            if (m_tvalid && m_tready)
                m_tvalid <= 1'b0;   // beat consumed - drop valid until next emit
            c_start <= 1'b0;
            d_start <= 1'b0;

            // collect bs frame
            if (s_tvalid && s_tready) begin
                dat[{s_tuser_pos, s_tuser[LB-1:0]}] <= s_tdata;
                if (s_frame_last) frame_have <= 1'b1;
            end

            case (st)
            M_IDLE: begin
                m_tvalid <= 1'b0;
                if (frame_have && go) begin
                    frame_have <= 1'b0;
                    busy <= 1'b1;
                    iz0  <= 16'd0;             // row 0 emitted as zeros (frame geometry)
                    st   <= M_IZ;
                end
            end

            M_IZ: begin
                if (iz0 >= N_IFFT) begin
                    st <= M_DONE;
                end else if (iz0 == 0 || iz0 >= max_bin) begin
                    // MATLAB fills these rows with zeros - keep the frame
                    // geometry intact for CFAR/cluster
                    zero_row <= 1'b1;
                    outpix   <= 16'd0;
                    ix       <= 0;
                    st       <= M_POUT;
                end else begin
                    zero_row <= 1'b0;
                    z_q20 <= $signed(iz0) * $signed({1'b0, dr_q20[30:0]});
                    nap   <= nap_rom[iz0];
                    // recip = 2^31/(nap+1) for the taper angle
                    d_num   <= 32'h8000_0000;
                    d_den   <= {8'd0, nap_rom[iz0]} + 16'd1;
                    d_start <= 1'b1;
                    st      <= M_IZ2;
                end
            end
            M_IZ2: if (d_done) begin
                recip_r <= d_quo;
                sp      <= 0;
                st      <= M_WS;
            end

            // ---------------- W table build ----------------
            M_WS: begin
                if (sp > {8'd0, nap} + {8'd0, nap}) begin
                    ix <= 0;
                    st <= M_PIX0;
                end else begin
                    xs_q20  <= xs_c[31:0];
                    c_mode  <= 1'b1;
                    c_x0    <= z_q20[25:0];
                    c_y0    <= xs_c[25:0];
                    c_start <= 1'b1;
                    st      <= M_WR;
                end
            end
            M_WR: if (c_done) begin
                r_q20 <= {6'd0, c_xo};         // magnitude >= 0
                st    <= M_FB;
            end
            M_FB: begin
                fb_q16s <= (fbprod >>> 24) + 48'sd65536;   // +1 in Q16
                st <= M_FB2;
            end
            M_FB2: begin
                if (fb_q16s < 48'sd131072 || (fb_q16s >>> 16) > (N_IFFT-1)) begin
                    b0_ram[sp[7:0]] <= 16'h7FFF;           // invalid tap
                    fr_ram[sp[7:0]] <= 16'd0;
                end else begin
                    b0_ram[sp[7:0]] <= fb_q16s[26:16] - 17'd1;  // 0-based
                    fr_ram[sp[7:0]] <= fb_q16s[15:0];
                end
                xsq_x <= {r_q20[27:0], 4'd0};              // X = R * 2^24
                g     <= 32'sd8192;
                nit   <= 4'd0;
                st    <= M_SQ0;
            end
            M_SQ0: begin
                d_num   <= xsq_x;
                d_den   <= g[15:0];
                d_start <= 1'b1;
                st      <= M_SQ1;
            end
            M_SQ1: if (d_done) begin
                g  <= (g + $signed({1'b0, d_quo[30:0]})) >>> 1;
                st <= M_SQ2;
            end
            M_SQ2: begin
                if (nit == 4'd6) begin
                    // ratio = z/R, Q15: num=(z>>8)<<15 = z<<7, den=R>>8
                    d_num   <= {z_q20[24:0], 7'd0};
                    d_den   <= r_q20[23:8];
                    d_start <= 1'b1;
                    st      <= M_RATIO;
                end else begin
                    nit <= nit + 1;
                    st  <= M_SQ0;
                end
            end
            M_RATIO: if (d_done) begin
                ratio_q15 <= $signed(d_quo[16:0]);
                d_num     <= 32'd33554432;     // 2^25 -> isqrt in Q13
                d_den     <= g[15:0];
                d_start   <= 1'b1;
                st        <= M_IR;
            end
            M_IR: if (d_done) begin
                isqr_q13 <= $signed(d_quo[19:0]);
                expv_q15 <= $signed({1'b0, exp_rom[exp_idx[9:0]]});
                // taper: cos(2*pi * |s|/(2(nap+1)))
                c_mode  <= 1'b0;
                c_x0    <= 26'sd16777216;      // 1.0 in Q24
                c_y0    <= 26'sd0;
                c_z0    <= tap_ang;
                c_start <= 1'b1;
                st      <= M_TAP;
            end
            M_TAP: if (c_done) begin
                // taper_q15 = (1 + cos)*2^14  (clamped)
                tapq_q15 <= (taper_sh > 27'sd32767) ? 16'sd32767 : taper_sh[15:0];
                w_q13    <= 0;                 // (final w computed at M_WMUL)
                // phase rotation
                c_mode   <= 1'b0;
                c_x0     <= 26'sd16777216;
                c_y0     <= 26'sd0;
                c_z0     <= ph_q32;
                c_start  <= 1'b1;
                st       <= M_PH;
            end
            M_PH: if (c_done) begin
                st <= M_WMUL;
            end
            M_WMUL: begin
                // w_q10 chain, then W = w * (cos_p + j sin_p) in Q25
                w_re[sp[7:0]] <= wmul_re >>> 9;
                w_im[sp[7:0]] <= wmul_im >>> 9;
                sp <= sp + 1;
                st <= M_WS;
            end

            // ---------------- pixel loop ----------------
            M_PIX0: begin
                acc_re <= 0; acc_im <= 0; nc <= 0; sp <= 0;
                st <= M_PTAP;
            end
            M_PTAP: begin
                if (sp > {8'd0, nap} + {8'd0, nap}) begin
                    if (nc == 0) begin
                        outpix <= 16'd0;
                        st     <= M_POUT;
                    end else begin
                        c_mode  <= 1'b1;
                        // acc is Q40 (w Q25 x val Q15) -> >>>25 gives Q15
                        c_x0    <= sat26(acc_re >>> 25);
                        c_y0    <= sat26(acc_im >>> 25);
                        c_start <= 1'b1;
                        st      <= M_PMAG;
                    end
                end else if (it_c < 0 || it_c >= N_POS || !tap_valid) begin
                    sp <= sp + 1;              // skip out-of-swath / invalid tap
                end else begin
                    st <= M_PRD0;              // issue dat read of bin b0
                end
            end
            M_PRD0: begin
                rdreg <= dat[rdaddr0];         // BRAM read latency 1
                st    <= M_PRD1;
            end
            M_PRD1: begin
                d0_re  <= $signed({{2{rdreg[31]}}, rdreg[31:16]});
                d0_im  <= $signed({{2{rdreg[15]}}, rdreg[15:0]});
                rdreg2 <= dat[rdaddr1];
                st     <= M_PRD2;
            end
            M_PRD2: begin
                // val = d0 + (fr*(d1-d0) >>> 16), Q15
                val_re <= sat16s(valx_re);
                val_im <= sat16s(valx_im);
                st     <= M_PACC;
            end
            M_PACC: begin
                acc_re <= acc_re + ($signed(w_re[sp[7:0]]) * val_re -
                                    $signed(w_im[sp[7:0]]) * val_im);
                acc_im <= acc_im + ($signed(w_re[sp[7:0]]) * val_im +
                                    $signed(w_im[sp[7:0]]) * val_re);
                nc <= nc + 1;
                sp <= sp + 1;
                st <= M_PTAP;
            end

            M_PMAG: if (c_done) begin
                st <= M_PMAGw;
            end
            M_PMAGw: begin
                // out = mag(Q15) * invsqrt(nc) Q15 >>15, * out_scale >>8
                outpix <= sat16u((($signed({1'b0, c_xo}) *
                                  $signed({1'b0, ins_rom[nc-1]})) >>> 15) *
                                 $signed({1'b0, out_scale_q8}) >>> 8);
                st <= M_POUT;
            end
            M_POUT: begin
                if (!m_tvalid || m_tready) begin
                    m_tvalid    <= 1'b1;
                    m_tdata     <= outpix;
                    m_tuser     <= iz0;
                    m_tuser_pos <= ix[7:0];
                    m_tlast     <= (ix == N_POS-1);
                    m_frame_last<= (ix == N_POS-1) && (iz0 == N_IFFT-1);
                    if (ix == N_POS-1) begin
                        ix  <= 0;
                        iz0 <= iz0 + 1;
                        st  <= M_IZ;
                    end else begin
                        ix <= ix + 1;
                        st <= zero_row ? M_POUT : M_PIX0;
                    end
                end
            end
            M_DONE: begin
                busy <= 1'b0;
                st   <= M_IDLE;
            end
            default: st <= M_IDLE;
            endcase
        end
    end
endmodule
