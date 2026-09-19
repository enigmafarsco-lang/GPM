// ---------------------------------------------------------------------------
// gpr_cfar.v - B13_Detection: 2-D CA-CFAR over the migrated frame, exactly
// mirroring code_detection.m: separable (2w+1)^2 training box minus
// (2g+1)^2 guard box over the validity-masked power map with 'same'
// (zero-padded) convolution semantics; mean estimator (default) or log.
//
// Integer-exact mean mode (no divider):
//   det = pmap*cnt*2^12 > alpha_q12*(big_sum - g_sum)
// Log mode compares in the log2 domain:
//   l_c > (Lbig-Lg)*inv_cnt[cnt] >> 30 + gamma/ln2 + log2(ln(1/pfa))
//
// Flow: collect the mig frame -> max pre-pass -> p_floor = (max^2 * K)>>SH
// (same integers as MATLAB) -> row pipeline: for each row pass rp the
// vertical delay-line chains (49 and 17 slots, circular BRAMs) update the
// column sums, then a horizontal pass finishes the box sums with 49/17-deep
// shift registers and taps the chain centre for the output pixel's own
// pmap - exact conv2('same') semantics with w zero rows/cols appended.
// Two output passes are streamed: {mig, det} (cluster pass 1) then {mig}
// (cluster pass 2 statistics), matching MATLAB's two-pass B14.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_cfar #(
    parameter N_IFFT = 2048,
    parameter N_POS  = 200,
    parameter GUARD  = 8,
    parameter TRAIN  = 16,
    parameter LB     = 11,
    parameter L2ROM  = "log2_lut.hex",
    parameter ICROM  = "inv_cnt.hex"
) (
    input  wire        clk,
    input  wire        rst,
    // mig frame stream in (iz outer, ix inner)
    input  wire        s_tvalid,
    output wire        s_tready,
    input  wire [15:0] s_tdata,
    input  wire [15:0] s_tuser,          // iz
    input  wire [7:0]  s_tuser_pos,      // ix
    input  wire        s_frame_last,
    input  wire        go,
    // registers
    input  wire [15:0] b_min0,
    input  wire [15:0] b_max0,
    input  wire        cfar_mode,        // 0 mean, 1 log
    input  wire [15:0] alpha_q12,
    input  wire signed [31:0] log2alpha_q26,
    input  wire signed [31:0] gamma2_q26,
    input  wire [15:0] pfloor_k,
    input  wire [4:0]  pfloor_sh,
    // output stream, emitted twice per frame (m_pass2 distinguishes)
    output reg         m_tvalid,
    input  wire        m_tready,
    output reg  [15:0] m_tdata,
    output reg         m_tdet,
    output reg         m_pass2,
    output reg  [15:0] m_tuser,          // iz
    output reg  [7:0]  m_tuser_pos,      // ix
    output reg         m_tlast,
    output reg         m_frame_last,
    output reg         busy,
    output reg         frame1_done
);
    localparam W      = GUARD + TRAIN;
    localparam BW     = 2*W + 1;
    localparam BG     = 2*GUARD + 1;
    // guard sums are built as (rolling A - rolling B), both aligned to the
    // leading row/column rp/cj, so that A-B is centred on the output pixel:
    //   A: rows rp-(W+G) .. rp        (BGA = W+G+1 slots)
    //   B: rows rp-(W-G)+1 .. rp      (BGB = W-G   slots)
    //   A-B = rows rp-(W+G) .. rp-(W-G) = guard box centred on io = rp-W
    localparam BGA    = W + GUARD + 1;
    localparam BGB    = W - GUARD;
    localparam ROWS_P = N_IFFT + W;
    localparam COLS_P = N_POS + W;

    // memories
    reg [15:0] frame [0:N_IFFT*N_POS-1];     // addr = (ix<<LB)|iz
    reg [31:0] lbbig [0:BW*256-1];           // addr = (slot<<8)|col
    reg [31:0] lbgA [0:BGA*256-1];
    reg [31:0] lbgB [0:BGB*256-1];
    reg [47:0] colbig [0:N_POS-1];           // async read (distributed)
    reg [47:0] colgA [0:N_POS-1];
    reg [47:0] colgB [0:N_POS-1];

    reg [25:0] l2rom [0:63];
    reg [29:0] icrom [0:2400];
    initial begin
        $readmemh(L2ROM, l2rom);
        $readmemh(ICROM, icrom);
    end

    localparam [3:0] C_IDLE=0, C_MAX=1, C_MAX2=13, C_PRE2=2,
                     C_V1=3, C_V2=4, C_VN=5,
                     C_H1=6, C_H2=7, C_HE=14, C_HN=8,
                     C_NROW=9, C_P2A=10, C_P2B=11, C_DONE=12,
                     C_CLR2=15;
    reg [3:0]  st;
    reg        frame_have;
    reg [31:0] mx;
    reg [31:0] p_floor;
    wire [63:0] mx_sq   = mx * mx;                  // <= 2^32, 64-bit context
    wire [63:0] pf_prod = mx_sq * {32'd0, pfloor_k};
    reg [12:0] rp;                 // row pass 0..ROWS_P-1
    reg [19:0] pcx;
    reg [11:0] iz2;                // pass-2 emit counters
    reg [7:0]  ix2;
    reg [8:0]  cj;                 // col step
    reg [15:0] m_rd;               // frame read register
    reg [31:0] old_b, old_gA, old_gB;  // eviction read registers
    reg [31:0] newpm;
    reg [47:0] hb, hgA, hgB;
    reg [47:0] hbd [0:BW-1];
    reg [47:0] hgdA [0:BGA-1];
    reg [47:0] hgdB [0:BGB-1];
    reg [BW-1:0] vrow_sh;
    reg [BGA-1:0] vrowg_sh;   // long enough to delay rowv by W-G (center-align guard window)
    // H2 pipeline registers
    reg [31:0] tap_pm;             // chain-centre tap (BRAM read reg)
    reg [15:0] tap_mig;            // frame(io,jo) (BRAM read reg)
    reg [47:0] h2_sum, h2_gsum;
    reg [11:0] h2_cnt;
    reg [15:0] h2_io, h2_jo;
    reg        h2_valid;
    reg        pass2;
    integer q;

    assign s_tready = !frame_have && (st == C_IDLE);

    // ------------------------------------------------------------------
    function [5:0] clz32(input [31:0] v);
        begin
            clz32 = 32;
            for (q = 31; q >= 0; q = q - 1)
                if (v[q] != 1'b0) begin
                    clz32 = 31 - q;
                    q = -1;                       // break
                end
        end
    endfunction
    function [31:0] log2q26(input [31:0] v);
        reg [5:0] e;
        reg [31:0] nrm;
        begin
            if (v == 0) log2q26 = 0;
            else begin
                e   = clz32(v);
                nrm = v << e;
                log2q26 = ((31 - e) << 26) + l2rom[nrm[30:25]];
            end
        end
    endfunction
    function [6:0] pop49(input [48:0] v);
        integer z2;
        begin
            pop49 = 0;
            for (z2 = 0; z2 < 49; z2 = z2 + 1) pop49 = pop49 + v[z2];
        end
    endfunction

    // row-pass helpers
    wire        row_live   = (rp < N_IFFT);
    wire signed [16:0] io_s = $signed({4'b0, rp}) - W;   // output row

    // slot addressing (circular)
    wire [5:0]  slot_b  = rp % BW;
    wire [5:0]  slot_gA = rp % BGA;
    wire [4:0]  slot_gB = rp % BGB;
    wire [5:0]  slot_mid_b = (rp + BW - W) % BW;

    // horizontal output column
    wire signed [16:0] jo_s = $signed({8'b0, cj}) - W;

    // vertical valid counts (windows already aligned to output row io)
    wire [48:0] vrow_sh_x  = vrow_sh;      // zero-extend to the pop width
    wire [48:0] vrowg_sh_x = {{(49-BG){1'b0}}, vrowg_sh[BGA-1:BGB]};  // rows rp-(W+G)..rp-(W-G)
    wire [6:0] vrows   = pop49(vrow_sh_x);
    wire [6:0] vrowsg7 = pop49(vrowg_sh_x);
    wire [5:0] vrowsg  = vrowsg7[5:0];

    // horizontal valid counts for jo
    wire signed [16:0] jl_s = jo_s - W;
    wire [15:0] jleft   = (jl_s < 0) ? 16'd0 : jl_s[15:0];
    wire signed [16:0] jright_s = jo_s + W;
    wire [15:0] jright  = (jright_s > N_POS-1) ? N_POS-1 : jright_s[15:0];
    wire [15:0] vcols   = (jo_s < 0 || jo_s > N_POS-1) ? 16'd0 : (jright - jleft + 16'd1);
    wire signed [16:0] gjl_s = jo_s - GUARD;
    wire [15:0] gleft   = (gjl_s < 0) ? 16'd0 : gjl_s[15:0];
    wire signed [16:0] gright_s = jo_s + GUARD;
    wire [15:0] gright  = (gright_s > N_POS-1) ? N_POS-1 : gright_s[15:0];
    wire [15:0] vcolsg  = (jo_s < 0 || jo_s > N_POS-1) ? 16'd0 : (gright - gleft + 1);

    // guard column value = A-B rolling difference (centred on io)
    wire [47:0] cgv = (cj < N_POS) ? (colgA[cj[7:0]] - colgB[cj[7:0]]) : 48'd0;

    // pm/pmf/newpm combinational chain
    wire [31:0] pm32  = m_rd * m_rd;
    wire [31:0] pmf   = (pm32 > p_floor) ? pm32 : p_floor;
    wire        rowv  = row_live && (rp >= b_min0) && (rp <= b_max0);

    always @(posedge clk) begin
        if (rst) begin
            st <= C_IDLE; busy <= 0; frame_have <= 0; frame1_done <= 0;
            mx <= 0; p_floor <= 0; rp <= 0; cj <= 0; pcx <= 0; iz2 <= 0; ix2 <= 0; pass2 <= 0;
            hb <= 0; hgA <= 0; hgB <= 0; vrow_sh <= 0; vrowg_sh <= 0;
            m_tvalid <= 0; m_tdata <= 0; m_tdet <= 0; m_pass2 <= 0;
            m_tuser <= 0; m_tuser_pos <= 0; m_tlast <= 0; m_frame_last <= 0;
            m_rd <= 0; old_b <= 0; old_gA <= 0; old_gB <= 0; newpm <= 0;
            tap_pm <= 0; tap_mig <= 0; h2_sum <= 0; h2_gsum <= 0;
            h2_cnt <= 0; h2_io <= 0; h2_jo <= 0; h2_valid <= 0;
            for (q = 0; q < BW; q = q + 1) hbd[q] <= 0;
            for (q = 0; q < BGA; q = q + 1) hgdA[q] <= 0;
            for (q = 0; q < BGB; q = q + 1) hgdB[q] <= 0;
            for (q = 0; q < N_POS; q = q + 1) begin
                colbig[q] <= 0; colgA[q] <= 0; colgB[q] <= 0;
            end
        end else begin
            if (m_tvalid && m_tready)
                m_tvalid <= 1'b0;   // beat consumed - drop valid until next emit
            frame1_done <= 1'b0;

            // ---------------- collect ----------------
            if (s_tvalid && s_tready) begin
                frame[{s_tuser_pos, s_tuser[LB-1:0]}] <= s_tdata;
                if (s_frame_last) frame_have <= 1'b1;
            end

            case (st)
            C_IDLE: begin
                m_tvalid <= 1'b0;
                if (frame_have && go) begin
                    frame_have <= 1'b0;
                    busy <= 1'b1;
                    mx   <= 0;
                    pcx  <= 0;
                    st   <= C_MAX;
                end
            end

            // ---------------- max pre-pass ----------------
            C_MAX: begin
                if (m_rd > mx[15:0]) mx <= {16'd0, m_rd};
                m_rd <= frame[pcx[19:0]];
                if (pcx == N_IFFT*N_POS-1) st <= C_MAX2;
                else                       pcx <= pcx + 1;
            end
            C_MAX2: begin
                if (m_rd > mx[15:0]) mx <= {16'd0, m_rd};
                st <= C_PRE2;
            end
            C_PRE2: begin
                p_floor <= pf_prod[63:0] >> pfloor_sh;
                rp      <= 0;
                pass2   <= 0;
                pcx     <= 0;
                vrow_sh <= 0;
                vrowg_sh<= 0;
                st      <= C_CLR2;
            end
            C_CLR2: begin
                // zero the ring chains + column accumulators so the first
                // windowed rows subtract defined zeros (BRAM has no reset)
                if (pcx < 256) begin
                    if (pcx < N_POS) begin
                        colbig[pcx[7:0]] <= 48'd0;
                        colgA [pcx[7:0]] <= 48'd0;
                        colgB [pcx[7:0]] <= 48'd0;
                    end
                end else if (pcx < 256 + BW*256)
                    lbbig[pcx - 256] <= 32'd0;
                else if (pcx < 256 + (BW+BGA)*256)
                    lbgA[pcx - 256 - BW*256] <= 32'd0;
                else if (pcx < 256 + (BW+BGA+BGB)*256)
                    lbgB[pcx - 256 - (BW+BGA)*256] <= 32'd0;
                if (pcx == 256 + (BW+BGA+BGB)*256 - 1) begin
                    pcx <= 0;
                    st  <= C_NROW;
                end else
                    pcx <= pcx + 1;
            end

            // ---------------- row pipeline ----------------
            C_NROW: begin
                // start of row pass rp
                cj    <= 0;
                hb    <= 0;
                hgA   <= 0;
                hgB   <= 0;
                vrow_sh  <= {vrow_sh[BW-2:0],  rowv};
                vrowg_sh <= {vrowg_sh[BGA-2:0], rowv};
                for (q = 0; q < BW;  q = q + 1) hbd [q] <= 0;
                for (q = 0; q < BGA; q = q + 1) hgdA[q] <= 0;
                for (q = 0; q < BGB; q = q + 1) hgdB[q] <= 0;
                st <= C_V1;
            end
            // V phase: 2 cycles per column
            C_V1: begin
                if (cj < N_POS) begin
                    m_rd  <= frame[{cj[7:0], rp[LB-1:0]}];
                    old_b <= lbbig[{slot_b, cj[7:0]}];
                    old_gA <= lbgA [{slot_gA, cj[7:0]}];
                    old_gB <= lbgB [{slot_gB, cj[7:0]}];
                    st    <= C_V2;
                end else begin
                    st <= C_H1;
                    cj <= 0;
                end
            end
            C_V2: begin
                // newpm from m_rd (registered frame value)
                if (!rowv)
                    newpm <= 32'd0;
                else if (cfar_mode)
                    newpm <= log2q26(pmf);
                else
                    newpm <= pmf;
                st <= C_VN;
            end
            C_VN: begin
                colbig[cj[7:0]] <= colbig[cj[7:0]] + newpm - old_b;
                colgA [cj[7:0]] <= colgA [cj[7:0]] + newpm - old_gA;
                colgB [cj[7:0]] <= colgB [cj[7:0]] + newpm - old_gB;
                lbbig[{slot_b, cj[7:0]}]   <= newpm;
                lbgA [{slot_gA, cj[7:0]}]  <= newpm;
                lbgB [{slot_gB, cj[7:0]}]  <= newpm;
                cj <= cj + 1;
                st <= C_V1;
            end
            // H phase: 2 cycles per column step (issue / emit)
            C_H1: begin
                if (m_tvalid && m_tready)
                    m_tvalid <= 1'b0;   // previous beat consumed
                if (cj < COLS_P) begin
                    // shift horizontal sums
                    hb <= hb + (cj < N_POS ? colbig[cj[7:0]] : 48'd0) - hbd[BW-1];
                    hgA <= hgA + cgv - hgdA[BGA-1];
                    hgB <= hgB + cgv - hgdB[BGB-1];
                    for (q = BW-1; q > 0; q = q - 1) hbd[q] <= hbd[q-1];
                    hbd[0] <= (cj < N_POS ? colbig[cj[7:0]] : 48'd0);
                    for (q = BGA-1; q > 0; q = q - 1) hgdA[q] <= hgdA[q-1];
                    for (q = BGB-1; q > 0; q = q - 1) hgdB[q] <= hgdB[q-1];
                    hgdA[0] <= cgv;
                    hgdB[0] <= cgv;
                    // issue tap + frame reads for the output pixel (io, jo)
                    if (io_s >= 0 && jo_s >= 0 && jo_s < N_POS) begin
                        tap_pm  <= lbbig[{slot_mid_b, jo_s[7:0]}];
                        tap_mig <= frame[{jo_s[7:0], io_s[LB-1:0]}];
                        h2_io   <= io_s[15:0];
                        h2_jo   <= jo_s[15:0];
                        h2_valid<= 1'b1;
                    end else
                        h2_valid <= 1'b0;
                    st <= C_H2;
                end else begin
                    st <= C_HN;
                end
            end
            C_H2: begin
                // register compare inputs one more cycle for BRAM latency
                h2_sum  <= hb;
                h2_gsum <= hgA - hgB;
                // cnt = vrows*vcols - vrowsg*vcolsg, clamped to >= 1
                if ((vrows * vcols) > (vrowsg * vcolsg))
                    h2_cnt <= (vrows * vcols) - (vrowsg * vcolsg);
                else
                    h2_cnt <= 1;
                st <= C_HE;
            end
            C_HE: begin
                if (!m_tvalid || m_tready) begin
                    if (h2_valid) begin
                        m_tvalid    <= 1'b1;
                        m_pass2     <= pass2;
                        m_tdata     <= tap_mig;
                        m_tuser     <= h2_io;
                        m_tuser_pos <= h2_jo[7:0];
                        m_tlast     <= (h2_jo == N_POS-1);
                        m_frame_last<= (h2_jo == N_POS-1) && (io_s == N_IFFT-1);
                        if (!cfar_mode)
                            m_tdet <= ({32'd0, tap_pm} * h2_cnt * 64'd4096 >
                                       {16'd0, alpha_q12} * (h2_sum - h2_gsum));
                        else
                            m_tdet <= ($signed({32'd0, tap_pm}) >
                                       $signed(((((h2_sum - h2_gsum) *
                                          {34'd0, icrom[h2_cnt-1]})) >> 30) +
                                        {32'd0, gamma2_q26} + {32'd0, log2alpha_q26}));
                    end else
                        m_tvalid <= 1'b0;
                    cj <= cj + 1;
                    st <= C_H1;
                end
            end
            C_HN: begin
                // row pass done
                if (rp == ROWS_P-1) begin
                    if (!pass2) begin
                        pass2       <= 1'b1;
                        frame1_done <= 1'b1;
                        rp          <= 0;
                        iz2         <= 0;
                        ix2         <= 0;
                        st          <= C_P2A;
                    end else begin
                        st <= C_DONE;
                    end
                end else begin
                    rp <= rp + 1;
                    st <= C_NROW;
                end
            end

            // ---------------- pass 2: re-stream the frame ----------------
            C_P2A: begin
                if (iz2 < N_IFFT) begin
                    m_rd <= frame[{ix2, iz2[LB-1:0]}];
                    st   <= C_P2B;
                end else begin
                    st <= C_DONE;
                end
            end
            C_P2B: begin
                if (!m_tvalid || m_tready) begin
                    m_tvalid    <= 1'b1;
                    m_pass2     <= 1'b1;
                    m_tdet      <= 1'b0;
                    m_tdata     <= m_rd;
                    m_tuser     <= iz2;
                    m_tuser_pos <= ix2;
                    m_tlast     <= (ix2 == N_POS-1);
                    m_frame_last<= (ix2 == N_POS-1) && (iz2 == N_IFFT-1);
                    if (ix2 == N_POS-1) begin
                        ix2 <= 0;
                        iz2 <= iz2 + 1;
                    end else
                        ix2 <= ix2 + 1;
                    st  <= C_P2A;
                end
            end
            C_DONE: begin
                // wait for the final (frame_last) beat to be accepted before
                // dropping m_tvalid - otherwise pass-2's last beat is lost
                if (!m_tvalid || m_tready) begin
                    busy     <= 1'b0;
                    m_tvalid <= 1'b0;
                    st       <= C_IDLE;
                end
            end
            default: st <= C_IDLE;
            endcase
        end
    end
endmodule
