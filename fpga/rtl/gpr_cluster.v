// ---------------------------------------------------------------------------
// gpr_cluster.v - B14_Report: cluster the CFAR detections and summarise.
// Exact hardware transcription of code_report.m:
//   pass 1 (stream beats with det=1): causal 8-neighbour labelling using
//     the MATLAB neighbour order (up-left, up, up-right, left) with
//     union-find on a 256-entry parent table (smaller root survives);
//     label overflow merges into the last label.  Non-hit pixels write 0,
//     which also zeroes the label RAM for the next frame.
//   pass 2 (re-stream): resolve each label to its root, write it back, and
//     accumulate per-cluster count / peak / peak-row / peak-col (strict >,
//     row-major scan => first maximum wins, exactly like MATLAB).
//   final scan: rep = [hits, nclust, mean_d, mean_x, d_min, d_max];
//     depths/cross-range in Q16 m (d = row0*dr_q16, x = col0*dx_q16),
//     means floored, min/max over cluster peak depths.
// Deviation: labels are 8-bit, so at most 255 clusters per frame (MATLAB:
// 256); overflow merges into label 255.  Real frames have O(10) clusters.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_cluster #(
    parameter N_IFFT = 2048,
    parameter N_POS  = 200,
    parameter LB     = 11,
    parameter MAXLAB = 256
) (
    input  wire        clk,
    input  wire        rst,
    // stream from CFAR (two passes per frame, in order)
    input  wire        s_tvalid,
    output wire        s_tready,
    input  wire [15:0] s_tdata,          // mig magnitude
    input  wire        s_tdet,
    input  wire        s_pass2,
    input  wire [15:0] s_tuser,          // iz (0-based)
    input  wire [7:0]  s_tuser_pos,      // ix (0-based)
    input  wire        s_frame_last,
    input  wire        go,
    input  wire [31:0] dr_q16,
    input  wire [31:0] dx_q16,
    // report
    output reg  [31:0] rep1, rep2, rep3, rep4, rep5, rep6,
    output reg         rep_done,
    output reg         busy
);
    reg [7:0]  lab    [0:N_IFFT*N_POS-1];
    reg [7:0]  parent [0:MAXLAB-1];
    reg [15:0] cnt [0:MAXLAB-1];
    reg [15:0] pk  [0:MAXLAB-1];
    reg [15:0] pkr [0:MAXLAB-1];
    reg [7:0]  pkc [0:MAXLAB-1];

    localparam [4:0]
        L_IDLE=0, L_ZERO=1,
        L_N0=2, L_N1=3, L_N2=4, L_N3=5, L_NW=6,
        L_K=7, L_FA=8, L_RET_BEST=9, L_RET_RA=10, L_RET_RB=11,
        L_MERGE=12, L_NEWLAB=13, L_WR=14,
        L_P2RD=15, L_P2F0=16, L_P2F1=17, L_P2ST=18,
        L_DONEPIX=19, L_FIN=20, L_FSCAN=21,
        L_DIV0=22, L_DIV1=23, L_DIV2=24, L_CLR=25, L_DONE=26;
    reg [4:0]  st;
    reg [4:0]  ret_st;

    reg [7:0]  nlab;
    reg [31:0] hits;
    reg signed [16:0] ci, cjj;
    reg [7:0]  nb [0:3];
    reg [2:0]  kk;
    reg [7:0]  best, ra, rb;
    reg [7:0]  fx, fout;
    reg [7:0]  labrd;
    reg        s_det_lat, s_p2_lat, s_fl_lat;
    reg [15:0] s_iz_lat, s_dat_lat;
    reg [7:0]  s_ix_lat;
    reg [7:0]  res_root;
    reg [7:0]  rr;
    reg [8:0]  clr_i;
    reg [8:0]  nclust;
    reg [31:0] sum_d, sum_x, dmin, dmax;

    reg        d_start;
    reg [31:0] d_num;
    reg [15:0] d_den;
    wire [31:0] d_quo;
    wire        d_done;
    gpr_div u_div (.clk(clk), .rst(rst), .start(d_start), .num(d_num),
                   .den(d_den), .quo(d_quo), .rem(), .busy(), .done(d_done));

    assign s_tready = (st == L_IDLE);

    wire [31:0] dcur = pkr[rr] * dr_q16;
    wire [31:0] xcur = {16'd0, pkc[rr]} * dx_q16;

    integer q;
    always @(posedge clk) begin
        if (rst) begin
            st <= L_IDLE; busy <= 0; rep_done <= 0; nlab <= 0; hits <= 0;
            rep1 <= 0; rep2 <= 0; rep3 <= 0; rep4 <= 0; rep5 <= 0; rep6 <= 0;
            kk <= 0; best <= 0; ra <= 0; rb <= 0; fx <= 0; fout <= 0;
            ci <= 0; cjj <= 0; labrd <= 0; rr <= 0; nclust <= 0; clr_i <= 0;
            sum_d <= 0; sum_x <= 0; dmin <= 0; dmax <= 0; res_root <= 0;
            d_start <= 0; d_num <= 0; d_den <= 0; ret_st <= 0;
            s_det_lat <= 0; s_p2_lat <= 0; s_fl_lat <= 0;
            s_iz_lat <= 0; s_ix_lat <= 0; s_dat_lat <= 0;
            for (q = 0; q < MAXLAB; q = q + 1) begin
                parent[q] <= 0; cnt[q] <= 0; pk[q] <= 0; pkr[q] <= 0; pkc[q] <= 0;
            end
        end else begin
            d_start  <= 1'b0;
            rep_done <= 1'b0;

            case (st)
            L_IDLE: begin
                if (s_tvalid && go) begin
                    ci        <= $signed({1'b0, s_tuser});
                    cjj       <= $signed({1'b0, s_tuser_pos});
                    s_det_lat <= s_tdet;
                    s_p2_lat  <= s_pass2;
                    s_fl_lat  <= s_frame_last;
                    s_iz_lat  <= s_tuser;
                    s_ix_lat  <= s_tuser_pos;
                    s_dat_lat <= s_tdata;
                    busy      <= 1'b1;
                    if (s_pass2)
                        st <= L_P2RD;
                    else if (s_tdet) begin
                        hits <= hits + 1;
                        st   <= L_N0;
                    end else
                        st <= L_ZERO;
                end
            end

            L_ZERO: begin
                lab[{s_ix_lat, s_iz_lat[LB-1:0]}] <= 8'd0;
                st <= L_DONEPIX;
            end

            // ---------------- pass 1: neighbours ----------------
            L_N0: begin
                if (ci > 0 && cjj > 0)
                    labrd <= lab[{(cjj[7:0] - 8'd1), (ci[LB-1:0] - 1'b1)}];
                else
                    labrd <= 8'd0;
                st <= L_N1;
            end
            L_N1: begin
                nb[0] <= labrd;
                labrd <= (ci > 0) ? lab[{cjj[7:0], (ci[LB-1:0] - 1'b1)}] : 8'd0;
                st <= L_N2;
            end
            L_N2: begin
                nb[1] <= labrd;
                labrd <= (ci > 0 && cjj < N_POS-1) ?
                         lab[{(cjj[7:0] + 8'd1), (ci[LB-1:0] - 1'b1)}] : 8'd0;
                st <= L_N3;
            end
            L_N3: begin
                nb[2] <= labrd;
                labrd <= (cjj > 0) ? lab[{(cjj[7:0] - 8'd1), ci[LB-1:0]}] : 8'd0;
                st <= L_NW;
            end
            L_NW: begin
                nb[3] <= labrd;
                best  <= 0;
                kk    <= 0;
                st    <= L_K;
            end

            // ---------------- union loop over the 4 neighbours ----------
            L_K: begin
                if (kk > 3) begin
                    st <= (best == 0) ? L_NEWLAB : L_WR;
                end else if (nb[kk[1:0]] == 0) begin
                    kk <= kk + 1;
                end else if (best == 0) begin
                    fx     <= nb[kk[1:0]];
                    ret_st <= L_RET_BEST;
                    st     <= L_FA;
                end else begin
                    fx     <= best;
                    ret_st <= L_RET_RA;
                    st     <= L_FA;
                end
            end
            L_FA: begin                       // find: fx -> fout
                if (parent[fx] == fx || parent[fx] == 0) begin
                    fout <= fx;
                    st   <= ret_st;
                end else
                    fx <= parent[fx];
            end
            L_RET_BEST: begin
                best <= fout;
                kk   <= kk + 1;
                st   <= L_K;
            end
            L_RET_RA: begin
                ra     <= fout;
                fx     <= nb[kk[1:0]];
                ret_st <= L_RET_RB;
                st     <= L_FA;
            end
            L_RET_RB: begin
                rb <= fout;
                st <= L_MERGE;
            end
            L_MERGE: begin
                if (ra == rb)
                    best <= ra;
                else if (ra < rb) begin
                    parent[rb] <= ra;
                    best <= ra;
                end else begin
                    parent[ra] <= rb;
                    best <= rb;
                end
                kk <= kk + 1;
                st <= L_K;
            end
            L_NEWLAB: begin
                if (nlab < MAXLAB-1) begin
                    nlab           <= nlab + 1;
                    parent[nlab+1] <= nlab + 1;
                    best           <= nlab + 1;
                end else
                    best           <= MAXLAB-1;   // overflow merges into 255
                st <= L_WR;
            end
            L_WR: begin
                lab[{s_ix_lat, s_iz_lat[LB-1:0]}] <= best;
                st <= L_DONEPIX;
            end

            // ---------------- pass 2: resolve + stats ----------------
            L_P2RD: begin
                labrd <= lab[{s_ix_lat, s_iz_lat[LB-1:0]}];
                st    <= L_P2F0;
            end
            L_P2F0: begin
                if (labrd == 0)
                    st <= L_DONEPIX;
                else begin
                    fx <= labrd;
                    st <= L_P2F1;
                end
            end
            L_P2F1: begin
                if (parent[fx] == fx || parent[fx] == 0) begin
                    res_root <= fx;
                    st <= L_P2ST;
                end else
                    fx <= parent[fx];
            end
            L_P2ST: begin
                lab[{s_ix_lat, s_iz_lat[LB-1:0]}] <= res_root;
                cnt[res_root] <= cnt[res_root] + 1;
                if (s_dat_lat > pk[res_root]) begin
                    pk[res_root]  <= s_dat_lat;
                    pkr[res_root] <= s_iz_lat;
                    pkc[res_root] <= s_ix_lat;
                end
                st <= L_DONEPIX;
            end

            L_DONEPIX: begin
                if (s_fl_lat && s_p2_lat) begin
                    st <= L_FIN;
                end else begin
                    st <= L_IDLE;
                end
            end

            // ---------------- final scan ----------------
            L_FIN: begin
                rr     <= 1;
                nclust <= 0;
                sum_d  <= 0; sum_x <= 0;
                dmin   <= 0; dmax  <= 0;
                st     <= L_FSCAN;
            end
            L_FSCAN: begin
                if (cnt[rr] != 0) begin
                    if (nclust == 0) begin
                        dmin <= dcur;
                        dmax <= dcur;
                    end else begin
                        if (dcur < dmin) dmin <= dcur;
                        if (dcur > dmax) dmax <= dcur;
                    end
                    nclust <= nclust + 1;
                    sum_d  <= sum_d + dcur;
                    sum_x  <= sum_x + xcur;
                end
                if (rr == MAXLAB-1) st <= L_DIV0;
                else                rr <= rr + 1;
            end
            L_DIV0: begin
                rep1 <= hits;
                rep2 <= {23'd0, nclust};
                rep5 <= dmin;
                rep6 <= dmax;
                if (nclust == 0) begin
                    rep3 <= 0;
                    rep4 <= 0;
                    st   <= L_CLR;
                end else begin
                    d_num   <= sum_d;
                    d_den   <= {7'd0, nclust};
                    d_start <= 1'b1;
                    st      <= L_DIV1;
                end
            end
            L_DIV1: if (d_done) begin
                rep3    <= d_quo;
                d_num   <= sum_x;
                d_start <= 1'b1;
                st      <= L_DIV2;
            end
            L_DIV2: if (d_done) begin
                rep4 <= d_quo;
                st   <= L_CLR;
            end
            L_CLR: begin
                // clear per-frame stat tables for the next frame
                cnt[clr_i[7:0]] <= 0;
                pk [clr_i[7:0]] <= 0;
                pkr[clr_i[7:0]] <= 0;
                pkc[clr_i[7:0]] <= 0;
                parent[clr_i[7:0]] <= 0;
                if (clr_i == MAXLAB-1) begin
                    clr_i <= 0;
                    st    <= L_DONE;
                end else
                    clr_i <= clr_i + 1;
            end
            L_DONE: begin
                rep_done <= 1'b1;
                busy     <= 1'b0;
                nlab     <= 0;
                hits     <= 0;
                st       <= L_IDLE;
            end
            default: st <= L_IDLE;
            endcase
        end
    end
endmodule
