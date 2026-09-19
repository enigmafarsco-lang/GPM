// ---------------------------------------------------------------------------
// gpr_background.v - B11_Background: position-invariant clutter removal.
// Mirrors code_background.m with cfg.bg_remove_modes = 1 (the validated
// default): component-wise median across the N_POS traces of a full frame,
// subtracted from every trace (even N_POS: floor of the mean of the two
// middle elements, exactly like the golden model).  The optional rank-1
// power-iteration mode stays a PS/MATLAB feature - see fpga/README.md.
//
// Ping-pong frame buffers, layout addr = (pos << LB) | bin.  When a buffer
// holds a complete frame the engine: loads each depth bin's real/imaginary
// columns into distributed sort RAMs, insertion-sorts both in parallel
// (~1 cycle per shift), stores the medians, subtracts in place and streams
// the bs frame out column by column.  Worst case ~170 ms/frame at
// 245.76 MHz vs a 2.56 s mode-A frame period.
//
// The FFT input is NEVER back-pressured (frame period >> post time is a
// design rule); a violation raises the sticky overrun flag.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_background #(
    parameter N_IFFT = 2048,
    parameter N_POS  = 200,
    parameter LB     = 11               // log2(N_IFFT)
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [3:0]  nbg,              // 0 = passthrough, >=1 = median
    // rp column stream from the FFT (one full column per position)
    input  wire        s_tvalid,
    input  wire [31:0] s_tdata,
    input  wire [15:0] s_tuser,          // depth bin
    input  wire        s_tlast,          // last bin of a column
    // bs frame stream to migration
    output reg         m_tvalid,
    input  wire        m_tready,
    output reg  [31:0] m_tdata,
    output reg  [15:0] m_tuser,          // depth bin
    output reg  [7:0]  m_tuser_pos,
    output reg         m_tlast,          // last bin of a column
    output reg         m_frame_last,     // last column of the frame
    output reg         frame_ready,      // pulse with the first beat
    output reg  [7:0]  frame_count,
    output reg         overrun           // sticky
);
    localparam DEPTH = N_IFFT * N_POS;

    reg [31:0] fb0 [0:DEPTH-1];
    reg [31:0] fb1 [0:DEPTH-1];
    reg [31:0] bgm [0:N_IFFT-1];

    reg signed [15:0] sre [0:N_POS-1];
    reg signed [15:0] sim [0:N_POS-1];

    localparam [3:0] S_IDLE=0, S_LOAD=1, S_SSTART=2, S_SSTEP=3, S_MED=4,
                     S_NEXTBIN=5, S_SUB=6, S_EMA=7, S_EMB=8, S_PASSA=9, S_PASSB=10;
    reg [3:0]  st;
    reg        cur;                      // buffer accepting new frames
    reg        proc;                     // buffer being processed
    reg [7:0]  pos_cnt;
    reg [11:0] d_idx;
    reg [8:0]  j_idx;
    reg signed [15:0] key_re, key_im;
    reg signed [8:0]  ii_re, ii_im;
    reg        act_re, act_im;
    reg [11:0] sub_d;
    reg [7:0]  sub_p, em_p;
    reg [11:0] em_d;
    reg [31:0] rd;

    function signed [15:0] subsat(input signed [15:0] a, input signed [15:0] b);
        reg signed [16:0] d;
        begin
            d = {a[15], a} - {b[15], b};
            if      (d >  17'sd32767) subsat =  16'sd32767;
            else if (d < -17'sd32768) subsat = -16'sd32768;
            else                      subsat = d[15:0];
        end
    endfunction

    // floor((a+b)/2) with 17-bit intermediate (matches the golden model)
    function signed [15:0] med_even(input signed [15:0] a, input signed [15:0] b);
        reg signed [16:0] t;
        begin
            t = {a[15], a} + {b[15], b};
            med_even = (t >>> 1);
        end
    endfunction

    // input never stalls
    wire col_end = s_tvalid && s_tlast;
    wire frame_full = col_end && (pos_cnt == N_POS-1);

    always @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE; cur <= 0; proc <= 0; pos_cnt <= 0;
            frame_count <= 0; overrun <= 0;
            m_tvalid <= 0; m_tdata <= 0; m_tuser <= 0; m_tuser_pos <= 0;
            m_tlast <= 0; m_frame_last <= 0; frame_ready <= 0;
            d_idx <= 0; j_idx <= 0; sub_d <= 0; sub_p <= 0; em_p <= 0; em_d <= 0;
            act_re <= 0; act_im <= 0; ii_re <= 0; ii_im <= 0;
            key_re <= 0; key_im <= 0; rd <= 0;
        end else begin
            frame_ready <= 1'b0;

            // ---------------- input write path ----------------
            if (s_tvalid) begin
                if (cur) fb1[{pos_cnt, s_tuser[LB-1:0]}] <= s_tdata;
                else     fb0[{pos_cnt, s_tuser[LB-1:0]}] <= s_tdata;
            end

            case (st)
            S_IDLE: begin
                m_tvalid <= 1'b0;
                if (frame_full) begin
                    pos_cnt <= 0;
                    proc    <= cur;
                    cur     <= ~cur;         // next frame goes to the other buf
                    if (nbg != 0) begin
                        st <= S_LOAD; d_idx <= 0; j_idx <= 0;
                    end else begin
                        st <= S_PASSA; sub_p <= 0; sub_d <= 0;
                    end
                end else if (col_end)
                    pos_cnt <= pos_cnt + 1;

                // overrun: a new frame completed while the engine still runs
                // (only reachable if S_IDLE is left - guarded below)
            end

            // ---------------- median engine ----------------
            S_LOAD: begin
                if (j_idx < N_POS) begin
                    if (proc) begin
                        sre[j_idx[7:0]] <= $signed(fb1[{j_idx[7:0], d_idx[LB-1:0]}][31:16]);
                        sim[j_idx[7:0]] <= $signed(fb1[{j_idx[7:0], d_idx[LB-1:0]}][15:0]);
                    end else begin
                        sre[j_idx[7:0]] <= $signed(fb0[{j_idx[7:0], d_idx[LB-1:0]}][31:16]);
                        sim[j_idx[7:0]] <= $signed(fb0[{j_idx[7:0], d_idx[LB-1:0]}][15:0]);
                    end
                    j_idx <= j_idx + 1;
                end else begin
                    j_idx <= 9'd1;        // restart as insertion-sort outer index
                    st <= S_SSTART;
                end
            end
            S_SSTART: begin
                if (j_idx >= N_POS) begin
                    st <= S_MED;
                end else begin
                    key_re <= sre[j_idx[7:0]];
                    key_im <= sim[j_idx[7:0]];
                    ii_re  <= $signed({1'b0, j_idx[7:0]}) - 1;
                    ii_im  <= $signed({1'b0, j_idx[7:0]}) - 1;
                    act_re <= 1'b1;
                    act_im <= 1'b1;
                    st     <= S_SSTEP;
                end
            end
            S_SSTEP: begin
                if (act_re) begin
                    if (ii_re < 0) begin
                        sre[0] <= key_re;
                        act_re <= 1'b0;
                    end else if (sre[ii_re[7:0]] > key_re) begin
                        sre[ii_re[7:0] + 1] <= sre[ii_re[7:0]];
                        ii_re <= ii_re - 1;
                    end else begin
                        sre[ii_re[7:0] + 1] <= key_re;
                        act_re <= 1'b0;
                    end
                end
                if (act_im) begin
                    if (ii_im < 0) begin
                        sim[0] <= key_im;
                        act_im <= 1'b0;
                    end else if (sim[ii_im[7:0]] > key_im) begin
                        sim[ii_im[7:0] + 1] <= sim[ii_im[7:0]];
                        ii_im <= ii_im - 1;
                    end else begin
                        sim[ii_im[7:0] + 1] <= key_im;
                        act_im <= 1'b0;
                    end
                end
                // both components placed -> next outer index
                if ((!act_re || (ii_re < 0) || (sre[ii_re[7:0]] <= key_re)) &&
                    (!act_im || (ii_im < 0) || (sim[ii_im[7:0]] <= key_im))) begin
                    j_idx <= j_idx + 1;
                    st    <= S_SSTART;
                end
            end
            S_MED: begin
                if (N_POS % 2 == 0)
                    bgm[d_idx] <= {med_even(sre[N_POS/2-1], sre[N_POS/2]),
                                   med_even(sim[N_POS/2-1], sim[N_POS/2])};
                else
                    bgm[d_idx] <= {sre[N_POS/2], sim[N_POS/2]};
                st <= S_NEXTBIN;
            end
            S_NEXTBIN: begin
                if (d_idx == N_IFFT-1) begin
                    st <= S_SUB; sub_p <= 0; sub_d <= 0;
                end else begin
                    d_idx <= d_idx + 1;
                    j_idx <= 0;
                    st    <= S_LOAD;
                end
            end
            S_SUB: begin
                // in-place subtract, read-first on the same address
                if (proc)
                    fb1[{sub_p, sub_d[LB-1:0]}] <=
                        {subsat($signed(fb1[{sub_p, sub_d[LB-1:0]}][31:16]), $signed(bgm[sub_d][31:16])),
                         subsat($signed(fb1[{sub_p, sub_d[LB-1:0]}][15:0]),  $signed(bgm[sub_d][15:0]))};
                else
                    fb0[{sub_p, sub_d[LB-1:0]}] <=
                        {subsat($signed(fb0[{sub_p, sub_d[LB-1:0]}][31:16]), $signed(bgm[sub_d][31:16])),
                         subsat($signed(fb0[{sub_p, sub_d[LB-1:0]}][15:0]),  $signed(bgm[sub_d][15:0]))};
                if (sub_d == N_IFFT-1) begin
                    sub_d <= 0;
                    if (sub_p == N_POS-1) begin
                        st <= S_EMA; em_p <= 0; em_d <= 0;
                    end else
                        sub_p <= sub_p + 1;
                end else
                    sub_d <= sub_d + 1;
            end
            S_EMA: begin
                if (m_tvalid && m_tready)
                    m_tvalid <= 1'b0;        // previous beat consumed
                if (proc) rd <= fb1[{em_p, em_d[LB-1:0]}];
                else      rd <= fb0[{em_p, em_d[LB-1:0]}];
                st <= S_EMB;
            end
            S_EMB: begin
                if (!m_tvalid || m_tready) begin
                    if (em_p == 0 && em_d == 0) begin
                        frame_ready <= 1'b1;
                        frame_count <= frame_count + 1;
                    end
                    m_tvalid    <= 1'b1;
                    m_tdata     <= rd;
                    m_tuser     <= em_d;
                    m_tuser_pos <= em_p;
                    m_tlast     <= (em_d == N_IFFT-1);
                    m_frame_last<= (em_d == N_IFFT-1) && (em_p == N_POS-1);
                    if (em_d == N_IFFT-1) begin
                        em_d <= 0;
                        if (em_p == N_POS-1)
                            st <= S_IDLE;
                        else begin
                            em_p <= em_p + 1;
                            st   <= S_EMA;
                        end
                    end else begin
                        em_d <= em_d + 1;
                        st   <= S_EMA;
                    end
                end
            end

            // ---------------- passthrough (nbg = 0) ----------------
            S_PASSA: begin
                if (m_tvalid && m_tready)
                    m_tvalid <= 1'b0;        // previous beat consumed
                if (proc) rd <= fb1[{sub_p, sub_d[LB-1:0]}];
                else      rd <= fb0[{sub_p, sub_d[LB-1:0]}];
                st <= S_PASSB;
            end
            S_PASSB: begin
                if (!m_tvalid || m_tready) begin
                    if (sub_p == 0 && sub_d == 0) begin
                        frame_ready <= 1'b1;
                        frame_count <= frame_count + 1;
                    end
                    m_tvalid    <= 1'b1;
                    m_tdata     <= rd;
                    m_tuser     <= sub_d;
                    m_tuser_pos <= sub_p;
                    m_tlast     <= (sub_d == N_IFFT-1);
                    m_frame_last<= (sub_d == N_IFFT-1) && (sub_p == N_POS-1);
                    if (sub_d == N_IFFT-1) begin
                        sub_d <= 0;
                        if (sub_p == N_POS-1)
                            st <= S_IDLE;
                        else begin
                            sub_p <= sub_p + 1;
                            st    <= S_PASSA;
                        end
                    end else begin
                        sub_d <= sub_d + 1;
                        st    <= S_PASSA;
                    end
                end
            end
            default: st <= S_IDLE;
            endcase

            // overrun: frame completed while the engine is mid-flight
            if (frame_full && (st != S_IDLE))
                overrun <= 1'b1;
        end
    end

endmodule
