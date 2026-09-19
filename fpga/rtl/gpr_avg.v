// ---------------------------------------------------------------------------
// gpr_avg.v - B08_Averaging: coherent integration of n_avg sweeps.
// Accumulates per TRUE tone index across n_avg = 2^LOG2_NAVG sweeps, then
// flushes one full n_tones-vector downstream (tlast on the final tone).
// Summing n_avg=32 Q15 samples needs guard bits; the flush divides by
// 2^log2_navg with round-to-nearest and saturates back to Q15.
// The accumulator RAM has ONE muxed write port: during streaming it adds
// the incoming sample, during flush it clears the entry being emitted
// (read-first), which keeps it inferable as block RAM.
// The MATLAB block also injects the post-integration noise figure; that is a
// modelling artefact - in hardware the noise is whatever the RFDC delivered.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_avg #(
    parameter N_TONES   = 256,
    parameter LOG2_NAVG = 5,
    parameter AW        = 16 + 1 + LOG2_NAVG
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        s_tvalid,
    output wire        s_tready,
    input  wire [31:0] s_tdata,
    input  wire        s_tlast,
    input  wire [7:0]  s_tuser_tone,
    input  wire [7:0]  s_tuser_pos,
    output reg         m_tvalid,
    input  wire        m_tready,
    output reg  [31:0] m_tdata,
    output reg         m_tlast,
    output reg  [7:0]  m_tuser_tone,
    output reg  [7:0]  m_tuser_pos,
    output reg         group_done,
    output reg         flush_start
);
    reg signed [AW-1:0] acc_i [0:N_TONES-1];
    reg signed [AW-1:0] acc_q [0:N_TONES-1];
    reg [7:0] sweep_cnt;
    reg [7:0] pos_lat;

    reg       flushing;
    reg [8:0] fk;

    assign s_tready = !flushing;

    function signed [15:0] sat16(input signed [31:0] v);
        begin
            if      (v >  32'sd32767) sat16 =  16'sd32767;
            else if (v < -32'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    wire signed [15:0] in_i = $signed(s_tdata[31:16]);
    wire signed [15:0] in_q = $signed(s_tdata[15:0]);

    // read-first views of the accumulator at the flush index
    wire signed [AW-1:0] rd_i = acc_i[fk[7:0]];
    wire signed [AW-1:0] rd_q = acc_q[fk[7:0]];

    reg signed [AW+2:0] ext_i, ext_q;
    always @(*) begin
        ext_i = {{3{rd_i[AW-1]}}, rd_i} + (1 <<< (LOG2_NAVG-1));
        ext_q = {{3{rd_q[AW-1]}}, rd_q} + (1 <<< (LOG2_NAVG-1));
    end

    integer k;
    always @(posedge clk) begin
        if (rst) begin
            sweep_cnt <= 0; flushing <= 0; fk <= 0; pos_lat <= 0;
            m_tvalid <= 0; m_tdata <= 0; m_tlast <= 0;
            m_tuser_tone <= 0; m_tuser_pos <= 0; group_done <= 0; flush_start <= 0;
            for (k = 0; k < N_TONES; k = k + 1) begin
                acc_i[k] <= 0; acc_q[k] <= 0;
            end
        end else begin
            group_done  <= 0;
            flush_start <= 0;

            if (!flushing) begin
                if (m_tvalid && m_tready) m_tvalid <= 0;
                if (s_tvalid && s_tready) begin
                    acc_i[s_tuser_tone] <= acc_i[s_tuser_tone] + {{(AW-16){in_i[15]}}, in_i};
                    acc_q[s_tuser_tone] <= acc_q[s_tuser_tone] + {{(AW-16){in_q[15]}}, in_q};
                    pos_lat <= s_tuser_pos;
                    if (s_tlast) begin
                        if (sweep_cnt == (1 << LOG2_NAVG) - 1) begin
                            sweep_cnt <= 0;
                            flushing  <= 1;
                            flush_start <= 1'b1;
                            fk        <= 0;
                        end else
                            sweep_cnt <= sweep_cnt + 1;
                    end
                end
            end else begin
                if (!m_tvalid || m_tready) begin
                    if (fk == N_TONES) begin
                        flushing   <= 0;
                        m_tvalid   <= 0;
                        group_done <= 1;
                    end else begin
                        m_tvalid     <= 1;
                        m_tdata      <= {sat16(ext_i >>> LOG2_NAVG),
                                         sat16(ext_q >>> LOG2_NAVG)};
                        m_tlast      <= (fk == N_TONES-1);
                        m_tuser_tone <= fk[7:0];
                        m_tuser_pos  <= pos_lat;
                        // read-first clear of the emitted entry
                        acc_i[fk[7:0]] <= 0;
                        acc_q[fk[7:0]] <= 0;
                        fk <= fk + 1;
                    end
                end
            end
        end
    end
endmodule
