// ---------------------------------------------------------------------------
// gpr_dds_txc.v - B02_Waveform + B03_TX_Chain: SFCW tone-stepped excitation
// generator for the ZCU208 RFDC DAC path.
//
// The MATLAB model treats the 256 SFCW tones as parallel frequency bins.
// Physically an SFCW radar dwells on one tone at a time, so this core steps
// through the tone sequence in sub-band-major order: the RFDC mixer covers
// one fabric-Nyquist-wide sub-band at a time and the 48-bit phase
// accumulator steps the tones inside it.  The reordering is invisible to the
// DSP chain - every sample carries its TRUE tone index on tuser, and the
// averaging / calibration / window stages index their tables by it.
//
// At a sub-band boundary the core pauses and pulses sb_req; the PS driver
// reprograms the RFDC mixer (XRFdc_SetMixerSettings) and pulses sb_ack.
// With N_SB = 1 the handshake never fires.
//
// Amplitude per tone = amp.hex, the baked B02+B03 result (predriver, PA
// soft compression, cable loss, peak normalisation) produced by
// make_golden.m from code_waveform/code_tx - no RTL-side approximation.
//
// Sequencing: per position, n_avg sweeps of all n_tones; each tone dwells
// dwell_cyc fabric clocks, emitting one IQ sample per accepted beat.
// pos_done pulses when a position's n_avg sweeps are complete.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_dds_txc #(
    parameter N_TONES   = 256,
    parameter N_SB      = 21,
    parameter N_POS     = 0,     // 0 = free-run; >0 = stop after one frame

    parameter FW_FILE   = "fw_seq_a.hex",
    parameter TIDX_FILE = "tidx_seq_a.hex",
    parameter AMP_FILE  = "amp_a.hex",
    parameter SB_FILE   = "sb_ends_a.hex",
    parameter SINE_FILE = "sine_rom.hex"
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        run,
    input  wire [31:0] dwell_cyc,        // samples per tone (>=1)
    input  wire [7:0]  n_avg_m1,         // sweeps per position minus 1
    // AXI4-Stream master -> RFDC DAC (interpolation + mixer in the RFDC)
    output reg         m_tvalid,
    input  wire        m_tready,
    output reg  [31:0] m_tdata,          // {I[31:16], Q[15:0]} Q15
    output reg         m_tlast,          // last beat of a full tone sweep
    output reg  [7:0]  m_tuser_tone,     // true tone index
    output reg  [7:0]  m_tuser_pos,      // position index (wraps at 256)
    // RFDC mixer re-tune handshake with the PS driver
    output reg         sb_req,
    input  wire        sb_ack,
    output reg         pos_done,
    output reg  [7:0]  tone_seq_idx
);
    reg [47:0] fw_rom   [0:N_TONES-1];
    reg [7:0]  tidx_rom [0:N_TONES-1];
    reg signed [15:0] amp_rom [0:N_TONES-1];
    reg [15:0] sb_ends  [0:31];
    reg [15:0] sine_rom [0:4095];
    initial begin
        $readmemh(FW_FILE,   fw_rom);
        $readmemh(TIDX_FILE, tidx_rom);
        $readmemh(AMP_FILE,  amp_rom);
        $readmemh(SB_FILE,   sb_ends);
        $readmemh(SINE_FILE, sine_rom);
    end

    reg [47:0] phase;
    reg [31:0] dwell_cnt;
    reg [7:0]  tone_i, avg_i, pos_i, sb_ptr;
    reg        waiting_ack;

    // quarter-wave sine evaluation at the current phase
    wire [1:0]  quad  = phase[47:46];
    wire [11:0] saddr = phase[45:34];
    wire signed [15:0] s0 = $signed(sine_rom[saddr]);
    wire signed [15:0] s1 = $signed(sine_rom[12'd4095 - saddr]);
    reg  signed [15:0] sin_v, cos_v;
    always @(*) begin
        case (quad)
            2'd0:    begin sin_v =  s0; cos_v =  s1; end
            2'd1:    begin sin_v =  s1; cos_v = -s0; end
            2'd2:    begin sin_v = -s0; cos_v = -s1; end
            default: begin sin_v = -s1; cos_v =  s0; end
        endcase
    end

    wire signed [15:0] ampq = amp_rom[tidx_rom[tone_i]];
    wire signed [32:0] pi_q = ampq * cos_v;   // Q30
    wire signed [32:0] pq_q = ampq * sin_v;

    function signed [15:0] sat16(input signed [32:0] v);
        begin
            if      (v >  33'sd32767) sat16 =  16'sd32767;
            else if (v < -33'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction
    wire signed [15:0] out_i = sat16((pi_q + 33'sd16384) >>> 15);
    wire signed [15:0] out_q = sat16((pq_q + 33'sd16384) >>> 15);

    // a beat is emitted when we are running and the stream can accept
    wire advance = run && !stop && !waiting_ack && (!m_tvalid || m_tready);
    wire dwell_end = (dwell_cnt + 32'd1 >= dwell_cyc);
    wire sweep_end = (tone_i == N_TONES-1);

    reg stop;        // frame complete - never advance again until reset
    reg last_pres;   // the beat currently presented is the frame's last

    always @(posedge clk) begin
        if (rst) begin
            m_tvalid <= 0; m_tlast <= 0; m_tdata <= 0;
            m_tuser_tone <= 0; m_tuser_pos <= 0;
            phase <= 0; dwell_cnt <= 0;
            tone_i <= 0; avg_i <= 0; pos_i <= 0; sb_ptr <= 0;
            sb_req <= 0; waiting_ack <= 0; pos_done <= 0; tone_seq_idx <= 0;
            stop <= 0; last_pres <= 0;
        end else begin
            pos_done <= 1'b0;
            sb_req   <= 1'b0;
            if (waiting_ack && sb_ack) waiting_ack <= 1'b0;

            if (!run) m_tvalid <= 1'b0;   // host stop: retire the beat at once

            if (advance) begin
                m_tvalid     <= 1'b1;
                m_tdata      <= {out_i, out_q};
                m_tuser_tone <= tidx_rom[tone_i];
                m_tuser_pos  <= pos_i;
                tone_seq_idx <= tone_i;
                m_tlast      <= 1'b0;
                last_pres    <= (N_POS != 0) && dwell_end && sweep_end &&
                                (avg_i == n_avg_m1) && (pos_i == N_POS-1);
                phase        <= phase + fw_rom[tone_i];

                if (dwell_end) begin
                    dwell_cnt <= 32'd0;
                    m_tlast   <= sweep_end;
                    phase     <= 48'd0;             // coherent tone start
                    if (sweep_end) begin
                        tone_i <= 8'd0;
                        sb_ptr <= 8'd0;
                        if (avg_i == n_avg_m1) begin
                            avg_i    <= 8'd0;
                            pos_done <= 1'b1;
                            pos_i    <= pos_i + 8'd1;
                        end else
                            avg_i <= avg_i + 8'd1;
                    end else begin
                        tone_i <= tone_i + 8'd1;
                        if ((N_SB > 1) && (tone_i == sb_ends[sb_ptr[4:0]])) begin
                            sb_ptr      <= sb_ptr + 8'd1;
                            sb_req      <= 1'b1;
                            waiting_ack <= 1'b1;
                        end
                    end
                end else
                    dwell_cnt <= dwell_cnt + 32'd1;
            end
            // frame stop: once the final beat is consumed, retire valid and
            // never present another.  Must follow the advance block so this
            // assignment overrides the re-presentation above.
            if (m_tvalid && m_tready && last_pres) begin
                m_tvalid <= 1'b0;
                stop     <= 1'b1;
            end
        end
    end
endmodule
