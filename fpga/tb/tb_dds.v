// tb_dds.v - tone-stepped DDS check: envelope magnitude, per-tone zero
// crossing count (= programmed frequency), tone index sideband, sweep tlast
`timescale 1ns / 1ps
module tb_dds;
    localparam NT = 16;
    reg clk = 0, rst = 1, run = 0, sb_ack = 0;
    wire m_v, m_l; wire [31:0] m_d; wire [7:0] m_tone, m_pos; wire sb_req, pos_done;
    wire [7:0] seq;
    integer errors = 0;

    gpr_dds_txc #(
        .N_TONES(NT), .N_SB(1),
        .FW_FILE("fw_seq_t.hex"), .TIDX_FILE("tidx_seq_t.hex"),
        .AMP_FILE("amp_t.hex"), .SB_FILE("sb_ends_t.hex")
    ) dut (
        .clk(clk), .rst(rst), .run(run), .dwell_cyc(32'd64), .n_avg_m1(8'd1),
        .m_tvalid(m_v), .m_tready(1'b1), .m_tdata(m_d), .m_tlast(m_l),
        .m_tuser_tone(m_tone), .m_tuser_pos(m_pos),
        .sb_req(sb_req), .sb_ack(sb_ack), .pos_done(pos_done),
        .tone_seq_idx(seq));

    always #2 clk = ~clk;

    // capture one full sweep (2 sweeps actually: n_avg=2)
    integer  tone_now, samples_in_tone, crossings, prev_i;
    integer  cur_tone, i_v, q_v, mag2, tt;
    real     ref;
    reg      seen_tlast = 0;
    integer  tones_checked = 0;

    initial begin
        cur_tone = -1; samples_in_tone = 0; crossings = 0; prev_i = 0;
        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);
        run <= 1;
    end

    always @(posedge clk) begin
        if (!rst && m_v) begin
            i_v = $signed(m_d[31:16]);
            q_v = $signed(m_d[15:0]);
            // envelope: I^2+Q^2 == amp^2 (amp = 32767)
            mag2 = i_v*i_v + q_v*q_v;
            ref  = 32767.0*32767.0;
            if ((mag2 - ref) > 0.004*ref || (ref - mag2) > 0.004*ref) begin
                if (samples_in_tone > 2) begin   // ignore pipeline warm-up
                    $display("DDS envelope fail tone=%0d sample=%0d mag2=%0d",
                             m_tone, samples_in_tone, mag2);
                    errors = errors + 1;
                end
            end
            // zero crossings of I for frequency check
            if (samples_in_tone > 0) begin
                if ((prev_i < 0 && i_v >= 0))
                    crossings = crossings + 1;
            end
            prev_i = i_v;
            samples_in_tone = samples_in_tone + 1;
            if (m_tone != cur_tone) begin
                // tone changed: evaluate the finished one
                if (samples_in_tone > 0 && cur_tone > 0 && cur_tone < 8) begin
                    // freq = cur_tone * F_CLK/64; 64 samples -> cur_tone
                    // cycles -> cur_tone rising zero crossings (+-1 edge)
                    if (crossings < cur_tone-1 || crossings > cur_tone+1) begin
                        $display("DDS freq fail tone=%0d crossings=%0d",
                                 cur_tone, crossings);
                        errors = errors + 1;
                    end
                    tones_checked = tones_checked + 1;
                end
                cur_tone = m_tone;
                crossings = 0;
                samples_in_tone = 0;
            end
            if (m_l) seen_tlast = 1;
        end
    end

    initial begin
        #60000;                      // ~2 sweeps of 16 tones x 64 samples
        run <= 0;
        #100;
        if (!seen_tlast) begin
            $display("DDS fail: no sweep tlast seen");
            errors = errors + 1;
        end
        if (tones_checked < 6) begin
            $display("DDS fail: only %0d tones frequency-checked", tones_checked);
            errors = errors + 1;
        end
        if (errors == 0) $display("TB_DDS PASS (%0d tones checked)", tones_checked);
        else             $display("TB_DDS FAIL (%0d)", errors);
        $finish;
    end
endmodule
