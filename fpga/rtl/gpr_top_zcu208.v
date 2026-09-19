// ---------------------------------------------------------------------------
// gpr_top_zcu208.v - GPM SFCW-GPR processing chain for the ZCU208 (ZU48DR).
//
//   B02/B03  gpr_dds_txc     tone-stepped SFCW excitation  -> RFDC DAC AXIS
//   (B01/B04/B05/B06: physical world + RFDC hard IP; see README)
//   B07      gpr_adc_if      RFDC ADC AXIS -> AGC/clip Q15
//   B08      gpr_avg         coherent averaging (2^LOG2_NAVG sweeps)
//   B09      gpr_cal         per-tone system-response removal (ROM)
//   B10      gpr_window +    Kaiser window + 2048-pt zero-padded IFFT
//            gpr_fft         (baked gain 18.4298)
//   B11      gpr_background  per-bin median across positions, subtract
//   B12      gpr_migrate     coherent Kirchhoff migration
//   B13      gpr_cfar        2-D CA-CFAR (mean/log)
//   B14      gpr_cluster     connected components + report registers
//
// Every inter-block stream is also exported as a monitor port (tp_*) so the
// Vivado ILA can watch the exact equivalents of the Simulink Scope test
// points TP01..TP14.  loopback=1 feeds the DDS output straight into the ADC
// interface for self-test without RF.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_top_zcu208 #(
    parameter N_TONES  = 256,
    parameter N_POS    = 200,
    parameter N_IFFT   = 2048,
    parameter LB       = 11,
    parameter N_SB     = 21,
    parameter LOG2NAV  = 5,
    parameter GUARD    = 8,
    parameter TRAIN    = 16,
    // ROM images (paths relative to the simulation/synthesis run dir)
    parameter FW_FILE   = "fw_seq_a.hex",
    parameter TIDX_FILE = "tidx_seq_a.hex",
    parameter AMP_FILE  = "amp_a.hex",
    parameter SB_FILE   = "sb_ends_a.hex",
    parameter CAL_FILE  = "cal_a.hex",
    parameter NAP_FILE  = "nap_a.hex"
) (
    input  wire        clk,               // fabric clock (245.76 MHz typ.)
    input  wire        rst,               // sync, active high
    input  wire        loopback,
    // RFDC DAC stream (m) - to interpolation/DAC tile
    output wire        m_axis_dac_tvalid,
    input  wire        m_axis_dac_tready,
    output wire [31:0] m_axis_dac_tdata,
    output wire        m_axis_dac_tlast,
    output wire [7:0]  m_axis_dac_tuser_tone,
    output wire [7:0]  m_axis_dac_tuser_pos,
    // RFDC ADC stream (s) - from decimation/ADC tile
    input  wire        s_axis_adc_tvalid,
    output wire        s_axis_adc_tready,
    input  wire [31:0] s_axis_adc_tdata,
    input  wire        s_axis_adc_tlast,
    input  wire [7:0]  s_axis_adc_tuser_tone,
    input  wire [7:0]  s_axis_adc_tuser_pos,
    // RFDC mixer re-tune handshake (PS driver)
    output wire        sb_req,
    input  wire        sb_ack,
    // AXI4-Lite control
    input  wire        s_axil_awvalid,
    output wire        s_axil_awready,
    input  wire [15:0] s_axil_awaddr,
    input  wire        s_axil_wvalid,
    output wire        s_axil_wready,
    input  wire [31:0] s_axil_wdata,
    input  wire [3:0]  s_axil_wstrb,
    output wire        s_axil_bvalid,
    input  wire        s_axil_bready,
    output wire [1:0]  s_axil_bresp,
    input  wire        s_axil_arvalid,
    output wire        s_axil_arready,
    input  wire [15:0] s_axil_araddr,
    output wire        s_axil_rvalid,
    input  wire        s_axil_rready,
    output wire [31:0] s_axil_rdata,
    output wire [1:0]  s_axil_rresp,
    output wire        irq,
    // monitor streams (= Simulink Scope test points), always-ready sinks
    output wire        tp_rp_valid,   output wire [31:0] tp_rp_data,
    output wire [15:0] tp_rp_user,
    output wire        tp_bs_valid,   output wire [31:0] tp_bs_data,
    output wire [15:0] tp_bs_user,    output wire [7:0] tp_bs_pos,
    output wire        tp_mig_valid,  output wire [15:0] tp_mig_data,
    output wire [15:0] tp_mig_user,   output wire [7:0] tp_mig_pos,
    output wire        tp_det_valid,  output wire [15:0] tp_det_data,
    output wire        tp_det_det,    output wire        tp_det_pass2,
    output wire [15:0] tp_det_user,   output wire [7:0] tp_det_pos
);
    // ---------------------------------------------------- register file
    wire        run, loop_mode;
    wire [31:0] dwell_cyc;
    wire [7:0]  n_avg_m1;
    wire [15:0] agc_gain, fft_gain;
    wire [3:0]  nbg;
    wire [31:0] inv_dr_q20, roff_q20, dx_q20, dr_q20, kbeta_q16;
    wire signed [15:0] alpha2_q8;
    wire [15:0] max_bin, out_scale_q8, b_min0, b_max0;
    wire        cfar_mode;
    wire [15:0] alpha_q12;
    wire signed [31:0] log2alpha_q26, gamma2_q26;
    wire [15:0] pfloor_k;
    wire [4:0]  pfloor_sh;
    wire [31:0] dr_q16, dx_q16;
    wire [31:0] rep1, rep2, rep3, rep4, rep5, rep6;
    wire        rep_done;
    wire [7:0]  frame_count;
    wire        bg_overrun;
    wire [31:0] cnt_adc, cnt_avggrp, cnt_rp, cnt_bs, cnt_mig, cnt_det;

    // ---------------------------------------------------- stream wires
    // dds -> dac / loopback
    wire        dds_v, dds_l;
    wire [31:0] dds_d;
    wire [7:0]  dds_tone, dds_pos;
    wire        dds_r;
    // adc -> avg
    wire        adc_v, adc_r, adc_l;
    wire [31:0] adc_d;
    wire [7:0]  adc_tone, adc_pos;
    // avg -> cal
    wire        avg_v, avg_r, avg_l, avg_flush;
    wire [31:0] avg_d;
    wire [7:0]  avg_tone, avg_pos;
    // cal -> window
    wire        cal_v, cal_r, cal_l;
    wire [31:0] cal_d;
    wire [7:0]  cal_tone, cal_pos;
    // window -> fft
    wire        win_v, win_r, win_l;
    wire [31:0] win_d;
    wire [7:0]  win_tone, win_pos;
    // fft -> background
    wire        fft_v, fft_r, fft_l, fft_busy, fft_done;
    wire [31:0] fft_d;
    wire [15:0] fft_bin;
    // background -> migrate
    wire        bg_v, bg_r, bg_l, bg_fl, bg_frdy;
    wire [31:0] bg_d;
    wire [15:0] bg_bin;
    wire [7:0]  bg_pos;
    // migrate -> cfar
    wire        mg_v, mg_r, mg_l, mg_fl, mg_busy;
    wire [15:0] mg_d, mg_iz;
    wire [7:0]  mg_ix;
    // cfar -> cluster
    wire        cf_v, cf_r, cf_l, cf_fl, cf_p2, cf_det, cf_f1d, cf_busy;
    wire [15:0] cf_d, cf_iz;
    wire [7:0]  cf_ix;

    // ---------------------------------------------------- one-shot frame run
    // run = acquire exactly one frame (N_POS positions); pos_done latches the
    // stop until the host clears and re-writes CTRL.run.
    wire        dds_pos_done;
    reg         frame_done;
    reg [15:0]  pos_cnt;
    wire        dds_run = run && !frame_done;
    // pos_done pulses at EVERY position - only latch frame_done after the
    // last one (N_POS positions per frame).
    always @(posedge clk) begin
        if (rst || !run) begin
            frame_done <= 1'b0;
            pos_cnt    <= 16'd0;
        end else if (dds_pos_done) begin
            pos_cnt <= pos_cnt + 1'b1;
            if (pos_cnt == N_POS-1)
                frame_done <= 1'b1;
        end
    end

    // ---------------------------------------------------- FFT load handshake
    // avg flush_start may arrive while the FFT is still busy with the
    // previous position - latch it and replay when the FFT is idle.
    reg lb_pend, lb_pulse;
    wire fft_idle;
    always @(posedge clk) begin
        if (rst) begin
            lb_pend <= 0; lb_pulse <= 0;
        end else begin
            lb_pulse <= 0;
            if (avg_flush)
                lb_pend <= 1;
            if (lb_pend && fft_idle && !avg_flush) begin
                lb_pulse <= 1;
                lb_pend  <= 0;
            end
        end
    end

    // ---------------------------------------------------- start_fft sequencer
    // pulse start_fft a few cycles after the last windowed tone of a
    // position (pipeline drain of cal/window)
    reg [3:0] last_dly;
    reg       start_fft;
    always @(posedge clk) begin
        if (rst) begin
            last_dly <= 0; start_fft <= 0;
        end else begin
            last_dly <= {last_dly[2:0], (win_v && win_r && win_l)};
            start_fft <= last_dly[3] && !fft_busy;
        end
    end

    // ---------------------------------------------------- blocks
    gpr_dds_txc #(
        .N_TONES(N_TONES), .N_SB(N_SB), .N_POS(N_POS),
        .FW_FILE(FW_FILE), .TIDX_FILE(TIDX_FILE), .AMP_FILE(AMP_FILE),
        .SB_FILE(SB_FILE)
    ) u_dds (
        .clk(clk), .rst(rst), .run(dds_run), .dwell_cyc(dwell_cyc),
        .n_avg_m1(n_avg_m1),
        .m_tvalid(dds_v), .m_tready(dds_r), .m_tdata(dds_d), .m_tlast(dds_l),
        .m_tuser_tone(dds_tone), .m_tuser_pos(dds_pos),
        .sb_req(sb_req), .sb_ack(sb_ack), .pos_done(dds_pos_done),
        .tone_seq_idx());

    // loopback mux: RFDC or internal
    wire        adcv = loopback ? dds_v  : s_axis_adc_tvalid;
    wire [31:0] adcd = loopback ? dds_d  : s_axis_adc_tdata;
    wire        adcl = loopback ? dds_l  : s_axis_adc_tlast;
    wire [7:0]  adct = loopback ? dds_tone : s_axis_adc_tuser_tone;
    wire [7:0]  adcp = loopback ? dds_pos  : s_axis_adc_tuser_pos;
    wire        adcr;
    assign dds_r = loopback ? adcr : m_axis_dac_tready;
    assign s_axis_adc_tready = loopback ? 1'b0 : adcr;
    assign m_axis_dac_tvalid = loopback ? 1'b0 : dds_v;
    assign m_axis_dac_tdata  = dds_d;
    assign m_axis_dac_tlast  = dds_l;
    assign m_axis_dac_tuser_tone = dds_tone;
    assign m_axis_dac_tuser_pos  = dds_pos;

    gpr_adc_if u_adc (
        .clk(clk), .rst(rst), .agc_gain_q14(agc_gain),
        .s_tvalid(adcv), .s_tready(adcr), .s_tdata(adcd), .s_tlast(adcl),
        .s_tuser_tone(adct), .s_tuser_pos(adcp),
        .m_tvalid(adc_v), .m_tready(adc_r), .m_tdata(adc_d), .m_tlast(adc_l),
        .m_tuser_tone(adc_tone), .m_tuser_pos(adc_pos));

    gpr_avg #(.N_TONES(N_TONES), .LOG2_NAVG(LOG2NAV)) u_avg (
        .clk(clk), .rst(rst),
        .s_tvalid(adc_v), .s_tready(adc_r), .s_tdata(adc_d), .s_tlast(adc_l),
        .s_tuser_tone(adc_tone), .s_tuser_pos(adc_pos),
        .m_tvalid(avg_v), .m_tready(avg_r), .m_tdata(avg_d), .m_tlast(avg_l),
        .m_tuser_tone(avg_tone), .m_tuser_pos(avg_pos),
        .group_done(), .flush_start(avg_flush));

    gpr_cal #(.N_TONES(N_TONES), .ROM_FILE(CAL_FILE)) u_cal (
        .clk(clk), .rst(rst),
        .s_tvalid(avg_v), .s_tready(avg_r), .s_tdata(avg_d), .s_tlast(avg_l),
        .s_tuser_tone(avg_tone), .s_tuser_pos(avg_pos),
        .m_tvalid(cal_v), .m_tready(cal_r), .m_tdata(cal_d), .m_tlast(cal_l),
        .m_tuser_tone(cal_tone), .m_tuser_pos(cal_pos));

    gpr_window #(.N_TONES(N_TONES)) u_win (
        .clk(clk), .rst(rst),
        .s_tvalid(cal_v), .s_tready(cal_r), .s_tdata(cal_d), .s_tlast(cal_l),
        .s_tuser_tone(cal_tone), .s_tuser_pos(cal_pos),
        .m_tvalid(win_v), .m_tready(win_r), .m_tdata(win_d), .m_tlast(win_l),
        .m_tuser_tone(win_tone), .m_tuser_pos(win_pos));

    gpr_fft #(.N(N_IFFT), .LOG2N(LB)) u_fft (
        .clk(clk), .rst(rst), .inv(1'b1), .gain_q10(fft_gain),
        .load_begin(lb_pulse),
        .s_tvalid(win_v), .s_tready(win_r), .s_tdata(win_d),
        .s_tuser_tone(win_tone),
        .start_fft(start_fft),
        .m_tvalid(fft_v), .m_tready(fft_r), .m_tdata(fft_d),
        .m_tuser(fft_bin), .m_tlast(fft_l), .busy(fft_busy), .done(fft_done),
        .idle(fft_idle));

    gpr_background #(.N_IFFT(N_IFFT), .N_POS(N_POS), .LB(LB)) u_bg (
        .clk(clk), .rst(rst), .nbg(nbg),
        .s_tvalid(fft_v), .s_tdata(fft_d), .s_tuser(fft_bin), .s_tlast(fft_l),
        .m_tvalid(bg_v), .m_tready(bg_r), .m_tdata(bg_d), .m_tuser(bg_bin),
        .m_tuser_pos(bg_pos), .m_tlast(bg_l), .m_frame_last(bg_fl),
        .frame_ready(bg_frdy), .frame_count(frame_count), .overrun(bg_overrun));
    assign fft_r = 1'b1;      // background accepts unconditionally (design rule)

    gpr_migrate #(
        .N_IFFT(N_IFFT), .N_POS(N_POS), .LB(LB), .NAP_FILE(NAP_FILE)
    ) u_mig (
        .clk(clk), .rst(rst),
        .s_tvalid(bg_v), .s_tready(bg_r), .s_tdata(bg_d), .s_tuser(bg_bin),
        .s_tuser_pos(bg_pos), .s_tlast(bg_l), .s_frame_last(bg_fl),
        .inv_dr_q20(inv_dr_q20), .roff_q20(roff_q20), .dx_q20(dx_q20),
        .dr_q20(dr_q20), .kbeta_q16(kbeta_q16), .alpha2_q8(alpha2_q8),
        .max_bin(max_bin), .out_scale_q8(out_scale_q8), .go(run),
        .m_tvalid(mg_v), .m_tready(mg_r), .m_tdata(mg_d), .m_tuser(mg_iz),
        .m_tuser_pos(mg_ix), .m_tlast(mg_l), .m_frame_last(mg_fl),
        .busy(mg_busy));

    gpr_cfar #(
        .N_IFFT(N_IFFT), .N_POS(N_POS), .GUARD(GUARD), .TRAIN(TRAIN), .LB(LB)
    ) u_cfar (
        .clk(clk), .rst(rst),
        .s_tvalid(mg_v), .s_tready(mg_r), .s_tdata(mg_d), .s_tuser(mg_iz),
        .s_tuser_pos(mg_ix), .s_frame_last(mg_fl), .go(run),
        .b_min0(b_min0), .b_max0(b_max0), .cfar_mode(cfar_mode),
        .alpha_q12(alpha_q12), .log2alpha_q26(log2alpha_q26),
        .gamma2_q26(gamma2_q26), .pfloor_k(pfloor_k), .pfloor_sh(pfloor_sh),
        .m_tvalid(cf_v), .m_tready(cf_r), .m_tdata(cf_d), .m_tdet(cf_det),
        .m_pass2(cf_p2), .m_tuser(cf_iz), .m_tuser_pos(cf_ix),
        .m_tlast(cf_l), .m_frame_last(cf_fl), .busy(cf_busy),
        .frame1_done(cf_f1d));

    gpr_cluster #(.N_IFFT(N_IFFT), .N_POS(N_POS), .LB(LB)) u_clu (
        .clk(clk), .rst(rst),
        .s_tvalid(cf_v), .s_tready(cf_r), .s_tdata(cf_d), .s_tdet(cf_det),
        .s_pass2(cf_p2), .s_tuser(cf_iz), .s_tuser_pos(cf_ix),
        .s_frame_last(cf_fl), .go(run),
        .dr_q16(dr_q16), .dx_q16(dx_q16),
        .rep1(rep1), .rep2(rep2), .rep3(rep3), .rep4(rep4),
        .rep5(rep5), .rep6(rep6), .rep_done(rep_done), .busy());

    // ---------------------------------------------------- test points
    // tee: monitors are always-ready sinks
    assign tp_rp_valid = fft_v;  assign tp_rp_data = fft_d;
    assign tp_rp_user  = fft_bin;
    assign tp_bs_valid = bg_v;   assign tp_bs_data = bg_d;
    assign tp_bs_user  = bg_bin; assign tp_bs_pos = bg_pos;
    assign tp_mig_valid = mg_v;  assign tp_mig_data = mg_d;
    assign tp_mig_user  = mg_iz; assign tp_mig_pos = mg_ix;
    assign tp_det_valid = cf_v && !cf_p2;
    assign tp_det_data  = cf_d;  assign tp_det_det = cf_det;
    assign tp_det_pass2 = cf_p2;
    assign tp_det_user  = cf_iz; assign tp_det_pos = cf_ix;
    // background consumes the FFT stream; the monitor must not load it:
    // fft_r already driven by bg_r above.

    // beat counters (visibility without ILA)
    reg [31:0] c_adc, c_avg, c_rp, c_bs, c_mig, c_det;
    always @(posedge clk) begin
        if (rst) begin
            c_adc <= 0; c_avg <= 0; c_rp <= 0; c_bs <= 0; c_mig <= 0; c_det <= 0;
        end else begin
            if (adcv && adcr)          c_adc <= c_adc + 1;
            if (avg_flush)             c_avg <= c_avg + 1;
            if (fft_v && fft_r)        c_rp  <= c_rp  + 1;
            if (bg_v && bg_r)          c_bs  <= c_bs  + 1;
            if (mg_v && mg_r)          c_mig <= c_mig + 1;
            if (cf_v && cf_r && !cf_p2) c_det <= c_det + 1;
        end
    end
    assign cnt_adc = c_adc;   assign cnt_avggrp = c_avg;
    assign cnt_rp  = c_rp;    assign cnt_bs     = c_bs;
    assign cnt_mig = c_mig;   assign cnt_det    = c_det;

    gpr_axil_regs u_regs (
        .clk(clk), .rst(rst),
        .s_awvalid(s_axil_awvalid), .s_awready(s_axil_awready),
        .s_awaddr(s_axil_awaddr),
        .s_wvalid(s_axil_wvalid), .s_wready(s_axil_wready),
        .s_wdata(s_axil_wdata), .s_wstrb(s_axil_wstrb),
        .s_bvalid(s_axil_bvalid), .s_bready(s_axil_bready),
        .s_bresp(s_axil_bresp),
        .s_arvalid(s_axil_arvalid), .s_arready(s_axil_arready),
        .s_araddr(s_axil_araddr),
        .s_rvalid(s_axil_rvalid), .s_rready(s_axil_rready),
        .s_rdata(s_axil_rdata), .s_rresp(s_axil_rresp),
        .run(run), .loop_mode(loop_mode), .dwell_cyc(dwell_cyc),
        .n_avg_m1(n_avg_m1), .agc_gain(agc_gain), .fft_gain(fft_gain),
        .nbg(nbg), .inv_dr_q20(inv_dr_q20), .roff_q20(roff_q20),
        .dx_q20(dx_q20), .dr_q20(dr_q20), .kbeta_q16(kbeta_q16),
        .alpha2_q8(alpha2_q8), .max_bin(max_bin), .out_scale_q8(out_scale_q8),
        .b_min0(b_min0), .b_max0(b_max0), .cfar_mode(cfar_mode),
        .alpha_q12(alpha_q12), .log2alpha_q26(log2alpha_q26),
        .gamma2_q26(gamma2_q26), .pfloor_k(pfloor_k), .pfloor_sh(pfloor_sh),
        .dr_q16(dr_q16), .dx_q16(dx_q16),
        .rep1(rep1), .rep2(rep2), .rep3(rep3), .rep4(rep4),
        .rep5(rep5), .rep6(rep6), .rep_done(rep_done),
        .frame_count(frame_count), .bg_overrun(bg_overrun),
        .cnt_adc(cnt_adc), .cnt_avggrp(cnt_avggrp), .cnt_rp(cnt_rp),
        .cnt_bs(cnt_bs), .cnt_mig(cnt_mig), .cnt_det(cnt_det),
        .irq(irq));
endmodule
