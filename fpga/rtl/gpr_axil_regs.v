// ---------------------------------------------------------------------------
// gpr_axil_regs.v - minimal AXI4-Lite slave holding every GPR chain control
// register + the B14 report readback + test-point beat counters.
// Register map (byte offsets, all 32-bit):
//   0x00 ID (RO, 0x47505200|ver)     0x04 CTRL  (b0 run, b1 loop)
//   0x08 STATUS (RO)                 0x0C IRQ (b0 event W1C, b1 enable)
//   0x10 DWELL_CYC   0x14 N_AVG_M1   0x18 AGC_GAIN_Q14  0x1C FFT_GAIN_Q10
//   0x20 NBG         0x24 INV_DR_Q20 0x28 ROFF_Q20      0x2C DX_Q20
//   0x30 DR_Q20      0x34 KBETA_Q16  0x38 ALPHA2_Q8     0x3C MAX_BIN
//   0x40 OUT_SCALE_Q8 0x44 B_MIN0    0x48 B_MAX0        0x4C CFAR_MODE
//   0x50 ALPHA_Q12   0x54 LOG2ALPHA_Q26 0x58 GAMMA2_Q26
//   0x5C PFLOOR (K[31:16] | SH[4:0]) 0x60 DR_Q16        0x64 DX_Q16
//   0x80..0x94 REP1..REP6 (RO)
//   0xA0 CNT_ADC 0xA4 CNT_AVGGRP 0xA8 CNT_RP 0xAC CNT_BS 0xB0 CNT_MIG
//   0xB4 CNT_DET (all RO, wrap)
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_axil_regs (
    input  wire        clk,
    input  wire        rst,
    // AXI4-Lite slave
    input  wire        s_awvalid,
    output reg         s_awready,
    input  wire [15:0] s_awaddr,
    input  wire        s_wvalid,
    output reg         s_wready,
    input  wire [31:0] s_wdata,
    input  wire [3:0]  s_wstrb,
    output reg         s_bvalid,
    input  wire        s_bready,
    output reg  [1:0]  s_bresp,
    input  wire        s_arvalid,
    output reg         s_arready,
    input  wire [15:0] s_araddr,
    output reg         s_rvalid,
    input  wire        s_rready,
    output reg  [31:0] s_rdata,
    output reg  [1:0]  s_rresp,
    // control outputs
    output reg         run, loop_mode,
    output reg  [31:0] dwell_cyc,
    output reg  [7:0]  n_avg_m1,
    output reg  [15:0] agc_gain, fft_gain,
    output reg  [3:0]  nbg,
    output reg  [31:0] inv_dr_q20, roff_q20, dx_q20, dr_q20, kbeta_q16,
    output reg  signed [15:0] alpha2_q8,
    output reg  [15:0] max_bin, out_scale_q8, b_min0, b_max0,
    output reg         cfar_mode,
    output reg  [15:0] alpha_q12,
    output reg  signed [31:0] log2alpha_q26, gamma2_q26,
    output reg  [15:0] pfloor_k,
    output reg  [4:0]  pfloor_sh,
    output reg  [31:0] dr_q16, dx_q16,
    // status inputs
    input  wire [31:0] rep1, rep2, rep3, rep4, rep5, rep6,
    input  wire        rep_done,
    input  wire [7:0]  frame_count,
    input  wire        bg_overrun,
    input  wire [31:0] cnt_adc, cnt_avggrp, cnt_rp, cnt_bs, cnt_mig, cnt_det,
    output reg         irq
);
    reg        irq_en, irq_ev;
    reg [15:0] awaddr_r, araddr_r;

    // write FSM: accept AW and W together, then respond
    wire w_accept = s_awvalid && s_wvalid && s_awready && s_wready;
    always @(posedge clk) begin
        if (rst) begin
            s_awready <= 0; s_wready <= 0; s_bvalid <= 0; s_bresp <= 0;
            run <= 0; loop_mode <= 0; dwell_cyc <= 32'd12288; n_avg_m1 <= 0;
            agc_gain <= 16384; fft_gain <= 18872; nbg <= 1;
            inv_dr_q20 <= 0; roff_q20 <= 0; dx_q20 <= 0; dr_q20 <= 0;
            kbeta_q16 <= 0; alpha2_q8 <= 0; max_bin <= 0; out_scale_q8 <= 256;
            b_min0 <= 0; b_max0 <= 0; cfar_mode <= 0; alpha_q12 <= 0;
            log2alpha_q26 <= 0; gamma2_q26 <= 0;
            pfloor_k <= 2874; pfloor_sh <= 28; dr_q16 <= 0; dx_q16 <= 0;
        end else begin
            s_awready <= s_awvalid && s_wvalid && !s_bvalid;
            s_wready  <= s_awvalid && s_wvalid && !s_bvalid;
            awaddr_r  <= s_awaddr;
            if (s_bvalid && s_bready) s_bvalid <= 0;
            if (w_accept) begin
                s_bvalid <= 1;
                s_bresp  <= 2'b00;
                case (s_awaddr[15:2])
                (16'h04 >> 2): begin run      <= s_wdata[0]; loop_mode <= s_wdata[1]; end
                (16'h10 >> 2): dwell_cyc     <= s_wdata;
                (16'h14 >> 2): n_avg_m1      <= s_wdata[7:0];
                (16'h18 >> 2): agc_gain      <= s_wdata[15:0];
                (16'h1C >> 2): fft_gain      <= s_wdata[15:0];
                (16'h20 >> 2): nbg           <= s_wdata[3:0];
                (16'h24 >> 2): inv_dr_q20    <= s_wdata;
                (16'h28 >> 2): roff_q20      <= s_wdata;
                (16'h2C >> 2): dx_q20        <= s_wdata;
                (16'h30 >> 2): dr_q20        <= s_wdata;
                (16'h34 >> 2): kbeta_q16     <= s_wdata;
                (16'h38 >> 2): alpha2_q8     <= $signed(s_wdata[15:0]);
                (16'h3C >> 2): begin max_bin <= s_wdata[15:0]; out_scale_q8 <= s_wdata[31:16]; end
                (16'h44 >> 2): b_min0        <= s_wdata[15:0];
                (16'h48 >> 2): b_max0        <= s_wdata[15:0];
                (16'h4C >> 2): cfar_mode     <= s_wdata[0];
                (16'h50 >> 2): alpha_q12     <= s_wdata[15:0];
                (16'h54 >> 2): log2alpha_q26 <= $signed(s_wdata);
                (16'h58 >> 2): gamma2_q26    <= $signed(s_wdata);
                (16'h5C >> 2): begin pfloor_k <= s_wdata[31:16]; pfloor_sh <= s_wdata[4:0]; end
                (16'h60 >> 2): dr_q16        <= s_wdata;
                (16'h64 >> 2): dx_q16        <= s_wdata;
                default: ;
                endcase
            end
        end
    end

    // read FSM
    reg [31:0] rdata_c;
    always @(*) begin
        case (araddr_r[15:2])
        (16'h00 >> 2): rdata_c = 32'h4750_5200 | 32'h18;    // id | ver 1.8
        (16'h04 >> 2): rdata_c = {30'd0, loop_mode, run};
        (16'h08 >> 2): rdata_c = {7'd0, bg_overrun, 7'd0, frame_count, 6'd0, irq_ev, 1'b0};
        (16'h0C >> 2): rdata_c = {30'd0, irq_en, irq_ev};
        (16'h80 >> 2): rdata_c = rep1;
        (16'h84 >> 2): rdata_c = rep2;
        (16'h88 >> 2): rdata_c = rep3;
        (16'h8C >> 2): rdata_c = rep4;
        (16'h90 >> 2): rdata_c = rep5;
        (16'h94 >> 2): rdata_c = rep6;
        (16'hA0 >> 2): rdata_c = cnt_adc;
        (16'hA4 >> 2): rdata_c = cnt_avggrp;
        (16'hA8 >> 2): rdata_c = cnt_rp;
        (16'hAC >> 2): rdata_c = cnt_bs;
        (16'hB0 >> 2): rdata_c = cnt_mig;
        (16'hB4 >> 2): rdata_c = cnt_det;
        default:       rdata_c = 32'd0;
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            s_arready <= 0; s_rvalid <= 0; s_rdata <= 0; s_rresp <= 0;
            araddr_r <= 0; irq <= 0; irq_en <= 0; irq_ev <= 0;
        end else begin
            if (!s_arready && s_arvalid) begin
                s_arready <= 1;
                araddr_r  <= s_araddr;
            end else if (s_arready) begin
                s_arready <= 0;
                s_rvalid  <= 1;
                s_rdata   <= rdata_c;
            end
            if (s_rvalid && s_rready) s_rvalid <= 0;

            if (rep_done) irq_ev <= 1'b1;
            if (irq_ev && irq_en) irq <= 1'b1;
            // W1C at 0x0C bit0
            if (w_accept && (s_awaddr == 16'h0C) && s_wdata[0]) begin
                irq_ev <= 1'b0;
                irq    <= 1'b0;
            end
            if (w_accept && (s_awaddr == 16'h0C))
                irq_en <= s_wdata[1];
        end
    end
endmodule
