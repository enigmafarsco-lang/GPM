// tb_chain.v - full-chain plumbing smoke test at small parameters:
// DDS -> loopback -> ADC -> AVG -> CAL -> WINDOW -> FFT -> BACKGROUND ->
// MIGRATE -> CFAR -> CLUSTER, programmed over AXI-Lite.  With a constant
// loopback the median removes everything, so the expected report is
// rep1=rep2=0; the test asserts every handshake counter and the absence of
// hangs/overruns.
`timescale 1ns / 1ps
module dbg_chain;
    localparam NT = 16, NP = 8, NI = 64, LB = 6;
    reg clk = 0, rst = 1, loopback = 1, sb_ack = 0;
    // axi-lite
    reg awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
    reg [15:0] awaddr = 0, araddr = 0;
    reg [31:0] wdata = 0;
    wire awready, wready, bvalid, arready, rvalid, irq;
    wire [1:0] bresp, rresp;
    wire [31:0] rdata;
    // dac/adc unused in loopback
    wire dac_v, dac_l; wire [31:0] dac_d; wire [7:0] dac_t, dac_p;
    wire sb_req;
    // monitors
    wire rp_v, bs_v, mig_v, det_v, det_d, det_p2;
    wire [31:0] rp_d, bs_d; wire [15:0] rp_u, bs_u, mig_u, det_u;
    wire [7:0] bs_p, mig_p, det_p; wire [15:0] mig_d, det_dv;

    gpr_top_zcu208 #(
        .N_TONES(NT), .N_POS(NP), .N_IFFT(NI), .LB(LB),
        .N_SB(1), .LOG2NAV(1), .GUARD(4), .TRAIN(8),
        .FW_FILE("fw_seq_t.hex"), .TIDX_FILE("tidx_seq_t.hex"),
        .AMP_FILE("amp_t.hex"), .SB_FILE("sb_ends_t.hex"),
        .CAL_FILE("cal_t.hex"), .NAP_FILE("nap_t.hex")
    ) dut (
        .clk(clk), .rst(rst), .loopback(loopback),
        .m_axis_dac_tvalid(dac_v), .m_axis_dac_tready(1'b1),
        .m_axis_dac_tdata(dac_d), .m_axis_dac_tlast(dac_l),
        .m_axis_dac_tuser_tone(dac_t), .m_axis_dac_tuser_pos(dac_p),
        .s_axis_adc_tvalid(1'b0), .s_axis_adc_tready(),
        .s_axis_adc_tdata(32'd0), .s_axis_adc_tlast(1'b0),
        .s_axis_adc_tuser_tone(8'd0), .s_axis_adc_tuser_pos(8'd0),
        .sb_req(sb_req), .sb_ack(sb_ack),
        .s_axil_awvalid(awvalid), .s_axil_awready(awready), .s_axil_awaddr(awaddr),
        .s_axil_wvalid(wvalid), .s_axil_wready(wready), .s_axil_wdata(wdata),
        .s_axil_wstrb(4'hF),
        .s_axil_bvalid(bvalid), .s_axil_bready(bready), .s_axil_bresp(bresp),
        .s_axil_arvalid(arvalid), .s_axil_arready(arready), .s_axil_araddr(araddr),
        .s_axil_rvalid(rvalid), .s_axil_rready(rready), .s_axil_rdata(rdata),
        .s_axil_rresp(rresp),
        .irq(irq),
        .tp_rp_valid(rp_v), .tp_rp_data(rp_d), .tp_rp_user(rp_u),
        .tp_bs_valid(bs_v), .tp_bs_data(bs_d), .tp_bs_user(bs_u), .tp_bs_pos(bs_p),
        .tp_mig_valid(mig_v), .tp_mig_data(mig_d), .tp_mig_user(mig_u), .tp_mig_pos(mig_p),
        .tp_det_valid(det_v), .tp_det_data(det_dv), .tp_det_det(det_d),
        .tp_det_pass2(det_p2), .tp_det_user(det_u), .tp_det_pos(det_p));

    always #2 clk = ~clk;

    integer errors = 0;
    reg [31:0] rd;

    task axi_wr(input [15:0] a, input [31:0] d);
        begin
            @(posedge clk);
            awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
            @(posedge clk);
            while (!(awready && wready)) @(posedge clk);
            awvalid <= 0; wvalid <= 0;
            bready <= 1;
            @(posedge clk);
            while (!bvalid) @(posedge clk);
            bready <= 0;
            @(posedge clk);
        end
    endtask

    task axi_rd(input [15:0] a, output [31:0] d);
        begin
            @(posedge clk);
            araddr <= a; arvalid <= 1;
            @(posedge clk);
            while (!arready) @(posedge clk);
            arvalid <= 0;
            rready <= 1;
            @(posedge clk);
            while (!rvalid) @(posedge clk);
            d = rdata;
            rready <= 0;
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        rst = 0;
        repeat (2) @(posedge clk);

        axi_rd(16'h00, rd);
        if (rd[31:8] !== 24'h475052) begin
            $display("CHAIN: bad ID %h", rd); errors = errors + 1;
        end
        // program the chain (mode-A-like scaled values)
        axi_wr(16'h10, 32'd1);            // dwell 1: one beat per tone (avg semantics)
        axi_wr(16'h14, 32'd1);            // n_avg = 2
        axi_wr(16'h18, 32'd16384);        // agc 1.0
        axi_wr(16'h1C, 32'd18872);        // fft gain
        axi_wr(16'h20, 32'd1);            // nbg = 1
        axi_wr(16'h24, 32'd243289095);    // inv_dr_q20
        axi_wr(16'h28, 32'd0);            // roff
        axi_wr(16'h2C, 32'd10486);        // dx_q20
        axi_wr(16'h30, 32'd4519);         // dr_q20
        axi_wr(16'h34, 32'd381485);       // kbeta_q16
        axi_wr(16'h38, 32'd276);          // alpha2_q8
        axi_wr(16'h3C, {16'd256, 16'd8});  // out_scale=1.0, max_bin=8
        axi_wr(16'h44, 32'd0);            // b_min0
        axi_wr(16'h48, 32'd63);           // b_max0
        axi_wr(16'h4C, 32'd0);            // mean CFAR
        axi_wr(16'h50, 32'd57313);        // alpha_q12
        axi_wr(16'h60, 32'd283);          // dr_q16
        axi_wr(16'h64, 32'd3277);         // dx_q16
        axi_wr(16'h0C, 32'h2);            // irq enable
        axi_wr(16'h04, 32'h1);            // RUN

        // wait for the report interrupt (one frame)
        fork
            begin : wait_irq
                @(posedge irq);
                disable timeout;   // fork/join waits for BOTH branches
            end
            begin : timeout
                #40000000;
                $display("CHAIN TIMEOUT waiting for irq");
                errors = errors + 1;
                disable wait_irq;
            end
        join

        axi_rd(16'h80, rd); $display("rep1 hits    = %0d", rd);
        if (rd !== 0) begin $display("CHAIN: expected 0 hits"); errors=errors+1; end
        axi_rd(16'h84, rd); $display("rep2 nclust  = %0d", rd);
        axi_rd(16'hA0, rd); $display("cnt_adc      = %0d (want %0d)", rd, NT*2*NP);
        if (rd !== NT*2*NP) begin errors=errors+1; end
        axi_rd(16'hA4, rd); $display("cnt_avggrp   = %0d (want %0d)", rd, NP);
        if (rd !== NP) begin errors=errors+1; end
        axi_rd(16'hA8, rd); $display("cnt_rp       = %0d (want %0d)", rd, NI*NP);
        if (rd !== NI*NP) begin errors=errors+1; end
        axi_rd(16'hAC, rd); $display("cnt_bs       = %0d (want %0d)", rd, NI*NP);
        if (rd !== NI*NP) begin errors=errors+1; end
        axi_rd(16'hB0, rd); $display("cnt_mig      = %0d (want %0d)", rd, NI*NP);
        if (rd !== NI*NP) begin errors=errors+1; end
        axi_rd(16'hB4, rd); $display("cnt_det      = %0d (want %0d)", rd, NI*NP);
        if (rd !== NI*NP) begin errors=errors+1; end
        axi_rd(16'h08, rd);
        if (rd[24]) begin $display("CHAIN: bg overrun set"); errors=errors+1; end

        axi_wr(16'h04, 32'h0);            // stop
        if (errors == 0) $display("TB_CHAIN PASS");
        else             $display("TB_CHAIN FAIL (%0d)", errors);
        $finish;
    end
endmodule
