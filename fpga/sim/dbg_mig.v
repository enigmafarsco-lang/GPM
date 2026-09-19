`timescale 1ns / 1ps
module dbg_mig;
    localparam NI = 64, NP = 16, LB = 6, MB = 12;
    reg clk = 0, rst = 1, go = 0;
    reg s_tvalid = 0, s_tlast = 0, s_frame_last = 0;
    reg [31:0] s_tdata; reg [15:0] s_tuser; reg [7:0] s_tpos;
    wire s_tready, m_tvalid, m_tlast, m_frame_last, busy;
    wire [15:0] m_tdata, m_tuser; wire [7:0] m_tpos;
    gpr_migrate #(.N_IFFT(NI), .N_POS(NP), .LB(LB), .NAP_MAX(8),
                  .NAP_FILE("mig_nap.hex")) dut (
        .clk(clk), .rst(rst),
        .s_tvalid(s_tvalid), .s_tready(s_tready), .s_tdata(s_tdata),
        .s_tuser(s_tuser), .s_tuser_pos(s_tpos), .s_tlast(s_tlast),
        .s_frame_last(s_frame_last),
        .inv_dr_q20(32'd243289095), .roff_q20(32'd0),
        .dx_q20(32'd10486), .dr_q20(32'd4519),
        .kbeta_q16(32'd381485), .alpha2_q8(16'sd276),
        .max_bin(MB), .out_scale_q8(16'd256), .go(go),
        .m_tvalid(m_tvalid), .m_tready(1'b1), .m_tdata(m_tdata),
        .m_tuser(m_tuser), .m_tuser_pos(m_tpos), .m_tlast(m_tlast),
        .m_frame_last(m_frame_last), .busy(busy));
    always #2 clk = ~clk;
    reg [31:0] inv [0:NI*NP-1];
    integer i, j, idx, emitted = 0, lastst = -1;
    initial begin
        $readmemh("mig_in.hex", inv);
        repeat (3) @(posedge clk); rst = 0;
        @(posedge clk); go <= 1;
        idx = 0;
        for (j = 0; j < NP; j = j + 1)
            for (i = 0; i < NI; i = i + 1) begin
                s_tvalid <= 1; s_tdata <= inv[idx]; s_tuser <= i; s_tpos <= j;
                s_tlast <= (i == NI-1);
                s_frame_last <= (j == NP-1 && i == NI-1);
                @(posedge clk);
                while (!s_tready) @(posedge clk);
                idx = idx + 1;
            end
        s_tvalid <= 0;
        $display("feed done @%0t", $time);
    end
    always @(posedge clk) if (m_tvalid) emitted = emitted + 1;
    // trace pixel iz0=1, ix=0 internals
    always @(posedge clk) begin
        if (dut.iz0 == 1 && dut.ix == 0) begin
            if (dut.st == 5'd25) // M_PACC
                $display("PACC sp=%0d w=(%0d,%0d) val=(%0d,%0d) b0=%0d fr=%0d acc=(%0d,%0d) nc=%0d",
                         dut.sp, dut.w_re[dut.sp[7:0]], dut.w_im[dut.sp[7:0]],
                         dut.val_re, dut.val_im,
                         dut.b0_ram[dut.sp[7:0]], dut.fr_ram[dut.sp[7:0]],
                         dut.acc_re, dut.acc_im, dut.nc);
            if (dut.st == 5'd17) // M_PTAP
                $display("PTAP sp=%0d it_c=%0d valid=%b", dut.sp, dut.it_c, dut.tap_valid);
            if (dut.st == 5'd18) // M_PRD0
                $display("PRD0 sp=%0d addr0=%0d addr1=%0d", dut.sp, dut.rdaddr0, dut.rdaddr1);
            if (dut.st == 5'd19) // M_PRD1
                $display("PRD1 sp=%0d rdreg=%h d0=(%0d,%0d)", dut.sp, dut.rdreg, dut.rdreg[31:16], dut.rdreg[15:0]);
            if (dut.st == 5'd24) // M_PRD2
                $display("PRD2 sp=%0d rdreg2=%h d0=(%0d,%0d) diff=(%0d,%0d) fmul=(%0d,%0d) fr=%0d",
                         dut.sp, dut.rdreg2, dut.d0_re, dut.d0_im,
                         dut.diff_re, dut.diff_im, dut.fmul_re, dut.fmul_im,
                         dut.fr_ram[dut.sp[7:0]]);
            if (dut.st == 5'd21) // M_PMAGw
                $display("PMAGw c_xo=%0d nc=%0d ins=%0d outpix_next", dut.c_xo, dut.nc,
                         dut.ins_rom[dut.nc-1]);
        end
        if (dut.iz0 == 1 && dut.st == 5'd15 && dut.sp <= 2) // M_WMUL
            $display("WMUL sp=%0d wc=%0d cos=%0d sin=%0d -> w=(%0d,%0d) ratio=%0d isqr=%0d expv=%0d tapq=%0d",
                     dut.sp, dut.wc, dut.c_xo, dut.c_yo,
                     (dut.wc*dut.c_xo)>>>9, (dut.wc*dut.c_yo)>>>9,
                     dut.ratio_q15, dut.isqr_q13, dut.expv_q15, dut.tapq_q15);
    end
    initial begin
        #100000;
        $display("emitted=%0d st=%0d iz0=%0d ix=%0d", emitted, dut.st, dut.iz0, dut.ix);
        if (dut.st != lastst) lastst = dut.st;
    end
    initial begin
        #600000;
        $display("final: emitted=%0d st=%0d iz0=%0d ix=%0d", emitted, dut.st, dut.iz0, dut.ix);
        $finish;
    end
endmodule
