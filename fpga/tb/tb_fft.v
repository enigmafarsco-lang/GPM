// tb_fft.v - 2048-point windowed zero-padded IFFT vs Octave/MATLAB golden
// (fft_in.hex -> fft_gold.hex, tolerance: per-stage rounding + saturation)
`timescale 1ns / 1ps
module tb_fft;
    localparam N = 2048;
    reg clk = 0, rst = 1;
    reg load_begin = 0, s_tvalid = 0, start_fft = 0;
    reg [31:0] s_tdata; reg [7:0] s_tuser;
    wire s_tready, m_tvalid, m_tlast, busy, done;
    wire [31:0] m_tdata; wire [15:0] m_tuser;

    gpr_fft #(.N(N), .LOG2N(11)) dut (
        .clk(clk), .rst(rst), .inv(1'b1), .gain_q10(16'd18872),
        .load_begin(load_begin),
        .s_tvalid(s_tvalid), .s_tready(s_tready), .s_tdata(s_tdata),
        .s_tuser_tone(s_tuser), .start_fft(start_fft),
        .m_tvalid(m_tvalid), .m_tready(1'b1), .m_tdata(m_tdata),
        .m_tuser(m_tuser), .m_tlast(m_tlast), .busy(busy), .done(done));

    always #2 clk = ~clk;

    reg [31:0] inv [0:255];
    reg [31:0] gold [0:N-1];
    integer i, errors = 0;
    integer e_re, e_im, maxe = 0;
    real sre = 0, sg = 0, sgold = 0;
    real corr;
    reg signed [15:0] gr, gi, rr, ri;

    initial begin
        $readmemh("fft_in.hex", inv);
        $readmemh("fft_gold.hex", gold);
        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);
        load_begin <= 1;
        @(posedge clk);
        load_begin <= 0;
        // wait until the engine accepts loads
        @(posedge clk);
        while (!s_tready) @(posedge clk);
        for (i = 0; i < 256; i = i + 1) begin
            s_tvalid <= 1;
            s_tdata  <= inv[i];
            s_tuser  <= i[7:0];
            @(posedge clk);
        end
        s_tvalid <= 0;
        repeat (4) @(posedge clk);
        start_fft <= 1;
        @(posedge clk);
        start_fft <= 0;

        // collect outputs
        i = 0;
        while (i < N) begin
            @(posedge clk);
            if (m_tvalid) begin
                gr = $signed(gold[i][31:16]); gi = $signed(gold[i][15:0]);
                rr = $signed(m_tdata[31:16]); ri = $signed(m_tdata[15:0]);
                e_re = rr - gr; e_im = ri - gi;
                if (e_re < 0) e_re = -e_re;
                if (e_im < 0) e_im = -e_im;
                if (e_re > maxe) maxe = e_re;
                if (e_im > maxe) maxe = e_im;
                sre  = sre + $itor(rr)*$itor(gr) + $itor(ri)*$itor(gi);
                sg   = sg  + $itor(rr)*$itor(rr) + $itor(ri)*$itor(ri);
                sgold= sgold + $itor(gr)*$itor(gr) + $itor(gi)*$itor(gi);
                i = i + 1;
            end
        end
        corr = sre / ($sqrt(sg) * $sqrt(sgold));
        $display("FFT: max |err| = %0d LSB, correlation = %.6f", maxe, corr);
        if (maxe > 96) begin   // 11 stages of Q15 rounding, <0.3% FS
            $display("FFT max error too large"); errors = errors + 1;
        end
        if (corr < 0.9999) begin
            $display("FFT correlation too low"); errors = errors + 1;
        end
        if (errors == 0) $display("TB_FFT PASS");
        else             $display("TB_FFT FAIL (%0d)", errors);
        $finish;
    end
    initial begin #20000000; $display("TB_FFT TIMEOUT"); $finish; end
endmodule
