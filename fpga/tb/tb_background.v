// tb_background.v - median-across-positions engine vs exact integer golden
// (N_IFFT=64, N_POS=8)
`timescale 1ns / 1ps
module tb_background;
    localparam NI = 64, NP = 8, LB = 6;
    reg clk = 0, rst = 1;
    reg s_tvalid = 0, s_tlast = 0;
    reg [31:0] s_tdata; reg [15:0] s_tuser;
    wire m_tvalid, m_tlast, m_frame_last, frame_ready, overrun;
    wire [31:0] m_tdata; wire [15:0] m_tuser; wire [7:0] m_pos;
    wire [7:0] frame_count;

    gpr_background #(.N_IFFT(NI), .N_POS(NP), .LB(LB)) dut (
        .clk(clk), .rst(rst), .nbg(4'd1),
        .s_tvalid(s_tvalid), .s_tdata(s_tdata), .s_tuser(s_tuser),
        .s_tlast(s_tlast),
        .m_tvalid(m_tvalid), .m_tready(1'b1), .m_tdata(m_tdata),
        .m_tuser(m_tuser), .m_tuser_pos(m_pos), .m_tlast(m_tlast),
        .m_frame_last(m_frame_last), .frame_ready(frame_ready),
        .frame_count(frame_count), .overrun(overrun));

    always #2 clk = ~clk;

    reg [31:0] inv [0:NI*NP-1];    // column-major: pos j outer, bin i inner
    reg [31:0] gold [0:NI*NP-1];
    integer i, j, idx, errors = 0, got = 0;
    reg signed [15:0] gr, gi, rr, ri;

    initial begin
        $readmemh("bg_in.hex", inv);
        $readmemh("bg_gold.hex", gold);
        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);
        idx = 0;
        for (j = 0; j < NP; j = j + 1) begin
            for (i = 0; i < NI; i = i + 1) begin
                s_tvalid <= 1;
                s_tdata  <= inv[idx];
                s_tuser  <= i;
                s_tlast  <= (i == NI-1);
                idx = idx + 1;
                @(posedge clk);
            end
        end
        s_tvalid <= 0; s_tlast <= 0;
    end

    always @(posedge clk) begin
        if (!rst && m_tvalid) begin
            gr = $signed(gold[got][31:16]); gi = $signed(gold[got][15:0]);
            rr = $signed(m_tdata[31:16]); ri = $signed(m_tdata[15:0]);
            if (rr !== gr || ri !== gi) begin
                if (errors < 5)
                    $display("BG mismatch @%0d (bin %0d pos %0d): got (%0d,%0d) want (%0d,%0d)",
                             got, m_tuser, m_pos, rr, ri, gr, gi);
                errors = errors + 1;
            end
            got = got + 1;
        end
    end

    initial begin
        #2000000;
        if (got != NI*NP) begin
            $display("BG fail: emitted %0d of %0d", got, NI*NP);
            errors = errors + 1;
        end
        if (overrun) begin
            $display("BG fail: overrun set"); errors = errors + 1;
        end
        if (errors == 0) $display("TB_BACKGROUND PASS");
        else             $display("TB_BACKGROUND FAIL (%0d)", errors);
        $finish;
    end
endmodule
