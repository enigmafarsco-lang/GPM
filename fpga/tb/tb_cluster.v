// tb_cluster.v - connected components + report vs exact golden
// (uses the CFAR test patch: pass 1 = det bits, pass 2 = magnitudes)
`timescale 1ns / 1ps
module tb_cluster;
    localparam NI = 64, NP = 64, LB = 6;
    reg clk = 0, rst = 1, go = 0;
    reg s_tvalid = 0, s_tdet = 0, s_pass2 = 0, s_frame_last = 0;
    reg [15:0] s_tdata, s_tuser; reg [7:0] s_tpos;
    wire s_tready, rep_done, busy;
    wire [31:0] rep1, rep2, rep3, rep4, rep5, rep6;

    gpr_cluster #(.N_IFFT(NI), .N_POS(NP), .LB(LB)) dut (
        .clk(clk), .rst(rst),
        .s_tvalid(s_tvalid), .s_tready(s_tready), .s_tdata(s_tdata),
        .s_tdet(s_tdet), .s_pass2(s_pass2), .s_tuser(s_tuser),
        .s_tuser_pos(s_tpos), .s_frame_last(s_frame_last), .go(go),
        .dr_q16(32'd283), .dx_q16(32'd3277),
        .rep1(rep1), .rep2(rep2), .rep3(rep3), .rep4(rep4),
        .rep5(rep5), .rep6(rep6), .rep_done(rep_done), .busy(busy));

    always #2 clk = ~clk;

    reg [15:0] img [0:NI*NP-1];
    integer det [0:NI*NP-1];
    integer gold [0:5];
    integer i, idx, errors = 0, gfd;
    reg [63:0] line;

    task feed_beat(input [15:0] d, input dt, input p2, input [15:0] iz,
                   input [7:0] ix, input fl);
        begin
            s_tvalid <= 1; s_tdata <= d; s_tdet <= dt; s_pass2 <= p2;
            s_tuser <= iz; s_tpos <= ix; s_frame_last <= fl;
            @(posedge clk);
            while (!s_tready) @(posedge clk);
        end
    endtask

    initial begin
        $readmemh("cfar_in.hex", img);
        gfd = $fopen("cfar_gold.hex", "r");
        idx = 0;
        while (!$feof(gfd) && idx < NI*NP) begin
            if ($fscanf(gfd, "%d\n", line) == 1) begin
                det[idx] = line; idx = idx + 1;
            end
        end
        $fclose(gfd);
        gfd = $fopen("cluster_gold.hex", "r");
        idx = 0;
        while (!$feof(gfd) && idx < 6) begin
            if ($fscanf(gfd, "%d\n", line) == 1) begin
                gold[idx] = line; idx = idx + 1;
            end
        end
        $fclose(gfd);

        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);
        go <= 1;

        // pass 1
        for (i = 0; i < NI*NP; i = i + 1)
            feed_beat(img[i], det[i][0], 1'b0, i / NP, i % NP,
                      (i == NI*NP-1));
        // pass 2
        for (i = 0; i < NI*NP; i = i + 1)
            feed_beat(img[i], 1'b0, 1'b1, i / NP, i % NP,
                      (i == NI*NP-1));
        s_tvalid <= 0;

        @(posedge rep_done);
        @(posedge clk);
        $display("CLUSTER rep = [%0d %0d %0d %0d %0d %0d]",
                 rep1, rep2, rep3, rep4, rep5, rep6);
        $display("CLUSTER gold= [%0d %0d %0d %0d %0d %0d]",
                 gold[0], gold[1], gold[2], gold[3], gold[4], gold[5]);
        if (rep1 !== gold[0]) begin $display("rep1 mismatch"); errors=errors+1; end
        if (rep2 !== gold[1]) begin $display("rep2 mismatch"); errors=errors+1; end
        if (rep3 !== gold[2]) begin $display("rep3 mismatch"); errors=errors+1; end
        if (rep4 !== gold[3]) begin $display("rep4 mismatch"); errors=errors+1; end
        if (rep5 !== gold[4]) begin $display("rep5 mismatch"); errors=errors+1; end
        if (rep6 !== gold[5]) begin $display("rep6 mismatch"); errors=errors+1; end
        if (errors == 0) $display("TB_CLUSTER PASS");
        else             $display("TB_CLUSTER FAIL (%0d)", errors);
        $finish;
    end
    initial begin #200000000; $display("TB_CLUSTER TIMEOUT"); $finish; end
endmodule
