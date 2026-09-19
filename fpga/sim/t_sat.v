module t_sat;
    reg signed [17:0] d0;
    reg signed [39:0] f;
    function signed [15:0] sat16s(input signed [39:0] v);
        begin
            if      (v >  40'sd32767) sat16s =  16'sd32767;
            else if (v < -40'sd32768) sat16s = -16'sd32768;
            else                      sat16s = v[15:0];
        end
    endfunction
    wire signed [39:0] valx = {{22{d0[17]}}, d0} + (f >>> 16);
    initial begin
        d0 = -18'sd836;
        f  = -40'sd80132;
        #1;
        $display("direct expr      : %0d (want -838)", sat16s({{22{d0[17]}}, d0} + (f >>> 16)));
        $display("via wire         : %0d (want -838)", sat16s(valx));
        $display("signed(concat)   : %0d (want -838)", sat16s($signed({{22{d0[17]}}, d0}) + (f >>> 16)));
        $display("valx bits        : %h", valx);
    end
endmodule
