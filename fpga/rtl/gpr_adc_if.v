// ---------------------------------------------------------------------------
// gpr_adc_if.v - B07_ADC digital side: AGC scale + saturation of the RFDC
// ADC stream.  On the ZCU208 the physical conversion, decimation and DDC
// happen inside the RFDC hard IP; this block applies the digital gain that
// mirrors the auto-ranging assumption of the MATLAB B07 model and clamps to
// Q15.  (The ENOB quantisation-noise model of B07 is a simulation artefact
// and lives in the testbench, not in silicon.)
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_adc_if (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] agc_gain_q14,
    // slave: from RFDC ADC AXIS
    input  wire        s_tvalid,
    output wire        s_tready,
    input  wire [31:0] s_tdata,          // {I,Q} int16
    input  wire        s_tlast,
    input  wire [7:0]  s_tuser_tone,
    input  wire [7:0]  s_tuser_pos,
    // master: to averaging
    output reg         m_tvalid,
    input  wire        m_tready,
    output reg  [31:0] m_tdata,
    output reg         m_tlast,
    output reg  [7:0]  m_tuser_tone,
    output reg  [7:0]  m_tuser_pos
);
    assign s_tready = m_tready || !m_tvalid;

    function signed [15:0] sat16(input signed [31:0] v);
        begin
            if      (v >  32'sd32767) sat16 =  16'sd32767;
            else if (v < -32'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    wire signed [15:0] in_i = $signed(s_tdata[31:16]);
    wire signed [15:0] in_q = $signed(s_tdata[15:0]);
    wire signed [15:0] g    = $signed(agc_gain_q14);   // <= 2^14, positive
    wire signed [31:0] oi   = (in_i * g + 32'sd8192) >>> 14;
    wire signed [31:0] oq   = (in_q * g + 32'sd8192) >>> 14;

    always @(posedge clk) begin
        if (rst) begin
            m_tvalid <= 0; m_tdata <= 0; m_tlast <= 0;
            m_tuser_tone <= 0; m_tuser_pos <= 0;
        end else if (s_tready) begin
            m_tvalid     <= s_tvalid;
            if (s_tvalid) begin
                m_tdata      <= {sat16(oi), sat16(oq)};
                m_tlast      <= s_tlast;
                m_tuser_tone <= s_tuser_tone;
                m_tuser_pos  <= s_tuser_pos;
            end
        end
    end
endmodule
