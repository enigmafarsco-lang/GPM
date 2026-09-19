// ---------------------------------------------------------------------------
// gpr_window.v - B10 first half: Kaiser window (beta = 8) across the tone
// axis, baked into win.hex by make_golden.m (parsed straight out of the
// code_rangeproc script).  Real Q15 multiply per component, 1-cycle
// latency, one sample per accepted beat.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_window #(
    parameter N_TONES  = 256,
    parameter ROM_FILE = "win.hex"
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        s_tvalid,
    output wire        s_tready,
    input  wire [31:0] s_tdata,
    input  wire        s_tlast,
    input  wire [7:0]  s_tuser_tone,
    input  wire [7:0]  s_tuser_pos,
    output reg         m_tvalid,
    input  wire        m_tready,
    output reg  [31:0] m_tdata,
    output reg         m_tlast,
    output reg  [7:0]  m_tuser_tone,
    output reg  [7:0]  m_tuser_pos
);
    reg [15:0] rom [0:N_TONES-1];
    initial $readmemh(ROM_FILE, rom);

    assign s_tready = m_tready || !m_tvalid;

    function signed [15:0] sat16(input signed [31:0] v);
        begin
            if      (v >  32'sd32767) sat16 =  16'sd32767;
            else if (v < -32'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    wire signed [15:0] xr = $signed(s_tdata[31:16]);
    wire signed [15:0] xi = $signed(s_tdata[15:0]);
    wire signed [15:0] w  = $signed({1'b0, rom[s_tuser_tone]});
    wire signed [31:0] yr = (xr*w + 32'sd16384) >>> 15;
    wire signed [31:0] yi = (xi*w + 32'sd16384) >>> 15;

    always @(posedge clk) begin
        if (rst) begin
            m_tvalid <= 0; m_tdata <= 0; m_tlast <= 0;
            m_tuser_tone <= 0; m_tuser_pos <= 0;
        end else if (s_tready) begin
            m_tvalid <= s_tvalid;
            if (s_tvalid) begin
                m_tdata      <= {sat16(yr), sat16(yi)};
                m_tlast      <= s_tlast;
                m_tuser_tone <= s_tuser_tone;
                m_tuser_pos  <= s_tuser_pos;
            end
        end
    end
endmodule
