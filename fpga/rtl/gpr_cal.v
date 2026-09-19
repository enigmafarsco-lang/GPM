// ---------------------------------------------------------------------------
// gpr_cal.v - B09_Calibration: divide out the system response per tone.
// cal_rom[k] = 1/H_sys(f_k) baked by make_golden.m from code_calibration
// (cable delay phase + passband ripple), Q14 complex.  One sample per
// accepted beat, 1-cycle latency.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps
module gpr_cal #(
    parameter N_TONES = 256,
    parameter ROM_FILE = "cal_a.hex"
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
    reg [31:0] rom [0:N_TONES-1];
    initial $readmemh(ROM_FILE, rom);

    assign s_tready = m_tready || !m_tvalid;

    function signed [15:0] sat16(input signed [33:0] v);
        begin
            if      (v >  34'sd32767) sat16 =  16'sd32767;
            else if (v < -34'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    wire signed [15:0] xr = $signed(s_tdata[31:16]);
    wire signed [15:0] xi = $signed(s_tdata[15:0]);
    wire signed [15:0] wr = $signed(rom[s_tuser_tone][31:16]);
    wire signed [15:0] wi = $signed(rom[s_tuser_tone][15:0]);

    // 33-bit intermediates: |x|<=2^15, |w|<=1.99*2^14 -> sum of two
    // products can reach 2^31
    wire signed [32:0] prr = xr*wr, pii = xi*wi, pri = xr*wi, pir = xi*wr;
    wire signed [33:0] yr = (prr - pii + 33'sd8192) >>> 14;
    wire signed [33:0] yi = (pri + pir + 33'sd8192) >>> 14;

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
