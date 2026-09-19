# =============================================================================
# zcu208.xdc - timing & board constraints for the GPR PL chain on the
#              AMD RFSoC ZCU208 (XCZU48DR-2FSVG1517E)
#
# The gpr_top_zcu208 design is SINGLE-CLOCK: every block (DDS, ADC IF, avg,
# cal, window, FFT, background, migrate, CFAR, cluster, AXI-Lite regs) runs
# from one fabric clock, nominally 245.76 MHz (F_CLK).  All golden fixed-point
# constants (DDS frequency words, dwell counters, Q-format scales) are baked
# for F_CLK = 245.76 MHz - do not retarget the frequency without regenerating
# golden/make_golden.m ROMs.
#
# In the block-design flow (scripts/vivado_build.tcl) the clock is produced by
# the Clocking Wizard / RFDC fabric clock IP and its create_clock is generated
# automatically by the BD; the create_clock below is only applied when
# gpr_top_zcu208 is synthesized flat (see guard).
# =============================================================================

# -----------------------------------------------------------------------------
# 1. Primary fabric clock - 245.76 MHz (period 4.0690 ns)
#    FLAT FLOW ONLY: uncomment and set the actual ZCU208 clock input pin from
#    the board schematic (SI570 user-programmable differential pair; the PS
#    Si570 driver programs it to 245.76 MHz at boot).  In the BD flow this
#    section must stay disabled - Vivado creates the clock on the IP output.
# -----------------------------------------------------------------------------
# create_clock -name gpr_fclk -period 4.0690 [get_ports pl_clk_p]

# For flat synthesis without a real pin (e.g. utilisation/timing estimation via
# vivado_build.tcl SYNTH_ONLY mode), constrain the top-level clock port.
# Guarded so the same file can stay attached in the BD flow (wrapper has no
# 'clk' port - the BD-generated clock on the clk_wiz output rules there).
if {[llength [get_ports -quiet clk]] > 0} {
    create_clock -name gpr_fclk -period 4.0690 [get_ports clk]
}


# -----------------------------------------------------------------------------
# 2. Asynchronous / quasi-static controls
#    rst is driven by the PS (EMIO or BD processor reset) and is synchronised
#    inside gpr_top_zcu208 usage (registered by every block's reset branch).
#    Treat it as a false path source; the design resets synchronously.
# -----------------------------------------------------------------------------
set_false_path -from [get_ports rst]
set_false_path -from [get_ports loopback]
set_false_path -from [get_ports sb_ack]
set_false_path -to   [get_ports irq]
set_false_path -to   [get_ports sb_req]

# -----------------------------------------------------------------------------
# 3. RFDC AXI4-Stream interfaces
#    In the BD flow the RFDC m_axis_adc*/s_axis_dac* interfaces are source-
#    synchronous to the RFDC fabric clock (same 245.76 MHz domain via the
#    shared clk_wiz).  Vivado's BD timing rules cover them; if the RFDC data
#    clock is ever a different generator, insert an AXIS clock converter and
#    add:
#      set_clock_groups -asynchronous -group gpr_fclk -group <rfdc_clk>
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# 4. AXI4-Lite slave from PS M_AXI_HPM0_FPD via SmartConnect
#    Same clock domain in the BD flow (pl_clk0 -> clk_wiz -> gpr clk).  The
#    interconnect IP constrains itself; nothing to add here.
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# 5. Bitstream / board properties (ZCU208)
# -----------------------------------------------------------------------------
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 85.0 [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]
set_property CFGBVS VCCO [current_design]
