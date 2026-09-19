# =============================================================================
# vivado_build.tcl - build the GPR PL chain for the AMD RFSoC ZCU208
#                    (XCZU48DR-2FSVG1517E), Vivado 2023.2 / 2024.x
#
# Usage:
#   # quick flat synthesis: utilisation + timing reports, no BD, no bitstream
#   vivado -mode batch -source fpga/scripts/vivado_build.tcl
#
#   # full block-design build: PS + RFDC + interconnect, bitstream + XSA
#   vivado -mode batch -source fpga/scripts/vivado_build.tcl -tclargs full
#
# Prerequisites for the FULL flow:
#   * Xilinx board files for ZCU208 installed (or set board_part manually)
#   * RFSoC RF Data Converter IP licensed (included with Vivado for ZU*DR)
#   * Run from the repository root or anywhere - paths here are script-relative
# =============================================================================

set script_dir [file dirname [file normalize [info script]]]
set fpga_dir   [file normalize "$script_dir/.."]
set rtl_dir    "$fpga_dir/rtl"
set xdc_dir    "$fpga_dir/xdc"
set prj_dir    "$fpga_dir/vivado_prj"

set part        xczu48dr-2fsvg1517e
set board_part  xilinx.com:zcu208:part0:3.0   ;# adjust to installed version
set top_module  gpr_top_zcu208
set full_build  [expr {[llength $argv] > 0 && [lindex $argv 0] eq "full"}]
file mkdir "$fpga_dir/reports"

# ----------------------------------------------------------------------------
# project
# ----------------------------------------------------------------------------
create_project gpr_zcu208 $prj_dir -part $part -force
if {[catch {set_property board_part $board_part [current_project]} err]} {
    puts "WARNING: ZCU208 board part not available ($err)."
    puts "         Continuing part-only; the FULL BD flow needs board files."
}

# RTL sources (golden ROM hex files live in fpga/golden and fpga/sim; they are
# simulation-only initialisation - synthesis initialises the ROMs from the
# PS-side or leaves them zero, see fpga/README.md section 'ROM loading')
add_files -norecurse [glob $rtl_dir/*.v]
add_files -fileset constrs_1 -norecurse $xdc_dir/zcu208.xdc
set_property top $top_module [current_fileset]
update_compile_order -fileset sources_1

if {!$full_build} {
    # ------------------------------------------------------------------------
    # SYNTH-ONLY: flat elaboration of gpr_top_zcu208 for resource/timing
    # ------------------------------------------------------------------------
    puts "=== SYNTH_ONLY: synthesising $top_module flat ==="
    synth_design -top $top_module -part $part -mode out_of_context
    opt_design
    report_utilization          -file "$fpga_dir/reports/util_synth.rpt"
    report_timing_summary       -file "$fpga_dir/reports/timing_synth.rpt"
    report_methodology          -file "$fpga_dir/reports/methodology_synth.rpt"
    write_checkpoint -force     "$fpga_dir/reports/synth_only.dcp"
    puts "=== reports in fpga/reports/ ==="
    exit
}

# ============================================================================
# FULL BUILD: block design with Zynq UltraScale+ MPSoC, RF Data Converter,
# clocking, AXI interconnect and the GPR chain as a module reference.
# ============================================================================
create_bd_design gpr_bd

# ---- Zynq MPSoC (PS) --------------------------------------------------------
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e ps_e]
catch {apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
           -config {apply_board_preset "1"} $ps}
# minimal fallback config if the board preset is missing
catch {
    set_property -dict [list \
        CONFIG.PSU__USE__M_AXI_GP0 {1} \
        CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
        CONFIG.PSU__GPIO0_MIO__PERIPHERAL__ENABLE {1} \
    ] $ps
}

# ---- PL fabric clock: 245.76 MHz -------------------------------------------
# ZCU208 note: exact 245.76 MHz cannot be derived from pl_clk0 (100 MHz) with
# an integer MMCM ratio.  Production boards feed the PL from the on-board
# SI570 (PS-programmable over I2C, e.g. via the Linux si570 driver) into a PL
# differential clock pin - replace clk_wiz's input accordingly and add the pin
# to zcu208.xdc.  For bring-up the clk_wiz below locks to the closest
# achievable rate (245.833 MHz, 0.03 % off); regenerate golden ROMs
# (make_golden.m, F_CLK) if you keep a non-nominal rate.
set cwin [create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wizard pl_clk_wiz]
set_property -dict [list \
    CONFIG.PRIM_IN_FREQ.VALUE_SRC USER \
    CONFIG.PRIM_SOURCE {Differential_clock_capable_pin} \
    CONFIG.USE_RESET {false} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {245.76} \
] $cwin
catch {
    # board bring-up alternative: drive clk_wiz from pl_clk0 single-ended
    set_property -dict [list CONFIG.PRIM_SOURCE {Single_ended_clock_capable_pin}] $cwin
}

# ---- RF Data Converter (RFDC) ----------------------------------------------
# GPR mode A: 0.5-3.0 GHz swept in 21 sub-bands (see README 'Sub-band plan').
# ADC tile 224 converter 0, fs = 4.9152 GSPS (LMK04828 refclk 122.88 MHz),
# fine-mixer NCO per sub-band (PS retunes on sb_req/sb_ack), 4x decimation,
# real 2GSPS->IQ.  DAC tile 224 converter 0, fs = 4.9152 GSPS, 4x
# interpolation, fine NCO per sub-band.  Property names vary between IP
# versions - each is applied under catch and reported.
set rfdc [create_bd_cell -type ip -vlnv xilinx.com:ip:usmp_rf_data_converter rfdc]
foreach {prop val} {
    CONFIG.ADC0_Enable              {1}
    CONFIG.ADC0_Sampling_Rate     {4.9152}
    CONFIG.ADC0_Refclk_Freq       {0.12288}
    CONFIG.ADC0_Fabric_Freq       {245.76}
    CONFIG.ADC0_Decimation_Mode   {4X}
    CONFIG.ADC0_Mixer_Mode        {Fine}
    CONFIG.ADC0_Data_Width        {4}
    CONFIG.DAC0_Enable            {1}
    CONFIG.DAC0_Sampling_Rate     {4.9152}
    CONFIG.DAC0_Refclk_Freq       {0.12288}
    CONFIG.DAC0_Fabric_Freq       {245.76}
    CONFIG.DAC0_Interpolation_Mode {4X}
    CONFIG.DAC0_Mixer_Mode        {Fine}
} {
    if {[catch {set_property $prop $val $rfdc} err]} {
        puts "WARNING: RFDC $prop not applied ($err) - configure in GUI."
    }
}

# ---- GPR chain (module reference) ------------------------------------------
set gpr [create_bd_cell -type module -reference $top_module gpr]

# ---- AXI-Lite interconnect: PS M_AXI_HPM0_FPD -> gpr ------------------------
set smc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect axil_smc]
set_property CONFIG.NUM_SI {1} CONFIG.NUM_MI {1} $smc
connect_bd_net [get_bd_pins ps_e/pl_clk0]    [get_bd_pins smc/aclk]
catch {connect_bd_net [get_bd_pins ps_e/pl_clk0] [get_bd_pins ps_e/maxihpm0_fpd_axi_aclk]}
connect_bd_intf_net [get_bd_intf_pins ps_e/M_AXI_HPM0_FPD] [get_bd_intf_pins smc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins smc/M00_AXI]         [get_bd_intf_pins gpr/s_axil]

# ---- clocks / resets ---------------------------------------------------------
set proc_rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst_245]
connect_bd_net [get_bd_pins cwin/clk_out1] [get_bd_pins gpr/clk]
connect_bd_net [get_bd_pins cwin/clk_out1] [get_bd_pins rst_245/slowest_sync_clk]
connect_bd_net [get_bd_pins ps_e/pl_resetn0] [get_bd_pins rst_245/ext_reset_in]
connect_bd_net [get_bd_pins rst_245/peripheral_reset] [get_bd_pins gpr/rst]
connect_bd_net [get_bd_pins cwin/clk_out1] [get_bd_pins rfdc/m0_axis_aclk]
catch {connect_bd_net [get_bd_pins cwin/clk_out1] [get_bd_pins rfdc/s_axis_dac_aclk]}

# ---- RFDC <-> GPR data (ADC tile 224 slice 0 -> gpr, gpr -> DAC) -------------
# Interface names depend on the enabled converters; connect the first ADC/DAC
# AXIS pairs.  Adjust slice indices if you enable more converters.
catch {connect_bd_intf_net [get_bd_intf_pins rfdc/m00_axis_adc0] \
                            [get_bd_intf_pins gpr/s_axis_adc]}
catch {connect_bd_intf_net [get_bd_intf_pins gpr/m_axis_dac] \
                            [get_bd_intf_pins rfdc/s00_axis_dac0]}

# ---- sideband retune handshake + irq to PS (EMIO GPIO) ----------------------
# sb_req/sb_ack and irq connect to PS EMIO GPIO (2 out, 1 in).  Enable
# PSU__GPIO1_EMIO in the PS configuration and map the nets; left as external
# ports here so the design validates without EMIO pin budgeting:
create_bd_port -dir O sb_req
create_bd_port -dir I sb_ack
create_bd_port -dir O irq
connect_bd_net [get_bd_pins gpr/sb_req] [get_bd_ports sb_req]
connect_bd_net [get_bd_ports sb_ack]    [get_bd_pins gpr/sb_ack]
connect_bd_net [get_bd_pins gpr/irq]    [get_bd_ports irq]

# loopback = 0: real RFDC data path (loopback=1 is the fabric self-test mode
# used by tb_chain and can be exposed as an external port for lab bring-up)
set tie0 [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant tie_gnd]
set_property CONFIG.CONST_VAL {0} $tie0
connect_bd_net [get_bd_pins tie0/dout] [get_bd_pins gpr/loopback]

# ---- validate, wrap, build ----------------------------------------------------
regenerate_bd_layout
if {[catch {validate_bd_design} err]} {
    puts "WARNING: validate_bd_design reported issues: $err"
    puts "         Fix RFDC/PS configuration in the GUI, then re-run."
}
save_bd_design
make_wrapper -files [get_files gpr_bd.bd] -top
add_files -norecurse [glob -nocomplain $prj_dir/gpr_zcu208.srcs/sources_1/bd/gpr_bd/hdl/*.v]
set_property top gpr_bd_wrapper [current_fileset]
update_compile_order -fileset sources_1

launch_runs synth_1 -jobs 8
wait_on_run synth_1
open_run synth_1 -name synth_1
report_utilization    -file "$fpga_dir/reports/util_bd.rpt"
report_timing_summary -file "$fpga_dir/reports/timing_bd.rpt"

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    error "implementation failed - see $prj_dir runs"
}
write_hw_platform -fixed -include_bit -force "$fpga_dir/gpr_zcu208.xsa"
puts "=== bitstream + XSA written: fpga/gpr_zcu208.xsa ==="
exit
