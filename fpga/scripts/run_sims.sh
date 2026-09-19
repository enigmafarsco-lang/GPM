#!/bin/bash
# Run every RTL testbench with Icarus Verilog.  Golden files must already be
# in fpga/sim (run_sims.sh copies them if missing).
set -u
cd "$(dirname "$0")/../sim"
cp ../golden/*.hex ../golden/*.vh . 2>/dev/null
RTL=../rtl
fail=0
run_tb () {
    name="$1"; shift
    echo "=== $name ==="
    iverilog -g2005 -I$RTL -o "$name.vvp" $RTL/*.v ../tb/"$name".v "$@" 2>"$name.compile.log" || {
        echo "COMPILE FAIL ($name)"; head -20 "$name.compile.log"; fail=1; return; }
    timeout 900 vvp "$name.vvp" | tail -6
    if ! timeout 5 true; then :; fi
}
for tb in tb_cordic tb_div tb_dds tb_fft tb_background tb_cfar tb_cluster tb_migrate tb_chain; do
    run_tb "$tb"
done
echo "================================"
[ $fail -eq 0 ] && echo "all tbs compiled"
