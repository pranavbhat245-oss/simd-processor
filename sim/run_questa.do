# ---------------------------------------------------------------------------
# Questa run script. Run from the sim/ folder:
#     vsim -c -do run_questa.do          (batch)
#     vsim    -do run_questa.do          (GUI: set GUI 1 below)
# Edit the variables below to pick the test, size and backpressure mode.
# Tests: simd_smoke_test simd_full_test simd_mask_test simd_random_test
#        simd_backpressure_test simd_reset_test simd_exhaustive_test (slow)
# ---------------------------------------------------------------------------
set TEST        simd_full_test
set N_TXN       5000
set READY_MODE  0
set GUI         0

quit -sim
if {[file exists work]} { vdel -all -lib work }
vlib work
vmap work work

# Compile order matters: RTL, reference model, interface, package, top.
vlog ../rtl/simd_processor.v
vlog -sv -L mtiUvm +incdir+../tb/uvm ../tb/uvm/simd_ref_pkg.sv ../tb/uvm/simd_if.sv ../tb/uvm/simd_pkg.sv ../tb/uvm/tb_top.sv
# vlog +define+SIMD_NO_SVA ...   -> compile out the interface assertions

vsim -voptargs=+acc -L mtiUvm work.tb_top \
     +UVM_TESTNAME=$TEST +N_TXN=$N_TXN +READY_MODE=$READY_MODE +UVM_VERBOSITY=UVM_LOW

if {$GUI} {
  add wave -r /tb_top/sif/*
}
run -all
if {!$GUI} { quit -f }
