// Sources for the fir_decimator sim. Paths are relative to the repo root.
// The tap files are read by path at run time (see TEST_DIR in the testbench).
--top-module tb_fir_decimator

src/interfaces/axis_if.sv
src/cdc/simple_dual_port_bram.sv
src/ram/rom.sv
src/dsp/building_blocks/fir_decimator.sv

sim/tests/fir_decimator/tb_fir_decimator.sv
