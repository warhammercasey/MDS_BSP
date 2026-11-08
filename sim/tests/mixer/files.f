// Sources for the mixer sim. Paths are relative to the repo root.
// The LO table is read by path at run time (see LUT_FILE in the testbench).
--top-module tb_mixer

src/interfaces/axis_if.sv
src/ram/rom.sv
src/dsp/building_blocks/mixer.sv

sim/tests/mixer/tb_mixer.sv
