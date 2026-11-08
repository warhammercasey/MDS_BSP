// Sources for the direction_estimator sim. Paths are relative to the repo root.
// The coefficient files are read by path at run time (see tb_direction_estimator). The
// tetrahedron one is made with:
//   python src/dsp/building_blocks/gen_direction_matrix.py --coef-frac 21
//     --locations "0.08,0.08,0.08; 0.08,-0.08,-0.08; -0.08,0.08,-0.08; -0.08,-0.08,0.08"
//     --out sim/tests/direction_estimator/tetra_matrix.hex
--top-module tb_direction_estimator

src/interfaces/axis_if.sv
src/ram/rom.sv
src/dsp/building_blocks/direction_estimator.sv

sim/tests/direction_estimator/tb_direction_estimator.sv
