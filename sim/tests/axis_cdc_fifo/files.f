// Sources for the axis_cdc_fifo sim. Paths are relative to the repo root.
--top-module tb_axis_cdc_fifo

src/interfaces/axis_if.sv
src/cdc/ff_sync.sv
src/cdc/two_port_bram.sv
src/cdc/axis_cdc_fifo.sv
src/cdc/axis_tid_cdc_fifo.sv

sim/common/cain_clocks_pkg.sv
sim/models/clk_gen.sv
sim/models/ff_sync_checker.sv
sim/tests/axis_cdc_fifo/axis_cdc_fifo_pair.sv
sim/tests/axis_cdc_fifo/tb_axis_cdc_fifo.sv
