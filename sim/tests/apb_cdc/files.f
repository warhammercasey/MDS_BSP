// Sources for the apb_cdc sim. Paths are relative to the repo root.
--top-module tb_apb_cdc

src/interfaces/apb_if.sv
src/cdc/ff_sync.sv
src/cdc/apb_cdc.sv

sim/common/cain_clocks_pkg.sv
sim/models/clk_gen.sv
sim/models/ff_sync_checker.sv
sim/models/apb_reg_model.sv
sim/tests/apb_cdc/tb_apb_cdc.sv
