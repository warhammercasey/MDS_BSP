// Sources for the uart_controller sim. Paths are relative to the repo root.
--top-module tb_uart_controller

src/interfaces/axis_if.sv
src/interfaces/apb_if.sv
src/cdc/ff_sync.sv
src/uart/uart_rx.sv
src/uart/uart_tx.sv
src/uart/uart.sv
src/infrastructure/axis_switch.sv
src/infrastructure/apb_splitter.sv
src/uart/uart_controller.sv

sim/models/apb_reg_model.sv
sim/tests/uart_controller/tb_uart_controller.sv
