// Sources for the uart sim. Paths are relative to the repo root.
// Any Verilator option is allowed here; --top-module is required.
--top-module tb_uart

src/interfaces/axis_if.sv
src/uart/uart_rx.sv
src/uart/uart_tx.sv
src/uart/uart.sv

sim/tests/uart/tb_uart.sv
