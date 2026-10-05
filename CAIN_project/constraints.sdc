create_clock -name clk_core -period 5.0 [get_ports clk_core]
create_clock -name clk_sram -period 10.0 [get_ports clk_sram]
create_clock -name clk_50 -period 20.0 [get_ports clk_50]
create_clock -name clk_io -period 13.592238288247879 [get_ports clk_io]