create_clock -name clk_core -period 5.0 [get_ports clk_core]
create_clock -name clk_sram -period 10.0 [get_ports clk_sram]
create_clock -name clk_50 -period 20.0 [get_ports clk_50]
create_clock -name clk_io -period 13.592238288247879 [get_ports clk_io]

set_clock_groups -asynchronous -group {clk_50} -group {clk_core} -group {clk_sram} -group {clk_io}


# ------------------------------------------------------------------------------
# RMII (LAN8720A, REF_CLK In mode)
# ------------------------------------------------------------------------------
# One 50 MHz oscillator clocks both the PHY and the FPGA. clk_50 is the in-phase
# PLL output (pll_inst1, internal feedback) and clocks the RMII logic. All times are
# in ns from the oscillator edge.
#
# Board (oscillator -> pin, and pin to pin):
#   clock -> FPGA 0.030, clock -> PHY 0.173
#   CRS_DV 0.169, RXER 0.157, RXD0 0.167, RXD1 0.160
#   TXD0 0.224, TXD1 0.230, TXEN 0.185
# PHY (datasheet table 5-11), relative to REF_CLK rising at the PHY:
#   RXD/RXER/CRS_DV valid 14.0 max, hold 3.0 min
#   TXD/TXEN setup 4.0, hold 1.5
# FPGA (Interface Designer timing report, outflow/CAIN_project.pt_timing.rpt):
#   Efinity times core paths only, from the core side of each GPIO, so the GPIO
#   buffer delays are added in here:
#   GPIO_IN  rmii_crs_dv 2.205/0.848, rmii_rxd[*] and rmii_rxerr 1.815/0.698 (max/min)
#   GPIO_OUT rmii_txd[*] and rmii_txen 5.084/1.956
#   With internal feedback the PLL lines clk_50 up with its reference at the PLL, after
#   the pll_clkin input buffer (2.335/0.898). So the clk_50 edge these delays are
#   measured from is at 0.030 + 2.335 (slow) / 0.030 + 0.898 (fast). Efinity adds the
#   clock tree from there.
#
# max (setup) uses slow-corner delays throughout, min (hold) uses fast-corner ones.
#
# Inputs:  max = 0.173 + 14.0 + trace + GPIO_IN max - (0.030 + 2.335) = 11.808 + trace + GPIO_IN max
#          min = 0.173 +  3.0 + trace + GPIO_IN min - (0.030 + 0.898) =  2.245 + trace + GPIO_IN min
set_input_delay -clock clk_50 -max 14.182 [get_ports {rmii_crs_dv}]
set_input_delay -clock clk_50 -min 3.262  [get_ports {rmii_crs_dv}]
set_input_delay -clock clk_50 -max 13.780 [get_ports {rmii_rxerr}]
set_input_delay -clock clk_50 -min 3.100  [get_ports {rmii_rxerr}]
set_input_delay -clock clk_50 -max 13.790 [get_ports {rmii_rxd[0]}]
set_input_delay -clock clk_50 -min 3.110  [get_ports {rmii_rxd[0]}]
set_input_delay -clock clk_50 -max 13.783 [get_ports {rmii_rxd[1]}]
set_input_delay -clock clk_50 -min 3.103  [get_ports {rmii_rxd[1]}]

# Outputs: max = GPIO_OUT max + trace + 4.0 - (0.173 - (0.030 + 2.335)) = GPIO_OUT max + trace + 6.192
#          min = GPIO_OUT min + trace - 1.5 - (0.173 - (0.030 + 0.898)) = GPIO_OUT min + trace - 0.745
set_output_delay -clock clk_50 -max 11.500 [get_ports {rmii_txd[0]}]
set_output_delay -clock clk_50 -min 1.435  [get_ports {rmii_txd[0]}]
set_output_delay -clock clk_50 -max 11.506 [get_ports {rmii_txd[1]}]
set_output_delay -clock clk_50 -min 1.441  [get_ports {rmii_txd[1]}]
set_output_delay -clock clk_50 -max 11.461 [get_ports {rmii_txen}]
set_output_delay -clock clk_50 -min 1.396  [get_ports {rmii_txen}]

# nINT is asynchronous (open drain). Enable this once it is used, through a synchronizer.
# set_false_path -from [get_ports {rmii_int}]
