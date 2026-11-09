// Sources for the top-level sim: every design file in the Efinity project, plus the
// board models. Paths are relative to the repo root.
--top-module tb_cain_top
+incdir+src/ethernet/packetization

src/interfaces/axis_if.sv
src/interfaces/apb_if.sv
src/interfaces/net_hdr_if.sv
src/ethernet/packetization/net_pkg.sv

src/cain_top.sv
src/uart/uart.sv
src/uart/uart_tx.sv
src/uart/uart_rx.sv
src/adc/adc.sv
src/audio/i2s.sv
src/sram/sram_controller.sv
src/cdc/two_port_bram.sv
src/cdc/axis_tid_cdc_fifo.sv
src/cdc/ff_sync.sv
src/cdc/pulse_cdc.sv
src/cdc/axis_cdc_fifo.sv
src/led_cube.sv
src/reset_sync.sv
src/cdc/simple_dual_port_bram.sv
src/uart/uart_controller.sv
src/ethernet/eth_smi.sv
src/infrastructure/axis_switch.sv
src/infrastructure/apb_splitter.sv
src/infrastructure/apb_width_conv.sv
src/cdc/apb_cdc.sv
src/infrastructure/axis_tid_reorder.sv
src/ethernet/rmii_tx.sv
src/ethernet/rmii_rx.sv
src/ethernet/ethernet.sv
src/ethernet/rmii.sv
src/ethernet/packetization/udp_packetizer.sv
src/ethernet/packetization/eth_depacketizer.sv
src/ethernet/packetization/ipv4_packetizer.sv
src/ethernet/packetization/udp_depacketizer.sv
src/ethernet/packetization/udp_stack.sv
src/ethernet/packetization/ipv4_depacketizer.sv
src/ethernet/packetization/eth_packetizer.sv
src/ethernet/packetization/udp_stack_regs.sv
src/ethernet/packetization/rtp_packetizer.sv
src/infrastructure/axis_packet_switch.sv
src/infrastructure/axis_packet_fifo.sv
src/infrastructure/axis_tid_broadcast.sv
src/dsp/building_blocks/peak_finder.sv
src/dsp/building_blocks/cross_correlator.sv
src/dsp/building_blocks/mixer.sv
src/dsp/building_blocks/moving_average_filter.sv
src/dsp/building_blocks/fir_decimator.sv
src/dsp/building_blocks/direction_estimator.sv
src/dsp/tdoa_direction_finder.sv
src/ram/dual_port_rom.sv
src/ram/rom.sv
src/infrastructure/axis_tid_serializer.sv
src/infrastructure/axis_register_slice.sv

sim/models/clk_gen.sv
sim/models/inmp441_model.sv
sim/models/adc088s022_model.sv

sim/tests/cain_top/tb_cain_top.sv
