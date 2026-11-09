// Sources for the ethernet TX test (also used by ethernet_tx_no_buffer and ethernet_rx).
// Paths are relative to the repo root.
--top-module tb_ethernet
+incdir+src/ethernet/packetization

src/interfaces/axis_if.sv
src/interfaces/apb_if.sv
src/interfaces/net_hdr_if.sv
src/ethernet/packetization/net_pkg.sv
src/cdc/simple_dual_port_bram.sv
src/cdc/ff_sync.sv
src/cdc/axis_cdc_fifo.sv
src/cdc/axis_tid_cdc_fifo.sv
src/infrastructure/axis_packet_fifo.sv
src/infrastructure/axis_packet_switch.sv
src/infrastructure/axis_tid_serializer.sv
src/infrastructure/axis_register_slice.sv
src/ethernet/rmii_tx.sv
src/ethernet/rmii_rx.sv
src/ethernet/rmii.sv
src/ethernet/packetization/eth_packetizer.sv
src/ethernet/packetization/eth_depacketizer.sv
src/ethernet/packetization/ipv4_packetizer.sv
src/ethernet/packetization/ipv4_depacketizer.sv
src/ethernet/packetization/udp_packetizer.sv
src/ethernet/packetization/udp_depacketizer.sv
src/ethernet/packetization/rtp_packetizer.sv
src/ethernet/packetization/udp_stack.sv
src/ethernet/packetization/udp_stack_regs.sv
src/ethernet/ethernet.sv

sim/tests/ethernet_tx/tb_ethernet.sv
