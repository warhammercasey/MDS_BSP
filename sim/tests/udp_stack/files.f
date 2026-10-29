// Sources for the udp_stack sim. Paths are relative to the repo root.
--top-module tb_udp_stack
+incdir+src/ethernet/packetization

src/interfaces/axis_if.sv
src/interfaces/apb_if.sv
src/interfaces/net_hdr_if.sv
src/ethernet/packetization/net_pkg.sv
src/cdc/simple_dual_port_bram.sv
src/infrastructure/axis_packet_fifo.sv
src/ethernet/rmii_tx.sv
src/ethernet/rmii_rx.sv
src/ethernet/rmii.sv
src/ethernet/packetization/eth_packetizer.sv
src/ethernet/packetization/eth_depacketizer.sv
src/ethernet/packetization/ipv4_packetizer.sv
src/ethernet/packetization/ipv4_depacketizer.sv
src/ethernet/packetization/udp_packetizer.sv
src/ethernet/packetization/udp_depacketizer.sv
src/ethernet/packetization/udp_stack.sv
src/ethernet/packetization/udp_stack_regs.sv

sim/tests/udp_stack/tb_udp_stack.sv
