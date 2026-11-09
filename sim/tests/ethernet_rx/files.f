// The ethernet RX test: tb_ethernet with RX = 1. Paths are relative to the repo root.
--top-module tb_ethernet
-GRX=1

-f sim/tests/ethernet_tx/files.f
