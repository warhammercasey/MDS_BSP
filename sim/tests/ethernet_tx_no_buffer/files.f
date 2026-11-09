// The ethernet TX test with the TX frame buffer left out (TX_BUFFER = 0).
// Paths are relative to the repo root.
--top-module tb_ethernet
-GTX_BUFFER=0

-f sim/tests/ethernet_tx/files.f
