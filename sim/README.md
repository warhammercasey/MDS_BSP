# CAIN simulation

Designer-level sims for individual components. The sims build with Verilator (in WSL), and you view waves in GTKWave.

To set up the toolchain on a fresh machine (instructions written for agents), see [SETUP.md](SETUP.md).

## Requirements

- WSL (Ubuntu) with `verilator` (5.x), `make` and `g++`
- Python 3.10+ on Windows (or run inside WSL with `python3`)
- GTKWave: uses `gtkwave` from PATH, otherwise `C:\iverilog\gtkwave\bin\gtkwave.exe`
- Optional: `sudo apt install zlib1g-dev` in WSL to get compact FST waves instead of VCD

Override defaults with the env vars `CAIN_WSL_DISTRO` and `CAIN_GTKWAVE`.

## Usage

Run from the repo root:

```
python sim/run.py --list            # list tests
python sim/run.py uart              # run one test
python sim/run.py uart --waves      # run, then open GTKWave
python sim/run.py --all             # run every test
python sim/run.py --view uart       # open waves from the last run
python sim/run.py --all --waves     # run all, view every test in one window
python sim/run.py --report          # latest result of every test
python sim/run.py uart --clean      # full rebuild
python sim/run.py --all --no-trace  # skip wave dumping
python sim/run.py uart --plusarg +foo=1
```

The exit code is 0 only when every selected test passes.

While a test runs, a status line shows its progress:
- **Build:** verilating, then compiling, then linking.
- **Run:** sim time, an error count, and the testbench's latest `tb_info(...)` message. Log each test phase with `tb_info`, and long sims will show where they are.

Outputs go to `sim/out/<test>/` (`build.log`, `sim.log`, `waves.vcd|fst`, `result.json`).
`sim/out/report.txt` holds the summary table.

## Adding a test

1. Create `sim/tests/<name>/files.f` with `--top-module <tb>` and the sources, using paths relative to the repo root. Any Verilator option works there too.
2. Write the testbench. Its skeleton:

```systemverilog
`timescale 1ns/1ps
`include "tb_macros.svh"

module tb_foo;
    import tb_pkg::*;

    `TB_INIT                   // ns timestamps + wave dumping
    `TB_WATCHDOG(1_000_000)    // fail if still running after 1 ms

    initial begin
        ...
        `CHECK_EQ(dut_out, 8'h42, "dut_out after write")
        `CHECK(fifo_empty, "fifo drained")
        tb_finish();           // prints TEST PASSED / TEST FAILED
    end
endmodule
```

3. Optional: list the signals to show in GTKWave in `sim/tests/<name>/waves.txt` (see below).

Reusable models of external devices go in `sim/models/`; add them to a test's `files.f`. For example, `adc088s022_model.sv` (ADC, SPI), `sph0645_model.sv` (I2S microphone) and `lan8720a_model.sv` (Ethernet PHY: MDC/MDIO management and its registers) are models built from the chips' datasheets, and they check interface timing. `apb_reg_model.sv` is a generic APB register block (wait states, PSLVERR, an address range that never responds) that checks the APB protocol. For a check that can fire often or from many instances, use `tb_error_once(key, msg)`: it reports the first occurrence and counts the rest.

### CDC tests

The `src/cdc` tests (`ff_sync`, `pulse_cdc`, `two_port_bram`, `simple_dual_port_bram`, `axis_cdc_fifo`, `axis_tid_cdc_fifo`, `apb_cdc`) instantiate one DUT per ordered pair of the design's four clocks, 12 in all, and run them in parallel. They share these helpers:

- `sim/common/cain_clocks_pkg.sv` holds the clock frequencies, which mirror `cain_top`, and the pair numbering. Pair 10 is `clk_io -> clk_core`, the pair the waves show.
- `sim/models/clk_gen.sv` is a clock with a random start phase and per-edge jitter. Because the PLL clocks have exact frequency ratios, the jitter is what makes their edges drift through every relative alignment.
- `sim/models/ff_sync_checker.sv` is bound into every `ff_sync`. It fails the test if a multi-bit synchronized bus changes more than one bit in a clock (i.e. it isn't Gray-coded). Add it to `files.f` for any test of logic that uses `ff_sync`.

Verilator is a 2-state simulator and does not model metastability, so the tests check the structural rules instead: the number of synchronizer stages (measured as latency) and Gray coding.

## Waves

`--waves` (after a run) and `--view` (no re-run) open one GTKWave window, zoomed to fit. Each test's signals go in a group named after the test. When you select several tests (e.g. `--all --waves`), their dumps are merged into `sim/out/_waves/waves.vcd`, with each test's hierarchy under a scope named after the test. All the tests then share one timeline, each starting at t=0.

Signals come from `sim/tests/<name>/waves.txt`. A test without that file shows its testbench's top-level signals. `--signals FILE` uses a different file for this view only.

```
# comment
clk                          # path relative to the testbench top
axis_tx.tdata   ascii        # format: bin hex dec sdec oct ascii analog analog-step
dut.uart_rx_i.*              # '*' = one hierarchy level, '**' = any depth
group Serial pins            # groups nest; close with 'end'
    rx
    tx
end
- a label                    # comment row ('-' alone = blank row)
```

Default formats are `bin` for 1-bit signals and `hex` for vectors. A path that matches nothing prints a warning and is skipped. The generated `waves.gtkw` is rewritten every time, so put lasting changes in `waves.txt`.

Statuses: `PASS`, `FAIL` (checks failed), `BUILD_FAIL`, `TIMEOUT` (wall clock),
`NO_VERDICT` (the sim ended without calling `tb_finish()`).
