# Sim environment setup and operation (for agents)

This file tells an agent how to rebuild the CAIN simulation toolchain on a fresh machine, check that it works, and run sims without tripping over known problems. [README.md](README.md) is the user guide; it covers runner flags, the `waves.txt` format and how to add tests.

## 1. What the environment looks like

| Piece | Where it runs | Known-good version |
|---|---|---|
| `sim/run.py` (runner, Python stdlib only, no pip packages) | Windows (PowerShell) or inside WSL | Python 3.12 (needs 3.10+) |
| Verilator + `make` + `g++` + `stdbuf` | WSL, called by the runner via `wsl -e bash -c ...` | Ubuntu 24.04, Verilator 5.020, g++ 13.3, GNU Make 4.3 |
| GTKWave (+ `fst2vcd.exe`) | Windows | 3.3.100 at `C:\iverilog\gtkwave\bin\` |

The repo lives on the Windows filesystem. The runner maps it to `/mnt/c/...` for WSL and runs every Verilator command from the repo root.

## 2. Install

### WSL + Verilator
```powershell
wsl --install -d Ubuntu-24.04        # skip if `wsl -l -v` already lists a distro
```
Then, inside WSL:
```bash
sudo apt update
sudo apt install -y verilator build-essential   # build-essential provides make and g++; stdbuf ships in coreutils
verilator --version                             # must be 5.x (Verilator 4 has no --timing / --binary)
```
- Verilator ≥ 5.0 and g++ ≥ 10 are required, because the testbenches use `--timing` (C++20 coroutines). Ubuntu 22.04's apt package is 4.x and too old; on that release, build Verilator 5 from source.
- **Optional:** `sudo apt install -y zlib1g-dev` switches dumps from VCD to the much smaller FST. The runner probes `/usr/include/zlib.h` and picks the format automatically. The current machine does **not** have zlib installed, so it produces VCD.
- If the tools are in a non-default distro, set `CAIN_WSL_DISTRO=<name>` (the name from `wsl -l -v`).

### GTKWave (Windows)
GTKWave came from the Windows Icarus Verilog installer (bleyer.org/icarus), installed to `C:\iverilog`. The runner looks for GTKWave in this order:
1. `CAIN_GTKWAVE` env var
2. `gtkwave` on PATH
3. `C:\iverilog\gtkwave\bin\gtkwave.exe` (or `/mnt/c/iverilog/...` when the runner runs inside WSL)

`fst2vcd.exe` must be in the same folder as `gtkwave.exe`; it is only needed when dumps are FST. Iverilog itself is **not** used, because it can't handle the RTL's interface ports or `$past`.

### Python
Install any CPython 3.10+ on Windows and make sure `python` is on PATH. Nothing else is required.

## 3. Verify

From the repo root:
```powershell
python sim/run.py --list    # expect: adc, i2s, uart
python sim/run.py --all     # expect: every line PASS, ending with "3/3 passed"; exit code 0
```
A first build of each test takes tens of seconds while the C++ compiles. Later runs reuse `sim/out/<test>/obj_dir` and only rebuild what changed. Use `--clean` to force a full rebuild.

If something fails, read these in order:
1. `sim/out/report.txt` — the summary table, with the first errors per failing test
2. `sim/out/<test>/build.log` for `BUILD_FAIL`, or `sim/out/<test>/sim.log` for any other failure
3. `sim/out/<test>/result.json` — a machine-readable copy of the same result

| Symptom | Cause / fix |
|---|---|
| `wsl` not found, or the runner hangs at its first command | WSL isn't installed or the distro is stopped/broken. Run `wsl -e true` to check. |
| `verilator: command not found` | Verilator isn't installed in the default distro. Install it, or set `CAIN_WSL_DISTRO`. |
| `%Error: Unknown option --timing` / `--binary` | Verilator 4.x is installed; Verilator 5 is required. |
| Build fails in `verilated_timing` / coroutine errors | g++ is too old (needs 10+). |
| `NO_VERDICT` | The TB exited without calling `tb_finish()`, or it crashed. Check the end of `sim.log`. |
| `TIMEOUT` | Wall-clock limit (default 600 s, change with `--timeout`). Each TB also has its own `TB_WATCHDOG`, which fails in sim time. |

## 4. Running sims as an agent

```powershell
python sim/run.py uart                 # one test (names accept globs: 'i*')
python sim/run.py adc i2s              # several tests
python sim/run.py --all                # everything
python sim/run.py --all --no-trace     # faster, no wave dump
python sim/run.py --report             # re-print the latest results without running anything
python sim/run.py i2s --plusarg +foo=1 # pass plusargs to the sim
```
- **Exit code** is 0 only if every selected test reports `PASS`. Use it, or `report.txt`, rather than parsing the console.
- When stdout is not a TTY, the runner prints one plain line per stage (`[uart] build...`, `[uart] run...`) instead of a live status line, so captured output stays readable.
- **Do not pipe the runner through `Select-Object -First N`, `head`, or anything else that closes the pipe early.** That kills the runner mid-sim and leaves partial results. Let it finish (redirect to `$null` or a file if needed), then read `sim/out/report.txt` and the logs.
- **Don't open GTKWave (`--waves` / `--view`) unless the user asks.** It launches a GUI window on their desktop. `--view` re-opens waves from the last run without re-simulating.
- Test verdicts come from the TB's final line: `TEST PASSED (N checks)` or `TEST FAILED (E errors, N checks)`. Runtime messages look like `[  1234.5 ns] INFO: ...` / `WARN:` / `ERROR:`.

## 5. Repo layout

```
sim/
  run.py              runner (build → run → report → optional waves)
  waves.py            waves.txt parser, VCD merge, .gtkw writer
  common/tb_pkg.sv    tb_info/warn/error, tb_error_once, tb_finish (compiled before every test)
  common/tb_macros.svh  CHECK_EQ, CHECK, TB_INIT, TB_WATCHDOG (found via +incdir+sim/common)
  models/             datasheet-based device models with timing checks
    adc088s022_model.sv   TI ADC088S022 (SPI ADC)
    sph0645_model.sv      Knowles SPH0645LM4H-B (I2S mic)
  tests/<name>/
    files.f           Verilator filelist: --top-module <tb>, then sources relative to the repo root
    tb_<name>.sv      testbench
    waves.txt         signals/groups to show in GTKWave
  out/                generated output, git-ignored; safe to delete
```
Verilator flags (in `run.py`): `--binary --timing --timescale 1ns/1ps -Wno-fatal -j 0 +incdir+sim/common`, plus `--trace` (VCD) or `--trace-fst`. Warnings are counted, not fatal; the count appears in the report's `WARNS` column.

## 6. Things that will bite you

- **Clock and rate constants are copied by hand** from `src/cain_top.sv` into each testbench. They must match, or a TB will test a configuration the design doesn't use. Currently every TB uses `CLK_FREQ = 73_571_400` (cain_top's `CLK_IO_FREQ`), and `tb_adc.sv` uses `SAMPLE_RATE = 100_000` (cain_top's `ADC_SAMPLE_RATE`). `tb_i2s.sv` deliberately does **not** set `I2S_CLK`; it uses the DUT's default. If `cain_top` changes, update the TBs to match.
- **Editor diagnostics are false positives.** The IDE linter reports `tb_macros.svh` not found and `tb_pkg` undefined because it doesn't know about `+incdir+sim/common` or `COMMON_SOURCES`. Only Verilator's output counts.
- **Verilator 5.020 limits** we have already hit in TB code:
  - No hierarchical type references (e.g. `adc_chip.conv_t`). Declare the type locally or use a package.
  - An assignment pattern (`'{...}`) can't be passed directly to `queue.push_back()`. Assign it to a variable first.
  - Initialized local variables inside `always`/`initial` blocks must be declared `automatic`.
  - Intra-assignment NBA delays (`x <= #(T) y;`) do work.
- **Multi-instance models:** a timing violation that can fire from all 8 mic instances (or on every edge) should use `tb_error_once(key, msg)`. It reports the first occurrence and `tb_finish()` prints a count of the rest.
- **TB conventions:** every TB includes `tb_macros.svh`, imports `tb_pkg::*`, calls `` `TB_INIT `` and `` `TB_WATCHDOG(...) ``, logs each phase with `tb_info()` (that text shows in the runner's progress line), and ends with `tb_finish()`. AXI-Stream sinks drive `tready` on posedge and sample handshakes on negedge.
- After changing RTL or TBs, re-run `python sim/run.py --all` and confirm it ends with `N/N passed`.
