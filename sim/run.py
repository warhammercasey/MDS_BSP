#!/usr/bin/env python3
"""CAIN simulation runner (Verilator in WSL, GTKWave for waves).

Usage (from the repo root, in PowerShell or WSL):
    python sim/run.py --list            list available tests
    python sim/run.py uart              run one test
    python sim/run.py uart --waves      run one test and open its waves
    python sim/run.py --all             run every test
    python sim/run.py --view uart       open waves from the last run (no re-run)
    python sim/run.py --all --waves     run everything, view all tests in one window
    python sim/run.py --report          print the latest result of every test

GTKWave opens with the signals listed in sim/tests/<name>/waves.txt, in a group
named after the test (format described in sim/waves.py). When several tests are
viewed, their dumps are merged so each test gets its own group in one window.

A test is a directory sim/tests/<name>/ containing a files.f (Verilator
filelist with --top-module and sources, paths relative to the repo root).
The testbench must call tb_pkg::tb_finish(), which prints the verdict line.

Outputs land in sim/out/<name>/: build.log, sim.log, waves.{fst,vcd}, result.json.
sim/out/report.txt summarizes the latest result of every test.
"""

import argparse
import fnmatch
import json
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

import waves

REPO = Path(__file__).resolve().parent.parent
SIM_DIR = REPO / "sim"
TESTS_DIR = SIM_DIR / "tests"
OUT_DIR = SIM_DIR / "out"
REPORT_FILE = OUT_DIR / "report.txt"
SIGNALS_FILE = "waves.txt"  # per-test GTKWave signal list, see sim/waves.py
DEFAULT_SIGNALS = [waves.Signal("*", None, "default: top-level testbench signals")]

ON_WINDOWS = os.name == "nt"
WSL_DISTRO = os.environ.get("CAIN_WSL_DISTRO")  # default WSL distro if unset
WIN_GTKWAVE = r"C:\iverilog\gtkwave\bin\gtkwave.exe"

# Sources compiled ahead of every test's files.f
COMMON_SOURCES = ["sim/common/tb_pkg.sv"]
VERILATOR_FLAGS = [
    "--binary", "--timing",
    "--timescale", "1ns/1ps",
    "-Wno-fatal",
    "-j", "0",
    "+incdir+sim/common",
]

VERDICT_RE = re.compile(r"^TEST (PASSED|FAILED)\b.*?(?:(\d+) errors?, )?(\d+) checks", re.M)


# ----------------------------------------------------------------------------
# Shell helpers: run Linux commands natively or through WSL
# ----------------------------------------------------------------------------

def to_linux_path(p: Path) -> str:
    if not ON_WINDOWS:
        return str(p)
    s = str(p.resolve()).replace("\\", "/")
    return f"/mnt/{s[0].lower()}{s[2:]}"


def linux_run(script: str, timeout: float | None = None, on_line=None) -> tuple[int, str]:
    """Run a bash script from the repo root. Returns (exit code, combined output).

    If on_line is given, it is called with each output line as it arrives, and with
    None about every 0.1 s while waiting (so callers can refresh a progress display).
    """
    script = f"cd '{to_linux_path(REPO)}' && {script}"
    if ON_WINDOWS:
        cmd = ["wsl"] + (["-d", WSL_DISTRO] if WSL_DISTRO else []) + ["-e", "bash", "-c", script]
    else:
        cmd = ["bash", "-c", script]
    if on_line is None:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        return proc.returncode, proc.stdout.decode("utf-8", errors="replace")

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    lines: queue.Queue = queue.Queue()

    def reader():
        for raw in proc.stdout:
            lines.put(raw.decode("utf-8", errors="replace"))
        lines.put(None)

    threading.Thread(target=reader, daemon=True).start()
    deadline = time.monotonic() + timeout if timeout else None
    output = []
    while True:
        try:
            line = lines.get(timeout=0.1)
        except queue.Empty:
            on_line(None)
        else:
            if line is None:
                break
            output.append(line)
            on_line(line.rstrip("\r\n"))
        if deadline and time.monotonic() > deadline:
            proc.kill()
            raise subprocess.TimeoutExpired(cmd, timeout, "".join(output))
    return proc.wait(), "".join(output)


_wave_format: str | None = None


def wave_format() -> str:
    """FST needs zlib headers for Verilator's C++ build; fall back to VCD without them."""
    global _wave_format
    if _wave_format is None:
        rc, _ = linux_run("test -f /usr/include/zlib.h")
        _wave_format = "fst" if rc == 0 else "vcd"
    return _wave_format


# ----------------------------------------------------------------------------
# Output formatting
# ----------------------------------------------------------------------------

USE_COLOR = sys.stdout.isatty() and "NO_COLOR" not in os.environ
if USE_COLOR and ON_WINDOWS:
    os.system("")  # enable ANSI escape handling in the Windows console

STATUS_COLOR = {"PASS": "32", "FAIL": "31", "BUILD_FAIL": "31", "TIMEOUT": "33", "NO_VERDICT": "33"}


def color_status(status: str) -> str:
    if not USE_COLOR:
        return status
    return f"\033[{STATUS_COLOR.get(status, '0')}m{status}\033[0m"


LIVE = sys.stdout.isatty()


class Progress:
    """Status line for one test: '[test] stage  elapsed  detail'.

    On a terminal the line is rewritten in place; otherwise each stage is printed
    once when it starts, so logs stay readable.
    """

    def __init__(self, test: str):
        self.test = test
        self.stage = ""
        self.detail = ""
        self.t0 = time.monotonic()
        self.width = 0

    def set_stage(self, stage: str, detail: str = ""):
        self.stage, self.detail, self.t0 = stage, detail, time.monotonic()
        if not LIVE:
            print(f"[{self.test}] {stage}...", flush=True)
        self.draw()

    def draw(self):
        if not LIVE:
            return
        text = f"[{self.test}] {self.stage:<5} {time.monotonic() - self.t0:5.1f}s  {self.detail}"
        text = text[:shutil.get_terminal_size().columns - 1]
        sys.stdout.write("\r" + text.ljust(self.width))
        sys.stdout.flush()
        self.width = len(text)

    def finish(self, text: str):
        if LIVE:
            sys.stdout.write("\r" + " " * self.width + "\r")
        print(text, flush=True)


def build_watcher(progress: Progress):
    """on_line callback that turns Verilator/make output into build progress."""
    state = {"phase": "verilating", "warnings": 0}
    obj_re = re.compile(r"\s-c\s+-o\s+(\S+\.o)\b")

    def on_line(line):
        if line is not None:
            if line.startswith("%Warning"):
                state["warnings"] += 1
            elif "Entering directory" in line:
                state["phase"] = "compiling C++"
            elif m := obj_re.search(line):
                state["phase"] = f"compiling {m.group(1)}"
            elif re.search(r"\s-o\s+Vsim\b", line):
                state["phase"] = "linking"
            elif "Nothing to be done" in line:
                state["phase"] = "up to date"
            warns = f"  ({state['warnings']} warnings)" if state["warnings"] else ""
            progress.detail = state["phase"] + warns
        progress.draw()

    return on_line


def format_sim_time(ns: float) -> str:
    if ns >= 1e6:
        return f"{ns / 1e6:.3f} ms"
    if ns >= 1e3:
        return f"{ns / 1e3:.3f} us"
    return f"{ns:.1f} ns"


def sim_watcher(progress: Progress):
    """on_line callback that shows the TB's latest tb_info message, sim time and error count."""
    state = {"time": None, "msg": "", "errors": 0}
    log_re = re.compile(r"^\[\s*([\d.]+) ns\]\s+(INFO|WARN|ERROR):\s*(.*)")

    def on_line(line):
        if line is not None:
            if m := log_re.match(line):
                state["time"] = float(m.group(1))
                if m.group(2) == "INFO":
                    state["msg"] = m.group(3)
                elif m.group(2) == "ERROR":
                    state["errors"] += 1
            elif line.startswith("%Error"):
                state["errors"] += 1
            parts = []
            if state["time"] is not None:
                parts.append(f"t={format_sim_time(state['time'])}")
            if state["errors"]:
                parts.append(f"[{state['errors']} errors]")
            if state["msg"]:
                parts.append(state["msg"])
            progress.detail = "  ".join(parts)
        progress.draw()

    return on_line


# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------

@dataclass
class Result:
    test: str
    status: str = "NO_VERDICT"  # PASS | FAIL | BUILD_FAIL | TIMEOUT | NO_VERDICT
    checks: int = 0
    errors: int = 0
    warnings: int = 0
    build_s: float = 0.0
    sim_s: float = 0.0
    timestamp: str = ""
    first_errors: list[str] = field(default_factory=list)


def discover_tests() -> list[str]:
    return sorted(d.name for d in TESTS_DIR.iterdir() if (d / "files.f").is_file())


def select_tests(patterns: list[str], available: list[str]) -> list[str]:
    selected = []
    for pat in patterns:
        matches = fnmatch.filter(available, pat)
        if not matches:
            sys.exit(f"error: no test matches '{pat}'. Available: {', '.join(available)}")
        selected += [m for m in matches if m not in selected]
    return selected


def waves_file(test: str) -> Path | None:
    for ext in ("fst", "vcd"):
        p = OUT_DIR / test / f"waves.{ext}"
        if p.is_file():
            return p
    return None


def run_test(test: str, args, progress: Progress) -> Result:
    out = OUT_DIR / test
    out_rel = out.relative_to(REPO).as_posix()
    if args.clean and out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob("waves.*"):
        old.unlink()

    res = Result(test, timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

    # Build
    progress.set_stage("build", "verilating")
    trace_flags = []
    if not args.no_trace:
        trace_flags = ["--trace-fst"] if wave_format() == "fst" else ["--trace"]
    build_cmd = " ".join(
        ["verilator"] + VERILATOR_FLAGS + trace_flags
        + ["--Mdir", f"{out_rel}/obj_dir", "-o", "Vsim"]
        + COMMON_SOURCES + ["-f", f"sim/tests/{test}/files.f"]
    )
    t0 = time.monotonic()
    # stdbuf: line-buffer output through the pipe so progress arrives as it happens
    rc, log = linux_run(f"stdbuf -oL -eL {build_cmd}", timeout=args.timeout, on_line=build_watcher(progress))
    res.build_s = time.monotonic() - t0
    build_log = out / "build.log"
    up_to_date = rc == 0 and "Nothing to be done" in log and build_log.is_file()
    if not up_to_date:  # keep the previous log (and its warnings) when Verilator skipped regeneration
        build_log.write_text(f"$ {build_cmd}\n\n{log}", encoding="utf-8")
    res.warnings = len(re.findall(r"^%Warning", build_log.read_text(encoding="utf-8"), re.M))
    if rc != 0:
        res.status = "BUILD_FAIL"
        res.first_errors = [l for l in log.splitlines() if l.startswith("%Error") or "error:" in l][:5]
        return res

    # Simulate
    plusargs = list(args.plusarg)
    if not args.no_trace:
        plusargs.append(f"+trace=waves.{wave_format()}")
    sim_cmd = " ".join(["./obj_dir/Vsim"] + plusargs)
    progress.set_stage("run")
    t0 = time.monotonic()
    try:
        rc, log = linux_run(f"cd {out_rel} && stdbuf -oL -eL {sim_cmd}", timeout=args.timeout,
                            on_line=sim_watcher(progress))
    except subprocess.TimeoutExpired:
        res.sim_s = time.monotonic() - t0
        res.status = "TIMEOUT"
        res.first_errors = [f"Simulation exceeded {args.timeout}s wall-clock timeout"]
        (out / "sim.log").write_text(f"$ {sim_cmd}\n\nTIMEOUT\n", encoding="utf-8")
        return res
    res.sim_s = time.monotonic() - t0
    (out / "sim.log").write_text(f"$ {sim_cmd}\n\n{log}", encoding="utf-8")

    error_lines = [l for l in log.splitlines() if "ERROR:" in l or l.startswith("%Error")]
    res.first_errors = error_lines[:5]
    m = VERDICT_RE.search(log)
    if m:
        res.checks = int(m.group(3))
        res.errors = int(m.group(2) or 0)
        res.status = "PASS" if m.group(1) == "PASSED" and rc == 0 else "FAIL"
    else:
        res.errors = len(error_lines)
        res.status = "NO_VERDICT"
        if not res.first_errors:
            res.first_errors = [f"No verdict line (exit code {rc}) - did the TB call tb_finish()?"]
    return res


# ----------------------------------------------------------------------------
# Reporting
# ----------------------------------------------------------------------------

def load_results() -> list[Result]:
    results = []
    for test in discover_tests():
        f = OUT_DIR / test / "result.json"
        if f.is_file():
            results.append(Result(**json.loads(f.read_text(encoding="utf-8"))))
        else:
            results.append(Result(test, status="NOT_RUN"))
    return results


def format_report(results: list[Result]) -> str:
    hdr = f"{'TEST':<20} {'STATUS':<11} {'CHECKS':>7} {'ERRORS':>7} {'WARNS':>6} {'BUILD':>7} {'SIM':>7}  LAST RUN"
    lines = [hdr, "-" * len(hdr)]
    for r in results:
        if r.status == "NOT_RUN":
            lines.append(f"{r.test:<20} {'NOT_RUN':<11}")
            continue
        lines.append(
            f"{r.test:<20} {r.status:<11} {r.checks:>7} {r.errors:>7} {r.warnings:>6} "
            f"{r.build_s:>6.1f}s {r.sim_s:>6.1f}s  {r.timestamp}"
        )
    ran = [r for r in results if r.status != "NOT_RUN"]
    passed = sum(r.status == "PASS" for r in ran)
    lines += ["-" * len(hdr), f"{passed}/{len(ran)} passed"]

    for r in results:
        if r.status not in ("PASS", "NOT_RUN") and r.first_errors:
            lines += ["", f"{r.test} ({r.status}):"] + [f"  {e}" for e in r.first_errors]
            log = "build.log" if r.status == "BUILD_FAIL" else "sim.log"
            lines.append(f"  -> sim/out/{r.test}/{log}")
    return "\n".join(lines)


def print_report(text: str):
    for status in STATUS_COLOR:
        text = re.sub(rf"(?<= ){status}(?= )", color_status(status), text)
    print(text)


def write_report() -> str:
    text = format_report(load_results())
    REPORT_FILE.write_text(f"CAIN sim report - {datetime.now():%Y-%m-%d %H:%M:%S}\n\n{text}\n", encoding="utf-8")
    return text


# ----------------------------------------------------------------------------
# Waves
# ----------------------------------------------------------------------------

def gtkwave_exe() -> str:
    exe = os.environ.get("CAIN_GTKWAVE") or shutil.which("gtkwave")
    return exe or (WIN_GTKWAVE if ON_WINDOWS else "/mnt/c/iverilog/gtkwave/bin/gtkwave.exe")


def gtkwave_tool(name: str) -> str:
    """A tool shipped next to GTKWave (e.g. fst2vcd)."""
    exe = Path(gtkwave_exe())
    return str(exe.with_name(name + exe.suffix))


def host_path(p: Path, exe: str) -> str:
    """Path argument for exe; Windows tools launched from WSL need Windows paths."""
    if not ON_WINDOWS and exe.endswith(".exe"):
        return subprocess.check_output(["wslpath", "-w", str(p)], text=True).strip()
    return str(p)


def fst2vcd_cmd(fst: Path, out: Path | None = None) -> list[str]:
    exe = gtkwave_tool("fst2vcd")
    return [exe] + (["-o", host_path(out, exe)] if out else []) + [host_path(fst, exe)]


def test_top(test: str) -> str:
    text = re.sub(r"//.*", "", (TESTS_DIR / test / "files.f").read_text(encoding="utf-8"))
    m = re.search(r"--top(?:-module)?\s+(\S+)", text)
    return m.group(1) if m else ""


def open_waves(tests: list[str], signals: Path | None = None):
    """Open one GTKWave window showing every test's signals, grouped per test.

    Signals come from sim/tests/<test>/waves.txt (or `signals` if given); a test
    without one shows its testbench's top-level signals.
    """
    have = []
    for t in tests:
        if waves_file(t):
            have.append(t)
        else:
            print(f"{t}: no waves found - run the test first (without --no-trace)")
    if not have:
        return

    def warn(msg):
        print(f"waves: {msg}")

    views = []
    for t in have:
        sig_file = signals or TESTS_DIR / t / SIGNALS_FILE
        try:
            items = waves.parse_signal_file(sig_file) if sig_file.is_file() else DEFAULT_SIGNALS
        except ValueError as e:
            sys.exit(f"waves: {t}: {e}")
        views.append(waves.WaveView(t, test_top(t), waves.read_header(waves_file(t), fst2vcd_cmd), items))

    if len(have) == 1:
        out_dir, dump = OUT_DIR / have[0], waves_file(have[0])
    else:
        # GTKWave shows one dump per window, so merge the tests' dumps into one
        out_dir = OUT_DIR / "_waves"
        out_dir.mkdir(parents=True, exist_ok=True)
        inputs = []
        for t in have:
            wf = waves_file(t)
            if wf.suffix == ".fst":
                vcd = out_dir / f"{t}.vcd"
                subprocess.run(fst2vcd_cmd(wf, vcd), check=True, stdout=subprocess.DEVNULL)
                wf = vcd
            inputs.append((t, wf))
        dump = out_dir / "waves.vcd"
        print(f"waves: merging {len(have)} dumps into {dump.relative_to(REPO)}")
        waves.merge_vcds(inputs, dump)

    save = out_dir / "waves.gtkw"
    waves.write_save_file(views, len(have) > 1, save, warn)
    script = out_dir / "waves.tcl"
    script.write_text("gtkwave::/Time/Zoom/Zoom_Best_Fit\n", encoding="utf-8")

    exe = gtkwave_exe()
    argv = [exe, "-S", host_path(script, exe), host_path(dump, exe), host_path(save, exe)]
    print(f"waves: opening {', '.join(have)} in GTKWave")
    subprocess.Popen(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="CAIN simulation runner",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("tests", nargs="*", help="test names or glob patterns (e.g. uart, 'i2s*')")
    ap.add_argument("-a", "--all", action="store_true", help="run every test")
    ap.add_argument("-l", "--list", action="store_true", help="list tests and exit")
    ap.add_argument("-w", "--waves", action="store_true", help="open GTKWave after running")
    ap.add_argument("-v", "--view", action="store_true", help="open waves from the last run without re-running")
    ap.add_argument("-r", "--report", action="store_true", help="print the latest result of every test")
    ap.add_argument("--signals", type=Path, metavar="FILE",
                    help=f"signal file for --waves/--view instead of each test's {SIGNALS_FILE}")
    ap.add_argument("--clean", action="store_true", help="delete previous build output first")
    ap.add_argument("--no-trace", action="store_true", help="skip wave dumping (faster)")
    ap.add_argument("--plusarg", action="append", default=[], metavar="+ARG",
                    help="extra plusarg for the sim, e.g. --plusarg +seed=3 (repeatable)")
    ap.add_argument("--timeout", type=float, default=600, help="build/sim wall-clock timeout in seconds")
    args = ap.parse_args()

    available = discover_tests()

    if args.list:
        print("\n".join(available))
        return 0

    if args.report:
        if not OUT_DIR.exists():
            print("No results yet.")
            return 0
        print_report(write_report())
        return 0

    if args.all:
        tests = available
    elif args.tests:
        tests = select_tests(args.tests, available)
    else:
        ap.print_help()
        return 1

    if args.view:
        open_waves(tests, args.signals)
        return 0

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    results = []
    for t in tests:
        progress = Progress(t)
        r = run_test(t, args, progress)
        (OUT_DIR / t / "result.json").write_text(json.dumps(asdict(r), indent=2), encoding="utf-8")
        results.append(r)
        detail = f"{r.checks} checks, {r.errors} errors  " if r.status in ("PASS", "FAIL") else ""
        times = f"build {r.build_s:.1f}s" + (f", run {r.sim_s:.1f}s" if r.status != "BUILD_FAIL" else "")
        progress.finish(f"[{t}] {color_status(r.status)}  {detail}({times})")

    print()
    print_report(write_report())

    if args.waves:
        open_waves(tests, args.signals)

    return 0 if all(r.status == "PASS" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
