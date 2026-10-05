"""GTKWave helpers for sim/run.py: signal files, VCD merging and save-file generation.

Signal file format (sim/tests/<name>/waves.txt):

    # comment                       anything after '#' is ignored
    clk                             signal path, relative to the testbench top
    axis_tx.tdata  hex              optional format: bin hex dec sdec oct ascii analog analog-step
    dut.uart_rx_i.*                 '*' matches within one hierarchy level, '**' across levels
    group Serial pins               start a (nestable) group
        rx
    end                             close the group
    - Some label                    a comment row ('-' alone gives a blank row)

When several tests are viewed together their dumps are merged into one VCD,
with each test's hierarchy nested under a scope named after the test, and
each test's signals are placed in a top-level group with the test's name.
"""

import heapq
import re
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

# GTKWave trace flags (gtkwave analyzer.h)
TR_HEX, TR_DEC, TR_BIN, TR_OCT = 0x2, 0x4, 0x8, 0x10
TR_RJUSTIFY, TR_BLANK, TR_SIGNED, TR_ASCII = 0x20, 0x200, 0x400, 0x800
TR_ANALOG_STEP, TR_ANALOG_INTERP, TR_ANALOG_STRETCH = 0x8000, 0x10000, 0x20000
TR_GRP_BEGIN, TR_GRP_END = 0x800000, 0x1000000

FORMATS = {
    "bin":         TR_RJUSTIFY | TR_BIN,
    "hex":         TR_RJUSTIFY | TR_HEX,
    "dec":         TR_RJUSTIFY | TR_DEC,
    "sdec":        TR_RJUSTIFY | TR_DEC | TR_SIGNED,
    "oct":         TR_RJUSTIFY | TR_OCT,
    "ascii":       TR_RJUSTIFY | TR_ASCII,
    "analog":      TR_RJUSTIFY | TR_DEC | TR_SIGNED | TR_ANALOG_INTERP,
    "analog-step": TR_RJUSTIFY | TR_DEC | TR_SIGNED | TR_ANALOG_STEP,
}
ANALOG_EXTRA_ROWS = 4  # analog traces get this many extra rows of height


# ----------------------------------------------------------------------------
# Signal files
# ----------------------------------------------------------------------------

@dataclass
class Signal:
    pattern: str
    fmt: str | None
    where: str  # file:line, for warnings


@dataclass
class Comment:
    text: str


@dataclass
class Group:
    name: str
    items: list = field(default_factory=list)


def parse_signal_file(path: Path) -> list:
    root = Group("")
    stack = [root]
    for n, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        where = f"{path.name}:{n}"
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("-"):
            stack[-1].items.append(Comment(line[1:].strip()))
        elif line == "end":
            if len(stack) == 1:
                raise ValueError(f"{where}: 'end' without matching 'group'")
            stack.pop()
        elif line.split()[0] == "group":
            g = Group(line[len("group"):].strip() or "group")
            stack[-1].items.append(g)
            stack.append(g)
        else:
            parts = line.split()
            if len(parts) > 2 or (len(parts) == 2 and parts[1] not in FORMATS):
                raise ValueError(f"{where}: expected '<signal> [{'|'.join(FORMATS)}]', got '{line}'")
            stack[-1].items.append(Signal(parts[0], parts[1] if len(parts) == 2 else None, where))
    if len(stack) != 1:
        raise ValueError(f"{path.name}: group '{stack[-1].name}' is missing its 'end'")
    return root.items


# ----------------------------------------------------------------------------
# Dump files
# ----------------------------------------------------------------------------

@dataclass
class Var:
    scope: str   # dotted scope path, e.g. TOP.tb_uart.axis_tx
    name: str    # e.g. tdata
    range: str   # e.g. [7:0], or '' for scalars
    width: int

    @property
    def full(self) -> str:  # the name GTKWave uses in save files
        return f"{self.scope}.{self.name}{self.range}"


@dataclass
class VcdHeader:
    timescale_fs: int
    defs: list    # ('scope', kind, name) | ('upscope',) | ('var', kind, width, id, ref_tokens)
    vars: list[Var]


TIME_UNITS_FS = {"s": 10**15, "ms": 10**12, "us": 10**9, "ns": 10**6, "ps": 10**3, "fs": 1}


def _parse_header(lines) -> VcdHeader:
    tokens = []
    for line in lines:
        tokens += line.split()
        if "$enddefinitions" in line:
            break
    timescale_fs, defs, variables, scope = 10**3, [], [], []
    i = 0
    while i < len(tokens):
        tok = tokens[i]
        end = tokens.index("$end", i)
        body = tokens[i + 1:end]
        if tok == "$scope":
            defs.append(("scope", body[0], body[1]))
            scope.append(body[1])
        elif tok == "$upscope":
            defs.append(("upscope",))
            scope.pop()
        elif tok == "$var":
            kind, width, ident, ref = body[0], int(body[1]), body[2], body[3:]
            defs.append(("var", kind, width, ident, ref))
            name = "".join(ref)
            m = re.match(r"^(.*?)(\[[^\]]*\])?$", name)
            variables.append(Var(".".join(scope), m.group(1), m.group(2) or "", width))
        elif tok == "$timescale":
            m = re.match(r"(\d+)\s*([a-z]+)", "".join(body))
            timescale_fs = int(m.group(1)) * TIME_UNITS_FS[m.group(2)]
        elif tok == "$enddefinitions":
            break
        i = end + 1
    return VcdHeader(timescale_fs, defs, variables)


def read_header(dump: Path, fst2vcd) -> VcdHeader:
    """Read the variable definitions of a VCD or FST dump."""
    if dump.suffix == ".vcd":
        with open(dump, encoding="utf-8", errors="replace") as f:
            return _parse_header(f)
    proc = subprocess.Popen(fst2vcd(dump), stdout=subprocess.PIPE, text=True, errors="replace")
    try:
        return _parse_header(proc.stdout)
    finally:
        proc.kill()


def _id_codes():
    """Unique VCD identifier codes: !, ", ..., ~, !!, !", ..."""
    chars = [chr(c) for c in range(33, 127)]
    n = 0
    while True:
        code, k = "", n
        while True:
            code = chars[k % len(chars)] + code
            k = k // len(chars) - 1
            if k < 0:
                break
        yield code
        n += 1


def _vcd_body(path: Path, idmap: dict, scale: int):
    """Yield (time, [value change lines]) per timestep, with ids remapped and time scaled."""
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if "$enddefinitions" in line:
                break
        t, buf = 0, []
        for line in f:
            line = line.strip()
            if not line or line[0] == "$":  # $dumpvars, $end, ...
                continue
            c = line[0]
            if c == "#":
                if buf:
                    yield t, buf
                    buf = []
                t = int(line[1:]) * scale
            elif c in "bBrRsS":
                value, ident = line.split(None, 1)
                buf.append(f"{value} {idmap[ident]}")
            else:
                buf.append(c + idmap[line[1:]])
        if buf:
            yield t, buf


def merge_vcds(inputs: list[tuple[str, Path]], out: Path):
    """Merge VCDs into one, nesting each input's hierarchy under a scope named by its label."""
    headers = []
    for _, p in inputs:
        with open(p, encoding="utf-8", errors="replace") as f:
            headers.append(_parse_header(f))
    unit = min(h.timescale_fs for h in headers)
    unit_name, unit_fs = next((n, fs) for n, fs in TIME_UNITS_FS.items() if unit % fs == 0)
    codes = _id_codes()
    idmaps = []
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write("$version Merged by sim/run.py $end\n")
        f.write(f"$timescale {unit // unit_fs}{unit_name} $end\n")
        for (label, _), h in zip(inputs, headers):
            idmap = {}
            f.write(f"$scope module {label} $end\n")
            for d in h.defs:
                if d[0] == "scope":
                    f.write(f"$scope {d[1]} {d[2]} $end\n")
                elif d[0] == "upscope":
                    f.write("$upscope $end\n")
                else:
                    _, kind, width, ident, ref = d
                    if ident not in idmap:
                        idmap[ident] = next(codes)
                    f.write(f"$var {kind} {width} {idmap[ident]} {' '.join(ref)} $end\n")
            f.write("$upscope $end\n")
            idmaps.append(idmap)
        f.write("$enddefinitions $end\n")

        bodies = [_vcd_body(p, idmap, h.timescale_fs // unit)
                  for (_, p), idmap, h in zip(inputs, idmaps, headers)]
        last_t = None
        for t, lines in heapq.merge(*bodies, key=lambda x: x[0]):
            if t != last_t:
                f.write(f"#{t}\n")
                last_t = t
            f.write("\n".join(lines) + "\n")


# ----------------------------------------------------------------------------
# Save files
# ----------------------------------------------------------------------------

def _pattern_regex(pattern: str) -> re.Pattern:
    out = ""
    i = 0
    while i < len(pattern):
        if pattern.startswith("**", i):
            out += ".*"
            i += 2
            continue
        c = pattern[i]
        out += "[^.]*" if c == "*" else "[^.]" if c == "?" else re.escape(c)
        i += 1
    return re.compile(out + "$")


def find_root(variables: list[Var], top: str) -> str:
    """Scope path of the testbench top module, e.g. TOP.tb_uart."""
    scopes = sorted({v.scope for v in variables}, key=lambda s: s.count("."))
    for s in scopes:
        if s.split(".")[-1] == top:
            return s
    return ""


def _resolve(sig: Signal, variables: list[Var], root: str, warn) -> list[Var]:
    rx = _pattern_regex(sig.pattern)
    prefix = root + "." if root else ""
    hits = [v for v in variables
            if f"{v.scope}.{v.name}".startswith(prefix) and rx.match(f"{v.scope}.{v.name}"[len(prefix):])]
    if not hits:
        warn(f"no signal matches '{sig.pattern}' ({sig.where})")
    return hits


def _emit(items: list, variables: list[Var], root: str, name_prefix: str, warn, out: list[str]):
    for item in items:
        if isinstance(item, Group):
            out += [f"@{TR_GRP_BEGIN | TR_BLANK:x}", f"-{item.name}"]
            _emit(item.items, variables, root, name_prefix, warn, out)
            out += [f"@{TR_GRP_END | TR_BLANK:x}", f"-{item.name}"]
        elif isinstance(item, Comment):
            out += [f"@{TR_BLANK:x}", f"-{item.text}"]
        else:
            for v in _resolve(item, variables, root, warn):
                fmt = item.fmt or ("bin" if v.width == 1 else "hex")
                out += [f"@{FORMATS[fmt]:x}", name_prefix + v.full]
                if fmt.startswith("analog"):
                    out += [f"@{TR_ANALOG_STRETCH:x}"] + ["-"] * ANALOG_EXTRA_ROWS


@dataclass
class WaveView:
    test: str
    top: str            # testbench top module name
    header: VcdHeader   # header of the test's own dump
    items: list         # parsed signal file


def write_save_file(views: list[WaveView], merged: bool, out: Path, warn):
    """Write a GTKWave save file with one top-level group per test."""
    lines = ["[*] Generated by sim/run.py - regenerated on every --waves/--view",
             "[timestart] 0", "[size] 1600 900", "[signals_width] 280",
             "[sst_expanded] 1", "[sst_vpaned_height] 260"]
    for v in views:
        root = find_root(v.header.vars, v.top)
        prefix = f"{v.test}." if merged else ""
        _emit([Group(v.test, v.items)], v.header.vars, root, prefix,
              lambda msg, t=v.test: warn(f"{t}: {msg}"), lines)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
