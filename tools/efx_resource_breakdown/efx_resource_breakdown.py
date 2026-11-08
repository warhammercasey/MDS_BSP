#!/usr/bin/env python3
"""
efx_resource_breakdown.py - hierarchical resource breakdown for Efinix (Trion/Titanium)
designs, built from the post-synthesis netlist that efx_map writes.

Works on designs that FAIL placement/resource checks, because it only needs the
synthesis output (outflow/<project>.map.v or, when the debugger is enabled,
outflow/<project>.dbg.map.v - the netlist efx_pnr actually packs).

How cells are attributed to the hierarchy
-----------------------------------------
Efinity flattens the design, but every primitive keeps a hierarchical instance
name, e.g.  \\ethernet_i/rtp_switch/g_in[0].packet_buffer/simple_dual_port_bram_i/ram__D$2
  * EFX_RAM_5K, EFX_MULT, EFX_FF, EFX_ADD : attributed exactly from their names.
  * EFX_LUT4 : LUT mapping renames almost all LUTs to flat 'LUT__<n>' names, so
    they are attributed by a majority vote over the hierarchical nets they touch,
    then by propagation through neighbouring cells. LUT counts are therefore an
    estimate (the same caveat Efinity puts on its own res.csv).

The module-instance tree comes from Efinity's <top>.res.csv / edb_top.res.csv
when present (gives module names + parameters); otherwise it is inferred from the
netlist names.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

# ----------------------------------------------------------------------------------------
# Device / primitive knowledge
# ----------------------------------------------------------------------------------------

MAIN_COLS = ["RAM_5K", "MULT", "LUT4", "FF", "ADD"]

# Trion EFX_RAM_5K: width -> depth for each supported configuration
RAM5K_DEPTH = {1: 4096, 2: 2048, 4: 1024, 5: 1024, 8: 512, 10: 512, 16: 256, 20: 256}
RAM5K_BITS = 5120

# Fallback capacities (used only when no place report is available). Keyed by device prefix.
DEVICE_CAPACITY = {
    "T20": {"RAM_5K": 204, "MULT": 36, "LE": 19728},
}

LUT_LEAF_RE = re.compile(r"^LUT__\d+$")
# Names of Verific operator/internal instances that are *not* module instances
# (only used when no res.csv is available to provide the real tree).
OPERATOR_SEG_RE = re.compile(
    r"^(?:[in]\d+|[A-Za-z]+_\d+|Select_\d+|LUT__\d+|CLKBUF__\d+|ceg_net\d+)$")
OUTPUT_PORTS = {"O", "CO", "Q", "RDATA"}

# ----------------------------------------------------------------------------------------
# Hierarchy tree
# ----------------------------------------------------------------------------------------


class Node:
    __slots__ = ("name", "module", "children", "parent", "self_counts", "incl", "depth")

    def __init__(self, name, module="", parent=None):
        self.name = name
        self.module = module
        self.children = {}
        self.parent = parent
        self.self_counts = Counter()
        self.incl = Counter()
        self.depth = 0 if parent is None else parent.depth + 1

    def child(self, name, module=""):
        c = self.children.get(name)
        if c is None:
            c = Node(name, module, self)
            self.children[name] = c
        elif module and not c.module:
            c.module = module
        return c

    def path(self):
        parts = []
        n = self
        while n.parent is not None:
            parts.append(n.name)
            n = n.parent
        return "/".join(reversed(parts))

    def is_within(self, anc):
        n = self
        while n is not None:
            if n is anc:
                return True
            n = n.parent
        return False

    def walk(self):
        yield self
        for c in self.children.values():
            yield from c.walk()


def parse_res_csv(path):
    """Parse Efinity's 'Module Resource Usage Distribution Estimates' CSV.
    Returns (root_instance, root_module, list of (depth, inst, module))."""
    rows = []
    with open(path, encoding="utf-8", errors="replace") as f:
        lines = f.read().splitlines()
    try:
        start = next(i for i, l in enumerate(lines) if l.startswith("Module\t"))
    except StopIteration:
        return None
    for line in lines[start + 1:]:
        if not line.strip():
            break
        name_field = line.split("\t", 1)[0].rstrip()
        stripped = name_field.lstrip(" ")
        depth = len(name_field) - len(stripped)
        stripped = stripped.lstrip("+")
        inst, _, module = stripped.partition(":")
        rows.append((depth, inst, module))
    if not rows:
        return None
    return rows


def tree_from_res_rows(rows, root):
    """Attach res.csv rows (row 0 is the root of the csv) under `root` node."""
    stack = [(0, root)]
    for depth, inst, module in rows[1:]:
        while stack and stack[-1][0] >= depth:
            stack.pop()
        parent = stack[-1][1] if stack else root
        n = parent.child(inst, module)
        stack.append((depth, n))


# ----------------------------------------------------------------------------------------
# Netlist parsing
# ----------------------------------------------------------------------------------------

INST_RE = re.compile(
    r"^[ \t]*(EFX_\w+)[ \t]+(\\\S+|[A-Za-z_][\w$]*)[ \t]*\((.*?)\)[ \t]*"
    r"(?:/\*(.*?)\*/)?[ \t]*;[ \t]*(?://[ \t]*([^\n]*))?$",
    re.M | re.S)
PORT_RE = re.compile(r"(?<=[\s(,])\.([A-Za-z_]\w*)\s*\(")
TOK_RE = re.compile(
    r"\\(\S+)(?:[ \t\n]*\[(\d+)(?::(\d+))?\])?"           # escaped identifier [+ select]
    r"|(\d*'[sS]?[bBhHdDoO][0-9a-fA-FxXzZ_?]+)"          # sized constant
    r"|(\d+)"                                              # bare number
    r"|([A-Za-z_][\w$]*)(?:[ \t\n]*\[(\d+)(?::(\d+))?\])?")  # simple identifier [+ select]
ATTR_RE = re.compile(r"(\w+)=(\"[^\"]*\"|[^,\s]+)")
MODULE_RE = re.compile(r"^\s*module\s+(\\\S+|\w+)", re.M)
DECL_RE = re.compile(r"^[ \t]*(?:wire|input|output|inout)[ \t]*\[(\d+):(\d+)\][ \t]*"
                     r"(?:\\(\S+)|([A-Za-z_][\w$]*))", re.M)


class Cell:
    __slots__ = ("type", "name", "ports", "attrs", "src", "node")

    def __init__(self, ctype, name, ports, attrs, src):
        self.type = ctype
        self.name = name
        self.ports = ports    # port -> list of (net_base, bit|None); constants dropped
        self.attrs = attrs
        self.src = src
        self.node = None


def parse_expr(expr, ranges):
    """Return (bits, const_bits): list of (base, bit) live bits and count of constant bits.
    `ranges` maps declared bus names to (msb, lsb) so whole-bus references expand."""
    bits = []
    const = 0
    for m in TOK_RE.finditer(expr):
        esc, emsb, elsb, cst, num, ident, imsb, ilsb = m.groups()
        if cst is not None:
            w = cst.split("'")[0]
            const += int(w) if w else 1
            continue
        if num is not None:
            continue
        base, msb, lsb = (esc, emsb, elsb) if esc is not None else (ident, imsb, ilsb)
        if msb is None:
            r = ranges.get(base)
            if r is None:
                bits.append((base, None))
                continue
            msb, lsb = r
        if lsb is None:
            bits.append((base, int(msb)))
        else:
            a, b = int(msb), int(lsb)
            step = -1 if a >= b else 1
            bits.extend((base, i) for i in range(a, b + step, step))
    return bits, const


def parse_netlist(path, want_attrs=("EFX_RAM_5K", "EFX_MULT")):
    with open(path, encoding="utf-8", errors="replace") as f:
        text = f.read()
    mods = MODULE_RE.findall(text)
    top = mods[0].lstrip("\\") if mods else "top"
    ranges = {(esc or ident): (int(a), int(b)) for a, b, esc, ident in DECL_RE.findall(text)}
    cells = []
    for m in INST_RE.finditer(text):
        ctype, name, port_txt, attr_txt, src = m.groups()
        name = name[1:] if name.startswith("\\") else name
        ports = {}
        pms = list(PORT_RE.finditer(" " + port_txt))
        for i, pm in enumerate(pms):
            end = pms[i + 1].start() if i + 1 < len(pms) else len(port_txt) + 1
            expr = (" " + port_txt)[pm.end():end]
            bits, const = parse_expr(expr, ranges)
            ports[pm.group(1)] = (bits, const)
        attrs = {}
        if attr_txt and ctype in want_attrs:
            for k, v in ATTR_RE.findall(attr_txt):
                if not k.startswith("INIT_"):
                    attrs[k] = v.strip('"')
        cells.append(Cell(ctype, name, ports, attrs, (src or "").strip()))
    return top, cells


# ----------------------------------------------------------------------------------------
# Attribution
# ----------------------------------------------------------------------------------------


class Resolver:
    """Maps hierarchical name prefixes to tree nodes."""

    def __init__(self, root, infer=False):
        self.root = root
        self.infer = infer
        self.cache = {}

    def node_for(self, segs):
        key = tuple(segs)
        n = self.cache.get(key)
        if n is not None:
            return n
        cur = self.root
        for s in segs:
            s = s.lstrip("~")          # inverted-signal nets are named '\~a/b/n12'
            if not s:
                continue
            nxt = cur.children.get(s)
            if nxt is not None:
                cur = nxt
            elif self.infer and not OPERATOR_SEG_RE.match(s):
                cur = cur.child(s)
            # else: operator / synthesis-generated level (e.g. add_12, i41) - skip it
        self.cache[key] = cur
        return cur

    def node_for_name(self, name):
        segs = name.split("/")
        return self.node_for(segs[:-1]), segs[-1]


def attribute(cells, resolver, max_net_fanout=48, rounds=8):
    root = resolver.root
    net_node = {}

    def nnode(base):
        r = net_node.get(base)
        if r is None:
            r = resolver.node_for_name(base)[0] if "/" in base else False
            net_node[base] = r
        return r

    pending = []
    for c in cells:
        anchor, leaf = resolver.node_for_name(c.name)
        if c.type == "EFX_LUT4" or LUT_LEAF_RE.match(leaf) or "/" not in c.name:
            if c.type in ("EFX_GBUFCE",):
                c.node = anchor
            else:
                pending.append((c, anchor))
        else:
            c.node = anchor

    # Pass 1: vote using hierarchical net names
    still = []
    for c, anchor in pending:
        votes = Counter()
        for port, (bits, _) in c.ports.items():
            w = 3 if port in OUTPUT_PORTS else 1
            for base in {b for b, _ in bits}:
                n = nnode(base)
                if n and n is not root and n.is_within(anchor):
                    votes[n] += w
        if votes:
            c.node = max(votes.items(), key=lambda kv: (kv[1], kv[0].depth))[0]
        else:
            still.append((c, anchor))

    if still:
        # Build net -> cells index over flat (non-hierarchical) nets with modest fanout
        net_cells = defaultdict(list)
        for c in cells:
            for bits, _ in c.ports.values():
                for base in {b for b, _ in bits}:
                    if "/" not in base:
                        net_cells[base].append(c)
        for _ in range(rounds):
            nxt = []
            for c, anchor in still:
                votes = Counter()
                for bits, _ in c.ports.values():
                    for base in {b for b, _ in bits}:
                        lst = net_cells.get(base)
                        if not lst or len(lst) > max_net_fanout:
                            continue
                        for o in lst:
                            if o is not c and o.node is not None and o.node is not root \
                                    and o.node.is_within(anchor):
                                votes[o.node] += 1
                if votes:
                    c.node = max(votes.items(), key=lambda kv: (kv[1], kv[0].depth))[0]
                else:
                    nxt.append((c, anchor))
            if len(nxt) == len(still):
                break
            still = nxt
        for c, anchor in still:
            c.node = anchor

    for c in cells:
        c.node.self_counts[c.type.replace("EFX_", "")] += 1

    def accumulate(n):
        n.incl = Counter(n.self_counts)
        for ch in n.children.values():
            accumulate(ch)
            n.incl.update(ch.incl)

    sys.setrecursionlimit(10000)
    accumulate(root)


# ----------------------------------------------------------------------------------------
# Memory analysis
# ----------------------------------------------------------------------------------------

RAM_SPLIT_RE = re.compile(r"__D\$\w+$")


def analyze_memories(cells, repo_root):
    groups = defaultdict(list)
    for c in cells:
        if c.type == "EFX_RAM_5K":
            groups[RAM_SPLIT_RE.sub("", c.name)].append(c)
    mems = []
    for name, blks in groups.items():
        wws = Counter(int(b.attrs.get("WRITE_WIDTH", 0) or 0) for b in blks)
        rws = Counter(int(b.attrs.get("READ_WIDTH", 0) or 0) for b in blks)
        wdata = set()
        rdata_bits = 0
        waddr_live = 0
        for b in blks:
            bits, _ = b.ports.get("WDATA", ([], 0))
            wdata.update(bits)
            rdata_bits += sum(1 for n, _ in b.ports.get("RDATA", ([], 0))[0]
                              if not n.startswith("Open_"))
            waddr_live = max(waddr_live, len({x for x in b.ports.get("WADDR", ([], 0))[0]}),
                             len({x for x in b.ports.get("RADDR", ([], 0))[0]}))
        ww = wws.most_common(1)[0][0]
        rw = rws.most_common(1)[0][0]
        mode_w = ww or rw
        blk_depth = RAM5K_DEPTH.get(mode_w, 0)
        rom = len(wdata) == 0
        width = len(wdata) if not rom else rdata_bits
        # how many blocks are stacked in depth
        per_row = max(1, math.ceil(width / mode_w)) if mode_w else 1
        slices = max(1, round(len(blks) / per_row))
        depth = blk_depth * slices if not rom else blk_depth
        if rom:
            width = max(1, rdata_bits // slices) if slices else rdata_bits
        phys = len(blks) * RAM5K_BITS
        src = blks[0].src
        mems.append({
            "name": name,
            "node": blks[0].node,
            "blocks": len(blks),
            "mode": f"{blk_depth}x{mode_w}" + ("" if ww == rw or not ww else f" (R{rw})"),
            "mixed_modes": len(wws) > 1,
            "rom": rom,
            "data_bits": width,
            "addr_bits": waddr_live,
            "capacity": f"{width}x{depth}",
            "capacity_bits": width * depth,
            "physical_bits": phys,
            "efficiency": (width * depth / phys) if phys else 0.0,
            "write_mode": blks[0].attrs.get("WRITE_MODE", ""),
            "output_reg": blks[0].attrs.get("OUTPUT_REG", ""),
            "source": shorten_src(src, repo_root),
        })
    mems.sort(key=lambda m: (-m["blocks"], m["name"]))
    return mems


def shorten_src(src, repo_root):
    m = re.match(r"(.*)\((\d+)\)\s*$", src)
    if not m:
        return src
    p, line = m.groups()
    if repo_root:
        try:
            rp = os.path.relpath(p, repo_root)
            if not rp.startswith(".."):
                p = rp
        except ValueError:
            pass
    return f"{p.replace(os.sep, '/')}:{line}"


# ----------------------------------------------------------------------------------------
# Project discovery
# ----------------------------------------------------------------------------------------


def find_inputs(target, netlist_kind):
    """Resolve the user's argument into (project_dir, outflow_dir, project_name, netlist)."""
    target = os.path.abspath(target)
    netlist = None
    if os.path.isfile(target) and target.endswith(".v"):
        netlist = target
        outflow = os.path.dirname(target)
        proj_dir = os.path.dirname(outflow)
        proj = os.path.basename(target).split(".")[0]
        return proj_dir, outflow, proj, netlist
    if os.path.isfile(target) and target.endswith(".xml"):
        proj_dir = os.path.dirname(target)
        proj = os.path.basename(target)[:-4]
    else:
        proj_dir = target
        if os.path.basename(proj_dir) == "outflow":
            proj_dir = os.path.dirname(proj_dir)
        xmls = [x for x in glob.glob(os.path.join(proj_dir, "*.xml"))
                if not x.endswith((".peri.xml",)) and "package_settings" not in x]
        if not xmls:
            sys.exit(f"error: no Efinity project (.xml) found in {proj_dir}")
        proj = os.path.basename(sorted(xmls, key=len)[0])[:-4]
    outflow = os.path.join(proj_dir, "outflow")
    plain = os.path.join(outflow, f"{proj}.map.v")
    dbg = os.path.join(outflow, f"{proj}.dbg.map.v")
    if netlist_kind == "user":
        netlist = plain
    elif netlist_kind == "debug":
        netlist = dbg
    else:
        # efx_pnr packs the debug netlist when the debugger is enabled; it is written
        # right after the plain netlist in the same run, so prefer it when it is current.
        if os.path.isfile(dbg) and (not os.path.isfile(plain) or
                                    os.path.getmtime(dbg) >= os.path.getmtime(plain) - 120):
            netlist = dbg
        else:
            netlist = plain
    if not os.path.isfile(netlist):
        sys.exit(f"error: netlist not found: {netlist}\n(run synthesis first)")
    return proj_dir, outflow, proj, netlist


def read_device(proj_dir, proj):
    try:
        tree = ET.parse(os.path.join(proj_dir, f"{proj}.xml"))
        for el in tree.iter():
            if el.tag.endswith("device") and el.get("name"):
                return el.get("name")
    except (OSError, ET.ParseError):
        pass
    return ""


def read_capacity(outflow, proj, device, netlist_mtime):
    cap, src = {}, ""
    rpt = os.path.join(outflow, f"{proj}.place.rpt")
    if os.path.isfile(rpt) and os.path.getmtime(rpt) >= netlist_mtime - 5:
        txt = open(rpt, encoding="utf-8", errors="replace").read()
        pats = {"RAM_5K": r"Memory Blocks:\s*\d+\s*/\s*(\d+)",
                "MULT": r"Multipliers:\s*\d+\s*/\s*(\d+)",
                "LE": r"Logic Elements:\s*\d+\s*/\s*(\d+)",
                "FF": r"LE: Registers:\s*\d+\s*/\s*(\d+)"}
        for k, p in pats.items():
            m = re.search(p, txt)
            if m:
                cap[k] = int(m.group(1))
        if cap:
            src = os.path.basename(rpt)
    if not cap:
        for prefix, c in DEVICE_CAPACITY.items():
            if device.upper().startswith(prefix):
                cap, src = dict(c), f"built-in table ({prefix})"
                break
    return cap, src


def build_tree(outflow, top, cells, use_res):
    """Build the hierarchy tree; returns (root, resolver, notes)."""
    notes = []
    root = Node(top, top)
    res_files = sorted(glob.glob(os.path.join(outflow, "*.res.csv"))) if use_res else []
    parsed = {}
    for f in res_files:
        rows = parse_res_csv(f)
        if rows:
            parsed.setdefault(rows[0][1], (f, rows))   # dedupe by root instance
    if top in parsed:
        tree_from_res_rows(parsed[top][1], root)
        root.module = parsed[top][1][0][2] or top
        top_level = {c.name.split("/", 1)[0] for c in cells if "/" in c.name}
        for rname, (f, rows) in parsed.items():
            if rname == top:
                continue
            # e.g. edb_top.res.csv is instantiated as 'edb_top_autodbg_inst__'
            cand = sorted(s for s in top_level if s == rname or s.startswith(rname + "_"))
            if cand:
                sub = root.child(cand[0], rows[0][2] or rname)
                tree_from_res_rows(rows, sub)
        infer = False
        notes.append("hierarchy from " + ", ".join(os.path.basename(f) for f, _ in parsed.values()))
    else:
        infer = True
        notes.append("no matching *.res.csv - hierarchy inferred from netlist names")
    return root, Resolver(root, infer=infer), notes


# ----------------------------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------------------------


def short_module(mod, full):
    if full or "(" not in mod:
        return mod
    return mod.split("(", 1)[0]


def visible(n, args):
    return n.incl.get("RAM_5K", 0) >= args.min_ram and (
        not args.hide_empty or any(n.incl.get(k, 0) for k in MAIN_COLS))


def sort_key(args):
    col = {"ram": "RAM_5K", "lut": "LUT4", "ff": "FF", "add": "ADD", "mult": "MULT"}.get(args.sort)
    if col is None:
        return lambda n: n.name.lower()
    return lambda n: (-n.incl.get(col, 0), n.name)


def ordered_children(n, args):
    ch = list(n.children.values())
    k = sort_key(args)
    if k:
        ch.sort(key=k)
    return ch


def render_text(root, mems, cap, info, args, out):
    w = out.write
    ram_cap = cap.get("RAM_5K")
    w(f"Netlist  : {info['netlist']}\n")
    if info.get("device"):
        w(f"Device   : {info['device']}\n")
    for n in info["notes"]:
        w(f"Note     : {n}\n")
    w("\n")

    # Totals
    tot = root.incl
    w(f"{'':12}" + "".join(f"{c:>9}" for c in MAIN_COLS) + "\n")
    w(f"{'used':12}" + "".join(f"{tot.get(c, 0):>9}" for c in MAIN_COLS) + "\n")
    if cap:
        cap_row = {"RAM_5K": cap.get("RAM_5K"), "MULT": cap.get("MULT"), "FF": cap.get("FF")}
        w(f"{'available':12}" + "".join(
            f"{cap_row.get(c) if cap_row.get(c) else '-':>9}" for c in MAIN_COLS) + "\n")
        w(f"{'util':12}" + "".join(
            f"{(f'{100.0 * tot.get(c, 0) / cap_row[c]:.1f}%') if cap_row.get(c) else '':>9}"
            for c in MAIN_COLS) + "\n")
        if cap.get("LE"):
            le = tot.get("LUT4", 0) + tot.get("ADD", 0)
            w(f"(LUT4+ADD = {le} of {cap['LE']} LEs, {100.0 * le / cap['LE']:.1f}%;"
              f" capacity from {info['cap_src']})\n")
        if ram_cap and tot.get("RAM_5K", 0) > ram_cap:
            w(f"*** RAM_5K over capacity by {tot['RAM_5K'] - ram_cap} blocks ***\n")
    w("\n")

    # Tree
    rows = []

    def rec(n, prefix, is_last, is_root):
        if not is_root and not visible(n, args):
            return
        if args.max_depth is not None and n.depth > args.max_depth:
            return
        label = n.name if is_root else prefix + ("└─ " if is_last else "├─ ") + n.name
        rows.append((label, short_module(n.module, args.params), n))
        kids = [c for c in ordered_children(n, args) if visible(c, args)]
        if args.max_depth is not None and n.depth + 1 > args.max_depth:
            kids = []
        child_prefix = "" if is_root else prefix + ("   " if is_last else "│  ")
        for i, c in enumerate(kids):
            rec(c, child_prefix, i == len(kids) - 1, False)

    rec(root, "", True, True)
    lw = min(max(len(r[0]) for r in rows), args.width)
    mw = min(max(len(r[1]) for r in rows), args.width)
    hdr = f"{'Instance':<{lw}}  {'Module':<{mw}}  {'RAM_5K':>6} {'%dev':>6} {'MULT':>5} " \
          f"{'LUT4':>6} {'FF':>6} {'ADD':>6}   self(RAM/LUT/FF)"
    w(hdr + "\n" + "-" * len(hdr) + "\n")
    for label, mod, n in rows:
        i = n.incl
        pct = f"{100.0 * i.get('RAM_5K', 0) / ram_cap:5.1f}%" if ram_cap else ""
        s = n.self_counts
        selfs = f"{s.get('RAM_5K', 0)}/{s.get('LUT4', 0)}/{s.get('FF', 0)}" if s else ""
        w(f"{trunc(label, lw):<{lw}}  {trunc(mod, mw):<{mw}}  {i.get('RAM_5K', 0):>6} {pct:>6} "
          f"{i.get('MULT', 0):>5} {i.get('LUT4', 0):>6} {i.get('FF', 0):>6} {i.get('ADD', 0):>6}"
          f"   {selfs}\n")
    w("\nCounts are inclusive of children. LUT4 attribution is estimated (see README).\n")

    # Memories
    if mems and not args.no_memories:
        w("\nLogical memories (EFX_RAM_5K primitives grouped per inferred RAM), largest first\n")
        hdr = f"{'blocks':>6} {'%dev':>6}  {'mode':<8} {'shape':<9} {'fill':>4}  memory / owner module @ source"
        w(hdr + "\n" + "-" * len(hdr) + "\n")
        pad = " " * 40
        for m in mems:
            if m["blocks"] < args.min_ram:
                continue
            node = m["node"]
            pct = f"{100.0 * m['blocks'] / ram_cap:5.1f}%" if ram_cap else ""
            w(f"{m['blocks']:>6} {pct:>6}  {m['mode']:<8} {m['capacity']:<9}"
              f" {100 * m['efficiency']:>3.0f}%  {m['name']}{'  (ROM)' if m['rom'] else ''}\n")
            w(f"{pad}{short_module(node.module, True) or node.name}"
              f"{' @ ' + m['source'] if m['source'] else ''}\n")
        w("\nshape = live data bits x addressable depth of the blocks used;  "
          f"fill = shape bits / ({RAM5K_BITS} x blocks).\nTrion RAM_5K modes: " +
          ", ".join(f"{d}x{wd}" for wd, d in sorted(RAM5K_DEPTH.items())) +
          " - the x1/x2/x4/x8/x16 modes only reach 4096 of the 5120 bits.\n")


def trunc(s, n):
    return s if len(s) <= n else s[:n - 1] + "…"


def tree_to_dict(n):
    return {
        "instance": n.name,
        "path": n.path(),
        "module": n.module,
        "inclusive": dict(n.incl),
        "self": dict(n.self_counts),
        "children": [tree_to_dict(c) for c in n.children.values()],
    }


def write_csv(root, path, cap):
    cols = sorted({k for n in root.walk() for k in n.incl}, key=lambda k: (
        MAIN_COLS.index(k) if k in MAIN_COLS else 99, k))
    with open(path, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["path", "depth", "instance", "module"] + cols +
                    [f"self_{c}" for c in cols] +
                    (["RAM_5K_pct_device"] if cap.get("RAM_5K") else []))
        for n in root.walk():
            row = [n.path() or n.name, n.depth, n.name, n.module]
            row += [n.incl.get(c, 0) for c in cols]
            row += [n.self_counts.get(c, 0) for c in cols]
            if cap.get("RAM_5K"):
                row.append(round(100.0 * n.incl.get("RAM_5K", 0) / cap["RAM_5K"], 2))
            wr.writerow(row)


# ----------------------------------------------------------------------------------------


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Hierarchical resource (BRAM/LUT/FF/ADD/MULT) breakdown of an Efinix "
                    "synthesized netlist - works even when place & route fails on resources.")
    ap.add_argument("target", nargs="?", default=".",
                    help="Efinity project dir, project .xml, outflow dir, or a *.map.v netlist "
                         "(default: current dir)")
    ap.add_argument("--netlist", choices=["auto", "user", "debug"], default="auto",
                    help="auto: use <proj>.dbg.map.v (what PnR packs) when it is current; "
                         "user: <proj>.map.v without the debug core; debug: force dbg netlist")
    ap.add_argument("--sort", choices=["ram", "lut", "ff", "add", "mult", "name"], default="ram",
                    help="child ordering (default: ram)")
    ap.add_argument("--min-ram", type=int, default=0, metavar="N",
                    help="hide subtrees using fewer than N RAM_5K blocks")
    ap.add_argument("--hide-empty", action="store_true",
                    help="hide subtrees with no counted resources")
    ap.add_argument("--max-depth", type=int, default=None, metavar="D",
                    help="limit tree depth (0 = top only)")
    ap.add_argument("--params", action="store_true",
                    help="show module parameters (from res.csv) in the tree")
    ap.add_argument("--width", type=int, default=48, help="max column width (default 48)")
    ap.add_argument("--no-memories", action="store_true", help="omit the memory detail section")
    ap.add_argument("--no-res-csv", action="store_true",
                    help="ignore Efinity's *.res.csv and infer the hierarchy from netlist names")
    ap.add_argument("-o", "--output", help="write the text report to this file instead of stdout")
    ap.add_argument("--csv", metavar="FILE", help="also write the flattened tree as CSV")
    ap.add_argument("--json", metavar="FILE", help="also write tree + memories as JSON")
    args = ap.parse_args(argv)

    proj_dir, outflow, proj, netlist = find_inputs(args.target, args.netlist)
    repo_root = os.path.dirname(proj_dir)
    top, cells = parse_netlist(netlist)
    if not cells:
        sys.exit(f"error: no EFX_* primitives found in {netlist}")

    root, resolver, notes = build_tree(outflow, top, cells, not args.no_res_csv)
    nl_mtime = os.path.getmtime(netlist)
    for f in glob.glob(os.path.join(outflow, "*.res.csv")):
        if not args.no_res_csv and os.path.getmtime(f) < nl_mtime - 300:
            notes.append(f"{os.path.basename(f)} is older than the netlist - "
                         "re-synthesize or use --no-res-csv if the hierarchy changed")
    attribute(cells, resolver)
    mems = analyze_memories(cells, repo_root)

    device = read_device(proj_dir, proj)
    cap, cap_src = read_capacity(outflow, proj, device, nl_mtime)
    info = {"netlist": os.path.relpath(netlist, os.getcwd()) if os.path.splitdrive(netlist)[0]
            .lower() == os.path.splitdrive(os.getcwd())[0].lower() else netlist,
            "device": device, "notes": notes, "cap_src": cap_src}

    out = open(args.output, "w", encoding="utf-8") if args.output else sys.stdout
    if out is sys.stdout:
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass
    try:
        render_text(root, mems, cap, info, args, out)
    finally:
        if out is not sys.stdout:
            out.close()

    if args.csv:
        write_csv(root, args.csv, cap)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({
                "netlist": netlist, "device": device, "capacity": cap,
                "tree": tree_to_dict(root),
                "memories": [dict(m, node=m["node"].path()) for m in mems],
            }, f, indent=1)


if __name__ == "__main__":
    main()
