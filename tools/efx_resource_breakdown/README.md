# efx_resource_breakdown

Hierarchical resource report (RAM_5K / MULT / LUT4 / FF / ADD) for Efinix Trion designs,
built from the **post-synthesis netlist**. Because it only needs synthesis output, it works
on designs that fail placement because they are over capacity, which is exactly when
Efinity's own reports stop being useful.

```
python tools/efx_resource_breakdown/efx_resource_breakdown.py CAIN_project
```

No dependencies beyond Python 3.8+. A run takes about a second.

## What it reads

| File (in `outflow/`) | Used for |
|---|---|
| `<proj>.dbg.map.v` (debugger enabled) or `<proj>.map.v` | The flattened netlist: every `EFX_*` primitive, its hierarchical name, its port nets and its attributes |
| `*.res.csv` (`cain_top.res.csv`, `edb_top.res.csv`) | Module-instance tree with module names and parameters (optional; see `--no-res-csv`) |
| `<proj>.place.rpt` | Device capacities (only if the report is newer than the netlist), with a built-in T20 fallback |

By default the tool picks `dbg.map.v` when it is current, because that is the netlist
`efx_pnr` actually packs (your design plus the Debugger's logic analyzer). Pass
`--netlist user` to see the design without the debug core.

## How the attribution works

Efinity flattens the design, but each primitive keeps its hierarchical instance name, e.g.
`\ethernet_i/rtp_switch/g_in[0].packet_buffer/simple_dual_port_bram_i/ram__D$2`.
Names are matched against the instance tree. Synthesis-generated levels such as `add_12/`,
`i41/` and `~` are skipped.

* **RAM_5K and MULT** are exact. They match Efinity's `res.csv` on every node and match
  the place report's totals.
* **FF and ADD** are attributed from their names, which Verific derives from the net they
  drive. A register that drives a port or interface signal of its parent can therefore
  land in the parent.
* **LUT4** cells are renamed to flat `LUT__<n>` during LUT mapping, so the tool estimates
  their location. Each LUT goes to the module that owns most of the hierarchical nets it
  touches; LUTs that touch none are placed by propagation through neighbouring LUTs.
  Treat module-level LUT numbers as about ±10%.

## Output

1. **Totals versus device capacity**, with an over-capacity warning.
2. **A hierarchy tree** with inclusive counts, `%dev` (the share of the device's RAM
   blocks that subtree uses) and self counts. Children are sorted by RAM by default.
3. **Logical memories.** RAM_5K primitives are grouped back into the inferred RAM, then
   listed with:
   * `mode`: the RAM_5K configuration that synthesis chose, for example `2048x2`.
   * `shape`: live data bits × the addressable depth of the blocks used.
   * `fill`: shape bits ÷ (5120 × blocks).
   * The owning module with its parameters, and the source line.

   This section shows why a memory costs so many blocks. Trion RAM_5K modes are
   4096x1, 2048x2, 1024x4/5, 512x8/10 and 256x16/20. Exceeding a power-of-two depth by a
   single entry moves the RAM into a narrower, deeper mode and can double its block count.

## Options

```
target                 project dir, project .xml, outflow dir, or a *.map.v file
--netlist auto|user|debug
--sort ram|lut|ff|add|mult|name
--min-ram N            hide subtrees (and memories) using fewer than N RAM_5K blocks
--hide-empty           hide subtrees with no resources
--max-depth D          limit tree depth
--params               show module parameters in the tree
--width N              max column width (default 48)
--no-memories          skip the memory section
--no-res-csv           infer the hierarchy from netlist names only
-o FILE                write the text report to a file
--csv FILE             flattened tree (inclusive and self counts per node)
--json FILE            tree and memories
```

Common uses:

```
# Only what uses BRAM
python tools/efx_resource_breakdown/efx_resource_breakdown.py CAIN_project --min-ram 1

# The design without the debug core
python tools/efx_resource_breakdown/efx_resource_breakdown.py CAIN_project --netlist user
```

## Why not VTR?

Efinity's place and route is derived from VPR, but its intermediate files are proprietary
and not VTR's text formats. The `.vdb` is a binary Verific database, and `.netlist` is
compressed. The packed netlist also only exists after packing succeeds. The `*.map.v`
Verilog that `efx_map` always writes has the same information in readable form, so the
tool uses that.
