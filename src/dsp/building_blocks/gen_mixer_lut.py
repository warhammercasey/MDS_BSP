"""Write the mixer's quarter-wave LO table.

    python src/dsp/building_blocks/gen_mixer_lut.py [--awidth 8]

Entry k holds {cos(phi), sin(phi)} for phi = (pi/2) * k / 2^awidth, each an 18-bit two's
complement word scaled to 2^17 - 1 (so it can be negated without overflow), cos in the
upper half. That's one 36-bit word, 9 hex digits, per line, in the rom's $readmemh
format. mixer gets the other three quadrants by swapping and negating the two halves,
so the LO has 2^(awidth + 2) points per cycle.

The default (awidth 8, 256 entries) matches mixer's default LUT_AWIDTH. It writes
mixer_lut.hex next to this script.
"""
import argparse
import math
from pathlib import Path

LO_WIDTH = 18
AMPLITUDE = (1 << (LO_WIDTH - 1)) - 1


def quantize(x):
    return math.floor(AMPLITUDE * x + 0.5)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--awidth", type=int, default=8, help="log2 of the table length (mixer LUT_AWIDTH)")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "mixer_lut.hex"))
    args = ap.parse_args()

    n = 1 << args.awidth
    mask = (1 << LO_WIDTH) - 1
    lines = []
    for k in range(n):
        phi = math.pi / 2 * k / n
        c, s = quantize(math.cos(phi)), quantize(math.sin(phi))
        lines.append(f"{((c & mask) << LO_WIDTH) | (s & mask):09x}\n")
    Path(args.out).write_text("".join(lines), encoding="ascii", newline="\n")
    print(f"{args.out}: {n} entries, {4 * n} points per cycle")


if __name__ == "__main__":
    main()
