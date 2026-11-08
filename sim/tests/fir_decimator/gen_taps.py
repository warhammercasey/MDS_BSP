"""Write the example tap files for the fir_decimator test.

    python sim/tests/fir_decimator/gen_taps.py

Files are in the rom's $readmemh format: one 18-bit two's complement word per line,
5 hex digits, no comments or prefixes.

  taps_lowpass.hex  32-tap Hamming-windowed sinc lowpass for decimation by 4: cutoff
                    0.1 fs (the output band edge is 0.125 fs). Symmetric (linear phase).
  taps_ramp.hex     20 taps, (-1)^k (k+1) * 1024. Not a useful filter: it's asymmetric
                    and every tap is different, so taps applied in the wrong order or
                    to the wrong sample show up.

fir_decimator keeps the top 18 bits of a 36-bit accumulator, so its gain is
sum(taps) / 2^18. The lowpass is scaled so sum(|taps|) <= 2^18 - 1: then even a
worst-case full-scale input can't overflow the accumulator. The negative lobes make
sum(|taps|) about 1.33 x sum(taps), so the DC gain comes out at 0.75.
"""
import math
from pathlib import Path

WIDTH = 18
HERE = Path(__file__).resolve().parent


def lowpass(taps, cutoff):
    mid = (taps - 1) / 2
    h = []
    for k in range(taps):
        t = k - mid
        sinc = 2 * cutoff if t == 0 else math.sin(2 * math.pi * cutoff * t) / (math.pi * t)
        h.append(sinc * (0.54 - 0.46 * math.cos(2 * math.pi * k / (taps - 1))))
    limit = (1 << WIDTH) - 1
    scale = limit / sum(abs(x) for x in h)
    q = [round(x * scale) for x in h]
    while sum(abs(x) for x in q) > limit:  # rounding can push it over
        scale *= 0.9999
        q = [round(x * scale) for x in h]
    return q


def write(name, taps):
    lo, hi = -(1 << (WIDTH - 1)), (1 << (WIDTH - 1)) - 1
    assert all(lo <= t <= hi for t in taps), name
    mask = (1 << WIDTH) - 1
    (HERE / name).write_text("".join(f"{t & mask:05x}\n" for t in taps), encoding="ascii", newline="\n")
    print(f"{name}: {len(taps)} taps, sum {sum(taps)} (DC gain {sum(taps) / (1 << WIDTH):.4f}), "
          f"sum |h| {sum(abs(t) for t in taps)}")


write("taps_lowpass.hex", lowpass(32, 0.1))
write("taps_ramp.hex", [(-1) ** k * (k + 1) * 1024 for k in range(20)])
