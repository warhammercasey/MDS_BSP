"""Write direction_estimator's least-squares matrix.

    python src/dsp/building_blocks/gen_direction_matrix.py [--coef-frac 23] [--fs HZ]
                                                           [--locations "x,y,z; x,y,z; ..."]

Same least squares as model/time_correlation_model.m: for each pair a < b (in the
correlator's order: (0,1), (0,2), .., (1,2), ..), row (loc_a - loc_b) of A, and
c * delay_ab on the right, where delay_ab is how far channel b lags channel a. The
direction is A \\ (c * delays) = pinv(A) * (c / fs) * lags, with lags in samples, so the
matrix written is M = (c / fs) * pinv(A): 3 rows (x, y, z) by one column per pair.

Each entry is round(M * 2^coef_frac) as an 18-bit two's complement word, 5 hex digits per
line, row-major (all the x coefficients, then y, then z): the rom's $readmemh format.
coef_frac must match direction_estimator's COEF_FRAC. The script stops if an entry
doesn't fit and says what the largest coef_frac that fits is.

The default locations are the 16 cm cube from the MATLAB model, in channel order. Edit
MIC_LOCATIONS (or pass --locations) when the array changes. It writes direction_matrix.hex
next to this script.
"""
import argparse
import math
from pathlib import Path

COEF_WIDTH = 18
SOUND_SPEED = 343.0                  # m/s
FS = 73_571_400 / (24 * 64) / 4      # clk_io / SCK divider / 64 SCK per frame / decimation

_S = 160e-3 / 2
MIC_LOCATIONS = [  # (x, y, z) in metres, one per channel
    (-_S, -_S, -_S),
    (-_S,  _S, -_S),
    (-_S, -_S,  _S),
    (-_S,  _S,  _S),
    ( _S, -_S,  _S),
    ( _S,  _S,  _S),
    ( _S, -_S, -_S),
    ( _S,  _S, -_S),
]


def inverse3(m):
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        raise SystemExit("A'A is singular: the mic locations don't span 3 dimensions")
    adj = [[e * i - f * h, c * h - b * i, b * f - c * e],
           [f * g - d * i, a * i - c * g, c * d - a * f],
           [d * h - e * g, b * g - a * h, a * e - b * d]]
    return [[x / det for x in row] for row in adj]


def direction_matrix(locations, fs, c=SOUND_SPEED):
    """(c / fs) * pinv(A), 3 rows by one column per pair."""
    n = len(locations)
    rows = [[locations[a][k] - locations[b][k] for k in range(3)]
            for a in range(n - 1) for b in range(a + 1, n)]
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(3)] for i in range(3)]
    inv = inverse3(ata)
    return [[c / fs * sum(inv[k][j] * r[j] for j in range(3)) for r in rows] for k in range(3)]


def parse_locations(text):
    locs = [tuple(float(v) for v in item.split(",")) for item in text.split(";") if item.strip()]
    if any(len(l) != 3 for l in locs):
        raise SystemExit("--locations wants x,y,z triples separated by ';'")
    return locs


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--coef-frac", type=int, default=23, help="fraction bits (direction_estimator COEF_FRAC)")
    ap.add_argument("--fs", type=float, default=FS, help=f"sample rate at the correlator (default {FS:.2f} Hz)")
    ap.add_argument("--locations", help="mic locations in metres, 'x,y,z; x,y,z; ...' in channel order")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "direction_matrix.hex"))
    args = ap.parse_args()

    locations = parse_locations(args.locations) if args.locations else MIC_LOCATIONS
    m = direction_matrix(locations, args.fs)
    biggest = max(abs(v) for row in m for v in row)
    limit = (1 << (COEF_WIDTH - 1)) - 1
    if round(biggest * (1 << args.coef_frac)) > limit:
        fits = math.floor(math.log2(limit / biggest))
        raise SystemExit(f"coef-frac {args.coef_frac} overflows {COEF_WIDTH} bits; the largest that fits is {fits}")

    mask = (1 << COEF_WIDTH) - 1
    words = [round(v * (1 << args.coef_frac)) & mask for row in m for v in row]
    Path(args.out).write_text("".join(f"{w:05x}\n" for w in words), encoding="ascii", newline="\n")
    pairs = len(m[0])
    print(f"{args.out}: {len(locations)} mics, {pairs} pairs, coef-frac {args.coef_frac}, "
          f"largest |coefficient| {biggest:.5f} ({round(biggest * (1 << args.coef_frac))} of {limit})")


if __name__ == "__main__":
    main()
