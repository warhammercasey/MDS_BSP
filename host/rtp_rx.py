"""Receive the FPGA's RTP streams: report packet counts and data rates, record the mics, and plot the directions.

The FPGA sends its RTP streams to one UDP port (5000, to broadcast, by default; see
udp_stack_regs.sv), told apart by SSRC ("ADC0", "MIC0" and "DIR0", see cain_top.sv).
Each mic is saved to its own mono 24-bit .wav, the direction estimates are plotted live
in a window, and ADC samples are dropped for now.

    python host/rtp_rx.py [--port 5000] [--interval 1] [--wav-dir DIR | --no-wav]
                          [--window 10] [--smooth 20] [--no-plot]

The plot needs matplotlib (pip install matplotlib). Close the window to stop; with
--no-plot it runs in the terminal only, until Ctrl-C.

Rates count UDP payload bytes (RTP header + samples). Lost packets are counted from
gaps in each stream's RTP sequence numbers.

Mic packets hold whole frames, channel 0 first (see rtp_packetizer.sv). Each sample is
L24: 3 bytes big endian, the 18-bit mic sample left justified, so it is written as is.
Frames missing between packets (by RTP timestamp, which counts frames) are written as
silence so the recording keeps time.

Direction packets hold whole {x, y, z} vectors (see direction_estimator.sv): 16-bit big
endian Q2.14, in the mic array's coordinates (MIC_LOCATIONS below). The RTP timestamp
counts directions, one per correlation window, which places them in time.
"""
import argparse
import collections
import math
import os
import socket
import struct
import threading
import time
import wave

RTP_HDR = struct.Struct("!BBHII")  # V/P/X/CC, M/PT, sequence, timestamp, SSRC

MIC_SSRC = 0x4D494330  # "MIC0", cain_top MIC_RTP_SSRC
MIC_CHANNELS = 8
MIC_SAMPLE_BYTES = 3
MIC_FRAME_BYTES = MIC_CHANNELS * MIC_SAMPLE_BYTES
# clk_io / SCK divider (even, so SCK <= I2S_CLK_FREQ = 3.072 MHz) / 64 SCK per frame
MIC_RATE = 73_571_400 // 24 // 64  # 47898 Hz
MAX_GAP_FRAMES = 10 * MIC_RATE  # a bigger timestamp jump is a restart, not lost packets

DIR_SSRC = 0x44495230  # "DIR0", cain_top DIR_RTP_SSRC
DIR_VECTOR = struct.Struct(">hhh")
DIR_FRAC = 14  # DSP_DF_OUT_FRAC
# Mic rate / decimation (DSP_DECIMATION) / correlation step (DSP_XCORR_WINDOW_STEP)
DIR_RATE = 73_571_400 / 24 / 64 / 4 / 16  # 374.2 directions/s

# Mic positions in metres, in channel order, as model/time_correlation_model.m and
# gen_direction_matrix.py. Only drawn, for orientation.
_S = 0.08
MIC_LOCATIONS = [(-_S, -_S, -_S), (-_S, _S, -_S), (-_S, -_S, _S), (-_S, _S, _S),
                 (_S, -_S, _S), (_S, _S, _S), (_S, -_S, -_S), (_S, _S, -_S)]


class Stream:
    def __init__(self):
        self.packets = 0
        self.bytes = 0
        self.lost = 0
        self.next_seq = None
        self.first = None      # Time of the first packet
        self.last_packets = 0  # Counts at the last report
        self.last_bytes = 0

    def add(self, seq, length, now):
        if self.first is None:
            self.first = now
        elif seq != self.next_seq:
            gap = (seq - self.next_seq) & 0xFFFF
            if gap < 0x8000:  # Otherwise it's late or repeated, not a gap
                self.lost += gap
        self.next_seq = (seq + 1) & 0xFFFF
        self.packets += 1
        self.bytes += length


class MicRecorder:
    """Writes each mic channel to <dir>/mic<N>_<start time>.wav."""

    def __init__(self, directory, rate):
        os.makedirs(directory, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.paths = [os.path.join(directory, f"mic{ch}_{stamp}.wav") for ch in range(MIC_CHANNELS)]
        self.rate = rate
        self.files = []
        for path in self.paths:
            w = wave.open(path, "wb")
            w.setnchannels(1)
            w.setsampwidth(MIC_SAMPLE_BYTES)
            w.setframerate(rate)
            self.files.append(w)
        self.next_ts = None
        self.frames = 0     # Frames written, silence included
        self.silence = 0    # Frames of silence written for gaps
        self.bad = 0        # Packets dropped for a payload that isn't whole frames

    def add(self, timestamp, payload):
        if len(payload) == 0 or len(payload) % MIC_FRAME_BYTES:
            self.bad += 1
            return
        frames = len(payload) // MIC_FRAME_BYTES

        if self.next_ts is not None:
            gap = (timestamp - self.next_ts) & 0xFFFFFFFF
            if 0 < gap <= MAX_GAP_FRAMES:
                silence = bytes(gap * MIC_SAMPLE_BYTES)
                for w in self.files:
                    w.writeframes(silence)
                self.frames += gap
                self.silence += gap
        self.next_ts = (timestamp + frames) & 0xFFFFFFFF

        # Big endian to the little endian .wav wants
        le = bytearray(len(payload))
        le[0::3] = payload[2::3]
        le[1::3] = payload[1::3]
        le[2::3] = payload[0::3]
        # Pick each channel's samples out of the interleaved frames
        mono = bytearray(frames * MIC_SAMPLE_BYTES)
        for ch, w in enumerate(self.files):
            for b in range(MIC_SAMPLE_BYTES):
                mono[b::MIC_SAMPLE_BYTES] = le[ch * MIC_SAMPLE_BYTES + b::MIC_FRAME_BYTES]
            w.writeframes(mono)
        self.frames += frames

    def close(self):
        for w in self.files:
            w.close()


class DirectionTrack:
    """The latest direction estimates, as (seconds, x, y, z), for the plot.

    Filled by the receive thread and read by the GUI: deque appends and copies are safe
    across threads without a lock.
    """

    def __init__(self, seconds, rate=DIR_RATE):
        self.rate = rate
        self.points = collections.deque(maxlen=int(seconds * rate) + 1)
        self.count = 0
        self.bad = 0         # Packets dropped for a payload that isn't whole vectors
        self.restarts = 0    # Timestamp jumps (FPGA reset): time carries on from where it was
        self._first_ts = None
        self._base = 0.0     # Time at _first_ts
        self._last_t = 0.0

    def add(self, timestamp, payload):
        if len(payload) == 0 or len(payload) % DIR_VECTOR.size:
            self.bad += 1
            return
        if self._first_ts is None:
            self._first_ts = timestamp
        t = self._base + ((timestamp - self._first_ts) & 0xFFFFFFFF) / self.rate
        if t < self._last_t or t > self._last_t + 10.0:
            # Carry on a step after the last direction
            if self.count:
                self.restarts += 1
                self._base = self._last_t + 1.0 / self.rate
            self._first_ts = timestamp
            t = self._base
        scale = 1.0 / (1 << DIR_FRAC)
        for i, (x, y, z) in enumerate(DIR_VECTOR.iter_unpack(payload)):
            self._last_t = t + i / self.rate
            self.points.append((self._last_t, x * scale, y * scale, z * scale))
            self.count += 1


def ssrc_name(ssrc):
    raw = ssrc.to_bytes(4, "big")
    if all(0x20 <= b < 0x7F for b in raw):
        return raw.decode("ascii")
    return f"0x{ssrc:08X}"


def mbps(nbytes, seconds):
    return nbytes * 8 / seconds / 1e6 if seconds > 0 else 0.0


def report(streams, other, now, interval):
    for ssrc, s in sorted(streams.items()):
        new_packets = s.packets - s.last_packets
        new_bytes = s.bytes - s.last_bytes
        s.last_packets, s.last_bytes = s.packets, s.bytes
        print(f"{ssrc_name(ssrc):>10}: {s.packets:9d} packets  {s.lost:6d} lost  "
              f"now {new_packets / interval:7.1f} pkt/s {mbps(new_bytes, interval):7.3f} Mbit/s  "
              f"avg {mbps(s.bytes, now - s.first):7.3f} Mbit/s")
    if other:
        print(f"{'non-RTP':>10}: {other:9d} packets")


class Receiver:
    """Reads the socket until stop is set, sorting packets into stats, recordings and the
    direction track, and printing a report every interval."""

    def __init__(self, sock, interval, recorder, track):
        self.sock = sock
        self.interval = interval
        self.recorder = recorder
        self.track = track
        self.streams = {}
        self.other = 0
        self.stop = threading.Event()

    def run(self):
        buf = bytearray(2048)
        next_report = time.monotonic() + self.interval
        while not self.stop.is_set():
            try:
                length = self.sock.recv_into(buf)
            except socket.timeout:
                length = None
            now = time.monotonic()

            if length is not None:
                if length >= RTP_HDR.size and buf[0] >> 6 == 2:
                    _, _, seq, timestamp, ssrc = RTP_HDR.unpack_from(buf)
                    self.streams.setdefault(ssrc, Stream()).add(seq, length, now)
                    if self.recorder and ssrc == MIC_SSRC:
                        self.recorder.add(timestamp, bytes(buf[RTP_HDR.size:length]))
                    elif ssrc == DIR_SSRC:
                        self.track.add(timestamp, bytes(buf[RTP_HDR.size:length]))
                else:
                    self.other += 1

            if now >= next_report:
                report(self.streams, self.other, now, self.interval + now - next_report)
                next_report = now + self.interval


# ----------------------------------------------------------------------------
# Plot
# ----------------------------------------------------------------------------
def make_plot(track, window, smooth):
    """The direction window: a 3D view of the direction and x, y, z over time.

    Returns (figure, update), update() redrawing it from track."""
    import matplotlib.pyplot as plt

    fig = plt.figure("CAIN direction", figsize=(12, 6))
    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    axes_t = [fig.add_subplot(3, 2, 2 * k + 2) for k in range(3)]

    # The mic array, scaled up to fit around the unit vectors, for orientation
    k = 0.5 / _S
    for i, (x, y, z) in enumerate(MIC_LOCATIONS):
        ax3.scatter([x * k], [y * k], [z * k], color="tab:gray", s=12)
        ax3.text(x * k, y * k, z * k, f" {i}", color="tab:gray", fontsize=8)
    for a, b in [(0, 1), (0, 2), (0, 6), (1, 3), (1, 7), (2, 3), (2, 4), (3, 5), (4, 5), (4, 6), (5, 7), (6, 7)]:
        ax3.plot(*zip([c * k for c in MIC_LOCATIONS[a]], [c * k for c in MIC_LOCATIONS[b]]),
                 color="tab:gray", linewidth=0.5)
    for axis in (ax3.set_xlim, ax3.set_ylim, ax3.set_zlim):
        axis(-1.2, 1.2)
    ax3.set_box_aspect((1, 1, 1))
    ax3.set_xlabel("x")
    ax3.set_ylabel("y")
    ax3.set_zlabel("z")
    title = ax3.set_title("Waiting for directions")
    arrows = []

    lines = []
    for ax, name, colour in zip(axes_t, "xyz", ("tab:red", "tab:green", "tab:blue")):
        raw, = ax.plot([], [], color=colour, alpha=0.3, linewidth=0.8)
        avg, = ax.plot([], [], color=colour, linewidth=1.8)
        lines.append((raw, avg))
        ax.set_ylim(-1.2, 1.2)
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
    axes_t[-1].set_xlabel("time (s)")
    axes_t[0].set_title(f"Directions (light) and their moving average over {smooth}")
    fig.tight_layout()

    def moving_average(values):
        out, total = [], 0.0
        for i, v in enumerate(values):
            total += v
            if i >= smooth:
                total -= values[i - smooth]
            out.append(total / min(i + 1, smooth))
        return out

    def update(_frame=None):
        points = list(track.points)
        if not points:
            return
        t = [p[0] for p in points]
        cols = [[p[1 + k] for p in points] for k in range(3)]
        avgs = [moving_average(c) for c in cols]
        for (raw, avg), c, a, ax in zip(lines, cols, avgs, axes_t):
            raw.set_data(t, c)
            avg.set_data(t, a)
            ax.set_xlim(max(t[-1] - window, 0.0), max(t[-1], window))

        for arrow in arrows:
            arrow.remove()
        arrows.clear()
        latest = [c[-1] for c in cols]
        smoothed = [a[-1] for a in avgs]
        arrows.append(ax3.quiver(0, 0, 0, *latest, color="tab:gray", linewidth=1, arrow_length_ratio=0.1))
        arrows.append(ax3.quiver(0, 0, 0, *smoothed, color="tab:red", linewidth=3, arrow_length_ratio=0.15))
        x, y, z = smoothed
        title.set_text(f"azimuth {math.degrees(math.atan2(y, x)):6.1f}°  "
                       f"elevation {math.degrees(math.atan2(z, math.hypot(x, y))):5.1f}°  "
                       f"|v| {math.sqrt(x * x + y * y + z * z):.2f}")

    return fig, update


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bind", default="", help="Local address to listen on (default: all)")
    ap.add_argument("--port", type=int, default=5000, help="UDP port (default: 5000)")
    ap.add_argument("--interval", type=float, default=1.0, help="Seconds between reports")
    ap.add_argument("--wav-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings"),
                    help="Where to save the mic .wav files (default: host/recordings)")
    ap.add_argument("--no-wav", action="store_true", help="Don't record the mics")
    ap.add_argument("--rate", type=int, default=MIC_RATE,
                    help=f"Mic sample rate written to the .wav files (default: {MIC_RATE})")
    ap.add_argument("--no-plot", action="store_true", help="Don't open the direction plot")
    ap.add_argument("--window", type=float, default=10.0, help="Seconds of directions plotted (default: 10)")
    ap.add_argument("--smooth", type=int, default=20,
                    help="Directions in the moving average, like the MATLAB model's movmean (default: 20)")
    ap.add_argument("--dir-rate", type=float, default=DIR_RATE,
                    help=f"Directions per second, to place them in time (default: {DIR_RATE:.1f})")
    args = ap.parse_args()

    if not args.no_plot:
        try:
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation
        except ImportError:
            raise SystemExit("The direction plot needs matplotlib: pip install matplotlib (or run with --no-plot)")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # The mic stream alone is ~1000 packets/s: give the OS room to queue them
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind((args.bind, args.port))
    # A timeout keeps reports going (and stopping prompt) with no packets coming in
    sock.settimeout(args.interval)
    print(f"Listening on {args.bind or '*'}:{args.port}, " + ("Ctrl-C to stop" if args.no_plot else "close the plot to stop"))

    recorder = None if args.no_wav else MicRecorder(args.wav_dir, args.rate)
    if recorder:
        print(f"Recording mics to {os.path.dirname(recorder.paths[0])}")
    track = DirectionTrack(args.window, args.dir_rate)
    rx = Receiver(sock, args.interval, recorder, track)

    try:
        if args.no_plot:
            rx.run()
        else:
            # The GUI has to run on the main thread, so the receiving moves to another
            thread = threading.Thread(target=rx.run, daemon=True)
            thread.start()
            fig, update = make_plot(track, args.window, args.smooth)
            anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)  # noqa: F841 (kept alive)
            plt.show()
            rx.stop.set()
            thread.join()
    except KeyboardInterrupt:
        rx.stop.set()
    finally:
        sock.close()
        if recorder:
            recorder.close()

    print("\nTotal:")
    for ssrc, s in sorted(rx.streams.items()):
        elapsed = time.monotonic() - s.first
        print(f"{ssrc_name(ssrc):>10}: {s.packets} packets, {s.lost} lost, {s.bytes} bytes "
              f"in {elapsed:.1f} s, avg {mbps(s.bytes, elapsed):.3f} Mbit/s")
    if rx.other:
        print(f"{'non-RTP':>10}: {rx.other} packets")
    if recorder:
        print(f"Mic recordings: {recorder.frames / recorder.rate:.2f} s per channel "
              f"({recorder.silence} frames of silence for gaps, {recorder.bad} bad packets) "
              f"in {os.path.dirname(recorder.paths[0])}")
    if track.count:
        print(f"Directions: {track.count} received, {track.bad} bad packets, {track.restarts} timestamp restarts")


if __name__ == "__main__":
    main()
