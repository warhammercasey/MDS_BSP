"""Receive the FPGA's RTP streams, report packet counts and data rates, and record the mics.

The FPGA sends its RTP streams to one UDP port (5000, to broadcast, by default; see
udp_stack_regs.sv), told apart by SSRC ("ADC0" and "MIC0", see cain_top.sv). Each mic
is saved to its own mono 24-bit .wav; ADC samples are dropped for now.

    python host/rtp_rx.py [--port 5000] [--interval 1] [--wav-dir DIR | --no-wav]

Rates count UDP payload bytes (RTP header + samples). Lost packets are counted from
gaps in each stream's RTP sequence numbers.

Mic packets hold whole frames, channel 0 first (see rtp_packetizer.sv). Each sample is
L24: 3 bytes big endian, the 18-bit mic sample left justified, so it is written as is.
Frames missing between packets (by RTP timestamp, which counts frames) are written as
silence so the recording keeps time.
"""
import argparse
import os
import socket
import struct
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
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # The mic stream alone is ~1000 packets/s: give the OS room to queue them
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind((args.bind, args.port))
    # A timeout keeps reports going (and Ctrl-C working) with no packets coming in
    sock.settimeout(args.interval)
    print(f"Listening on {args.bind or '*'}:{args.port}, Ctrl-C to stop")

    recorder = None if args.no_wav else MicRecorder(args.wav_dir, args.rate)
    if recorder:
        print(f"Recording mics to {os.path.dirname(recorder.paths[0])}")

    streams = {}
    other = 0
    buf = bytearray(2048)
    next_report = time.monotonic() + args.interval
    try:
        while True:
            try:
                length = sock.recv_into(buf)
            except socket.timeout:
                length = None
            now = time.monotonic()

            if length is not None:
                if length >= RTP_HDR.size and buf[0] >> 6 == 2:
                    _, _, seq, timestamp, ssrc = RTP_HDR.unpack_from(buf)
                    streams.setdefault(ssrc, Stream()).add(seq, length, now)
                    if recorder and ssrc == MIC_SSRC:
                        recorder.add(timestamp, bytes(buf[RTP_HDR.size:length]))
                else:
                    other += 1

            if now >= next_report:
                report(streams, other, now, args.interval + now - next_report)
                next_report = now + args.interval
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        if recorder:
            recorder.close()

    print("\nTotal:")
    for ssrc, s in sorted(streams.items()):
        elapsed = time.monotonic() - s.first
        print(f"{ssrc_name(ssrc):>10}: {s.packets} packets, {s.lost} lost, {s.bytes} bytes "
              f"in {elapsed:.1f} s, avg {mbps(s.bytes, elapsed):.3f} Mbit/s")
    if other:
        print(f"{'non-RTP':>10}: {other} packets")
    if recorder:
        print(f"Mic recordings: {recorder.frames / recorder.rate:.2f} s per channel "
              f"({recorder.silence} frames of silence for gaps, {recorder.bad} bad packets) "
              f"in {os.path.dirname(recorder.paths[0])}")


if __name__ == "__main__":
    main()
