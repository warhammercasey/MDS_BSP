"""Receive the FPGA's RTP streams and report packet counts and data rates.

The FPGA sends its RTP streams to one UDP port (5000, to broadcast, by default; see
udp_stack_regs.sv), told apart by SSRC ("ADC0" and "MIC0", see cain_top.sv). Samples
are dropped for now.

    python host/rtp_rx.py [--port 5000] [--interval 1]

Rates count UDP payload bytes (RTP header + samples). Lost packets are counted from
gaps in each stream's RTP sequence numbers.
"""
import argparse
import socket
import struct
import time

RTP_HDR = struct.Struct("!BBHII")  # V/P/X/CC, M/PT, sequence, timestamp, SSRC


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
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # The mic stream alone is ~1300 packets/s: give the OS room to queue them
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind((args.bind, args.port))
    # A timeout keeps reports going (and Ctrl-C working) with no packets coming in
    sock.settimeout(args.interval)
    print(f"Listening on {args.bind or '*'}:{args.port}, Ctrl-C to stop")

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
                    _, _, seq, _, ssrc = RTP_HDR.unpack_from(buf)
                    streams.setdefault(ssrc, Stream()).add(seq, length, now)
                else:
                    other += 1

            if now >= next_report:
                report(streams, other, now, args.interval + now - next_report)
                next_report = now + args.interval
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    print("\nTotal:")
    for ssrc, s in sorted(streams.items()):
        elapsed = time.monotonic() - s.first
        print(f"{ssrc_name(ssrc):>10}: {s.packets} packets, {s.lost} lost, {s.bytes} bytes "
              f"in {elapsed:.1f} s, avg {mbps(s.bytes, elapsed):.3f} Mbit/s")
    if other:
        print(f"{'non-RTP':>10}: {other} packets")


if __name__ == "__main__":
    main()
