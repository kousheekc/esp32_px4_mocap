#!/usr/bin/env python3
"""Measure how QTM 6D frames arrive at this machine, the same way the bridge firmware does.

Opens a QTM RT session over TCP, asks QTM to stream 6D frames to a local UDP port and prints a
once-per-second stats line in the bridge's format, then a summary on Ctrl+C. Run it once over a
cable and once over WiFi to see whether drops and stalls come from QTM or from the WiFi link.

    python3 tools/qtm_probe.py 192.168.3.92
    python3 tools/qtm_probe.py 192.168.3.92 --bind 192.168.3.50   # force a given interface

Standard library only.
"""

import argparse
import math
import select
import socket
import struct
import time

HEADER_LEN = 8
TYPE_ERROR, TYPE_COMMAND, TYPE_DATA, TYPE_EVENT = 0, 1, 3, 6
COMPONENT_6D = 5


def encode_command(cmd):
    body = cmd.encode() + b"\0"
    return struct.pack("<II", HEADER_LEN + len(body), TYPE_COMMAND) + body


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("TCP closed")
        buf += chunk
    return buf


def read_packet(sock):
    size, kind = struct.unpack("<II", recv_exact(sock, HEADER_LEN))
    return kind, recv_exact(sock, size - HEADER_LEN)


def text(payload):
    return payload.split(b"\0", 1)[0].decode(errors="replace")


def parse_6d(payload, index):
    """Position (mm) and validity of the body at `index`, or None"""
    off = 12
    (n_components,) = struct.unpack_from("<I", payload, off)
    off += 4
    for _ in range(n_components):
        size, kind = struct.unpack_from("<II", payload, off)
        if kind != COMPONENT_6D:
            off += size
            continue
        (n_bodies,) = struct.unpack_from("<I", payload, off + 8)
        if index >= n_bodies:
            return None
        vals = struct.unpack_from("<12f", payload, off + 16 + index * 48)
        return vals[:3], all(math.isfinite(v) for v in vals)
    return None


class ClockSync:
    """Offset = smallest arrival - capture over the last 1-2 s, as in the firmware"""

    def __init__(self, window_us=1_000_000):
        self.window_us = window_us
        self.block_start = 0
        self.cur_min = self.prev_min = 0
        self.last_capture = None

    def update(self, capture_us, arrival_us):
        d = arrival_us - capture_us
        if self.last_capture is None or capture_us < self.last_capture:
            self.cur_min = self.prev_min = d
            self.block_start = arrival_us
        elif arrival_us - self.block_start >= self.window_us:
            self.prev_min, self.cur_min = self.cur_min, d
            self.block_start = arrival_us
        else:
            self.cur_min = min(self.cur_min, d)
        self.last_capture = capture_us
        return capture_us + min(self.cur_min, self.prev_min)


def percentile(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    return sorted_vals[min(len(sorted_vals) - 1, int(p / 100 * len(sorted_vals)))]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host", help="QTM PC address")
    ap.add_argument("--port", type=int, default=22223, help="QTM RT port (default 22223)")
    ap.add_argument("--udp-port", type=int, default=22230, help="local UDP port for frames")
    ap.add_argument("--rate", type=int, default=100, help="requested stream rate in Hz")
    ap.add_argument("--body", type=int, default=1, help="rigid body number, 1-based")
    ap.add_argument("--bind", default="", help="local IP to use, picks the interface (cable/WiFi)")
    args = ap.parse_args()

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    udp.bind((args.bind, args.udp_port))

    # QTM streams UDP back to the address the TCP session comes from.
    tcp = socket.create_connection((args.host, args.port), timeout=5, source_address=(args.bind, 0))
    tcp.settimeout(5)
    print(f"connected from {tcp.getsockname()[0]} to {args.host}:{args.port}")
    print("qtm:", text(read_packet(tcp)[1]))
    tcp.sendall(encode_command("Version 1.19"))
    kind, payload = read_packet(tcp)
    if kind == TYPE_ERROR:
        raise SystemExit(f"version rejected: {text(payload)}")
    print("qtm:", text(payload))
    stream_cmd = f"StreamFrames Frequency:{args.rate} UDP:{args.udp_port} 6D"
    tcp.sendall(encode_command(stream_cmd))
    print(f'requested "{stream_cmd}"')

    clock = ClockSync()
    start = window_start = time.monotonic()
    last_frame = frame_step = None
    last_arrival = None
    pkts = missed = dup = 0
    max_gap_ms = max_age_ms = 0.0
    last_pose = None
    total_pkts = total_missed = total_dup = 0
    gaps_ms, ages_ms = [], []

    try:
        while True:
            ready, _, _ = select.select([tcp, udp], [], [], 0.2)
            # Control channel: errors, events, or EOF when QTM goes away.
            if tcp in ready:
                kind, payload = read_packet(tcp)
                if kind == TYPE_ERROR:
                    print("qtm: error:", text(payload))
                elif kind == TYPE_EVENT:
                    print("qtm: event", payload[0] if payload else None)

            if udp in ready:
                data = udp.recv(4096)
                arrival = time.monotonic()
                size, kind = struct.unpack_from("<II", data)
                if kind == TYPE_DATA and size >= HEADER_LEN + 12:
                    payload = data[HEADER_LEN:size]
                    timestamp_us, frame = struct.unpack_from("<QI", payload)
                    pkts += 1
                    if last_arrival is not None:
                        gap = (arrival - last_arrival) * 1000
                        gaps_ms.append(gap)
                        max_gap_ms = max(max_gap_ms, gap)
                    last_arrival = arrival

                    d = None if last_frame is None else (frame - last_frame) & 0xFFFFFFFF
                    if d is not None and d >= 0x80000000:
                        d -= 1 << 32
                    if d is not None and -1000 <= d <= 0:
                        dup += 1
                    else:
                        if d is not None and 0 < d < 100_000:
                            frame_step = d if frame_step is None else min(frame_step, d)
                            missed += d // frame_step - 1
                        last_frame = frame
                        arrival_us = int(arrival * 1e6)
                        age = (arrival_us - clock.update(timestamp_us, arrival_us)) / 1000
                        ages_ms.append(age)
                        max_age_ms = max(max_age_ms, age)
                        last_pose = parse_6d(payload, args.body - 1)

            now = time.monotonic()
            if now - window_start >= 1.0:
                hz = pkts / (now - window_start)
                if last_pose:
                    (x, y, z), valid = last_pose
                    pose = f"body {args.body}: pos [{x:.1f} {y:.1f} {z:.1f}] mm valid={valid}"
                else:
                    pose = f"body {args.body} not seen"
                print(
                    f"qtm: {pkts} pkts ({hz:.1f} Hz) | missed {missed} (frame step {frame_step or 0}), "
                    f"dup {dup} | gap max {max_gap_ms:.0f} ms, age max {max_age_ms:.0f} ms | {pose}"
                )
                total_pkts += pkts
                total_missed += missed
                total_dup += dup
                pkts = missed = dup = 0
                max_gap_ms = max_age_ms = 0.0
                last_pose = None
                window_start = now
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tcp.sendall(encode_command("StreamFrames Stop"))
        except OSError:
            pass
        tcp.close()
        udp.close()

    total_pkts += pkts
    total_missed += missed
    total_dup += dup
    elapsed = time.monotonic() - start
    expected = total_pkts + total_missed
    gaps_ms.sort()
    ages_ms.sort()
    print(f"\n--- {elapsed:.1f} s ---")
    print(f"received {total_pkts} ({total_pkts / elapsed:.1f} Hz), missed {total_missed} "
          f"({100 * total_missed / expected if expected else 0:.1f}%), dup {total_dup}")
    print(f"QTM frame rate seen: {expected / elapsed:.1f} frames/s (received + missed)")
    for name, vals in (("gap", gaps_ms), ("age", ages_ms)):
        print(f"{name} ms: p50 {percentile(vals, 50):.1f}  p95 {percentile(vals, 95):.1f}  "
              f"p99 {percentile(vals, 99):.1f}  max {vals[-1] if vals else 0:.1f}")


if __name__ == "__main__":
    main()
