"""Real-time ESP32-S3 ADC stream -> pyfar.Signal bridge.

Firmware frame format (little-endian, 28-byte header):
    u32 magic 0xAA55AD98 | u16 version | u32 seq | u32 sampling_rate
    u32 n_samples | u16 n_channels | u32 time_stamp_us | f32 full_scale
    f32 time_data[n_samples * n_channels]

Usage as a library:
    from ad9833_stream import AD9833Stream
    stream = AD9833Stream("COM7").start()
    sig = stream.signal           # pyfar.Signal of the latest window
    block = stream.latest_block() # pyfar.Signal of the newest frame

CLI:
    python ad9833_stream.py --list
    python ad9833_stream.py --port COM7 --window 0.5 --stats
    python ad9833_stream.py --simulate --stats
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import threading
import time
from collections import deque

import numpy as np
import pyfar as pf

MAGIC = 0xAA55AD98
HEADER_SIZE = 28
HEADER = struct.Struct("<IHI IIH If")


class Frame:
    def __init__(self, seq, sampling_rate, n_channels, time_stamp_us,
                 full_scale, data):
        self.seq = seq
        self.sampling_rate = sampling_rate
        self.n_channels = n_channels
        self.time_stamp_us = time_stamp_us
        self.full_scale = full_scale
        self.data = data  # float32 ndarray

    def to_signal(self) -> pf.Signal:
        return pf.Signal(
            self.data.reshape(-1, self.n_channels).T,
            self.sampling_rate,
            comment=f"seq={self.seq} t_us={self.time_stamp_us} "
                    f"full_scale={self.full_scale}",
        )


def parse_frames(buf: bytearray):
    """Yield (Frame, leftover_bytes_consumed) from a byte buffer.

    Text lines starting with '#' (firmware metric output) are ignored.
    """
    frames = []
    pos = 0
    while True:
        # Skip anything before the magic (also filters '#' text lines)
        idx = buf.find(MAGIC.to_bytes(4, "little"), pos)
        if idx < 0:
            break
        pos = idx
        if len(buf) - pos < HEADER_SIZE:
            break
        (magic, version, seq, fs, n_samples, n_channels, ts, full_scale) = \
            HEADER.unpack_from(buf, pos)
        if magic != MAGIC or version != 1 or n_samples == 0:
            pos += 1  # stale sync, resync
            continue
        payload_len = n_samples * n_channels * 4
        if len(buf) - pos - HEADER_SIZE < payload_len:
            break
        data = np.frombuffer(
            buf, dtype="<f4", count=n_samples * n_channels,
            offset=pos + HEADER_SIZE,
        ).copy()
        frames.append(Frame(seq, fs, n_channels, ts, full_scale, data))
        pos += HEADER_SIZE + payload_len
    del buf[:pos]
    return frames


class AD9833Stream:
    """Reads the firmware binary stream and keeps data as pyfar Signals."""

    def __init__(self, port: str, baud: int = 115200, window_s: float = 1.0):
        import serial  # pyserial, imported lazily for --simulate
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.window_s = window_s
        self._lock = threading.Lock()
        self._blocks: deque[Frame] = deque(maxlen=1024)
        self._window: deque[np.ndarray] = deque()
        self._window_samples = 0
        self._fs = 50000
        self.dropped_frames = 0
        self.last_seq = None
        self.text_lines: deque[str] = deque(maxlen=100)
        self._stop = threading.Event()
        self._thread = None

    # -- background reader ------------------------------------------------
    def _push(self, frame: Frame):
        with self._lock:
            if self.last_seq is not None and frame.seq != self.last_seq + 1:
                self.dropped_frames += frame.seq - self.last_seq - 1
            self.last_seq = frame.seq
            self._fs = frame.sampling_rate
            self._blocks.append(frame)
            self._window.append(frame.data)
            self._window_samples += frame.data.size
            limit = int(self.window_s * frame.sampling_rate)
            while self._window_samples > limit and len(self._window) > 1:
                self._window_samples -= self._window.popleft().size

    def _run(self):
        buf = bytearray()
        while not self._stop.is_set():
            chunk = self.ser.read(4096)
            if not chunk:
                continue
            buf += chunk
            for frame in parse_frames(buf):
                self._push(frame)

    # -- public API --------------------------------------------------------
    def start(self) -> "AD9833Stream":
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self.ser.close()

    def latest_block(self) -> pf.Signal | None:
        with self._lock:
            return self._blocks[-1].to_signal() if self._blocks else None

    @property
    def sampling_rate(self) -> int:
        return self._fs

    @property
    def signal(self) -> pf.Signal | None:
        """pyfar.Signal over the most recent `window_s` seconds."""
        with self._lock:
            if not self._window:
                return None
            data = np.concatenate(list(self._window))
            return pf.Signal(data, self._fs, comment="streaming window")


class SimulatedAD9833Stream(AD9833Stream):
    """Same interface, synthesizes frames locally (no hardware needed)."""

    def __init__(self, window_s: float = 1.0, fs: int = 50000,
                 n_samples: int = 2048, period_s: float = 0.05):
        self.window_s = window_s
        self._fs = fs
        self._n = n_samples
        self._period = period_s
        self._lock = threading.Lock()
        self._blocks = deque(maxlen=1024)
        self._window = deque()
        self._window_samples = 0
        self.dropped_frames = 0
        self.last_seq = None
        self.text_lines = deque(maxlen=100)
        self._stop = threading.Event()
        self._thread = None
        self.ser = None

    def _run(self):
        seq = 0
        t = 0.0
        while not self._stop.is_set():
            n = np.arange(seq * self._n, (seq + 1) * self._n)
            data = (2048 + 1000 * np.sin(2 * math.pi * 1000 * n / self._fs)
                    + np.random.normal(0, 1, self._n)).astype("<f4")
            header = HEADER.pack(MAGIC, 1, seq, self._fs, self._n, 1,
                                 int(t * 1e6) & 0xFFFFFFFF, 4096.0)
            self._push(Frame(seq, self._fs, 1, int(t * 1e6), 4096.0, data))
            seq += 1
            t += self._n / self._fs
            time.sleep(self._period)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)


def list_ports():
    from serial.tools import list_ports
    for p in list_ports.comports():
        print(f"{p.device:12s} {p.description}")


def run_stats(stream, window_s: float):
    print(f"Streaming... Ctrl+C to stop. window={window_s}s")
    t0 = time.time()
    blocks0 = 0
    try:
        while True:
            time.sleep(1.0)
            sig = stream.signal
            with stream._lock:
                nblocks = len(stream._blocks)
                dropped = stream.dropped_frames
            if sig is None or nblocks == blocks0:
                print("waiting for frames...")
                continue
            rate = (nblocks - blocks0) * sig.sampling_rate * 0 + \
                (nblocks - blocks0)
            print(f"[{time.time() - t0:6.1f}s] blocks/s={rate:.0f} "
                  f"window={sig.n_samples} samples @ {sig.sampling_rate} Hz "
                  f"| rms={float(np.sqrt(np.mean(sig.time**2))):8.2f} LSB "
                  f"| dropped={dropped}")
            blocks0 = nblocks
    except KeyboardInterrupt:
        pass


def run_plot(stream):
    import matplotlib.pyplot as plt
    plt.ion()
    fig, (ax_t, ax_f) = plt.subplots(2, 1)
    while not stream._stop.is_set():
        sig = stream.signal
        if sig is not None:
            ax_t.cla()
            ax_f.cla()
            pf.plot.time(sig, ax=ax_t)
            pf.plot.freq(sig, ax=ax_f)
            ax_f.set_yscale("linear")
            fig.canvas.draw_idle()
        plt.pause(0.2)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--list", action="store_true", help="list serial ports")
    ap.add_argument("--port")
    ap.add_argument("--window", type=float, default=1.0,
                    help="signal window length in seconds")
    ap.add_argument("--stats", action="store_true")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--simulate", action="store_true")
    args = ap.parse_args()

    if args.list:
        list_ports()
        return

    if args.simulate:
        stream = SimulatedAD9833Stream(window_s=args.window).start()
    else:
        if not args.port:
            ap.error("--port required (use --list to enumerate)")
        stream = AD9833Stream(args.port, window_s=args.window).start()

    try:
        if args.plot:
            run_plot(stream)
        else:
            run_stats(stream, args.window)
    finally:
        stream.stop()
        print(f"total dropped frames: {stream.dropped_frames}")


if __name__ == "__main__":
    main()
