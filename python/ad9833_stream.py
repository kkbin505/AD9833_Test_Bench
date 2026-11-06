"""Real-time ESP32-S3 I2S microphone stream -> pyfar.Signal bridge.

Firmware frame formats (little-endian, 28-byte header):
    v1:
        u32 magic 0xAA55AD98 | u16 version=1 | u32 seq | u32 sampling_rate
        u32 n_samples | u16 n_channels | u32 time_stamp_us | f32 full_scale
        f32 time_data[n_samples * n_channels]
    v2:
        u32 magic 0xAA55AD98 | u16 version=2 | u32 seq | u32 sampling_rate
        u32 n_samples | u16 n_channels | u32 time_stamp_us | f32 full_scale
        packed signed 24-bit little-endian PCM, 3 bytes per sample
    v3:
        u32 magic 0xAA55AD98 | u16 version=3 | u32 seq | u32 sampling_rate
        u32 n_samples | u16 n_channels | u32 time_stamp_us | f32 full_scale
        u32 payload_bytes
        packed signed 24-bit little-endian PCM, 3 bytes per sample

Usage as a library:
    from ad9833_stream import AD9833Stream, save_to_tdms
    stream = AD9833Stream("COM7").start()
    sig = stream.signal           # pyfar.Signal of the latest window
    block = stream.latest_block() # pyfar.Signal of the newest frame

    # Record 3s of continuous data and save to NI LabVIEW-compatible TDMS:
    stream.record_to_tdms("capture.tdms", duration_s=3.0)

CLI:
    python ad9833_stream.py --list
    python ad9833_stream.py --port COM7 --window 0.5 --stats
    python ad9833_stream.py --port COM7 --tdms --tdms-interval 5 --tdms-duration 3
    python ad9833_stream.py --simulate --tdms
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pyfar as pf

MAGIC = 0xAA55AD98
HEADER_V1 = struct.Struct("<IHI IIH If")
HEADER_V2 = struct.Struct("<IHI IIH If")
HEADER_V3 = struct.Struct("<IHI IIH If I")


def _unpack_int24_le(buffer: bytes) -> np.ndarray:
    """Convert packed signed little-endian 24-bit PCM to float32."""
    raw = np.frombuffer(buffer, dtype=np.uint8)
    if raw.size % 3 != 0:
        raise ValueError("packed 24-bit payload size is not a multiple of 3")
    raw = raw.reshape(-1, 3).astype(np.uint32)
    values = raw[:, 0] | (raw[:, 1] << 8) | (raw[:, 2] << 16)
    signed = ((values ^ 0x800000) - 0x800000).astype(np.int32)
    return signed.astype(np.float32) / float(1 << 23)


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
        if len(buf) - pos < 6:
            break
        version = struct.unpack_from("<H", buf, pos + 4)[0]
        if version == 1:
            header = HEADER_V1
            header_size = HEADER_V1.size
        elif version == 2:
            header = HEADER_V2
            header_size = HEADER_V2.size
        elif version == 3:
            header = HEADER_V3
            header_size = HEADER_V3.size
        else:
            pos += 1  # stale sync, resync
            continue
        if len(buf) - pos < header_size:
            break
        if version in (1, 2):
            (magic, version, seq, fs, n_samples, n_channels, ts, full_scale) = \
                header.unpack_from(buf, pos)
            payload_bytes = n_samples * n_channels * (4 if version == 1 else 3)
        else:
            (magic, version, seq, fs, n_samples, n_channels, ts, full_scale,
             payload_bytes) = header.unpack_from(buf, pos)
        if magic != MAGIC or n_samples == 0 or n_channels == 0:
            pos += 1  # stale sync, resync
            continue
        expected_len = n_samples * n_channels * (4 if version == 1 else 3)
        payload_len = payload_bytes if version == 3 else expected_len
        if version == 3 and payload_len != expected_len:
            pos += 1
            continue
        if len(buf) - pos - header_size < payload_len:
            break
        payload = bytes(buf[pos + header_size:pos + header_size + payload_len])
        if version == 1:
            data = np.frombuffer(payload, dtype="<f4",
                                 count=n_samples * n_channels).copy()
        else:
            data = _unpack_int24_le(payload)
        frames.append(Frame(seq, fs, n_channels, ts, full_scale, data))
        pos += header_size + payload_len
    del buf[:pos]
    return frames


def save_to_tdms(
    data: np.ndarray,
    sampling_rate: int,
    filepath: str | Path,
    group_name: str = "AD9833_Data",
    channel_names: list[str] | None = None,
    start_time: datetime | None = None,
    properties: dict | None = None,
    unit_string: str = "FS",
) -> str:
    """Save continuous signal data as a National Instruments compatible TDMS file.

    Waveform properties (wf_start_time, wf_increment, wf_start_offset, wf_samples)
    are populated to ensure seamless compatibility with NI LabVIEW, DIAdem, and MATLAB.
    """
    try:
        import nptdms
        from nptdms import ChannelObject, GroupObject, RootObject
    except ImportError as exc:
        raise ImportError(
            "npTDMS is required to save TDMS files. "
            "Please install it using: pip install nptdms>=1.6.0"
        ) from exc

    if start_time is None:
        start_time = datetime.now()

    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)

    arr = np.asarray(data)
    if arr.ndim == 1:
        arr_2d = arr[np.newaxis, :]
    elif arr.ndim == 2:
        if arr.shape[0] > arr.shape[1] and arr.shape[1] <= 32:
            arr_2d = arr.T
        else:
            arr_2d = arr
    else:
        raise ValueError(f"Expected 1D or 2D array, got shape {arr.shape}")

    n_channels, n_samples = arr_2d.shape
    if channel_names is None:
        channel_names = [f"Channel_{i}" for i in range(n_channels)]
    elif len(channel_names) != n_channels:
        raise ValueError(
            f"Length of channel_names ({len(channel_names)}) does not match channel count ({n_channels})"
        )

    # INMP441 Acoustic Calibration Constants:
    # Sensitivity: -26 dBFS @ 1 kHz, 94 dB SPL (1 Pa)
    inmp441_sens_dbfs = -26.0
    inmp441_fs_per_pa = float(10.0 ** (inmp441_sens_dbfs / 20.0))  # ~0.0501187 FS/Pa
    inmp441_pa_per_fs = float(1.0 / inmp441_fs_per_pa)             # ~19.9526 Pa/FS
    inmp441_aop_dbspl = 120.0                                       # Acoustic Overload Point

    root_props = {
        "title": "AD9833 Stream Acquisition",
        "description": "Continuous I2S stream recording from ESP32-S3 / INMP441 / AD9833",
        "created_at": start_time.isoformat(),
        "sampling_rate": float(sampling_rate),
        "n_channels": int(n_channels),
        "total_samples": int(n_samples),
        "duration_s": float(n_samples / sampling_rate) if sampling_rate > 0 else 0.0,
        "sensor_model": "INMP441",
        "mic_sensitivity_dbfs": inmp441_sens_dbfs,
        "mic_pa_per_fs": inmp441_pa_per_fs,
    }
    if properties:
        root_props.update(properties)

    group_props = {
        "group_name": group_name,
        "sampling_rate": float(sampling_rate),
        "n_channels": int(n_channels),
        "sensor_model": "INMP441",
    }

    channels = []
    dt = 1.0 / float(sampling_rate) if sampling_rate > 0 else 0.0
    for i, ch_name in enumerate(channel_names):
        ch_data = np.ascontiguousarray(arr_2d[i], dtype=np.float32)
        ch_props = {
            "wf_start_time": start_time,
            "wf_increment": dt,
            "wf_start_offset": 0.0,
            "wf_samples": int(n_samples),
            "unit_string": unit_string,
            "NI_ChannelName": ch_name,
            # INMP441 Microphone Transducer Properties
            "transducer_model": "INMP441",
            "transducer_type": "MEMS Microphone (I2S)",
            "sensitivity_dbfs": inmp441_sens_dbfs,
            "sensitivity_reference": "94 dB SPL @ 1 kHz (1 Pa)",
            "fs_per_pa": inmp441_fs_per_pa,
            "pa_per_fs": inmp441_pa_per_fs,
            "acoustic_overload_point_dbspl": inmp441_aop_dbspl,
        }
        channels.append(ChannelObject(group_name, ch_name, ch_data, properties=ch_props))

    root_obj = RootObject(properties=root_props)
    group_obj = GroupObject(group_name, properties=group_props)

    with nptdms.TdmsWriter(str(filepath)) as writer:
        writer.write_segment([root_obj, group_obj] + channels)

    return str(filepath.resolve())


class AD9833Stream:
    """Reads the firmware binary stream and keeps data as pyfar Signals."""

    def __init__(self, port: str, baud: int = 2000000, window_s: float = 1.0):
        import serial  # pyserial, imported lazily for --simulate
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.window_s = window_s
        self._lock = threading.Lock()
        self._blocks: deque[Frame] = deque(maxlen=1024)
        self._window: deque[np.ndarray] = deque()
        self._window_samples = 0
        self._fs = 48000
        self.dropped_frames = 0
        self.last_seq = None
        self.text_lines: deque[str] = deque(maxlen=100)
        self._recorders: list[dict] = []
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

            # Dispatch frame data to active continuous recorders
            for rec in list(self._recorders):
                if rec["last_seq"] is not None and frame.seq != rec["last_seq"] + 1:
                    rec["dropped"] += frame.seq - rec["last_seq"] - 1
                rec["last_seq"] = frame.seq
                rec["fs"] = frame.sampling_rate
                rec["n_channels"] = frame.n_channels
                rec["frames"].append(frame.data)
                rec["collected_samples"] += frame.data.size // frame.n_channels
                if rec["collected_samples"] >= rec["target_samples"]:
                    rec["done_event"].set()
                    self._recorders.remove(rec)

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
        if self.ser:
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

    def record(self, duration_s: float, timeout: float | None = None) -> tuple[np.ndarray, int, dict]:
        """Record continuous stream for `duration_s` seconds.

        Returns:
            (data, sampling_rate, metadata_dict)
            where data has shape (n_channels, total_samples).
        """
        if duration_s <= 0:
            raise ValueError("duration_s must be positive")

        start_dt = datetime.now()
        target_samples = max(1, int(duration_s * (self._fs or 48000)))
        rec = {
            "start_time": start_dt,
            "target_samples": target_samples,
            "collected_samples": 0,
            "frames": [],
            "done_event": threading.Event(),
            "dropped": 0,
            "last_seq": None,
            "fs": self._fs or 48000,
            "n_channels": 1,
        }

        with self._lock:
            self._recorders.append(rec)

        actual_timeout = timeout if timeout is not None else (duration_s + 5.0)
        rec["done_event"].wait(timeout=actual_timeout)

        with self._lock:
            if rec in self._recorders:
                self._recorders.remove(rec)

        if not rec["frames"]:
            raise TimeoutError("No data frames received during recording period")

        fs = rec["fs"]
        n_channels = rec["n_channels"]
        all_data = np.concatenate(rec["frames"])
        reshaped = all_data.reshape(-1, n_channels).T
        exact_samples = int(duration_s * fs)
        if reshaped.shape[1] > exact_samples:
            reshaped = reshaped[:, :exact_samples]

        meta = {
            "start_time": rec["start_time"],
            "dropped_frames": rec["dropped"],
            "n_channels": n_channels,
            "sampling_rate": fs,
            "duration_s": float(reshaped.shape[1] / fs),
        }
        return reshaped, fs, meta

    def record_to_tdms(
        self,
        filepath: str | Path,
        duration_s: float = 3.0,
        group_name: str = "AD9833_Data",
        channel_names: list[str] | None = None,
        timeout: float | None = None,
        properties: dict | None = None,
    ) -> str:
        """Record `duration_s` seconds of continuous data and save to a TDMS file."""
        data, fs, meta = self.record(duration_s=duration_s, timeout=timeout)
        props = {"dropped_frames": meta["dropped_frames"]}
        if properties:
            props.update(properties)
        return save_to_tdms(
            data=data,
            sampling_rate=fs,
            filepath=filepath,
            group_name=group_name,
            channel_names=channel_names,
            start_time=meta["start_time"],
            properties=props,
        )


class SimulatedAD9833Stream(AD9833Stream):
    """Same interface, synthesizes frames locally (no hardware needed)."""

    def __init__(self, window_s: float = 1.0, fs: int = 48000,
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
        self._recorders: list[dict] = []
        self._stop = threading.Event()
        self._thread = None
        self.ser = None

    def _run(self):
        seq = 0
        t = 0.0
        while not self._stop.is_set():
            n = np.arange(seq * self._n, (seq + 1) * self._n)
            data = (0.5 * np.sin(2 * math.pi * 1000 * n / self._fs)
                    + np.random.normal(0, 0.005, self._n)).astype("<f4")
            header = HEADER_V1.pack(MAGIC, 1, seq, self._fs, self._n, 1,
                                    int(t * 1e6) & 0xFFFFFFFF, 1.0)
            self._push(Frame(seq, self._fs, 1, int(t * 1e6), 1.0, data))
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
            rate = nblocks - blocks0
            rms = float(np.sqrt(np.mean(sig.time**2)))
            peak = float(np.max(np.abs(sig.time)))
            rms_dbfs = 20.0 * np.log10(max(rms, 1e-12))
            print(f"[{time.time() - t0:6.1f}s] blocks/s={rate:.0f} "
                  f"window={sig.n_samples} samples @ {sig.sampling_rate} Hz "
                  f"| rms={rms:9.6f} FS "
                  f"| peak={peak:9.6f} FS "
                  f"| rms={rms_dbfs:7.2f} dBFS "
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


def run_periodic_tdms(
    stream: AD9833Stream,
    record_duration: float = 3.0,
    interval: float = 5.0,
    output_dir: str | Path = "tdms_data",
    count: int | None = None,
    prefix: str = "ad9833",
):
    """Periodically record data and save to timestamped TDMS files.

    Parameters:
        stream: Active AD9833Stream or SimulatedAD9833Stream instance.
        record_duration: Duration of each acquisition in seconds (default: 3.0s).
        interval: Periodic trigger interval in seconds (default: 5.0s).
        output_dir: Directory where TDMS files will be saved (default: 'tdms_data').
        count: Maximum number of acquisition cycles (None for indefinite).
        prefix: Filename prefix for TDMS files (default: 'ad9833').
    """
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    print("=== TDMS Periodic Acquisition Started ===")
    print(f"Output Directory : {out_path.resolve()}")
    print(f"Cycle Interval   : {interval:.1f} s")
    print(f"Capture Duration : {record_duration:.1f} s")
    if count is not None:
        print(f"Cycle Count Limit: {count}")
    print("Press Ctrl+C to stop.\n")

    iteration = 0
    try:
        while not stream._stop.is_set():
            cycle_start = time.time()
            iteration += 1
            now_dt = datetime.now()
            timestamp_str = now_dt.strftime("%Y%m%d_%H%M%S")
            target_file = out_path / f"{prefix}_{timestamp_str}.tdms"

            # Avoid filename collisions if multiple files are saved in the same second
            file_idx = 1
            while target_file.exists():
                target_file = out_path / f"{prefix}_{timestamp_str}_{file_idx}.tdms"
                file_idx += 1

            print(
                f"[{now_dt.strftime('%H:%M:%S')}] Cycle {iteration}: Recording {record_duration:.1f}s ... ",
                end="",
                flush=True,
            )
            try:
                data, fs, meta = stream.record(duration_s=record_duration)
                save_to_tdms(
                    data=data,
                    sampling_rate=fs,
                    filepath=target_file,
                    start_time=meta["start_time"],
                    properties={
                        "cycle_index": iteration,
                        "dropped_frames": meta["dropped_frames"],
                    },
                )
                samples_per_ch = data.shape[1] if data.ndim > 1 else data.size
                n_ch = data.shape[0] if data.ndim > 1 else 1
                print(
                    f"Saved -> {target_file.name} "
                    f"({samples_per_ch} samples @ {fs} Hz, {n_ch} ch, dropped={meta['dropped_frames']})"
                )
            except Exception as e:
                print(f"Failed: {e}")

            if count is not None and iteration >= count:
                print(f"\nCompleted {iteration} acquisition cycle(s).")
                break

            elapsed = time.time() - cycle_start
            wait_time = interval - elapsed
            if wait_time > 0:
                deadline = time.time() + wait_time
                interrupted = False
                while time.time() < deadline:
                    if stream._stop.is_set():
                        interrupted = True
                        break
                    time.sleep(min(0.1, deadline - time.time()))
                if interrupted:
                    break
            else:
                print(
                    f"  [Notice] Recording took {elapsed:.2f}s, longer than interval {interval:.1f}s. Starting next cycle immediately."
                )
    except KeyboardInterrupt:
        print("\nPeriodic acquisition interrupted by user.")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--list", action="store_true", help="list serial ports")
    ap.add_argument("--port")
    ap.add_argument("--window", type=float, default=1.0,
                    help="signal window length in seconds")
    ap.add_argument("--stats", action="store_true")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--simulate", action="store_true")
    ap.add_argument("--tdms", action="store_true",
                    help="enable periodic TDMS recording (every 5s, 3s capture by default)")
    ap.add_argument("--tdms-interval", type=float, default=5.0,
                    help="interval between acquisitions in seconds (default: 5.0)")
    ap.add_argument("--tdms-duration", type=float, default=3.0,
                    help="duration of each acquisition in seconds (default: 3.0)")
    ap.add_argument("--tdms-dir", type=str, default="tdms_data",
                    help="output directory for TDMS files (default: tdms_data)")
    ap.add_argument("--tdms-count", type=int, default=None,
                    help="maximum number of acquisitions before exit (default: infinite)")
    ap.add_argument("--tdms-prefix", type=str, default="ad9833",
                    help="filename prefix for TDMS files (default: ad9833)")
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
        if args.tdms:
            run_periodic_tdms(
                stream,
                record_duration=args.tdms_duration,
                interval=args.tdms_interval,
                output_dir=args.tdms_dir,
                count=args.tdms_count,
                prefix=args.tdms_prefix,
            )
        elif args.plot:
            run_plot(stream)
        else:
            run_stats(stream, args.window)
    finally:
        stream.stop()
        print(f"total dropped frames: {stream.dropped_frames}")


if __name__ == "__main__":
    main()
