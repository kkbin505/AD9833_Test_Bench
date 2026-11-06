"""Real-time USB2406 4-Channel Acquisition Stream -> TDMS & pyfar.Signal bridge.

Hardware specifications (USB2406 USB 4-channel acquisition card):
    Channels: 4 differential / single-ended channels (CH0 - CH3)
    ADC: AD7764 / AD7765 (24-bit high-speed ADC)
    Max Sampling Rate: 156 kHz / 312 kHz (configurable via FreqMode)
    Gain Ranges: ±10V (0), ±1V (1), ±100mV (2), ±10mV (3)

Usage as a library:
    from usb2406_stream import USB2406Stream, save_to_tdms

    # Real hardware mode:
    stream = USB2406Stream(card_id=0, fs_khz=100.0, gain=0, n_channels=4).start()
    sig = stream.signal            # pyfar.Signal of the latest window (4 channels)
    block = stream.latest_block()  # pyfar.Signal of the newest frame (4 channels)

    # Record 3 seconds of 4-channel continuous data and save to NI LabVIEW TDMS:
    stream.record_to_tdms("capture_4ch.tdms", duration_s=3.0)

    # Simulation mode (no hardware required):
    from usb2406_stream import SimulatedUSB2406Stream
    sim_stream = SimulatedUSB2406Stream(fs=100000, n_channels=4).start()
    sim_stream.record_to_tdms("sim_4ch.tdms", duration_s=3.0)

CLI commands:
    python usb2406_stream.py --simulate --stats
    python usb2406_stream.py --simulate --tdms --tdms-interval 5 --tdms-duration 3
    python usb2406_stream.py --card 0 --channels 4 --fs 100 --tdms
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from collections import deque
from ctypes import CDLL, POINTER, c_byte, c_double, c_float, c_int, c_int16, c_long, c_uint, c_uint8, c_uint16
from datetime import datetime
from pathlib import Path

import numpy as np
import pyfar as pf

# Resolve bundled drivers relative to this script, independently of the working directory.
_DRIVER_DIR = Path(__file__).resolve().parent
SEARCH_DLL_PATHS = [
    _DRIVER_DIR / "USB2406.dll",
    _DRIVER_DIR / "usb2406.dll",
    _DRIVER_DIR.parent / "SDK" / "dll64" / "usb2406.dll",
    _DRIVER_DIR / "../../BIN/x64/USB2406.dll",
    _DRIVER_DIR / "../../BIN/x64/usb2406.dll",
]

DEFAULT_DLL_PATH = os.environ.get("USB2406_DLL_PATH", "USB2406.dll")


def save_to_tdms(
    data: np.ndarray,
    sampling_rate: int | float,
    filepath: str | Path,
    group_name: str = "USB2406_Data",
    channel_names: list[str] | None = None,
    start_time: datetime | None = None,
    properties: dict | None = None,
    unit_string: str = "V",
) -> str:
    """Save continuous multi-channel signal data as a National Instruments compatible TDMS file.

    Parameters
    ----------
    data : np.ndarray
        2D numpy array of shape (n_channels, total_samples) or 1D array for single channel.
    sampling_rate : int | float
        Sampling frequency in Hz.
    filepath : str | Path
        Path to output TDMS file.
    group_name : str
        TDMS Group Name (default: 'USB2406_Data').
    channel_names : list[str] | None
        List of channel names matching n_channels count. Default: CH0, CH1, CH2, CH3...
    start_time : datetime | None
        Acquisition start time timestamp. Defaults to datetime.now().
    properties : dict | None
        Additional root level TDMS properties.
    unit_string : str
        Physical unit string (default: 'V').

    Returns
    -------
    str
        Absolute resolved filepath of saved TDMS file.
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

    arr = np.asarray(data, dtype=np.float32)
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
        channel_names = [f"CH{i}" for i in range(n_channels)]
    elif len(channel_names) != n_channels:
        raise ValueError(
            f"Length of channel_names ({len(channel_names)}) does not match channel count ({n_channels})"
        )

    root_props = {
        "title": "USB2406 4-Channel Stream Acquisition",
        "description": "Continuous multi-channel high-speed voltage recording from USB2406 acquisition card",
        "created_at": start_time.isoformat(),
        "sampling_rate": float(sampling_rate),
        "n_channels": int(n_channels),
        "total_samples": int(n_samples),
        "duration_s": float(n_samples / sampling_rate) if sampling_rate > 0 else 0.0,
        "hardware_model": "USB2406",
    }
    if properties:
        root_props.update(properties)

    group_props = {
        "group_name": group_name,
        "sampling_rate": float(sampling_rate),
        "n_channels": int(n_channels),
        "hardware_model": "USB2406",
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
            "transducer_type": "USB2406 Voltage Input",
            "channel_index": i,
        }
        channels.append(ChannelObject(group_name, ch_name, ch_data, properties=ch_props))

    root_obj = RootObject(properties=root_props)
    group_obj = GroupObject(group_name, properties=group_props)

    with nptdms.TdmsWriter(str(filepath)) as writer:
        writer.write_segment([root_obj, group_obj] + channels)

    return str(filepath.resolve())


class USB2406Driver:
    """Low-level ctypes driver wrapper for USB2406.dll."""

    GAIN_MAP = {
        0: "±10V",
        1: "±1V",
        2: "±100mV",
        3: "±10mV",
    }

    def __init__(
        self,
        card_id: int = 0,
        dll_path: str | Path = DEFAULT_DLL_PATH,
        ch_start: int = 0,
        ch_count: int = 4,
        freq_mode: int = 0,
        fs_khz: float = 100.0,
        gain: int = 0,
        constant_current: int = 0,
        calibration: int = 0,
    ):
        self.card_id = int(card_id)
        self.dll_path = Path(dll_path)
        self.ch_start = c_byte(ch_start)
        self.ch_count = c_byte(ch_count)
        self.freq_mode = c_uint(freq_mode)
        self.fs_khz = c_float(fs_khz)
        self.gain = c_uint16(gain)
        self.constant_current = c_int(constant_current)
        self.calibration = c_int(calibration)
        self.h_device = c_int(0)
        self.dll = None

        self._k_value = (c_int * 10)()
        self._b_value = (c_int * 10)()

    def load_dll(self):
        candidate_paths = [self.dll_path, Path(__file__).parent / self.dll_path] + [Path(p) for p in SEARCH_DLL_PATHS]
        found_path = None
        for p in candidate_paths:
            try:
                if p.exists() and p.is_file():
                    found_path = p
                    break
            except Exception:
                continue

        if found_path is None:
            searched_str = "\n  - ".join([str(p.resolve()) for p in candidate_paths if p])
            raise FileNotFoundError(
                f"USB2406.dll not found. Searched locations:\n  - {searched_str}"
            )

        self.dll_path = found_path
        self.dll = CDLL(str(self.dll_path.resolve()))

    def open(self):
        if self.dll is None:
            self.load_dll()

        ph_device = POINTER(c_int)(self.h_device)
        res = self.dll.USB2406_OpenDevice(self.card_id, ph_device)
        if res != 1:
            raise RuntimeError(f"Failed to open USB2406 card ID {self.card_id}")

        # Stop previous operations & clear FIFO
        self.dll.USB2406_ADStop(self.card_id)
        self.dll.USB2406_CLRRAM(self.card_id)

    def configure(self) -> float:
        """Configure sampling parameters on hardware and return actual sampling rate in Hz."""
        p_fs_khz = POINTER(c_float)(self.fs_khz)
        self.dll.USB2406_SetFreDiv_2(self.card_id, self.freq_mode.value, p_fs_khz)

        # Set gain for all 4 channels
        gain_array = (c_byte * 4)()
        for i in range(4):
            gain_array[i] = self.gain.value
        self.dll.USB2406_Set_Gain(self.card_id, gain_array)

        # Soft trigger mode
        ad_trig = c_int(1 + ((1 + 1) << 1))
        self.dll.USB2406_SetADMode(self.card_id, ad_trig)

        # Enable selected channels
        ch_sel = c_byte(0)
        for i in range(self.ch_start.value, self.ch_start.value + self.ch_count.value):
            ch_sel.value += 1 << i
        self.dll.USB2406_SetADCh(self.card_id, ch_sel)

        # Constant current source & RAM clear
        self.dll.USb2406_Set_ConstantVolSource(self.card_id, self.constant_current)
        self.dll.USB2406_CLRRAM(self.card_id)
        self.dll.USB2406_IOCTL_VENDOR_REQUEST(self.card_id, 0xB2)

        actual_fs_hz = float(self.fs_khz.value) * 1000.0
        return actual_fs_hz

    def start(self):
        self.dll.USB2406_ADStart(self.card_id)

    def stop(self):
        if self.dll:
            self.dll.USB2406_ADStop(self.card_id)
            self.dll.USB2406_CLRRAM(self.card_id)

    def close(self):
        if self.dll:
            self.stop()
            self.dll.USB2406_CloseDevice(self.card_id)
            self.dll = None

    def read_block(self, buffer_size: int = 4096) -> tuple[np.ndarray, int]:
        """Read data block from DRAM FIFO.

        Returns:
            (data_2d, fifo_status) where data_2d has shape (n_channels, n_samples).
        """
        status = c_byte(1)
        p_status = POINTER(c_byte)(status)
        self.dll.USB2406_ReadDramState(self.card_id, p_status)

        if status.value != 2:
            return np.empty((self.ch_count.value, 0), dtype=np.float32), status.value

        n_bytes = c_long(0)
        p_bytes = POINTER(c_long)(n_bytes)
        barray = (c_uint8 * (2 * 1024 * 1024 * 2))()

        self.dll.USB2406_IOCTL_BULK_READ(self.card_id, barray, buffer_size, p_bytes)
        bytes_read = n_bytes.value

        n_ch = self.ch_count.value
        bytes_per_multich = 4 * n_ch
        num_samples = bytes_read // bytes_per_multich

        if num_samples <= 0:
            return np.empty((n_ch, 0), dtype=np.float32), status.value

        # Unpack raw bytes into 4-channel voltage values
        raw_bytes = np.frombuffer(barray, dtype=np.uint8, count=num_samples * bytes_per_multich)
        reshaped = raw_bytes.reshape(num_samples, n_ch, 4)

        lbyte = reshaped[:, :, 1].astype(np.float64)
        mbyte = reshaped[:, :, 2].astype(np.float64)
        hbyte = reshaped[:, :, 3].astype(np.float64)

        dfret_vals = lbyte + mbyte * 256.0 + hbyte * 65536.0

        # Convert ADC code to voltage using DLL helper
        dfret = c_double(0.0)
        p_dfret = POINTER(c_double)(dfret)
        volts = np.zeros((n_ch, num_samples), dtype=np.float32)

        x0 = float(math.pow(10.0, self.gain.value))
        for s in range(num_samples):
            for c in range(n_ch):
                dfret.value = dfret_vals[s, c]
                self.dll.USB2406_Fun_DataToV(p_dfret, self.gain)
                v = (dfret.value * x0 * 1.0 + 0.0) / x0
                volts[c, s] = v

        return volts, status.value


class USB2406Stream:
    """Reads the USB2406 hardware stream and keeps data as multi-channel pyfar Signals."""

    def __init__(
        self,
        card_id: int = 0,
        dll_path: str | Path = DEFAULT_DLL_PATH,
        ch_start: int = 0,
        ch_count: int = 4,
        fs_khz: float = 100.0,
        gain: int = 0,
        window_s: float = 1.0,
    ):
        self.driver = USB2406Driver(
            card_id=card_id,
            dll_path=dll_path,
            ch_start=ch_start,
            ch_count=ch_count,
            fs_khz=fs_khz,
            gain=gain,
        )
        self.window_s = window_s
        self.n_channels = ch_count
        self._lock = threading.Lock()
        self._blocks: deque[np.ndarray] = deque(maxlen=1024)
        self._window: deque[np.ndarray] = deque()
        self._window_samples = 0
        self._fs = int(fs_khz * 1000)
        self.dropped_frames = 0
        self.fifo_overflow_count = 0

        self._recorders: list[dict] = []
        self._stop = threading.Event()
        self._thread = None

    def _push(self, block_2d: np.ndarray):
        """Push a block of shape (n_channels, n_samples) into window and recorders."""
        if block_2d.size == 0:
            return

        with self._lock:
            self._blocks.append(block_2d)
            self._window.append(block_2d)
            self._window_samples += block_2d.shape[1]

            limit = int(self.window_s * self._fs)
            while self._window_samples > limit and len(self._window) > 1:
                dropped = self._window.popleft()
                self._window_samples -= dropped.shape[1]

            # Dispatch block to active recorders
            for rec in list(self._recorders):
                rec["frames"].append(block_2d)
                rec["collected_samples"] += block_2d.shape[1]
                if rec["collected_samples"] >= rec["target_samples"]:
                    rec["done_event"].set()
                    self._recorders.remove(rec)

    def _run(self):
        self.driver.open()
        actual_fs = self.driver.configure()
        self._fs = int(actual_fs)
        self.driver.start()

        buf_size = self.n_channels * 1024 * 4
        while not self._stop.is_set():
            try:
                block_2d, fifo_status = self.driver.read_block(buffer_size=buf_size)
                if fifo_status == 6:
                    self.fifo_overflow_count += 1
                if block_2d.shape[1] > 0:
                    self._push(block_2d)
                else:
                    time.sleep(0.005)
            except Exception as exc:
                print(f"[USB2406Stream Error] {exc}", file=sys.stderr)
                time.sleep(0.01)

        self.driver.close()

    def start(self) -> "USB2406Stream":
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def latest_block(self) -> pf.Signal | None:
        with self._lock:
            if not self._blocks:
                return None
            return pf.Signal(self._blocks[-1], self._fs, comment="latest_block")

    @property
    def sampling_rate(self) -> int:
        return self._fs

    @property
    def signal(self) -> pf.Signal | None:
        """pyfar.Signal over the most recent `window_s` seconds (shape: n_channels, n_samples)."""
        with self._lock:
            if not self._window:
                return None
            data = np.hstack(list(self._window))
            return pf.Signal(data, self._fs, comment="streaming window")

    def record(self, duration_s: float, timeout: float | None = None) -> tuple[np.ndarray, int, dict]:
        """Record continuous stream for `duration_s` seconds across all channels.

        Returns:
            (data_2d, sampling_rate, metadata_dict) where data_2d shape is (n_channels, total_samples).
        """
        if duration_s <= 0:
            raise ValueError("duration_s must be positive")

        start_dt = datetime.now()
        target_samples = max(1, int(duration_s * self._fs))
        rec = {
            "start_time": start_dt,
            "target_samples": target_samples,
            "collected_samples": 0,
            "frames": [],
            "done_event": threading.Event(),
            "fs": self._fs,
            "n_channels": self.n_channels,
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
        all_data = np.hstack(rec["frames"])
        exact_samples = int(duration_s * fs)
        if all_data.shape[1] > exact_samples:
            all_data = all_data[:, :exact_samples]

        meta = {
            "start_time": rec["start_time"],
            "n_channels": self.n_channels,
            "sampling_rate": fs,
            "duration_s": float(all_data.shape[1] / fs),
            "fifo_overflows": self.fifo_overflow_count,
        }
        return all_data, fs, meta

    def record_to_tdms(
        self,
        filepath: str | Path,
        duration_s: float = 3.0,
        group_name: str = "USB2406_Data",
        channel_names: list[str] | None = None,
        timeout: float | None = None,
        properties: dict | None = None,
    ) -> str:
        """Record `duration_s` seconds of continuous multi-channel data and save to a TDMS file."""
        data, fs, meta = self.record(duration_s=duration_s, timeout=timeout)
        props = {"fifo_overflows": meta["fifo_overflows"]}
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


class SimulatedUSB2406Stream(USB2406Stream):
    """Same interface as USB2406Stream, synthesizes 4-channel frames locally without hardware."""

    def __init__(
        self,
        window_s: float = 1.0,
        fs: int = 100000,
        n_channels: int = 4,
        n_samples_per_block: int = 2048,
        period_s: float = 0.02,
    ):
        self.window_s = window_s
        self.n_channels = n_channels
        self._fs = fs
        self._n = n_samples_per_block
        self._period = period_s
        self._lock = threading.Lock()
        self._blocks: deque[np.ndarray] = deque(maxlen=1024)
        self._window: deque[np.ndarray] = deque()
        self._window_samples = 0
        self.dropped_frames = 0
        self.fifo_overflow_count = 0

        self._recorders: list[dict] = []
        self._stop = threading.Event()
        self._thread = None
        self.driver = None

    def _run(self):
        seq = 0
        while not self._stop.is_set():
            t = np.arange(seq * self._n, (seq + 1) * self._n) / float(self._fs)

            # Generate 4 distinct signals:
            # CH0: 1000 Hz Sine + noise
            # CH1: 500 Hz Sine + noise
            # CH2: 200 Hz Triangular wave
            # CH3: Ambient Gaussian noise
            ch0 = 2.5 * np.sin(2 * math.pi * 1000 * t) + np.random.normal(0, 0.05, self._n)
            ch1 = 1.5 * np.sin(2 * math.pi * 500 * t) + np.random.normal(0, 0.03, self._n)
            ch2 = 1.0 * (2.0 * np.abs(2.0 * (t * 200 - np.floor(t * 200 + 0.5))) - 1.0)
            ch3 = np.random.normal(0, 0.2, self._n)

            block = np.vstack([ch0, ch1, ch2, ch3][: self.n_channels]).astype(np.float32)
            self._push(block)
            seq += 1
            time.sleep(self._period)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)


def run_stats(stream: USB2406Stream, window_s: float):
    print(f"Streaming USB2406 ({stream.n_channels} Channels)... Press Ctrl+C to stop. window={window_s}s\n")
    t0 = time.time()
    blocks0 = 0
    try:
        while True:
            time.sleep(1.0)
            sig = stream.signal
            with stream._lock:
                nblocks = len(stream._blocks)
                overflows = stream.fifo_overflow_count
            if sig is None or nblocks == blocks0:
                print("Waiting for samples...")
                continue

            rate = nblocks - blocks0
            # sig.time shape: (n_channels, n_samples)
            time_data = sig.time
            rms_per_ch = np.sqrt(np.mean(time_data ** 2, axis=-1))
            peak_per_ch = np.max(np.abs(time_data), axis=-1)

            ch_summary = " | ".join(
                [f"CH{i}: rms={rms_per_ch[i]:.4f}V peak={peak_per_ch[i]:.4f}V" for i in range(stream.n_channels)]
            )

            print(f"[{time.time() - t0:6.1f}s] blocks/s={rate:.0f} | fs={sig.sampling_rate} Hz | {ch_summary} | fifo_overflows={overflows}")
            blocks0 = nblocks
    except KeyboardInterrupt:
        print("\nStreaming stopped.")


def run_plot(stream: USB2406Stream):
    import matplotlib.pyplot as plt

    plt.ion()
    fig, axes = plt.subplots(stream.n_channels, 1, figsize=(10, 2 * stream.n_channels), sharex=True)
    if stream.n_channels == 1:
        axes = [axes]

    print("Plotting real-time waveforms... Close window or Ctrl+C to stop.")
    try:
        while not stream._stop.is_set():
            sig = stream.signal
            if sig is not None:
                t = np.arange(sig.n_samples) / sig.sampling_rate
                for i in range(stream.n_channels):
                    axes[i].cla()
                    axes[i].plot(t, sig.time[i], label=f"CH{i}")
                    axes[i].set_ylabel("Voltage (V)")
                    axes[i].grid(True, alpha=0.3)
                    axes[i].legend(loc="upper right")

                axes[-1].set_xlabel("Time (s)")
                fig.canvas.draw_idle()
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass


def run_periodic_tdms(
    stream: USB2406Stream,
    record_duration: float = 3.0,
    interval: float = 5.0,
    output_dir: str | Path = "tdms_data",
    count: int | None = None,
    prefix: str = "usb2406",
):
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    print("=== USB2406 Periodic TDMS Acquisition Started ===")
    print(f"Channels         : {stream.n_channels}")
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

            file_idx = 1
            while target_file.exists():
                target_file = out_path / f"{prefix}_{timestamp_str}_{file_idx}.tdms"
                file_idx += 1

            print(
                f"[{now_dt.strftime('%H:%M:%S')}] Cycle {iteration}: Recording {record_duration:.1f}s ({stream.n_channels} ch) ... ",
                end="",
                flush=True,
            )
            try:
                data, fs, meta = stream.record(duration_s=record_duration)
                ch_names = [f"CH{i}" for i in range(stream.n_channels)]
                save_to_tdms(
                    data=data,
                    sampling_rate=fs,
                    filepath=target_file,
                    channel_names=ch_names,
                    start_time=meta["start_time"],
                    properties={
                        "cycle_index": iteration,
                        "fifo_overflows": meta["fifo_overflows"],
                    },
                )
                samples_per_ch = data.shape[1]
                print(
                    f"Saved -> {target_file.name} "
                    f"({samples_per_ch} samples/ch @ {fs} Hz, {stream.n_channels} ch)"
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
    ap.add_argument("--card", type=int, default=0, help="USB card DIP ID (0-15)")
    ap.add_argument("--dll-path", type=str, default=DEFAULT_DLL_PATH, help="Path to USB2406.dll (default: USB2406_DLL_PATH or local driver search)")
    ap.add_argument("--channels", type=int, default=4, help="Number of channels to record (default: 4)")
    ap.add_argument("--fs", type=float, default=100.0, help="Sampling rate in kHz (default: 100.0)")
    ap.add_argument("--gain", type=int, default=0, help="Gain setting (0: ±10V, 1: ±1V, 2: ±100mV, 3: ±10mV)")
    ap.add_argument("--window", type=float, default=1.0, help="Signal window length in seconds")
    ap.add_argument("--stats", action="store_true", help="Display real-time multi-channel statistics")
    ap.add_argument("--plot", action="store_true", help="Plot real-time multi-channel waveforms")
    ap.add_argument("--simulate", action="store_true", help="Run in simulation mode without USB hardware")
    ap.add_argument("--tdms", action="store_true", help="Enable periodic TDMS recording")
    ap.add_argument("--tdms-interval", type=float, default=5.0, help="Interval between acquisitions in seconds")
    ap.add_argument("--tdms-duration", type=float, default=3.0, help="Duration of each acquisition in seconds")
    ap.add_argument("--tdms-dir", type=str, default="tdms_data", help="Output directory for TDMS files")
    ap.add_argument("--tdms-count", type=int, default=None, help="Maximum number of acquisition cycles")
    ap.add_argument("--tdms-prefix", type=str, default="usb2406", help="Filename prefix for TDMS files")
    args = ap.parse_args()

    if args.simulate:
        stream = SimulatedUSB2406Stream(
            window_s=args.window,
            fs=int(args.fs * 1000),
            n_channels=args.channels,
        ).start()
    else:
        stream = USB2406Stream(
            card_id=args.card,
            dll_path=args.dll_path,
            ch_start=0,
            ch_count=args.channels,
            fs_khz=args.fs,
            gain=args.gain,
            window_s=args.window,
        ).start()

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


if __name__ == "__main__":
    main()
