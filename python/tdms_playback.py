"""TDMS Audio Playback and Acoustic Inspection Tool.

Usage:
    # Play a specific file (by filename or full/relative path):
    python tdms_playback.py ad9833_20260903_132338.tdms
    python tdms_playback.py tdms_data/ad9833_20260903_132338.tdms

    # Automatically play the latest recorded TDMS file:
    python tdms_playback.py

    # List all available TDMS files:
    python tdms_playback.py --list

    # Specify custom gain or plot waveform:
    python tdms_playback.py ad9833_20260903_132338.tdms --gain 5.0
    python tdms_playback.py ad9833_20260903_132338.tdms --plot
"""

import argparse
import sys
from pathlib import Path

import nptdms
import numpy as np


def find_tdms_file(filename_or_path: str | None, search_dir: Path = Path("tdms_data")) -> Path:
    """Resolve file path whether given as pure filename, relative, or absolute path."""
    if filename_or_path:
        p = Path(filename_or_path)
        # 1. Direct path exists
        if p.exists() and p.is_file():
            return p
        # 2. Check with .tdms suffix appended
        if not p.suffix and p.with_suffix(".tdms").exists():
            return p.with_suffix(".tdms")
        # 3. Check inside search_dir
        in_dir = search_dir / p.name
        if in_dir.exists() and in_dir.is_file():
            return in_dir
        if not in_dir.suffix and in_dir.with_suffix(".tdms").exists():
            return in_dir.with_suffix(".tdms")

        raise FileNotFoundError(
            f"Could not find TDMS file '{filename_or_path}'. "
            f"Searched in current directory and '{search_dir.resolve()}'."
        )

    # If no filename provided, pick the latest one in search_dir
    files = sorted(search_dir.glob("*.tdms"), key=lambda x: x.stat().st_mtime)
    if not files:
        raise FileNotFoundError(f"No .tdms files found in directory: '{search_dir.resolve()}'")
    latest = files[-1]
    print(f"[Notice] No filename specified, playing latest file: {latest.name}\n")
    return latest


def list_files(search_dir: Path = Path("tdms_data")):
    """List all available TDMS files in directory."""
    if not search_dir.exists():
        print(f"Directory '{search_dir}' does not exist.")
        return
    files = sorted(search_dir.glob("*.tdms"), key=lambda x: x.stat().st_mtime, reverse=True)
    if not files:
        print(f"No .tdms files found in '{search_dir.resolve()}'.")
        return
    print(f"\nAvailable TDMS files in '{search_dir.resolve()}':")
    for i, f in enumerate(files, start=1):
        mtime = f.stat().st_mtime
        from datetime import datetime
        dt_str = datetime.fromtimestamp(mtime).strftime("%Y-%m-%d %H:%M:%S")
        size_kb = f.stat().st_size / 1024
        print(f"  [{i:2d}] {f.name:<32s}  {size_kb:7.1f} KB  ({dt_str})")
    print()


def play_audio(play_data: np.ndarray, fs: int):
    """Play audio via sounddevice or fallback to winsound."""
    try:
        import sounddevice as sd
        print("播放中 (sounddevice) ... ", end="", flush=True)
        sd.play(play_data, fs)
        sd.wait()
        print("播放完成。")
    except ImportError:
        import io
        import wave
        import winsound
        print("播放中 (Windows Audio) ... ", end="", flush=True)
        pcm16 = (np.clip(play_data, -1.0, 1.0) * 32767).astype(np.int16)
        buf = io.BytesIO()
        with wave.open(buf, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(fs)
            wf.writeframes(pcm16.tobytes())
        winsound.PlaySound(buf.getvalue(), winsound.SND_MEMORY)
        print("播放完成。")


def main():
    parser = argparse.ArgumentParser(description="TDMS Audio Playback and Acoustic Inspection Tool")
    parser.add_argument(
        "file",
        nargs="?",
        default=None,
        help="TDMS filename or path to play (e.g. ad9833_20260903_132338.tdms). If omitted, plays latest file.",
    )
    parser.add_argument("--list", action="store_true", help="List all available TDMS files in tdms_data/")
    parser.add_argument("--gain", type=float, default=None, help="Playback gain multiplier (default: auto-safe gain <= 0.8)")
    parser.add_argument("--plot", action="store_true", help="Plot waveform and sound pressure curve")
    parser.add_argument("--dir", default="tdms_data", help="Directory containing TDMS files (default: 'tdms_data')")
    args = parser.parse_args()

    search_dir = Path(args.dir)

    if args.list:
        list_files(search_dir)
        return

    try:
        tdms_file = find_tdms_file(args.file, search_dir=search_dir)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        list_files(search_dir)
        sys.exit(1)

    # 1. Read TDMS File
    print(f"Loading: {tdms_file.resolve()}")
    with nptdms.TdmsFile.read(tdms_file) as tdms:
        if "AD9833_Data" in tdms:
            group = tdms["AD9833_Data"]
        else:
            group = tdms.groups()[0]

        channels = group.channels()
        ch = channels[0]
        data = ch[:]

        dt = ch.properties.get("wf_increment", 1.0 / 48000.0)
        fs = int(round(1.0 / dt)) if dt > 0 else 48000

        # Read transducer calibration properties
        mic_model = ch.properties.get("transducer_model", "INMP441")
        pa_per_fs = ch.properties.get("pa_per_fs", 19.9526)
        sens_dbfs = ch.properties.get("sensitivity_dbfs", -26.0)

    # 2. Acoustic & Signal Metrics
    pressure_pa = data * pa_per_fs
    peak_fs = float(np.max(np.abs(data)))
    rms_fs = float(np.sqrt(np.mean(data ** 2)))
    peak_pa = float(np.max(np.abs(pressure_pa)))
    rms_pa = float(np.sqrt(np.mean(pressure_pa ** 2)))
    spl_db = 20.0 * np.log10(max(rms_pa, 1e-12) / 20e-6)

    print(f"=== 麦克风与信号特征 ===")
    print(f"  传感器型号   : {mic_model} (灵敏度: {sens_dbfs:.1f} dBFS @ 94 dB SPL)")
    print(f"  采样率 / 点数: {fs} Hz / {len(data)} 点 ({len(data)/fs:.2f} 秒)")
    print(f"  数字满量程峰值: {peak_fs:.6f} FS ({20*np.log10(max(peak_fs, 1e-12)):.1f} dBFS)")
    print(f"  物理声压峰值 : {peak_pa:.4f} Pa")
    print(f"  物理声压 RMS : {rms_pa:.4f} Pa")
    print(f"  等效声压级   : {spl_db:.1f} dB SPL")

    # 3. Playback Gain Handling
    if args.gain is not None:
        gain = args.gain
        print(f"  播放增益     : 手动设置 {gain:.2f}x")
    else:
        if peak_fs > 0:
            gain = min(20.0, 0.8 / peak_fs)
            print(f"  播放增益     : 自动自适应 {gain:.2f}x (限制最高电平 <= 0.8 防止破音)")
        else:
            gain = 1.0

    play_data = data * gain

    # 4. Optional Plotting
    if args.plot:
        import matplotlib.pyplot as plt
        t = np.arange(len(data)) / fs
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
        ax1.plot(t, data, color="#1f77b4", lw=0.8)
        ax1.set_ylabel("Digital (FS)")
        ax1.set_title(f"{tdms_file.name} - Signal Waveform")
        ax1.grid(True, alpha=0.3)

        ax2.plot(t, pressure_pa, color="#d62728", lw=0.8)
        ax2.set_ylabel("Sound Pressure (Pa)")
        ax2.set_xlabel("Time (s)")
        ax2.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show(block=False)

    # 5. Play Audio
    play_audio(play_data, fs)


if __name__ == "__main__":
    main()
