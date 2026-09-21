"""Multi-device GUI demo (PyQt5 + matplotlib + numpy) over the sensor
ctypes binding (dist name sensor-sdk).

Run (from the repo root, prebuilt lib/windows/x64/Debug/sensor.dll):
    python example_py/pyqt_demo.py

Requires: pip install PyQt5 matplotlib numpy
Optional: pip install scipy   (enables the Live Filter band selector)
"""

import sys
import signal
import time
import subprocess
import os
import threading
import collections
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from datetime import datetime
from typing import List, Optional

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

try:
    from scipy import signal as scipy_signal
except ImportError:
    scipy_signal = None    # Live Filter combo disabled, everything else works

from PyQt5 import QtWidgets, QtCore

#sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bindings", "python"))

from sensor import *
from sensor.sensor_data import SAMPLE_SIZE


SCAN_DEVICE_PERIOD_IN_MS   = 3000
PACKAGE_COUNT              = 32
POWER_REFRESH_PERIOD_IN_MS = 60000
PLOT_UPDATE_INTERVAL       = 50
FFT_UPDATE_INTERVAL        = 0.5
DEMO_VERSION               = "0.1.17"
BUFFER_SECONDS             = 5
BIO_BUFFER_SECONDS         = 1
POWER_STABLE_BAND          = 4

matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'Arial Unicode MS', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False
matplotlib.rcParams['lines.antialiased'] = False
matplotlib.rcParams['agg.path.chunksize'] = 10000


def _device_info_from_bin(d: Optional[dict]) -> DeviceInfo:
    info = DeviceInfo()
    for key, value in (d or {}).items():
        if not hasattr(info, key):
            continue
        try:
            setattr(info, key, value)
        except (TypeError, ValueError):
            pass
    return info

CHANNEL_LABELS = {
    DataType.NTF_ACC:        ["ACC-X", "ACC-Y", "ACC-Z"],
    DataType.NTF_GYRO:       ["GYRO-X", "GYRO-Y", "GYRO-Z"],
    DataType.NTF_EULER_DATA: ["Pitch(Y)", "Roll(X)", "Yaw(Z)"],
    DataType.NTF_QUATERNION: ["W", "X", "Y", "Z"],
}

DATA_TYPE_NAMES = {
    DataType.NTF_ACC:        "Acceleration (ACC)",
    DataType.NTF_GYRO:       "Gyroscope (GYRO)",
    DataType.NTF_EULER_DATA: "Euler Angle (Euler)",
    DataType.NTF_QUATERNION: "Quaternion (Quaternion)",
}

# Live Filter band options: (display name, (low, high)), None = off
FILTER_BANDS = (
    ("Off", None),
    ("delta 0.5-4Hz", (0.5, 4.0)),
    ("theta 4-8Hz", (4.0, 8.0)),
    ("alpha 8-13Hz", (8.0, 13.0)),
    ("beta 13-30Hz", (13.0, 30.0)),
    ("gamma 30-45Hz", (30.0, 45.0)),
)

EEG_CHANNEL_COLORS = plt.cm.tab10(np.linspace(0, 1, 8))

FIXED_Y_RANGES = {
    DataType.NTF_ACC: (-8, 8),
    DataType.NTF_GYRO: (-2000, 2000),
    DataType.NTF_EULER_DATA: (-180, 180),
    DataType.NTF_QUATERNION: (-1, 1),
}

# NTF_IMU aggregate channel windows: acc 0-2 / gyro 3-5 / euler 6-8 / quat 9-12
_IMU_AGGREGATE_SLICES = (
    (DataType.NTF_ACC,        0, 3),
    (DataType.NTF_GYRO,       3, 6),
    (DataType.NTF_EULER_DATA, 6, 9),
    (DataType.NTF_QUATERNION, 9, 13),
)


def split_imu_aggregate(data: SensorData) -> List[SensorData]:
    """Split one NTF_IMU aggregate batch into per-segment batches."""
    n_ch = data.channelCount
    n = data.sampleCount
    subs = []
    for dt, start, end in _IMU_AGGREGATE_SLICES:
        if n_ch < end:
            continue
        sub = data.clone()
        sub.dataType = dt
        sub.channelCount = end - start
        sub.lostPackageCount = data.lostPackageCount if not subs else 0
        sub._buf = sub._buf[start * n * SAMPLE_SIZE:end * n * SAMPLE_SIZE]
        subs.append(sub)
    return subs


def _ring_write(buf_row, idx_row, write_index, vals, indices):
    """One channel circular-buffer write."""
    buf_len = buf_row.shape[0]
    n = len(vals)
    if n == 0:
        return
    write_end = write_index + n
    if write_end <= buf_len:
        buf_row[write_index:write_end] = vals
        idx_row[write_index:write_end] = indices
    else:
        first = buf_len - write_index
        buf_row[write_index:] = vals[:first]
        buf_row[:n - first] = vals[first:]
        idx_row[write_index:] = indices[:first]
        idx_row[:n - first] = indices[first:]


GESTURE_DEFAULT_TEXT = (
    "Gesture:\n"
    "  gesture: -- (0-8)\n"
    "  raw gesture: -- (0-8)\n"
    "  possiblity: -- (0-100)\n"
    "  strength: -- (0-100)"
)

# PPG device right-side plot config: (data type, channel, title, color)
BIO_PLOT_CONFIG = [
    (DataType.NTF_EEG,  0, "fp1",   plt.cm.tab10(0)),
    (DataType.NTF_EEG,  1, "fp2",   plt.cm.tab10(1)),
    (DataType.NTF_PPG,  0, "red_led", plt.cm.tab10(2)),
    (DataType.NTF_PPG,  1, "ir_led",  plt.cm.tab10(3)),
    (DataType.NTF_SPO2, 0, "spo2",    plt.cm.tab10(4)),
    (DataType.NTF_SPO2, 1, "heart_rate", plt.cm.tab10(5)),
]

EEG_AXIS_COUNT = 8
PPG_AXIS_COUNT = len(BIO_PLOT_CONFIG)

SAMPLE_RATE_CANDIDATES = (250, 500, 1000, 2000)
EMG_SAMPLE_RATE_CANDIDATES = (500, 1000)
IMU_SAMPLE_RATE_CANDIDATES = (50, 100, 200, 250, 400, 500, 1000, 2000)
PPG_SAMPLE_RATE_CANDIDATES = (50, 100, 200, 400, 800, 1000, 1600, 3200)


class DeviceDataState:
    """Per-connected-device data buffers and display state."""

    def __init__(self, sensor: SensorProfile):
        self.sensor = sensor
        self.info: Optional[DeviceInfo] = None
        self.last_power: Optional[int] = None
        self.status_text = ""
        self.lost_counts: dict = {}
        self.ntf_states: dict = {}     # key -> (enabled, checked)
        self.filter_states: dict = {}  # key -> (enabled, checked)
        self.sample_rate_state: tuple = ([], 0)  # (options, current rate) EEG/ECG
        self.emg_sample_rate_state: tuple = ([], 0)  # (options, current rate) EMG
        self.imu_sample_rate_state: tuple = ([], 0)  # (options, current rate) IMU
        self.ppg_sample_rate_state: tuple = ([], 0)  # (options, current rate) PPG
        self.gesture = None            # (gesture, raw_gesture, possiblity, strength)

        self.buffers: dict = {}
        self.sample_rates: dict = {}
        self.sample_index_buffers: dict = {}
        self.buffer_indices: dict = {}
        self.buffer_locks: dict = {}

        self.eeg_buffer = None
        self.eeg_sample_index_buffer = None
        self.eeg_buffer_index = 0
        self.eeg_sample_rate = 0
        self.eeg_total_channels = 0
        self.eeg_page_index = 0
        self.eeg_channels_per_page = 8
        self.eeg_impedance: list = []
        self.eeg_buffer_lock = QtCore.QMutex()

        self.has_ecg = False
        self.ecg_buffer = None
        self.ecg_sample_index_buffer = None
        self.ecg_buffer_index = 0
        self.ecg_sample_rate = 0
        self.ecg_impedance: list = []
        self.ecg_buffer_lock = QtCore.QMutex()

        self.has_brth = False
        self.brth_buffer = None
        self.brth_sample_index_buffer = None
        self.brth_buffer_index = 0
        self.brth_sample_rate = 0
        self.brth_impedance: list = []
        self.brth_buffer_lock = QtCore.QMutex()

        self.has_mag_angle = False
        self.mag_angle_buffer = None
        self.mag_angle_sample_index_buffer = None
        self.mag_angle_buffer_index = 0
        self.mag_angle_sample_rate = 0
        self.mag_angle_buffer_lock = QtCore.QMutex()

        self.has_emg = False
        self.emg_buffer = None
        self.emg_sample_index_buffer = None
        self.emg_buffer_index = 0
        self.emg_sample_rate = 0
        self.emg_display_channels = 0
        self.emg_impedance: list = []
        self.emg_buffer_lock = QtCore.QMutex()

        # PPG-mode bio buffers (EEG fp1/fp2 + PPG + SpO2)
        self.bio_buffers: dict = {}
        self.bio_sample_index_buffers: dict = {}
        self.bio_buffer_indices: dict = {}
        self.bio_sample_rates: dict = {}
        self.bio_impedance: dict = {}
        self.bio_buffer_lock = QtCore.QMutex()

        # Live Filter state
        self.live_filter_band = None           # (lo, hi) or None
        self._filter_sos = None
        self._filter_sos_key = None
        self._filter_zi = {}

        # Right-side display mode: "eeg" / "emg" / "ppg"
        self.bio_kind: Optional[str] = None

        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.quaternion_lock = QtCore.QMutex()

        # Status line entries: (label, channels, nominal rate, data type)
        self.status_parts = None
        self.rate_lock = threading.Lock()
        self.rate_total_counts: dict = {}
        self.rate_stream_start: dict = {}
        self.rate_stream_tags: dict = {}
        self.rate_last_data_time = time.time()
        self.actual_rates: dict = {}
        self.nominal_rates: dict = {}
        self.nominal_channels: dict = {}
        self.stream_delay_ms = 0
        self.stream_start_time_sec = 0.0

    def note_data_received(self, data: SensorData):
        if data.channelCount <= 0 or data.sampleCount <= 0:
            return
        delay = data.delay
        if delay:
            self.stream_delay_ms = delay
        start_sec = data.startTimeSec
        if start_sec > 0:
            self.stream_start_time_sec = start_sec
        if data.dataType == DataType.NTF_IMU:
            for sub in split_imu_aggregate(data):
                self.note_data_received(sub)
            return
        arr = data.as_numpy()
        n = int(np.count_nonzero(arr["isLost"][0] == 0))
        dt = data.dataType
        now = time.time()
        with self.rate_lock:
            if n > 0:
                tag = data.startTimeStamp
                if self.rate_stream_tags.get(dt) != tag:
                    self.rate_stream_tags[dt] = tag
                    self.rate_total_counts[dt] = 0
                    self.rate_stream_start[dt] = now
                self.rate_total_counts[dt] += n
                self.rate_last_data_time = now
            if data.sampleRate and data.sampleRate > 0:
                self.nominal_rates[dt] = data.sampleRate
            ch = data.channelCount
            if ch > 0:
                self.nominal_channels[dt] = ch

    def refresh_actual_rates(self, now: float):
        with self.rate_lock:
            if now - self.rate_last_data_time > 2.0:
                self.actual_rates = {}
                self.rate_total_counts = {}
                self.rate_stream_start = {}
                self.rate_stream_tags = {}
                return
            self.actual_rates = {
                dt: total / max(now - self.rate_stream_start.get(dt, now), 1e-3)
                for dt, total in self.rate_total_counts.items()
            }

    def build_status_text(self) -> str:
        name = self.sensor.BLEDevice.Name if self.sensor is not None else ""
        parts = [f"Connected: {name}"]
        with self.rate_lock:
            nominal_rates = dict(self.nominal_rates)
            nominal_channels = dict(self.nominal_channels)
        for label, ch, sr, dt in (self.status_parts or []):
            nominal = nominal_rates.get(dt) or sr
            ch = nominal_channels.get(dt, ch)
            nominal_txt = f"{nominal:g}" if nominal else "--"
            entry = f"{label} {ch}ch @ {nominal_txt}Hz" if ch else f"{label} @ {nominal_txt}Hz"
            parts.append(entry)
        return " | ".join(parts)

    def build_rate_text(self) -> str:
        entries = []
        with self.rate_lock:
            actual_rates = dict(self.actual_rates)
            nominal_rates = dict(self.nominal_rates)
        for label, ch, sr, dt in (self.status_parts or []):
            actual = actual_rates.get(dt)
            if actual is None:
                continue
            nominal = nominal_rates.get(dt) or sr
            nominal_txt = f"{nominal:g}" if nominal else "--"
            entries.append(f"{label} {actual:.1f} / {nominal_txt}Hz")
        if self.stream_start_time_sec > 0:
            sec = self.stream_start_time_sec
            start_txt = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(sec)) + f".{int(sec * 1000) % 1000:03d}"
            entries.append(f"start {start_txt}")
        if self.stream_delay_ms:
            entries.append(f"delay {self.stream_delay_ms}ms")
        return "Actual: " + " | ".join(entries) if entries else ""

    def get_buffer_lock(self, data_type):
        lock = self.buffer_locks.get(data_type)
        if lock is None:
            lock = QtCore.QMutex()
            self.buffer_locks[data_type] = lock
        return lock

    def init_buffers(self, info: DeviceInfo, eeg_axis_count: int):
        configs = [
            (DataType.NTF_ACC,        info.AccSampleRate,   info.AccChannelCount),
            (DataType.NTF_GYRO,       info.GyroSampleRate,  info.GyroChannelCount),
            (DataType.NTF_EULER_DATA, info.EulerSampleRate, info.EulerChannelCount),
            (DataType.NTF_QUATERNION, info.QuatSampleRate,  info.QuatChannelCount),
        ]
        for dt, sr, ch in configs:
            if sr > 0 and ch > 0:
                buf_len = max(sr * BUFFER_SECONDS, 1)
                self.buffers[dt]               = np.zeros((ch, buf_len))
                self.sample_index_buffers[dt]  = np.zeros((ch, buf_len), dtype=np.int64)
                self.sample_rates[dt]          = sr
                self.buffer_indices[dt]        = 0

        if info.EegSampleRate > 0 and info.EegChannelCount > 0:
            self.eeg_sample_rate = info.EegSampleRate
            self.eeg_total_channels = info.EegChannelCount
            self.eeg_page_index = 0
            buf_len = max(info.EegSampleRate * BIO_BUFFER_SECONDS, 1)
            self.eeg_buffer = np.zeros((info.EegChannelCount, buf_len))
            self.eeg_sample_index_buffer = np.zeros((info.EegChannelCount, buf_len), dtype=np.int64)
            self.eeg_buffer_index = 0

        self.has_ecg = info.EcgSampleRate > 0 and info.EcgChannelCount > 0
        if self.has_ecg:
            self.ecg_sample_rate = info.EcgSampleRate
            buf_len = max(info.EcgSampleRate * BIO_BUFFER_SECONDS, 1)
            self.ecg_buffer = np.zeros((info.EcgChannelCount, buf_len))
            self.ecg_sample_index_buffer = np.zeros((info.EcgChannelCount, buf_len), dtype=np.int64)
            self.ecg_buffer_index = 0

        self.has_brth = info.BrthSampleRate > 0 and info.BrthChannelCount > 0
        if self.has_brth:
            self.brth_sample_rate = info.BrthSampleRate
            buf_len = max(info.BrthSampleRate * BIO_BUFFER_SECONDS, 1)
            self.brth_buffer = np.zeros((info.BrthChannelCount, buf_len))
            self.brth_sample_index_buffer = np.zeros((info.BrthChannelCount, buf_len), dtype=np.int64)
            self.brth_buffer_index = 0

        self.has_mag_angle = info.MagAngleSampleRate > 0 and info.MagAngleChannelCount > 0
        if self.has_mag_angle:
            self.mag_angle_sample_rate = info.MagAngleSampleRate
            buf_len = max(info.MagAngleSampleRate * BIO_BUFFER_SECONDS, 1)
            self.mag_angle_buffer = np.zeros((info.MagAngleChannelCount, buf_len))
            self.mag_angle_sample_index_buffer = np.zeros((info.MagAngleChannelCount, buf_len), dtype=np.int64)
            self.mag_angle_buffer_index = 0

        self.has_emg = info.EmgSampleRate > 0 and info.EmgChannelCount > 0
        if self.has_emg:
            self.emg_sample_rate = info.EmgSampleRate
            # A mag-angle stream takes the last bio row from the EMG channels
            self.emg_display_channels = min(info.EmgChannelCount, eeg_axis_count - int(self.has_mag_angle))
            buf_len = max(info.EmgSampleRate * BIO_BUFFER_SECONDS, 1)
            self.emg_buffer = np.zeros((self.emg_display_channels, buf_len))
            self.emg_sample_index_buffer = np.zeros((self.emg_display_channels, buf_len), dtype=np.int64)
            self.emg_buffer_index = 0
            self.emg_impedance = [None] * self.emg_display_channels

        # Bio display mode: PPG > EEG > EMG/MAG_ANGLE
        if info.PpgSampleRate > 0:
            self.bio_kind = "ppg"
        elif self.eeg_buffer is not None:
            self.bio_kind = "eeg"
        elif self.emg_buffer is not None or self.has_mag_angle:
            self.bio_kind = "emg"
        else:
            self.bio_kind = None

        if self.bio_kind == "ppg":
            bio_configs = [
                (DataType.NTF_EEG,  info.EegSampleRate,  info.EegChannelCount),
                (DataType.NTF_PPG,  info.PpgSampleRate,  info.PpgChannelCount),
                (DataType.NTF_SPO2, info.Spo2SampleRate, info.Spo2ChannelCount),
            ]
            for dt, sr, ch in bio_configs:
                if sr > 0 and ch > 0:
                    buf_len = max(sr * BUFFER_SECONDS, 1)
                    self.bio_buffers[dt] = np.zeros((ch, buf_len))
                    self.bio_sample_index_buffers[dt] = np.zeros((ch, buf_len), dtype=np.int64)
                    self.bio_buffer_indices[dt] = 0
                    self.bio_sample_rates[dt] = sr
                    self.bio_impedance[dt] = []

        extra_axes = int(self.has_ecg) + int(self.has_brth) + int(self.has_mag_angle)
        self.eeg_channels_per_page = eeg_axis_count - extra_axes

    def sync_bio_sample_rates(self, info: DeviceInfo) -> bool:
        """Rebuild bio ring buffers after a sample-rate change. Returns True
        when any buffer was rebuilt."""
        changed = False
        if (info.EegSampleRate > 0 and self.eeg_buffer is not None
                and self.eeg_sample_rate != info.EegSampleRate):
            ch = self.eeg_buffer.shape[0]
            buf_len = max(info.EegSampleRate * BIO_BUFFER_SECONDS, 1)
            self.eeg_buffer_lock.lock()
            try:
                self.eeg_sample_rate = info.EegSampleRate
                self.eeg_buffer = np.zeros((ch, buf_len))
                self.eeg_sample_index_buffer = np.zeros((ch, buf_len), dtype=np.int64)
                self.eeg_buffer_index = 0
            finally:
                self.eeg_buffer_lock.unlock()
            changed = True

        if (info.EcgSampleRate > 0 and self.has_ecg and self.ecg_buffer is not None
                and self.ecg_sample_rate != info.EcgSampleRate):
            ch = self.ecg_buffer.shape[0]
            buf_len = max(info.EcgSampleRate * BIO_BUFFER_SECONDS, 1)
            self.ecg_buffer_lock.lock()
            try:
                self.ecg_sample_rate = info.EcgSampleRate
                self.ecg_buffer = np.zeros((ch, buf_len))
                self.ecg_sample_index_buffer = np.zeros((ch, buf_len), dtype=np.int64)
                self.ecg_buffer_index = 0
            finally:
                self.ecg_buffer_lock.unlock()
            changed = True

        if (info.EmgSampleRate > 0 and self.emg_buffer is not None
                and self.emg_sample_rate != info.EmgSampleRate):
            ch = self.emg_buffer.shape[0]
            buf_len = max(info.EmgSampleRate * BIO_BUFFER_SECONDS, 1)
            self.emg_buffer_lock.lock()
            try:
                self.emg_sample_rate = info.EmgSampleRate
                self.emg_buffer = np.zeros((ch, buf_len))
                self.emg_sample_index_buffer = np.zeros((ch, buf_len), dtype=np.int64)
                self.emg_buffer_index = 0
            finally:
                self.emg_buffer_lock.unlock()
            changed = True

        if (info.BrthSampleRate > 0 and self.has_brth and self.brth_buffer is not None
                and self.brth_sample_rate != info.BrthSampleRate):
            ch = self.brth_buffer.shape[0]
            buf_len = max(info.BrthSampleRate * BIO_BUFFER_SECONDS, 1)
            self.brth_buffer_lock.lock()
            try:
                self.brth_sample_rate = info.BrthSampleRate
                self.brth_buffer = np.zeros((ch, buf_len))
                self.brth_sample_index_buffer = np.zeros((ch, buf_len), dtype=np.int64)
                self.brth_buffer_index = 0
            finally:
                self.brth_buffer_lock.unlock()
            changed = True

        if (info.MagAngleSampleRate > 0 and self.has_mag_angle
                and self.mag_angle_buffer is not None
                and self.mag_angle_sample_rate != info.MagAngleSampleRate):
            ch = self.mag_angle_buffer.shape[0]
            buf_len = max(info.MagAngleSampleRate * BIO_BUFFER_SECONDS, 1)
            self.mag_angle_buffer_lock.lock()
            try:
                self.mag_angle_sample_rate = info.MagAngleSampleRate
                self.mag_angle_buffer = np.zeros((ch, buf_len))
                self.mag_angle_sample_index_buffer = np.zeros((ch, buf_len), dtype=np.int64)
                self.mag_angle_buffer_index = 0
            finally:
                self.mag_angle_buffer_lock.unlock()
            changed = True

        # PPG mode: EEG fp1/fp2 + PPG + SpO2 live in the 5 s bio buffers
        if self.bio_kind == "ppg":
            bio_rate_map = {
                DataType.NTF_EEG:  info.EegSampleRate,
                DataType.NTF_PPG:  info.PpgSampleRate,
                DataType.NTF_SPO2: info.Spo2SampleRate,
            }
            for dt, sr in bio_rate_map.items():
                if sr <= 0 or self.bio_sample_rates.get(dt) in (None, sr):
                    continue
                buf = self.bio_buffers.get(dt)
                if buf is None:
                    continue
                ch = buf.shape[0]
                buf_len = max(sr * BUFFER_SECONDS, 1)
                self.bio_buffer_lock.lock()
                try:
                    self.bio_buffers[dt] = np.zeros((ch, buf_len))
                    self.bio_sample_index_buffers[dt] = np.zeros((ch, buf_len), dtype=np.int64)
                    self.bio_buffer_indices[dt] = 0
                    self.bio_sample_rates[dt] = sr
                finally:
                    self.bio_buffer_lock.unlock()
                changed = True
        return changed

    def sync_imu_sample_rates(self, info: DeviceInfo) -> list:
        """Rebuild the IMU ring buffers after a sample-rate change. Returns
        the rebuilt data types."""
        configs = [
            (DataType.NTF_ACC,        info.AccSampleRate),
            (DataType.NTF_GYRO,       info.GyroSampleRate),
            (DataType.NTF_EULER_DATA, info.EulerSampleRate),
            (DataType.NTF_QUATERNION, info.QuatSampleRate),
        ]
        changed = []
        for dt, sr in configs:
            buf = self.buffers.get(dt)
            if sr <= 0 or buf is None or self.sample_rates.get(dt) == sr:
                continue
            ch = buf.shape[0]
            buf_len = max(sr * BUFFER_SECONDS, 1)
            lock = self.get_buffer_lock(dt)
            lock.lock()
            try:
                self.buffers[dt] = np.zeros((ch, buf_len))
                self.sample_index_buffers[dt] = np.zeros((ch, buf_len), dtype=np.int64)
                self.buffer_indices[dt] = 0
                self.sample_rates[dt] = sr
            finally:
                lock.unlock()
            changed.append(dt)
        return changed

    def set_live_filter_band(self, band):
        self.live_filter_band = band
        self._filter_sos_key = None
        self._filter_zi = {}

    def apply_live_filter(self, dt, ch_idx: int, vals, sample_rate):
        """Causal bandpass on one channel batch; pass-through when off or
        the parameters are invalid."""
        band = self.live_filter_band
        if band is None or vals is None or len(vals) == 0:
            return vals
        if scipy_signal is None or not sample_rate or sample_rate <= 0:
            return vals
        lo, hi = band
        if hi >= sample_rate / 2:
            return vals
        try:
            key = (band, int(sample_rate))
            if self._filter_sos_key != key:
                self._filter_sos = scipy_signal.butter(
                    4, [lo, hi], btype="band", fs=sample_rate, output="sos")
                self._filter_sos_key = key
                self._filter_zi = {}
            sos = self._filter_sos
            zi = self._filter_zi.get(dt)
            if zi is None or zi.shape[1] <= ch_idx:
                zi0 = scipy_signal.sosfilt_zi(sos)
                zi = np.repeat(zi0[:, None, :], ch_idx + 1, axis=1)
                self._filter_zi[dt] = zi
            out, zi[:, ch_idx, :] = scipy_signal.sosfilt(sos, vals, zi=zi[:, ch_idx, :])
            return out.astype(np.float32)
        except Exception:
            return vals

    def append_data(self, data: SensorData):
        dt = data.dataType

        # PPG-mode bio buffers
        if self.bio_buffers and dt in self.bio_buffers:
            self.bio_buffer_lock.lock()
            try:
                buf = self.bio_buffers.get(dt)
                idx_buf = self.bio_sample_index_buffers.get(dt)
                if buf is None or idx_buf is None or data.channelCount == 0:
                    return
                arr = data.as_numpy()
                n = min(data.sampleCount, buf.shape[1])
                if n == 0:
                    return
                buf_idx = self.bio_buffer_indices.get(dt, 0)
                for ch_idx in range(min(arr.shape[0], buf.shape[0])):
                    vals = np.asarray(arr["data"][ch_idx], dtype=np.float32)
                    vals = self.apply_live_filter(dt, ch_idx, vals, data.sampleRate)
                    indices = np.asarray(arr["sampleIndex"][ch_idx], dtype=np.int64)
                    _ring_write(buf[ch_idx], idx_buf[ch_idx], buf_idx, vals[-n:], indices[-n:])
                    if dt == DataType.NTF_EEG:
                        imp = self.bio_impedance[dt]
                        while len(imp) <= ch_idx:
                            imp.append(0)
                        imp[ch_idx] = float(arr["impedance"][ch_idx, -1])
                self.bio_buffer_indices[dt] = (buf_idx + n) % buf.shape[1]
            finally:
                self.bio_buffer_lock.unlock()
            return

        if dt == DataType.NTF_EMG:
            self.emg_buffer_lock.lock()
            try:
                buf = self.emg_buffer
                idx_buf = self.emg_sample_index_buffer
                if buf is None or idx_buf is None:
                    return
                arr = data.as_numpy()
                n = min(data.sampleCount, buf.shape[1])
                if n == 0:
                    return
                write_start = self.emg_buffer_index
                for ch_idx in range(min(arr.shape[0], buf.shape[0])):
                    vals = np.asarray(arr["data"][ch_idx], dtype=np.float32)
                    vals = self.apply_live_filter(dt, ch_idx, vals, data.sampleRate)
                    indices = np.asarray(arr["sampleIndex"][ch_idx], dtype=np.int64)
                    _ring_write(buf[ch_idx], idx_buf[ch_idx], write_start, vals[-n:], indices[-n:])
                    while len(self.emg_impedance) <= ch_idx:
                        self.emg_impedance.append(None)
                    self.emg_impedance[ch_idx] = float(arr["impedance"][ch_idx, -1])
                self.emg_buffer_index = (write_start + n) % buf.shape[1]
            finally:
                self.emg_buffer_lock.unlock()
            return

        if dt == DataType.NTF_EEG:
            self.eeg_buffer_lock.lock()
            try:
                buf = self.eeg_buffer
                idx_buf = self.eeg_sample_index_buffer
                if buf is None or idx_buf is None:
                    return
                arr = data.as_numpy()
                n = min(data.sampleCount, buf.shape[1])
                if n == 0:
                    return
                write_start = self.eeg_buffer_index
                for ch_idx in range(min(arr.shape[0], buf.shape[0])):
                    vals = np.asarray(arr["data"][ch_idx], dtype=np.float32)
                    vals = self.apply_live_filter(dt, ch_idx, vals, data.sampleRate)
                    indices = np.asarray(arr["sampleIndex"][ch_idx], dtype=np.int64)
                    _ring_write(buf[ch_idx], idx_buf[ch_idx], write_start, vals[-n:], indices[-n:])
                    while len(self.eeg_impedance) <= ch_idx:
                        self.eeg_impedance.append(0)
                    self.eeg_impedance[ch_idx] = float(arr["impedance"][ch_idx, -1])
                self.eeg_buffer_index = (write_start + n) % buf.shape[1]
            finally:
                self.eeg_buffer_lock.unlock()
            return

        if dt == DataType.NTF_ECG:
            self.ecg_buffer_lock.lock()
            try:
                buf = self.ecg_buffer
                idx_buf = self.ecg_sample_index_buffer
                if buf is None or idx_buf is None:
                    return
                arr = data.as_numpy()
                n = min(data.sampleCount, buf.shape[1])
                if n == 0:
                    return
                write_start = self.ecg_buffer_index
                for ch_idx in range(min(arr.shape[0], buf.shape[0])):
                    vals = np.asarray(arr["data"][ch_idx], dtype=np.float32)
                    vals = self.apply_live_filter(dt, ch_idx, vals, data.sampleRate)
                    indices = np.asarray(arr["sampleIndex"][ch_idx], dtype=np.int64)
                    _ring_write(buf[ch_idx], idx_buf[ch_idx], write_start, vals[-n:], indices[-n:])
                    while len(self.ecg_impedance) <= ch_idx:
                        self.ecg_impedance.append(0)
                    self.ecg_impedance[ch_idx] = float(arr["impedance"][ch_idx, -1])
                self.ecg_buffer_index = (write_start + n) % buf.shape[1]
            finally:
                self.ecg_buffer_lock.unlock()
            return

        if dt == DataType.NTF_BRTH:
            self.brth_buffer_lock.lock()
            try:
                buf = self.brth_buffer
                idx_buf = self.brth_sample_index_buffer
                if buf is None or idx_buf is None:
                    return
                arr = data.as_numpy()
                n = min(data.sampleCount, buf.shape[1])
                if n == 0:
                    return
                write_start = self.brth_buffer_index
                for ch_idx in range(min(arr.shape[0], buf.shape[0])):
                    vals = np.asarray(arr["data"][ch_idx], dtype=np.float32)
                    vals = self.apply_live_filter(dt, ch_idx, vals, data.sampleRate)
                    indices = np.asarray(arr["sampleIndex"][ch_idx], dtype=np.int64)
                    _ring_write(buf[ch_idx], idx_buf[ch_idx], write_start, vals[-n:], indices[-n:])
                    while len(self.brth_impedance) <= ch_idx:
                        self.brth_impedance.append(0)
                    self.brth_impedance[ch_idx] = float(arr["impedance"][ch_idx, -1])
                self.brth_buffer_index = (write_start + n) % buf.shape[1]
            finally:
                self.brth_buffer_lock.unlock()
            return

        if dt == DataType.NTF_MAG_ANGLE_DATA:
            self.mag_angle_buffer_lock.lock()
            try:
                buf = self.mag_angle_buffer
                idx_buf = self.mag_angle_sample_index_buffer
                if buf is None or idx_buf is None:
                    return
                arr = data.as_numpy()
                n = min(data.sampleCount, buf.shape[1])
                if n == 0:
                    return
                write_start = self.mag_angle_buffer_index
                for ch_idx in range(min(arr.shape[0], buf.shape[0])):
                    vals = np.asarray(arr["data"][ch_idx], dtype=np.float32)
                    indices = np.asarray(arr["sampleIndex"][ch_idx], dtype=np.int64)
                    _ring_write(buf[ch_idx], idx_buf[ch_idx], write_start, vals[-n:], indices[-n:])
                self.mag_angle_buffer_index = (write_start + n) % buf.shape[1]
            finally:
                self.mag_angle_buffer_lock.unlock()
            return

        # IMU ring buffers
        lock = self.get_buffer_lock(dt)
        lock.lock()
        try:
            buf = self.buffers.get(dt)
            idx_buf = self.sample_index_buffers.get(dt)
            if buf is None or idx_buf is None:
                return
            arr = data.as_numpy()
            n = min(data.sampleCount, buf.shape[1])
            if n == 0:
                return
            buffer_index = self.buffer_indices.get(dt, 0)
            for ch_idx in range(min(arr.shape[0], buf.shape[0])):
                vals = np.asarray(arr["data"][ch_idx], dtype=np.float32)
                indices = np.asarray(arr["sampleIndex"][ch_idx], dtype=np.int64)
                _ring_write(buf[ch_idx], idx_buf[ch_idx], buffer_index, vals[-n:], indices[-n:])
            self.buffer_indices[dt] = (buffer_index + n) % buf.shape[1]
        finally:
            lock.unlock()

    def clear_buffers(self):
        for dt in list(self.buffers.keys()):
            lock = self.get_buffer_lock(dt)
            lock.lock()
            try:
                self.buffers[dt].fill(0)
                self.sample_index_buffers[dt].fill(0)
                self.buffer_indices[dt] = 0
            finally:
                lock.unlock()

        self.eeg_buffer_lock.lock()
        self.ecg_buffer_lock.lock()
        self.brth_buffer_lock.lock()
        self.emg_buffer_lock.lock()
        self.mag_angle_buffer_lock.lock()
        try:
            if self.eeg_buffer is not None:
                self.eeg_buffer.fill(0)
                self.eeg_sample_index_buffer.fill(0)
                self.eeg_buffer_index = 0
            if self.ecg_buffer is not None:
                self.ecg_buffer.fill(0)
                self.ecg_sample_index_buffer.fill(0)
                self.ecg_buffer_index = 0
            if self.brth_buffer is not None:
                self.brth_buffer.fill(0)
                self.brth_sample_index_buffer.fill(0)
                self.brth_buffer_index = 0
            if self.emg_buffer is not None:
                self.emg_buffer.fill(0)
                self.emg_sample_index_buffer.fill(0)
                self.emg_buffer_index = 0
            if self.mag_angle_buffer is not None:
                self.mag_angle_buffer.fill(0)
                self.mag_angle_sample_index_buffer.fill(0)
                self.mag_angle_buffer_index = 0
        finally:
            self.mag_angle_buffer_lock.unlock()
            self.emg_buffer_lock.unlock()
            self.brth_buffer_lock.unlock()
            self.ecg_buffer_lock.unlock()
            self.eeg_buffer_lock.unlock()

        self.bio_buffer_lock.lock()
        try:
            for dt in list(self.bio_buffers.keys()):
                self.bio_buffers[dt].fill(0)
                self.bio_sample_index_buffers[dt].fill(0)
                self.bio_buffer_indices[dt] = 0
        finally:
            self.bio_buffer_lock.unlock()


class IMUQuaternionEMGEEGDemo(QtWidgets.QWidget):
    power_changed_sig = QtCore.pyqtSignal(object, int)         # (sensor, power)
    device_info_sig = QtCore.pyqtSignal(object, object)        # (sensor, info)
    add_device_sig = QtCore.pyqtSignal(str)
    update_device_sig = QtCore.pyqtSignal(str, int)    # (address, rssi)
    lost_packet_signal = QtCore.pyqtSignal(str, str, int)    # (address, type_name, count)
    gesture_signal = QtCore.pyqtSignal(str, int, int, int, int)   # (address, gesture, raw, possiblity, strength)
    device_disconnected_sig = QtCore.pyqtSignal(str)         # address
    auto_reconnect_sig = QtCore.pyqtSignal(str, bool)        # (address, restore)
    ui_call_sig = QtCore.pyqtSignal(object)                  # callable to run on the UI thread
    replay_done_sig = QtCore.pyqtSignal(str)
    replay_member_done_sig = QtCore.pyqtSignal(object)       # (sensor)
    analyze_done_sig = QtCore.pyqtSignal(str, str)
    dongle_check_sig = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.discovered_devices = []
        self._scan_missed_rounds: dict = {}     # Address -> consecutive absent scan rounds
        self.current_sensor: SensorProfile = None
        self.device_states: dict = {}               # Address -> DeviceDataState
        self._connecting_addrs: set = set()         # Address with an in-flight connect
        self.sensor_controller = SensorController()

        self.active_data_type = DataType.NTF_ACC
        self._last_plotted_sample_indices = {}
        self.lines_2d = []

        self._last_drawn_quaternion = None
        self._last_3d_update_time = 0.0
        self.cube_vertices = None
        self.cube_faces = None

        self.eeg_lines = []
        self.ecg_line = None
        self.brth_line = None
        self.mag_angle_line = None
        self.emg_lines = []
        self.bio_lines = []
        self._eeg_display_channels = 0

        self._updating_ntf_controls = False
        self._updating_filter_controls = False
        self._updating_sample_rate_controls = False
        self._updating_emg_sample_rate_controls = False
        self._updating_imu_sample_rate_controls = False
        self._updating_ppg_sample_rate_controls = False
        self._debug_log_checkbox = None
        self._data_debug_log_checkbox = None
        self._dongle_debug_checkbox = None
        self._ntf_checkboxes: dict = {}
        self._filter_checkboxes: dict = {}
        self._sample_rate_radios: dict = {}
        self._sample_rate_button_group = None
        self._sample_rate_group = None
        self._emg_sample_rate_radios: dict = {}
        self._emg_sample_rate_button_group = None
        self._emg_sample_rate_group = None
        self._imu_sample_rate_radios: dict = {}
        self._imu_sample_rate_button_group = None
        self._imu_sample_rate_group = None
        self._ppg_sample_rate_radios: dict = {}
        self._ppg_sample_rate_button_group = None
        self._ppg_sample_rate_group = None
        self._debug_log_enabled = True
        self._data_debug_log_enabled = True
        self._dongle_debug_enabled = True
        self._last_log_paths: dict = {}
        self._last_data_log_paths: dict = {}
        self._saved_params_by_addr: dict = {}
        self._replay_sensor = None
        self._replay_active = False
        self._replay_paused = False
        self._replay_stop_requested = False
        self._replay_done_fired = False
        self._replay_path = ""
        self._replay_sensors = []
        self._replay_paths = []
        self._replay_multi_counts = None

        # FFT spectrum state
        self.fft_lines = []
        self._fft_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="FFT")
        self._fft_pending = False
        self._fft_last_submit = 0.0
        # Worker-thread results picked up by the plot timer (no queued signals)
        self._fft_result_lock = threading.Lock()
        self._fft_result = None        # (data_type, freqs, mags)
        self._bio_fft_result = None    # (data_type, {row: freqs}, {row: mags})

        # Bio per-row FFT state
        self.axes_bio_fft = []
        self.bio_fft_lines = []
        self._bio_fft_pending = False
        self._bio_fft_last_submit = 0.0
        self._eeg_axes_signature = None

        # Live Filter selection: (lo, hi) or None
        self._filter_band = None

        # Backend display in the SDK header label
        self._shown_backend = ""
        self._backend_query_pending = False
        self._backend_query_ms = 0.0

        # Data queue: the data callback only enqueues (a clone when Use Clone
        # Data is checked, the borrowed batch otherwise), a worker thread
        # dispatches into the display buffers
        self._use_clone_data = False
        self._auto_reconnect_enabled = True
        self._data_queue = collections.deque()
        self._data_queue_lock = threading.Lock()
        self._data_queue_event = threading.Event()
        self._data_worker_stop = False
        self._data_worker = threading.Thread(
            target=self._drain_data_queue, daemon=True, name="DataQueueDrain")
        self._data_worker.start()

        self._init_ui()

        if self._debug_log_enabled:
            self._apply_sdk_debug_log()

        self._apply_dongle_debug()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(PLOT_UPDATE_INTERVAL)
        self._rate_last_refresh = 0.0

        self.add_device_sig.connect(self._add_device_item)
        self.update_device_sig.connect(self._update_device_rssi)
        self.power_changed_sig.connect(self._update_power_display)
        self.device_info_sig.connect(self._update_link_info_display)
        self.lost_packet_signal.connect(self._update_lost_packet_display)
        self.gesture_signal.connect(self._update_gesture_display)
        self.device_disconnected_sig.connect(self._on_device_disconnected)
        self.auto_reconnect_sig.connect(self._press_connect_for_address)
        self.ui_call_sig.connect(self._run_ui_call)
        self.replay_done_sig.connect(self._on_replay_done)
        self.replay_member_done_sig.connect(self._finish_replay_member)
        self.analyze_done_sig.connect(self._on_analyze_done)
        self.dongle_check_sig.connect(self._on_dongle_check_result)

        self.sensor_controller.on_sensor_scan_result = self._on_device_found

    # -- UI --------------------------------------------------------------------

    def _init_ui(self):
        main_layout = QtWidgets.QHBoxLayout()

        left_layout = QtWidgets.QVBoxLayout()

        top_left_layout = QtWidgets.QVBoxLayout()
        self.figure_3d = plt.figure(figsize=(6, 6))
        self.canvas_3d = FigureCanvas(self.figure_3d)
        self.toolbar_3d = NavigationToolbar2QT(self.canvas_3d, self)
        top_left_layout.addWidget(QtWidgets.QLabel("3D Quaternion Visualization"))
        top_left_layout.addWidget(self.canvas_3d, stretch=18)
        top_left_layout.addWidget(self.toolbar_3d, stretch=1)

        self.ax_3d = self.figure_3d.add_subplot(111, projection='3d')
        self._setup_3d_plot()

        bottom_left_layout = QtWidgets.QVBoxLayout()
        self.figure_2d, (self.ax_2d, self.ax_fft) = plt.subplots(
            2, 1, gridspec_kw={"height_ratios": [3, 2]})
        self.figure_2d.subplots_adjust(hspace=0.45)
        self.canvas_2d = FigureCanvas(self.figure_2d)
        bottom_left_layout.addWidget(QtWidgets.QLabel("2D Waveform + FFT Spectrum (ACC/GYRO/Euler)"))
        bottom_left_layout.addWidget(self.canvas_2d)

        left_layout.addLayout(top_left_layout, stretch=1)
        left_layout.addLayout(bottom_left_layout, stretch=1)

        right_layout = QtWidgets.QVBoxLayout()

        controls_layout = QtWidgets.QVBoxLayout()

        self.btn_scan = QtWidgets.QPushButton("Start Scan")
        self.btn_scan.clicked.connect(self._start_scan)

        self.btn_stop_scan = QtWidgets.QPushButton("Stop Scan")
        self.btn_stop_scan.clicked.connect(self._stop_scan)
        self.btn_stop_scan.setEnabled(False)

        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self._connect_selected_device)
        self.btn_connect.setEnabled(False)

        self.btn_disconnect = QtWidgets.QPushButton("Disconnect")
        self.btn_disconnect.clicked.connect(self._disconnect_selected_device)
        self.btn_disconnect.setEnabled(False)

        self.btn_multi_sync = QtWidgets.QPushButton("Multi Start")
        self.btn_multi_sync.clicked.connect(self._multi_sync)
        self.btn_multi_sync.setEnabled(False)

        self.btn_multi_replay = QtWidgets.QPushButton("Multi Replay Bin")
        self.btn_multi_replay.clicked.connect(self._multi_replay_bin)

        self.btn_check_dongle = QtWidgets.QPushButton("Check Setup Dongle")
        self.btn_check_dongle.clicked.connect(self._check_setup_dongle)
        _bold_font = self.btn_check_dongle.font()
        _bold_font.setBold(True)
        self.btn_check_dongle.setFont(_bold_font)

        self.btn_replay = QtWidgets.QPushButton("Replay Bin File")
        self.btn_replay.clicked.connect(self._replay_bin_file)

        self.btn_analyze = QtWidgets.QPushButton("Analyze Bin")
        self.btn_analyze.clicked.connect(self._analyze_bin_file)

        self.btn_replay_pause = QtWidgets.QPushButton("Pause Replay")
        self.btn_replay_pause.clicked.connect(self._toggle_replay_pause)
        self.btn_replay_pause.setEnabled(False)

        self.btn_replay_stop = QtWidgets.QPushButton("Stop Replay")
        self.btn_replay_stop.clicked.connect(self._stop_replay)
        self.btn_replay_stop.setEnabled(False)

        sdk_header_layout = QtWidgets.QHBoxLayout()
        self.sdk_label = QtWidgets.QLabel(
            f"SDK: {self.sensor_controller.getVersion()} | Backend: --")
        sdk_header_layout.addWidget(self.sdk_label)
        sdk_header_layout.addStretch()
        sdk_header_layout.addWidget(self.btn_multi_sync)
        sdk_header_layout.addWidget(self.btn_multi_replay)
        sdk_header_layout.addWidget(self.btn_check_dongle)
        controls_layout.addLayout(sdk_header_layout)

        button_layout = QtWidgets.QVBoxLayout()
        button_layout.addWidget(self.btn_scan)
        button_layout.addWidget(self.btn_stop_scan)
        button_layout.addWidget(self.btn_connect)
        button_layout.addWidget(self.btn_disconnect)
        button_layout.addStretch()

        replay_button_layout = QtWidgets.QVBoxLayout()
        replay_button_layout.addWidget(self.btn_replay)
        replay_button_layout.addWidget(self.btn_analyze)
        replay_button_layout.addWidget(self.btn_replay_pause)
        replay_button_layout.addWidget(self.btn_replay_stop)
        replay_button_layout.addStretch()

        self.device_list = QtWidgets.QListWidget()
        self.device_list.setMaximumHeight(80)
        self.device_list.itemClicked.connect(self._on_device_selected)

        device_layout = QtWidgets.QVBoxLayout()
        device_header_layout = QtWidgets.QHBoxLayout()
        self.chk_auto_reconnect = QtWidgets.QCheckBox("Auto Reconnect")
        self.chk_auto_reconnect.setChecked(True)
        self.chk_auto_reconnect.toggled.connect(self._on_auto_reconnect_toggled)
        device_header_layout.addWidget(self.chk_auto_reconnect)
        self.chk_use_clone_data = QtWidgets.QCheckBox("Use Clone Data")
        self.chk_use_clone_data.setChecked(False)
        self.chk_use_clone_data.toggled.connect(self._on_use_clone_data_toggled)
        device_header_layout.addWidget(self.chk_use_clone_data)
        device_header_layout.addWidget(QtWidgets.QLabel("Discovered Devices:"))
        device_header_layout.addStretch()
        device_layout.addLayout(device_header_layout)
        device_layout.addWidget(self.device_list)

        scan_layout = QtWidgets.QHBoxLayout()
        scan_layout.addLayout(button_layout)
        scan_layout.addLayout(device_layout, stretch=1)
        scan_layout.addLayout(replay_button_layout)
        controls_layout.addLayout(scan_layout)

        type_layout = QtWidgets.QVBoxLayout()
        type_layout.addWidget(QtWidgets.QLabel("Bottom-left Display Data Type:"))
        self.type_combo = QtWidgets.QComboBox()
        for dt, name in DATA_TYPE_NAMES.items():
            self.type_combo.addItem(name, dt)
        self.type_combo.currentIndexChanged.connect(self._on_type_changed)
        type_layout.addWidget(self.type_combo)

        type_layout.addWidget(QtWidgets.QLabel("Live Filter:"))
        self.filter_combo = QtWidgets.QComboBox()
        for name, band in FILTER_BANDS:
            self.filter_combo.addItem(name, band)
        if scipy_signal is None:
            self.filter_combo.setEnabled(False)
        self.filter_combo.currentIndexChanged.connect(self._on_filter_combo_changed)
        type_layout.addWidget(self.filter_combo)

        type_layout.addStretch()

        self.value_labels: dict = {}
        self.value_box = QtWidgets.QGroupBox("Real-time Values")
        self.value_layout = QtWidgets.QVBoxLayout()
        self.value_box.setLayout(self.value_layout)

        self.lost_packet_label = QtWidgets.QLabel("Packet Loss Stats: None")
        self.lost_packet_label.setWordWrap(True)
        self.lost_packet_box = QtWidgets.QGroupBox("Packet Loss Stats")
        lost_packet_layout = QtWidgets.QVBoxLayout()
        lost_packet_layout.addWidget(self.lost_packet_label)
        self.lost_packet_box.setLayout(lost_packet_layout)

        self.gesture_label = QtWidgets.QLabel(GESTURE_DEFAULT_TEXT)
        self.gesture_label.setWordWrap(True)
        self.gesture_box = QtWidgets.QGroupBox("Gesture")
        gesture_layout = QtWidgets.QVBoxLayout()
        gesture_layout.addWidget(self.gesture_label)
        self.gesture_box.setLayout(gesture_layout)

        status_layout = QtWidgets.QHBoxLayout()
        status_layout.addWidget(self.value_box, stretch=1)
        status_layout.addWidget(self.lost_packet_box, stretch=1)
        status_layout.addWidget(self.gesture_box, stretch=1)

        display_layout = QtWidgets.QHBoxLayout()
        display_layout.addLayout(type_layout)
        display_layout.addLayout(status_layout, stretch=1)
        controls_layout.addLayout(display_layout)

        self.status_label = QtWidgets.QLabel("Not Connected")
        controls_layout.addWidget(self.status_label)
        self.rate_label = QtWidgets.QLabel("")
        controls_layout.addWidget(self.rate_label)

        device_info_layout = QtWidgets.QHBoxLayout()
        self.model_label = QtWidgets.QLabel("Model: --")
        self.hw_version_label = QtWidgets.QLabel("HW Version: --")
        self.fw_version_label = QtWidgets.QLabel("FW Version: --")
        self.link_label = QtWidgets.QLabel("Link: --")
        self.mtu_label = QtWidgets.QLabel("MTU: --")
        self.power_label = QtWidgets.QLabel("Power: --%")
        device_info_layout.addWidget(self.model_label)
        device_info_layout.addWidget(self.hw_version_label)
        device_info_layout.addWidget(self.fw_version_label)
        device_info_layout.addWidget(self.link_label)
        device_info_layout.addWidget(self.mtu_label)
        device_info_layout.addWidget(self.power_label)
        device_info_layout.addStretch()
        controls_layout.addLayout(device_info_layout)

        debug_log_group = QtWidgets.QGroupBox("Debug Log")
        debug_log_layout = QtWidgets.QVBoxLayout()
        self._debug_log_checkbox = QtWidgets.QCheckBox("Enable SDK Debug Log")
        self._debug_log_checkbox.setChecked(True)
        self._debug_log_checkbox.stateChanged.connect(self._on_debug_log_toggled)
        debug_log_layout.addWidget(self._debug_log_checkbox)
        self._data_debug_log_checkbox = QtWidgets.QCheckBox("Enable Debug Bin Data")
        self._data_debug_log_checkbox.setChecked(True)
        self._data_debug_log_checkbox.stateChanged.connect(self._on_data_debug_log_toggled)
        debug_log_layout.addWidget(self._data_debug_log_checkbox)
        self._dongle_debug_checkbox = QtWidgets.QCheckBox("Enable debug dongle")
        self._dongle_debug_checkbox.setChecked(True)
        self._dongle_debug_checkbox.stateChanged.connect(self._on_dongle_debug_toggled)
        debug_log_layout.addWidget(self._dongle_debug_checkbox)
        debug_log_group.setLayout(debug_log_layout)

        ntf_group = QtWidgets.QGroupBox("Data Notification")
        ntf_layout = QtWidgets.QHBoxLayout()
        self._ntf_checkboxes = {
            "NTF_EEG":  QtWidgets.QCheckBox("EEG"),
            "NTF_EMG":  QtWidgets.QCheckBox("EMG"),
            "NTF_GEST": QtWidgets.QCheckBox("GESTURE"),
            "NTF_PPG":  QtWidgets.QCheckBox("PPG"),
            "NTF_SPO2": QtWidgets.QCheckBox("SpO2"),
            "NTF_IMU":  QtWidgets.QCheckBox("IMU"),
            "NTF_MAG_ANGLE": QtWidgets.QCheckBox("Angle"),
        }
        for key, cb in self._ntf_checkboxes.items():
            cb.setChecked(True)
            cb.setEnabled(False)
            cb.stateChanged.connect(lambda state, k=key: self._on_ntf_toggled(k))
            ntf_layout.addWidget(cb)
        ntf_group.setLayout(ntf_layout)

        filter_group = QtWidgets.QGroupBox("Filter")
        filter_layout = QtWidgets.QHBoxLayout()
        self._filter_checkboxes = {
            "FILTER_50HZ": QtWidgets.QCheckBox("50Hz"),
            "FILTER_60HZ": QtWidgets.QCheckBox("60Hz"),
            "FILTER_HPF":  QtWidgets.QCheckBox("HPF"),
            "FILTER_LPF":  QtWidgets.QCheckBox("LPF"),
        }
        for key, cb in self._filter_checkboxes.items():
            cb.setChecked(True)
            cb.setEnabled(False)
            cb.stateChanged.connect(lambda state, k=key: self._on_filter_toggled(k))
            filter_layout.addWidget(cb)
        filter_group.setLayout(filter_layout)

        sample_rate_group = QtWidgets.QGroupBox("EEG Sample Rate")
        sample_rate_group.setVisible(False)
        self._sample_rate_group = sample_rate_group
        sample_rate_layout = QtWidgets.QHBoxLayout()
        self._sample_rate_button_group = QtWidgets.QButtonGroup(self)
        for rate in SAMPLE_RATE_CANDIDATES:
            rb = QtWidgets.QRadioButton(f"{rate} Hz")
            rb.setAutoExclusive(False)
            rb.setEnabled(False)
            rb.toggled.connect(lambda checked, r=rate: self._on_sample_rate_toggled(r, checked))
            self._sample_rate_radios[rate] = rb
            self._sample_rate_button_group.addButton(rb)
            sample_rate_layout.addWidget(rb)
        sample_rate_group.setLayout(sample_rate_layout)

        emg_sample_rate_group = QtWidgets.QGroupBox("EMG Sample Rate")
        emg_sample_rate_group.setVisible(False)
        self._emg_sample_rate_group = emg_sample_rate_group
        emg_sample_rate_layout = QtWidgets.QHBoxLayout()
        self._emg_sample_rate_button_group = QtWidgets.QButtonGroup(self)
        for rate in EMG_SAMPLE_RATE_CANDIDATES:
            rb = QtWidgets.QRadioButton(f"{rate} Hz")
            rb.setAutoExclusive(False)
            rb.setEnabled(False)
            rb.toggled.connect(lambda checked, r=rate: self._on_emg_sample_rate_toggled(r, checked))
            self._emg_sample_rate_radios[rate] = rb
            self._emg_sample_rate_button_group.addButton(rb)
            emg_sample_rate_layout.addWidget(rb)
        emg_sample_rate_group.setLayout(emg_sample_rate_layout)

        imu_sample_rate_group = QtWidgets.QGroupBox("IMU Sample Rate")
        imu_sample_rate_group.setVisible(False)
        self._imu_sample_rate_group = imu_sample_rate_group
        imu_sample_rate_layout = QtWidgets.QHBoxLayout()
        self._imu_sample_rate_button_group = QtWidgets.QButtonGroup(self)
        for rate in IMU_SAMPLE_RATE_CANDIDATES:
            rb = QtWidgets.QRadioButton(f"{rate} Hz")
            rb.setAutoExclusive(False)
            rb.setEnabled(False)
            rb.toggled.connect(lambda checked, r=rate: self._on_imu_sample_rate_toggled(r, checked))
            self._imu_sample_rate_radios[rate] = rb
            self._imu_sample_rate_button_group.addButton(rb)
            imu_sample_rate_layout.addWidget(rb)
        imu_sample_rate_group.setLayout(imu_sample_rate_layout)

        ppg_sample_rate_group = QtWidgets.QGroupBox("PPG Sample Rate")
        ppg_sample_rate_group.setVisible(False)
        self._ppg_sample_rate_group = ppg_sample_rate_group
        ppg_sample_rate_layout = QtWidgets.QHBoxLayout()
        self._ppg_sample_rate_button_group = QtWidgets.QButtonGroup(self)
        for rate in PPG_SAMPLE_RATE_CANDIDATES:
            rb = QtWidgets.QRadioButton(f"{rate} Hz")
            rb.setAutoExclusive(False)
            rb.setEnabled(False)
            rb.toggled.connect(lambda checked, r=rate: self._on_ppg_sample_rate_toggled(r, checked))
            self._ppg_sample_rate_radios[rate] = rb
            self._ppg_sample_rate_button_group.addButton(rb)
            ppg_sample_rate_layout.addWidget(rb)
        ppg_sample_rate_group.setLayout(ppg_sample_rate_layout)

        options_layout = QtWidgets.QHBoxLayout()
        options_layout.addWidget(debug_log_group, stretch=1)
        options_layout.addWidget(ntf_group, stretch=1)
        options_layout.addWidget(filter_group, stretch=1)
        options_layout.addWidget(sample_rate_group, stretch=1)
        options_layout.addWidget(emg_sample_rate_group, stretch=1)
        options_layout.addWidget(imu_sample_rate_group, stretch=1)
        options_layout.addWidget(ppg_sample_rate_group, stretch=1)
        controls_layout.addLayout(options_layout)

        controls_layout.addStretch()

        eeg_layout = QtWidgets.QVBoxLayout()

        page_controls_layout = QtWidgets.QHBoxLayout()
        self.btn_prev_page = QtWidgets.QPushButton("Prev")
        self.btn_prev_page.clicked.connect(self._prev_page)
        self.btn_prev_page.setEnabled(False)
        page_controls_layout.addWidget(self.btn_prev_page)

        self.page_label = QtWidgets.QLabel("Page 1 / 1")
        self.page_label.setAlignment(QtCore.Qt.AlignCenter)
        page_controls_layout.addWidget(self.page_label, stretch=1)

        self.btn_next_page = QtWidgets.QPushButton("Next")
        self.btn_next_page.clicked.connect(self._next_page)
        self.btn_next_page.setEnabled(False)
        page_controls_layout.addWidget(self.btn_next_page)

        eeg_layout.addLayout(page_controls_layout)

        self.figure_eeg, self.axes_eeg = plt.subplots(EEG_AXIS_COUNT, 1, sharex=True, figsize=(8, 12))
        self.figure_eeg.subplots_adjust(left=0.05, right=0.9, hspace=0.4)
        self.axes_bio_fft = [None] * EEG_AXIS_COUNT
        self._eeg_axes_signature = (EEG_AXIS_COUNT, None)
        self.canvas_eeg = FigureCanvas(self.figure_eeg)
        self.bio_title_label = QtWidgets.QLabel("EMG / EEG Waveform")
        eeg_layout.addWidget(self.bio_title_label)
        eeg_layout.addWidget(self.canvas_eeg)

        right_layout.addLayout(controls_layout, stretch=1)
        right_layout.addLayout(eeg_layout, stretch=4)

        main_layout.addLayout(left_layout, stretch=3)
        main_layout.addLayout(right_layout, stretch=7)
        self.setLayout(main_layout)
        self.setWindowTitle(f"SensorSDKCXX IMU + Quaternion + EMG + EEG Demo (sensor-sdk v{self.sensor_controller.getVersion()}, demo v{DEMO_VERSION})")
        self.resize(1600, 900)
        self.show()

    # -- Scan / Connect ----------------------------------------------------------

    def _start_scan(self):
        if not self.sensor_controller.isEnable:
            self._app_log("User: start scan rejected (Bluetooth disabled)", "W")
            self.status_label.setText("Please enable Bluetooth first")
            return
        self._app_log("User: start scan")
        if not self.sensor_controller.isScanning:
            self.sensor_controller.startScan(SCAN_DEVICE_PERIOD_IN_MS)
        self.btn_scan.setEnabled(False)
        self.btn_stop_scan.setEnabled(True)

    def _stop_scan(self):
        self._app_log("Stop scan")
        self.sensor_controller.stopScan()
        self.btn_scan.setEnabled(True)
        self.btn_stop_scan.setEnabled(False)

    def _check_setup_dongle(self):
        self._app_log("User: check setup dongle")
        self.btn_check_dongle.setEnabled(False)
        self.btn_check_dongle.setText("Checking Dongle...")

        if sys.platform.startswith("linux") and hasattr(os, "geteuid") and os.geteuid() != 0:
            shell = os.path.basename(os.environ.get("SHELL") or "") or "shell"
            QtWidgets.QMessageBox.information(
                self, "Check Dongle",
                f"Please input sudo password in !!{shell}!!")

        def work():
            try:
                result = self.sensor_controller.checkSetupDongle()
            except Exception as e:
                result = f"Error: {e}"
            self.dongle_check_sig.emit(result)

        threading.Thread(target=work, daemon=True).start()

    def _on_dongle_check_result(self, result: str):
        self._app_log(f"App: check dongle result: {result.splitlines()[0] if result else result}")
        self.btn_check_dongle.setEnabled(True)
        self.btn_check_dongle.setText("Check Setup Dongle")
        if result.startswith("OK"):
            first_line, _, extra = result.partition("\n")
            count = first_line.split(":", 1)[1].strip() if ":" in first_line else None
            msg = "USB BLE dongle is ready (driver installed and usable by the SDK)."
            if count is not None:
                msg += f"\nUsable dongle count: {count}"
            if extra:
                msg += f"\n{extra.strip()}"
            QtWidgets.QMessageBox.information(self, "Check Setup Dongle", msg)
        else:
            QtWidgets.QMessageBox.warning(self, "Check Setup Dongle", result)

    def _multi_sync(self):
        streaming = any(state.sensor.isDataTransfering
                        for state in self.device_states.values())
        if streaming:
            self._multi_stop()
        else:
            self._multi_start()

    def _multi_start(self):
        sensors = [state.sensor for state in self.device_states.values()
                   if state.sensor.isReady
                   and state.sensor.hasInited]
        if not sensors:
            self._app_log("User: multi start rejected (no connected device)", "W")
            self.status_label.setText("No connected device to sync-start")
            return
        self._app_log(f"User: multi start on {len(sensors)} device(s)")
        self.btn_multi_sync.setEnabled(False)
        try:
            transferring = [s for s in sensors if s.isDataTransfering]
            if transferring:
                stop_results = self.sensor_controller.multiStopDataNotification(transferring)
                stop_failed = [mac for mac, ok in stop_results.items() if not ok]
                if stop_failed:
                    self._app_log(f"App: multi stop failed on: {', '.join(stop_failed)}", "W")
                    self.status_label.setText(
                        f"Multi stop failed on: {', '.join(stop_failed)}")
                    return
            results = self._start_with_model_params(sensors)
            failed = [mac for mac, ok in results.items() if not ok]
            if failed:
                self._app_log(f"App: multi start failed on: {', '.join(failed)}", "W")
                self.status_label.setText(
                    f"Multi start failed on: {', '.join(failed)}")
            else:
                self._app_log(f"App: multi start OK: {len(results)} device(s) started")
                self.status_label.setText(
                    f"Multi start: {len(results)} device(s) started")
        finally:
            self._update_button_states()

    def _multi_stop(self):
        sensors = [state.sensor for state in self.device_states.values()
                   if state.sensor.isReady
                   and state.sensor.hasInited]
        if not sensors:
            self._app_log("User: multi stop rejected (no connected device)", "W")
            self.status_label.setText("No connected device to sync-stop")
            return
        self._app_log(f"User: multi stop on {len(sensors)} device(s)")
        self.btn_multi_sync.setEnabled(False)
        try:
            results = self.sensor_controller.multiStopDataNotification(sensors)
            failed = [mac for mac, ok in results.items() if not ok]
            if failed:
                self._app_log(f"App: multi stop failed on: {', '.join(failed)}", "W")
                self.status_label.setText(
                    f"Multi stop failed on: {', '.join(failed)}")
            else:
                self._app_log(f"App: multi stop OK: {len(results)} device(s) stopped")
                self.status_label.setText(
                    f"Multi stop: {len(results)} device(s) stopped")
        finally:
            self._update_button_states()

    def _start_with_model_params(self, sensors):
        # Same model for all devices -> defaults; mixed models -> no
        # dispersion check, longer timeout, more attempts
        model_names = set()
        for s in sensors:
            info = s.getDeviceInfo()
            model_names.add(info.ModelName if info else None)
        if len(model_names) == 1 and None not in model_names:
            results = self.sensor_controller.multiStartDataNotification(sensors)
        else:
            results = self.sensor_controller.multiStartDataNotification(
                sensors, timeout=60.0, maxDelayDispersionMs=-1, maxAttempts=5)
        return results

    def _on_auto_reconnect_toggled(self, checked: bool):
        self._auto_reconnect_enabled = bool(checked)
        self._app_log(f"User: auto reconnect {'ON' if checked else 'OFF'}")
        for state in self.device_states.values():
            state.sensor.setAutoReconnect(checked)

    def _on_use_clone_data_toggled(self, checked: bool):
        self._use_clone_data = checked
        self._app_log(f"User: use clone data {'ON' if checked else 'OFF'}")

    def _on_device_found(self, device_list: List[BLEDevice]):
        # Every batch is a full scan-round snapshot; merge it into the list in
        # place and keep scanning until the user presses Stop Scan.
        present = set()
        for d in device_list:
            present.add(d.Address)
            existing = next((x for x in self.discovered_devices if x.Address == d.Address), None)
            if existing is None:
                self.discovered_devices.append(d)
                self._scan_missed_rounds[d.Address] = 0
                self.add_device_sig.emit(f"RSSI: {d.RSSI}, Name: {d.Name}, Address: {d.Address}")
            else:
                existing.RSSI = d.RSSI
                self._scan_missed_rounds[d.Address] = 0
                self.update_device_sig.emit(d.Address, d.RSSI)
        self._evict_stale_devices(present)

    def _evict_stale_devices(self, present):
        # Rows absent from four consecutive scan rounds are dropped; connected
        # devices and replay rows are exempt.
        evicted = []
        for x in list(self.discovered_devices):
            if x.Address in present or x.Address in self.device_states:
                continue
            missed = self._scan_missed_rounds.get(x.Address, 0) + 1
            self._scan_missed_rounds[x.Address] = missed
            if missed < 4:
                continue
            self.discovered_devices.remove(x)
            self._scan_missed_rounds.pop(x.Address, None)
            evicted.append(x.Address)
        for addr in evicted:
            self._ui(lambda a=addr: self._remove_device_item(a))

    def _add_device_item(self, text: str):
        item = QtWidgets.QListWidgetItem(text)
        try:
            rssi = int(text.split("RSSI: ")[1].split(",")[0])
        except (IndexError, ValueError):
            rssi = None
        item.setData(QtCore.Qt.UserRole, rssi)
        # Insert by RSSI descending; existing rows keep their place, rows
        # without an RSSI sort last.
        pos = self.device_list.count()
        if rssi is not None:
            for i in range(self.device_list.count()):
                other = self.device_list.item(i).data(QtCore.Qt.UserRole)
                if not isinstance(other, int) or other < rssi:
                    pos = i
                    break
        self.device_list.insertItem(pos, item)

    def _remove_device_item(self, addr: str):
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            if f"Address: {addr}" not in item.text():
                continue
            was_current = self.device_list.currentItem() is item
            self.device_list.takeItem(i)
            if was_current:
                self.device_list.setCurrentItem(None)
            break

    def _update_device_rssi(self, addr: str, rssi: int):
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            text = item.text()
            if f"Address: {addr}" not in text:
                continue
            d = next((x for x in self.discovered_devices if x.Address == addr), None)
            if d is None:
                return
            new_text = f"RSSI: {rssi}, Name: {d.Name}, Address: {d.Address}"
            for p in self._DEVICE_ITEM_PREFIXES:
                if text.startswith(p):
                    new_text = p + new_text
                    break
            item.setText(new_text)
            item.setData(QtCore.Qt.UserRole, rssi)
            break

    def _selected_address(self) -> Optional[str]:
        item = self.device_list.currentItem()
        if item is None or "Address: " not in item.text():
            return None
        return item.text().split("Address: ")[1].strip()

    def _selected_list_device(self) -> Optional[BLEDevice]:
        addr = self._selected_address()
        if addr is None:
            return None
        return next((d for d in self.discovered_devices if d.Address == addr), None)

    def _current_state(self) -> Optional[DeviceDataState]:
        if self.current_sensor is None:
            return None
        return self.device_states.get(self.current_sensor.BLEDevice.Address)

    def _on_device_selected(self, item):
        addr = item.text().split("Address: ")[1].strip() if "Address: " in item.text() else None
        state = self.device_states.get(addr) if addr else None
        self.current_sensor = state.sensor if state is not None else None
        self._refresh_display_for_state(state)
        self._update_button_states()

    def _update_button_states(self):
        addr = self._selected_address()
        connected = addr is not None and addr in self.device_states
        connecting = addr is not None and addr in self._connecting_addrs
        self.btn_connect.setEnabled(addr is not None and not connected and not connecting)
        self.btn_disconnect.setEnabled(connected)
        streaming = any(state.sensor.isDataTransfering
                        for state in self.device_states.values())
        self.btn_multi_sync.setText("Multi Stop" if streaming else "Multi Start")
        self.btn_multi_sync.setEnabled(len(self.device_states) >= 1)

    _DEVICE_ITEM_PREFIXES = ("[Connected] ", "[Connecting...] ", "[Disconnecting...] ")

    def _set_device_item_prefix(self, addr: str, prefix: str):
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            text = item.text()
            if f"Address: {addr}" not in text:
                continue
            for p in self._DEVICE_ITEM_PREFIXES:
                if text.startswith(p):
                    text = text[len(p):]
                    break
            item.setText(prefix + text)
            break

    def _update_device_item_text(self, addr: str, connected: bool):
        self._set_device_item_prefix(addr, "[Connected] " if connected else "")

    def _run_ui_call(self, fn):
        fn()

    def _ui(self, fn):
        app = QtWidgets.QApplication.instance()
        if app is not None and QtCore.QThread.currentThread() is app.thread():
            fn()
        else:
            self.ui_call_sig.emit(fn)

    def _connect_selected_device(self):
        device = self._selected_list_device()
        # connect/init/startDataNotification 都是阻塞式 SDK 调用，放到
        # sensor.submit 的后台线程执行，避免长时间冻结 UI 线程
        submit(self._connect_device, device,
               self.chk_auto_reconnect.isChecked(), True)

    def _connect_device(self, device, auto_reconnect: bool, select_current: Optional[bool] = None):
        if device is None:
            self._app_log("User: connect rejected (no device selected)", "W")
            self._ui(lambda: self.status_label.setText("Please select a device in the list first"))
            return
        addr = device.Address
        if addr in self.device_states:
            return

        self._app_log(f"User: connect {device.Name} ({addr})")
        if self.sensor_controller.isScanning:
            # 本函数可能运行在 sensor.submit 的工作线程：SDK 停止扫描直接调用，
            # 按钮状态更新交回 UI 线程
            self._app_log("Stop scan")
            self.sensor_controller.stopScan()
            self._ui(lambda: (self.btn_scan.setEnabled(True),
                              self.btn_stop_scan.setEnabled(False)))
        sensor = self.sensor_controller.requireSensor(device)
        if sensor is None:
            self._app_log(f"App: failed to create SensorProfile for {addr}", "E")
            self._ui(lambda: self.status_label.setText("Failed to create SensorProfile"))
            return

        self._connecting_addrs.add(addr)
        self._ui(lambda a=addr: self._set_device_item_prefix(a, "[Connecting...] "))
        self._ui(self._update_button_states)

        sensor.on_sensor_notify_data = self._on_data
        sensor.on_state_change = self._on_state_changed
        sensor.on_error_callback = self._on_error
        sensor.on_power_changed = self._on_power_changed
        sensor.on_device_info_update = self._on_device_info_update
        sensor.on_auto_reconnect = self._on_auto_reconnect
        sensor.setAutoReconnect(auto_reconnect)

        self._ui(lambda: self.status_label.setText(f"Connecting: {device.Name} ..."))
        self._ui(lambda: self.btn_connect.setEnabled(False))

        if not sensor.isReady:
            ok = sensor.connect()
            if not ok:
                self._app_log(f"App: failed to connect to {device.Name} ({addr})", "E", sensor)
                self._connecting_addrs.discard(addr)
                self._ui(lambda a=addr: self._set_device_item_prefix(a, ""))
                self._ui(lambda: self.status_label.setText(f"Failed to connect to {device.Name}"))
                self._ui(self._update_button_states)
                return

        state = DeviceDataState(sensor)

        if not sensor.hasInited:
            ok = sensor.init(PACKAGE_COUNT,
                             POWER_REFRESH_PERIOD_IN_MS)
            if not ok:
                self._app_log(f"App: failed to initialize {device.Name} ({addr})", "E", sensor)
                self._connecting_addrs.discard(addr)
                self._ui(lambda a=addr: self._set_device_item_prefix(a, ""))
                self._ui(lambda: self.status_label.setText(f"Failed to initialize {device.Name}"))
                self._ui(self._update_button_states)
                return

            info = sensor.getDeviceInfo()
            state.info = info
            state.init_buffers(info, EEG_AXIS_COUNT)
            state.status_parts = [
                ("ACC",   info.AccChannelCount,   info.AccSampleRate,   DataType.NTF_ACC),
                ("Euler", info.EulerChannelCount, info.EulerSampleRate, DataType.NTF_EULER_DATA),
                ("Quat",  info.QuatChannelCount,  info.QuatSampleRate,  DataType.NTF_QUATERNION),
            ]
            if state.bio_kind == "emg":
                if info.EmgChannelCount > 0:
                    state.status_parts.append(
                        ("EMG", info.EmgChannelCount, info.EmgSampleRate, DataType.NTF_EMG))
            elif state.bio_kind == "ppg":
                state.status_parts.extend([
                    ("EEG",  info.EegChannelCount,  info.EegSampleRate,  DataType.NTF_EEG),
                    ("PPG",  info.PpgChannelCount,  info.PpgSampleRate,  DataType.NTF_PPG),
                    ("SpO2", info.Spo2ChannelCount, info.Spo2SampleRate, DataType.NTF_SPO2),
                ])
            else:
                state.status_parts.extend([
                    ("EEG",  info.EegChannelCount,  info.EegSampleRate,  DataType.NTF_EEG),
                    ("ECG",  info.EcgChannelCount,  info.EcgSampleRate,  DataType.NTF_ECG),
                    ("BRTH", info.BrthChannelCount, info.BrthSampleRate, DataType.NTF_BRTH),
                ])
            if info.GestChannelCount > 0:
                state.status_parts.append(("GEST", info.GestChannelCount, info.GestSampleRate, DataType.NTF_GEST))
            if info.MagAngleChannelCount > 0:
                state.status_parts.append(("Angle", info.MagAngleChannelCount, info.MagAngleSampleRate, DataType.NTF_MAG_ANGLE_DATA))
            state.status_text = state.build_status_text()

        if not sensor.isDataTransfering:
            ok = sensor.startDataNotification()
            if not ok:
                self._app_log(f"App: failed to start data stream on {addr}", "E", sensor)
                self._connecting_addrs.discard(addr)
                self._ui(lambda a=addr: self._set_device_item_prefix(a, ""))
                self._ui(lambda: self.status_label.setText("Failed to start data stream"))
                self._ui(self._update_button_states)
                return

        self._app_log(f"App: device connected and streaming: {device.Name} ({addr})", sensor=sensor)
        self.device_states[addr] = state
        self._ui(lambda: self._update_device_item_text(addr, connected=True))

        try:
            power = sensor.getBatteryLevel()
            if (power >= 0 and (state.last_power is None
                                or abs(power - state.last_power) >= POWER_STABLE_BAND)):
                state.last_power = power
        except Exception:
            pass

        if select_current is None:
            select_current = self._selected_address() == addr
        if select_current:
            self.current_sensor = sensor

        if self._debug_log_enabled:
            log_path = self._last_log_paths.get(addr) or "True"
            sensor.setParam("DEBUG_LOG_PATH", log_path)
            current = sensor.getParam("DEBUG_LOG_PATH")
            if current and not str(current).startswith("Error"):
                self._last_log_paths[addr] = current
        if self._data_debug_log_enabled:
            data_path = self._last_data_log_paths.get(addr) or "True"
            sensor.setParam("DEBUG_BLE_DATA_PATH", data_path)
            current = sensor.getParam("DEBUG_BLE_DATA_PATH")
            if current and not str(current).startswith("Error"):
                self._last_data_log_paths[addr] = current

        self._connecting_addrs.discard(addr)
        self._ui(lambda: self._refresh_control_states(sensor))

        if self.current_sensor == sensor:
            self._ui(lambda: self._refresh_display_for_state(state))

        self._ui(self._update_button_states)

    def _disconnect_selected_device(self):
        sensor = self.current_sensor
        if sensor is None:
            return
        addr = sensor.BLEDevice.Address
        self._app_log(f"User: disconnect {addr}", sensor=sensor)
        self._set_device_item_prefix(addr, "[Disconnecting...] ")
        self.btn_disconnect.setEnabled(False)
        self.btn_connect.setEnabled(False)
        for cb in self._ntf_checkboxes.values():
            cb.setEnabled(False)
        for cb in self._filter_checkboxes.values():
            cb.setEnabled(False)
        self.status_label.setText("Disconnecting...")
        # disconnect() 是阻塞式 SDK 调用，放后台线程执行；完成后由
        # _on_device_disconnected 清掉列表前缀并恢复按钮状态
        submit(sensor.disconnect)

    # -- Bin replay ----------------------------------------------------------------

    def _set_replay_mode_ui(self, replaying: bool):
        if replaying:
            if self.sensor_controller.isScanning:
                self.sensor_controller.stopScan()
            self.btn_stop_scan.setEnabled(False)
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(False)
            self.btn_multi_sync.setEnabled(False)
            self.btn_multi_replay.setEnabled(False)
        else:
            self._update_button_states()
            self.btn_multi_replay.setEnabled(True)
        self.btn_scan.setEnabled(not replaying)
        self.device_list.setEnabled(not replaying)
        self._debug_log_checkbox.setEnabled(not replaying)
        self._data_debug_log_checkbox.setEnabled(not replaying)
        self._dongle_debug_checkbox.setEnabled(not replaying)

    def _replay_bin_file(self):
        if self.device_states:
            self.status_label.setText("Please disconnect all devices before replaying a bin file")
            return
        if self._replay_active:
            return

        default_dir = Path.home() / "Documents" / "sensorsdklog"
        start_dir = str(default_dir) if default_dir.exists() else str(Path.home())
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Bin File", start_dir, "Bin Files (*.bin)"
        )
        if not path:
            return
        self._start_replay(path)

    def _multi_replay_bin(self):
        if self.device_states:
            self.status_label.setText("Please disconnect all devices before replaying bin files")
            return
        if self._replay_active:
            return

        default_dir = Path.home() / "Documents" / "sensorsdklog"
        start_dir = str(default_dir) if default_dir.exists() else str(Path.home())
        paths, _ = QtWidgets.QFileDialog.getOpenFileNames(
            self, "Select Bin Files", start_dir, "Bin Files (*.bin)"
        )
        if not paths:
            return
        if len(paths) == 1:
            self._start_replay(paths[0])
        else:
            self._start_multi_replay(paths)

    def _start_replay(self, path: str):
        self._app_log(f"User: replay bin file: {path}")
        try:
            config = self.sensor_controller.getBinFileInfo(path)
        except Exception as e:
            self._app_log(f"App: replay failed to read bin info: {e}", "E")
            self.status_label.setText(f"Replay failed: {e}")
            return
        if config is None:
            self._app_log(f"App: invalid bin file (no config record): {path}", "W")
            self.status_label.setText("Invalid bin file: no config record found")
            return

        mac = config.get("device_mac")
        if not mac:
            self.status_label.setText("Invalid bin file: config missing device_mac")
            return
        name = config.get("device_name") or ""

        info = _device_info_from_bin(config.get("device_info"))

        # The replay profile is created by the controller
        try:
            sensor = self.sensor_controller.replayBinFile(path, None, realtime=True, block=False)
        except Exception as e:
            self._app_log(f"App: replay failed to start: {e}", "E")
            self.status_label.setText(f"Replay failed to start: {e}")
            return
        if sensor is None:
            self.status_label.setText("Replay failed to start")
            return
        sensor.on_sensor_notify_data = self._on_data
        sensor.on_error_callback = self._on_error
        sensor.on_device_info_update = self._on_device_info_update
        sensor.on_data_transfer_state_change = self._on_replay_transfer_state
        sensor.setAutoReconnect(self.chk_auto_reconnect.isChecked())

        self._replay_sensor = sensor

        state = DeviceDataState(sensor)
        state.info = info
        state.init_buffers(info, EEG_AXIS_COUNT)
        duration = config.get("replay_duration", 0.0)
        version = config.get("version", "?")
        state.status_text = (
            f"Replaying: {Path(path).name} (config v{version}, duration {duration:.1f}s, realtime) ...")
        self.device_states[mac] = state
        self.current_sensor = sensor
        self._refresh_display_for_state(state)
        if info.EegSampleRate > 0:
            state.sample_rate_state = ([], int(info.EegSampleRate))
            self._set_sample_rate_checked(int(info.EegSampleRate))
        if info.EmgSampleRate > 0:
            state.emg_sample_rate_state = ([], int(info.EmgSampleRate))
            self._set_emg_sample_rate_checked(int(info.EmgSampleRate))
        if info.AccSampleRate > 0:
            state.imu_sample_rate_state = ([], int(info.AccSampleRate))
            self._set_imu_sample_rate_checked(int(info.AccSampleRate))

        self._replay_paused = False
        self._replay_stop_requested = False
        self._replay_done_fired = False
        self._replay_path = path
        self._replay_sensors = []
        self._replay_paths = []
        self._replay_multi_counts = None
        self._replay_active = True
        self.btn_replay.setEnabled(False)
        self.btn_replay_pause.setEnabled(True)
        self.btn_replay_pause.setText("Pause Replay")
        self.btn_replay_stop.setEnabled(True)
        self._set_replay_mode_ui(True)

    def _start_multi_replay(self, paths: list):
        self._app_log(f"User: replay bin files: {'; '.join(paths)}")
        members = []
        macs = []
        for path in paths:
            try:
                config = self.sensor_controller.getBinFileInfo(path)
            except Exception as e:
                self._app_log(f"App: invalid bin file ({e}): {path}", "W")
                continue
            mac = config.get("device_mac") if config is not None else None
            if config is None:
                self._app_log(f"App: invalid bin file (no config record): {path}", "W")
                continue
            if not mac:
                self._app_log(f"App: invalid bin file (config missing device_mac): {path}", "W")
                continue
            if mac in macs:
                self._app_log(f"App: duplicate device mac skipped: {mac}", "W")
                continue
            macs.append(mac)
            members.append((path, config))
        if len(members) < 2:
            self.status_label.setText("Multi replay needs at least 2 valid bin files")
            return

        member_paths = [p for p, _ in members]

        # The replay profiles are created by the controller
        try:
            profiles = self.sensor_controller.multiReplayBinFile(member_paths, macs, realtime=True, block=False)
        except Exception as e:
            self._app_log(f"App: multi replay failed to start: {e}", "E")
            self.status_label.setText(f"Multi replay failed to start: {e}")
            return

        started = []
        for (path, config), sensor in zip(members, profiles):
            if sensor is None:
                self._app_log(f"App: replay member failed to start: {path}", "W")
                continue
            sensor.on_sensor_notify_data = self._on_data
            sensor.on_error_callback = self._on_error
            sensor.on_device_info_update = self._on_device_info_update
            sensor.on_data_transfer_state_change = self._on_replay_transfer_state
            sensor.setAutoReconnect(self.chk_auto_reconnect.isChecked())

            info = _device_info_from_bin(config.get("device_info"))
            state = DeviceDataState(sensor)
            state.info = info
            state.init_buffers(info, EEG_AXIS_COUNT)
            duration = config.get("replay_duration", 0.0)
            state.status_text = (
                f"Replaying: {Path(path).name} (duration {duration:.1f}s, multi-sync) ...")
            self.device_states[config.get("device_mac")] = state
            if info.EegSampleRate > 0:
                state.sample_rate_state = ([], int(info.EegSampleRate))
            if info.EmgSampleRate > 0:
                state.emg_sample_rate_state = ([], int(info.EmgSampleRate))
            if info.AccSampleRate > 0:
                state.imu_sample_rate_state = ([], int(info.AccSampleRate))
            started.append(sensor)

        if not started:
            self.status_label.setText("Multi replay failed to start")
            return

        self._replay_sensor = None
        self._replay_sensors = started
        self._replay_paths = member_paths
        self._replay_multi_counts = (len(started), len(member_paths))
        self._replay_path = ""

        first_state = self.device_states[started[0].BLEDevice.Address]
        self.current_sensor = started[0]
        self._refresh_display_for_state(first_state)
        if first_state.sample_rate_state[1] > 0:
            self._set_sample_rate_checked(first_state.sample_rate_state[1])
        if first_state.emg_sample_rate_state[1] > 0:
            self._set_emg_sample_rate_checked(first_state.emg_sample_rate_state[1])
        if first_state.imu_sample_rate_state[1] > 0:
            self._set_imu_sample_rate_checked(first_state.imu_sample_rate_state[1])

        self._replay_paused = False
        self._replay_stop_requested = False
        self._replay_done_fired = False
        self._replay_active = True
        self.btn_replay.setEnabled(False)
        self.btn_replay_pause.setEnabled(True)
        self.btn_replay_pause.setText("Pause Replay")
        self.btn_replay_stop.setEnabled(True)
        self._set_replay_mode_ui(True)

    def _on_replay_transfer_state(self, sensor: SensorProfile, is_transferring: bool):
        if is_transferring:
            self._replay_active = True
        self._app_log(
            f"App: replay data transfer {'started' if is_transferring else 'stopped'}",
            sensor=sensor)
        if not is_transferring:
            if self._replay_sensors:
                self.replay_member_done_sig.emit(sensor)
            else:
                self._replay_active = False
                self._fire_replay_done()

    def _finish_replay_member(self, sensor: SensorProfile):
        if sensor not in self._replay_sensors:
            return
        mac = sensor.BLEDevice.Address
        self._replay_sensors.remove(sensor)
        self.device_states.pop(mac, None)
        if self.current_sensor is sensor:
            self.current_sensor = self._replay_sensors[0] if self._replay_sensors else None
            next_state = None
            if self.current_sensor is not None:
                next_state = self.device_states.get(self.current_sensor.BLEDevice.Address)
            self._refresh_display_for_state(next_state)
        if not self._replay_sensors:
            self._replay_active = False
            self._fire_replay_done()

    def _fire_replay_done(self, message: str = None):
        if self._replay_done_fired:
            return
        self._replay_done_fired = True
        if message is None:
            if self._replay_stop_requested:
                message = ("Multi replay stopped" if self._replay_paths
                           else "Replay stopped")
            elif self._replay_paths:
                ok, total = self._replay_multi_counts or (
                    len(self._replay_paths), len(self._replay_paths))
                message = f"Multi replay finished: {ok}/{total} device(s) ok"
            else:
                message = f"Replay finished: {Path(self._replay_path).name}"
        self.replay_done_sig.emit(message)

    def _toggle_replay_pause(self):
        sensors = list(self._replay_sensors) if self._replay_sensors else (
            [self._replay_sensor] if self._replay_sensor is not None else [])
        if not sensors:
            return
        action = "resume" if self._replay_paused else "pause"
        result = "OK"
        for sensor in sensors:
            if self._replay_paused:
                result = self.sensor_controller.resumeBinReplay(sensor)
            else:
                result = self.sensor_controller.pauseBinReplay(sensor)
            self._app_log(f"User: {action} replay -> {result}",
                          "I" if result == "OK" else "W", sensor)
        if result != "OK":
            self.status_label.setText(f"Replay pause/resume failed: {result}")
            return
        self._replay_paused = not self._replay_paused
        if self._replay_paused:
            self.btn_replay_pause.setText("Resume Replay")
            self.status_label.setText("Replay paused")
        else:
            self.btn_replay_pause.setText("Pause Replay")
            self.status_label.setText("Replaying ...")

    def _stop_replay(self):
        sensors = list(self._replay_sensors) if self._replay_sensors else (
            [self._replay_sensor] if self._replay_sensor is not None else [])
        if not sensors:
            return
        self._replay_stop_requested = True
        self.btn_replay_stop.setEnabled(False)
        self.btn_replay_pause.setEnabled(False)
        results = []
        for sensor in sensors:
            r = self.sensor_controller.stopBinReplay(sensor)
            results.append(r)
            self._app_log(f"User: stop replay -> {r}",
                          "I" if r == "OK" else "W", sensor)
        if all(r != "OK" for r in results):
            self.status_label.setText(f"Stop replay failed: {results[0]}")
            return
        self.status_label.setText("Stopping replay ...")

    def _on_replay_done(self, message: str):
        self._app_log(f"App: replay done: {message}")
        sensors = list(self._replay_sensors)
        if self._replay_sensor is not None and self._replay_sensor not in sensors:
            sensors.append(self._replay_sensor)
        for sensor in sensors:
            self.device_states.pop(sensor.BLEDevice.Address, None)
            if self.current_sensor is sensor:
                self.current_sensor = None
        self._replay_sensors = []
        self._replay_sensor = None
        self._refresh_display_for_state(None)
        self.status_label.setText(message)
        self.btn_replay.setEnabled(True)
        self.btn_replay_pause.setEnabled(False)
        self.btn_replay_pause.setText("Pause Replay")
        self.btn_replay_stop.setEnabled(False)
        self._set_replay_mode_ui(False)
        self._replay_paused = False
        self._replay_stop_requested = False
        self._replay_active = False
        self._replay_paths = []
        self._replay_multi_counts = None

    # -- Bin analyze -----------------------------------------------------------------

    def _analyze_bin_file(self):
        if getattr(self, "_analyze_busy", False):
            return
        default_dir = Path.home() / "Documents" / "sensorsdklog"
        start_dir = str(default_dir) if default_dir.exists() else str(Path.home())
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Bin File to Analyze", start_dir, "Bin Files (*.bin)"
        )
        if not path:
            return
        self._start_analyze(path)

    def _start_analyze(self, path: str):
        self._app_log(f"User: analyze bin file: {path}")
        self._analyze_busy = True
        self.btn_analyze.setEnabled(False)
        self.status_label.setText(f"Analyzing: {Path(path).name} ...")

        def work():
            try:
                result = self.sensor_controller.parseBinToCsv(path)
            except Exception as e:
                result = f"Error: {e}"
            self._analyze_busy = False
            if result.startswith("Error"):
                self.analyze_done_sig.emit("", result)
            else:
                self.analyze_done_sig.emit(result, "")

        threading.Thread(target=work, daemon=True).start()

    def _on_analyze_done(self, csv_path: str, error: str):
        self.btn_analyze.setEnabled(True)
        if error:
            self._app_log(f"App: analyze failed: {error}", "E")
            self.status_label.setText(f"Analyze failed: {error}")
            return
        self._app_log(f"App: CSV saved: {csv_path}")
        self.status_label.setText(f"CSV saved: {csv_path}")
        self._open_in_system_editor(csv_path)

    @staticmethod
    def _open_in_system_editor(path: str):
        try:
            if sys.platform == "darwin":
                subprocess.Popen(["open", path])
            elif sys.platform.startswith("win"):
                os.startfile(path)  # type: ignore[attr-defined]
            else:
                subprocess.Popen(["xdg-open", path])
        except Exception:
            pass

    # -- Data routing ------------------------------------------------------------------

    def _on_data(self, sensor: SensorProfile, data_list: list):
        addr = sensor.BLEDevice.Address
        state = self.device_states.get(addr)
        if state is None:
            return
        use_clone = self._use_clone_data
        for data in data_list:
            if data is None or data.channelCount == 0 or data.sampleCount == 0:
                continue
            state.note_data_received(data)
            self._enqueue_data(addr, data.clone() if use_clone else data)

    def _dispatch_sensor_data(self, addr: str, data: SensorData):
        state = self.device_states.get(addr)
        if state is None:
            return
        if not data.isDataValid():
            self._app_log("App: Your data process runs too slow", "W", state.sensor)
            return
        if state.live_filter_band is not self._filter_band:
            state.set_live_filter_band(self._filter_band)

        # Batch absolute timestamp (LSL)
        lsl_timestamp = data.getAbsTimeStampInSec(0, 0)

        if data.dataType == DataType.NTF_IMU:
            for sub in split_imu_aggregate(data):
                if sub.dataType in state.buffers:
                    self._append_sensor_data(addr, sub)
                if sub.dataType == DataType.NTF_QUATERNION:
                    self._update_quaternion(state, sub)
            return
        if (data.dataType in state.buffers
                or data.dataType in (DataType.NTF_EEG, DataType.NTF_ECG, DataType.NTF_BRTH,
                                     DataType.NTF_EMG, DataType.NTF_MAG_ANGLE_DATA)
                or (state.bio_buffers and data.dataType in state.bio_buffers)):
            self._append_sensor_data(addr, data)
        if data.dataType == DataType.NTF_QUATERNION:
            self._update_quaternion(state, data)
        if data.dataType == DataType.NTF_GEST:
            self._handle_gesture_data(addr, data)

    DATA_QUEUE_MAX_BATCHES = 1000

    def _enqueue_data(self, addr: str, data: SensorData):
        with self._data_queue_lock:
            while len(self._data_queue) >= self.DATA_QUEUE_MAX_BATCHES:
                self._data_queue.popleft()
            self._data_queue.append((addr, data))
        self._data_queue_event.set()

    def _drain_data_queue(self):
        while True:
            self._data_queue_event.wait()
            with self._data_queue_lock:
                batch = list(self._data_queue)
                self._data_queue.clear()
                self._data_queue_event.clear()
                stop = self._data_worker_stop
            for addr, data in batch:
                try:
                    self._dispatch_sensor_data(addr, data)
                except Exception as e:
                    print(f"[QueueData] dispatch error: {e}")
            if stop:
                return

    def _append_sensor_data(self, addr: str, data: SensorData):
        state = self.device_states.get(addr)
        if state is None:
            return
        if data.lostPackageCount > 0:
            dt = data.dataType
            type_name = dt.name if isinstance(dt, DataType) else "Unknown"
            self.lost_packet_signal.emit(addr, type_name, data.lostPackageCount)
        state.append_data(data)

    def closeEvent(self, event):
        self._app_log("App: demo window closing")
        self.timer.stop()
        self._data_worker_stop = True
        self._data_queue_event.set()
        self._data_worker.join(timeout=2)
        try:
            self._fft_executor.shutdown(wait=False)
        except Exception:
            pass
        try:
            self.sensor_controller.terminate()
        except Exception as e:
            print(f"[closeEvent] terminate error: {e}")
        event.accept()

    # -- Bottom-left 2D waveform -----------------------------------------------------

    def _on_type_changed(self, _):
        self.active_data_type = self.type_combo.currentData()
        self._app_log(f"User: display data type -> {self.type_combo.currentText()}")
        self._rebuild_2d_plot()

    def _rebuild_2d_plot(self):
        self.ax_2d.cla()
        self.lines_2d = []
        dt = self.active_data_type
        labels = CHANNEL_LABELS.get(dt, [])

        for lbl in self.value_labels.values():
            lbl.setParent(None)
        self.value_labels.clear()

        buf_copy = None
        idx_buf_copy = None
        buffer_index = 0
        state = self._current_state()
        if state is not None:
            lock = state.get_buffer_lock(dt)
            lock.lock()
            try:
                buf = state.buffers.get(dt)
                idx_buf = state.sample_index_buffers.get(dt)
                if buf is not None and idx_buf is not None:
                    buf_copy = buf.copy()
                    idx_buf_copy = idx_buf.copy()
                    buffer_index = state.buffer_indices.get(dt, 0)
            finally:
                lock.unlock()

        if buf_copy is None or idx_buf_copy is None:
            suffix = "(Not connected)" if state is None else "(Device not supported or disabled)"
            self.ax_2d.set_title(f"{DATA_TYPE_NAMES.get(dt, '')} {suffix}")
            self.ax_fft.cla()
            self.fft_lines = []
            self.canvas_2d.draw_idle()
            self._last_plotted_sample_indices.pop(dt, None)
            return

        self._last_plotted_sample_indices[dt] = int(idx_buf_copy.max())

        t = np.linspace(-BUFFER_SECONDS, 0, buf_copy.shape[1])
        for ch in range(buf_copy.shape[0]):
            label = labels[ch] if ch < len(labels) else f"ch{ch}"
            y_data = np.roll(buf_copy[ch], -buffer_index)
            (line,) = self.ax_2d.plot(t, y_data, label=label)
            self.lines_2d.append(line)

            row = QtWidgets.QLabel(f"{label}: --")
            row.setStyleSheet("font-family: monospace; font-size: 13px;")
            self.value_layout.addWidget(row)
            self.value_labels[label] = row

        self.ax_2d.set_title(DATA_TYPE_NAMES.get(dt, ""))
        self.ax_2d.set_xlabel("Time (s)")
        self.ax_2d.set_ylabel("Value")
        self.ax_2d.legend(loc="upper right")

        self.ax_fft.cla()
        self.fft_lines = []
        for line in self.lines_2d:
            (fft_line,) = self.ax_fft.plot([], [], label=line.get_label())
            self.fft_lines.append(fft_line)
        self.ax_fft.set_title(f"{DATA_TYPE_NAMES.get(dt, '')} Spectrum")
        self.ax_fft.set_xlabel("Frequency (Hz)")
        self.ax_fft.set_ylabel("Amplitude")
        if self.fft_lines:
            self.ax_fft.legend(loc="upper right")
        self.canvas_2d.draw_idle()

    # -- FFT spectrum -------------------------------------------------------------------

    def _submit_fft(self, dt, sample_rate: int, buf_snapshot):
        self._fft_pending = True

        def _compute_fft():
            try:
                window = np.hanning(buf_snapshot.shape[1])
                windowed = buf_snapshot * window
                mags = np.abs(np.fft.rfft(windowed, axis=1)) / max(window.sum(), 1e-12) * 2
                freqs = np.fft.rfftfreq(buf_snapshot.shape[1], d=1.0 / sample_rate)
                with self._fft_result_lock:
                    self._fft_result = (int(dt), freqs, mags)
            except Exception as e:
                print(f"[FFT] compute error: {e}")
            finally:
                self._fft_pending = False

        try:
            self._fft_executor.submit(_compute_fft)
        except RuntimeError:
            self._fft_pending = False

    def _poll_fft_results(self):
        result = None
        bio_result = None
        with self._fft_result_lock:
            result, self._fft_result = self._fft_result, None
            bio_result, self._bio_fft_result = self._bio_fft_result, None
        if result is not None:
            self._apply_fft_result(*result)
        if bio_result is not None:
            self._apply_bio_fft_result(*bio_result)

    def _apply_fft_result(self, dt_value: int, freqs, mags):
        if dt_value != int(self.active_data_type) or not self.fft_lines:
            return
        for ch, line in enumerate(self.fft_lines):
            if ch < mags.shape[0]:
                line.set_data(freqs, mags[ch])
        if freqs.size:
            self.ax_fft.set_xlim(0, freqs[-1])
            self.ax_fft.relim()
            self.ax_fft.autoscale_view(scalex=False)
        self.canvas_2d.draw_idle()

    def _submit_bio_fft(self, dt, row_specs):
        # row_specs: [(row, time-ordered 1-D snapshot, sample_rate), ...]
        self._bio_fft_pending = True

        def _compute_bio_fft():
            try:
                freqs_map = {}
                mags_map = {}
                for row, data, sr in row_specs:
                    if sr <= 0 or data.size == 0:
                        continue
                    window = np.hanning(data.size)
                    windowed = data * window
                    mags_map[row] = np.abs(np.fft.rfft(windowed)) / max(window.sum(), 1e-12) * 2
                    freqs_map[row] = np.fft.rfftfreq(data.size, d=1.0 / sr)
                with self._fft_result_lock:
                    self._bio_fft_result = (int(dt), freqs_map, mags_map)
            except Exception as e:
                print(f"[FFT] bio compute error: {e}")
            finally:
                self._bio_fft_pending = False

        try:
            self._fft_executor.submit(_compute_bio_fft)
        except RuntimeError:
            self._bio_fft_pending = False

    def _apply_bio_fft_result(self, dt_value: int, freqs_map, mags_map):
        state = self._current_state()
        expected = DataType.NTF_EEG
        if state is not None:
            if state.bio_kind == "emg":
                expected = DataType.NTF_EMG
            elif state.bio_kind == "ppg":
                expected = DataType.NTF_PPG
        if dt_value != int(expected) or not self.bio_fft_lines:
            return
        updated = False
        for row, line in enumerate(self.bio_fft_lines):
            if line is None:
                continue
            freqs = freqs_map.get(row)
            mags = mags_map.get(row)
            if freqs is None or mags is None:
                continue
            line.set_data(freqs, mags)
            ax = line.axes
            if freqs.size:
                ax.set_xlim(0, freqs[-1])
            ax.relim()
            ax.autoscale_view(scalex=False)
            updated = True
        if updated:
            self.canvas_eeg.draw_idle()

    # -- Right-side EMG / EEG (+ ECG + BRTH + Angle) waveform ---------------------------

    def _eeg_page_count(self) -> int:
        state = self._current_state()
        if state is None or state.eeg_total_channels <= 0:
            return 1
        return max(1, (state.eeg_total_channels + state.eeg_channels_per_page - 1) // state.eeg_channels_per_page)

    def _eeg_page_range(self):
        state = self._current_state()
        if state is None:
            return 0, 0
        page_count = self._eeg_page_count()
        state.eeg_page_index = max(0, min(state.eeg_page_index, page_count - 1))
        start = state.eeg_page_index * state.eeg_channels_per_page
        end = min(start + state.eeg_channels_per_page, state.eeg_total_channels)
        return start, end

    def _update_page_label(self):
        state = self._current_state()
        page_count = self._eeg_page_count()
        page_index = state.eeg_page_index if state is not None else 0
        self.page_label.setText(f"Page {page_index + 1} / {page_count}")

    def _update_page_buttons(self):
        state = self._current_state()
        page_count = self._eeg_page_count()
        page_index = state.eeg_page_index if state is not None else 0
        self.btn_prev_page.setEnabled(state is not None and page_index > 0)
        self.btn_next_page.setEnabled(state is not None and page_index < page_count - 1)

    def _prev_page(self):
        state = self._current_state()
        if state is not None and state.eeg_page_index > 0:
            state.eeg_page_index -= 1
            self._app_log(f"User: prev page -> {state.eeg_page_index}", "D")
            self._rebuild_eeg_plot()
            self._update_page_label()
            self._update_page_buttons()

    def _next_page(self):
        state = self._current_state()
        if state is None:
            return
        page_count = self._eeg_page_count()
        if state.eeg_page_index < page_count - 1:
            state.eeg_page_index += 1
            self._app_log(f"User: next page -> {state.eeg_page_index}", "D")
            self._rebuild_eeg_plot()
            self._update_page_label()
            self._update_page_buttons()

    def _reset_eeg_axes(self, count: int, fft_rows=None):
        # fft_rows: rows with a left FFT spectrum + right waveform split;
        # None = all rows full-width
        signature = (count, None if fft_rows is None else tuple(sorted(fft_rows)))
        if self._eeg_axes_signature == signature:
            return
        self._eeg_axes_signature = signature
        self.figure_eeg.clf()
        self.bio_fft_lines = []
        if fft_rows is None:
            axes = [self.figure_eeg.add_subplot(count, 1, 1)]
            for i in range(1, count):
                axes.append(self.figure_eeg.add_subplot(count, 1, i + 1, sharex=axes[0]))
            self.axes_eeg = axes
            self.axes_bio_fft = [None] * count
        else:
            grid = self.figure_eeg.add_gridspec(count, 2, width_ratios=[1, 1])
            axes = []
            fft_axes = []
            first_wave = None
            for i in range(count):
                if i in fft_rows:
                    fax = self.figure_eeg.add_subplot(grid[i, 0])
                    wax = self.figure_eeg.add_subplot(grid[i, 1], sharex=first_wave)
                else:
                    fax = None
                    wax = self.figure_eeg.add_subplot(grid[i, :], sharex=first_wave)
                if first_wave is None:
                    first_wave = wax
                axes.append(wax)
                fft_axes.append(fax)
            self.axes_eeg = axes
            self.axes_bio_fft = fft_axes
        self.figure_eeg.subplots_adjust(left=0.05, right=0.9, hspace=0.4, wspace=0.25)

    def _rebuild_eeg_plot(self):
        state = self._current_state()
        if state is not None and state.bio_kind == "ppg":
            self._rebuild_ppg_plot(state)
            return
        if state is not None and state.bio_kind == "emg":
            self._rebuild_emg_plot(state)
            return
        eeg_available = False
        ecg_available = False
        brth_available = False
        mag_angle_available = False
        eeg_buffer_copy = None
        eeg_idx_buf_copy = None
        eeg_buffer_index = 0
        ecg_buffer_copy = None
        ecg_idx_buf_copy = None
        ecg_buffer_index = 0
        brth_buffer_copy = None
        brth_idx_buf_copy = None
        brth_buffer_index = 0
        mag_angle_buffer_copy = None
        mag_angle_idx_buf_copy = None
        mag_angle_buffer_index = 0

        if state is not None:
            state.eeg_buffer_lock.lock()
            state.ecg_buffer_lock.lock()
            state.brth_buffer_lock.lock()
            state.mag_angle_buffer_lock.lock()
            try:
                eeg_available = state.eeg_buffer is not None and state.eeg_sample_index_buffer is not None
                ecg_available = state.has_ecg and state.ecg_buffer is not None and state.ecg_sample_index_buffer is not None
                brth_available = state.has_brth and state.brth_buffer is not None and state.brth_sample_index_buffer is not None
                mag_angle_available = (state.has_mag_angle and state.mag_angle_buffer is not None
                                       and state.mag_angle_sample_index_buffer is not None)
                if eeg_available:
                    eeg_buffer_copy = state.eeg_buffer.copy()
                    eeg_idx_buf_copy = state.eeg_sample_index_buffer.copy()
                    eeg_buffer_index = state.eeg_buffer_index
                if ecg_available:
                    ecg_buffer_copy = state.ecg_buffer.copy()
                    ecg_idx_buf_copy = state.ecg_sample_index_buffer.copy()
                    ecg_buffer_index = state.ecg_buffer_index
                if brth_available:
                    brth_buffer_copy = state.brth_buffer.copy()
                    brth_idx_buf_copy = state.brth_sample_index_buffer.copy()
                    brth_buffer_index = state.brth_buffer_index
                if mag_angle_available:
                    mag_angle_buffer_copy = state.mag_angle_buffer.copy()
                    mag_angle_idx_buf_copy = state.mag_angle_sample_index_buffer.copy()
                    mag_angle_buffer_index = state.mag_angle_buffer_index
            finally:
                state.mag_angle_buffer_lock.unlock()
                state.brth_buffer_lock.unlock()
                state.ecg_buffer_lock.unlock()
                state.eeg_buffer_lock.unlock()

        if not eeg_available:
            self._reset_eeg_axes(EEG_AXIS_COUNT, set())
            self.bio_fft_lines = []
            for ax in self.axes_eeg:
                ax.cla()
                ax.set_visible(True)
            suffix = "(Not connected)" if state is None else "(Device not supported or disabled)"
            self.bio_title_label.setText("EMG / EEG Waveform")
            self.axes_eeg[0].set_title(f"EMG / EEG {suffix}")
            self.canvas_eeg.draw_idle()
            self._last_plotted_sample_indices.pop(DataType.NTF_EEG, None)
            self._last_plotted_sample_indices.pop(DataType.NTF_ECG, None)
            self._last_plotted_sample_indices.pop(DataType.NTF_BRTH, None)
            self._last_plotted_sample_indices.pop(DataType.NTF_EMG, None)
            self._last_plotted_sample_indices.pop(DataType.NTF_MAG_ANGLE_DATA, None)
            self.eeg_lines = []
            self.ecg_line = None
            self.brth_line = None
            self.mag_angle_line = None
            self.emg_lines = []
            self.bio_lines = []
            self._eeg_display_channels = 0
            self._update_page_label()
            self._update_page_buttons()
            return

        bio_title = "EEG + ECG + BRTH Waveform"
        if mag_angle_available:
            bio_title = "EEG + ECG + BRTH + Angle Waveform"
        self.bio_title_label.setText(bio_title)
        self.emg_lines = []
        self.bio_lines = []
        self._last_plotted_sample_indices[DataType.NTF_EEG] = int(eeg_idx_buf_copy.max())
        if ecg_available:
            self._last_plotted_sample_indices[DataType.NTF_ECG] = int(ecg_idx_buf_copy.max())
        else:
            self.ecg_line = None
        if brth_available:
            self._last_plotted_sample_indices[DataType.NTF_BRTH] = int(brth_idx_buf_copy.max())
        else:
            self.brth_line = None
        if mag_angle_available:
            self._last_plotted_sample_indices[DataType.NTF_MAG_ANGLE_DATA] = int(mag_angle_idx_buf_copy.max())
        else:
            self.mag_angle_line = None

        start_ch, end_ch = self._eeg_page_range()
        page_eeg_count = max(0, end_ch - start_ch)
        self._eeg_display_channels = page_eeg_count

        # Extra rows are assigned from the bottom up: BRTH last, ECG above it, Angle above that
        brth_axis_index = EEG_AXIS_COUNT - 1 if brth_available else None
        ecg_axis_index = EEG_AXIS_COUNT - 1 - int(brth_available) if ecg_available else None
        mag_angle_axis_index = (EEG_AXIS_COUNT - 1 - int(brth_available) - int(ecg_available)
                                if mag_angle_available else None)

        # EEG channel rows and the ECG/Angle rows get the left FFT + right waveform split
        fft_rows = set(range(page_eeg_count))
        if ecg_axis_index is not None:
            fft_rows.add(ecg_axis_index)
        if mag_angle_axis_index is not None:
            fft_rows.add(mag_angle_axis_index)
        self._reset_eeg_axes(EEG_AXIS_COUNT, fft_rows)
        self.bio_fft_lines = [None] * len(self.axes_eeg)

        self.eeg_lines = []
        t = np.linspace(-BIO_BUFFER_SECONDS, 0, eeg_buffer_copy.shape[1])
        t_ecg = np.linspace(-BIO_BUFFER_SECONDS, 0, ecg_buffer_copy.shape[1]) if ecg_available else None
        t_brth = np.linspace(-BIO_BUFFER_SECONDS, 0, brth_buffer_copy.shape[1]) if brth_available else None
        t_mag_angle = (np.linspace(-BIO_BUFFER_SECONDS, 0, mag_angle_buffer_copy.shape[1])
                       if mag_angle_available else None)

        for ch, ax in enumerate(self.axes_eeg):
            ax.cla()
            if ch < page_eeg_count:
                eeg_ch = start_ch + ch
                color = EEG_CHANNEL_COLORS[eeg_ch % len(EEG_CHANNEL_COLORS)]
                y_data = np.roll(eeg_buffer_copy[eeg_ch], -eeg_buffer_index)
                (line,) = ax.plot(t, y_data, color=color, linewidth=0.8)
                self.eeg_lines.append(line)
                ax.tick_params(axis='both', labelsize=7)
                ax.ticklabel_format(axis='y', style='plain', useOffset=False)
                ax.set_xlim(-BIO_BUFFER_SECONDS, 0)
                ax.set_ylabel(f"EEG-{eeg_ch + 1}", fontsize=8, color=color, rotation=0, va='center', ha='left', labelpad=10)
                ax.yaxis.set_label_position("right")
                for spine in ax.spines.values():
                    spine.set_color(color)
                ax.set_visible(True)
                fax = self.axes_bio_fft[ch]
                if fax is not None:
                    fax.cla()
                    (fft_line,) = fax.plot([], [], color=color, linewidth=0.8)
                    self.bio_fft_lines[ch] = fft_line
                    fax.tick_params(axis='both', labelsize=7)
                    fax.set_ylabel(f"EEG-{eeg_ch + 1}", fontsize=8, color=color,
                                   rotation=0, va='center', ha='right', labelpad=10)
                    for spine in fax.spines.values():
                        spine.set_color(color)
            elif ch == ecg_axis_index and ecg_available:
                color = plt.cm.tab10(7)
                y_data = np.roll(ecg_buffer_copy[0], -ecg_buffer_index)
                (line,) = ax.plot(t_ecg, y_data, color=color, linewidth=0.8)
                self.ecg_line = line
                ax.tick_params(axis='both', labelsize=7)
                ax.ticklabel_format(axis='y', style='plain', useOffset=False)
                ax.set_xlim(-BIO_BUFFER_SECONDS, 0)
                ax.set_ylabel("ECG", fontsize=8, color=color, rotation=0, va='center', ha='left', labelpad=10)
                ax.yaxis.set_label_position("right")
                for spine in ax.spines.values():
                    spine.set_color(color)
                ax.set_visible(True)
                fax = self.axes_bio_fft[ch]
                if fax is not None:
                    fax.cla()
                    (fft_line,) = fax.plot([], [], color=color, linewidth=0.8)
                    self.bio_fft_lines[ch] = fft_line
                    fax.tick_params(axis='both', labelsize=7)
                    fax.set_ylabel("ECG", fontsize=8, color=color,
                                   rotation=0, va='center', ha='right', labelpad=10)
                    for spine in fax.spines.values():
                        spine.set_color(color)
            elif ch == mag_angle_axis_index and mag_angle_available:
                color = plt.cm.tab10(8)
                y_data = np.roll(mag_angle_buffer_copy[0], -mag_angle_buffer_index)
                (line,) = ax.plot(t_mag_angle, y_data, color=color, linewidth=0.8)
                self.mag_angle_line = line
                ax.tick_params(axis='both', labelsize=7)
                ax.ticklabel_format(axis='y', style='plain', useOffset=False)
                ax.set_xlim(-BIO_BUFFER_SECONDS, 0)
                ax.set_ylim(0, 180)
                ax.set_ylabel("Angle", fontsize=8, color=color, rotation=0, va='center', ha='left', labelpad=10)
                ax.yaxis.set_label_position("right")
                for spine in ax.spines.values():
                    spine.set_color(color)
                ax.set_visible(True)
                fax = self.axes_bio_fft[ch]
                if fax is not None:
                    fax.cla()
                    (fft_line,) = fax.plot([], [], color=color, linewidth=0.8)
                    self.bio_fft_lines[ch] = fft_line
                    fax.tick_params(axis='both', labelsize=7)
                    fax.set_ylabel("Angle", fontsize=8, color=color,
                                   rotation=0, va='center', ha='right', labelpad=10)
                    for spine in fax.spines.values():
                        spine.set_color(color)
            elif ch == brth_axis_index and brth_available:
                color = plt.cm.tab10(6)
                y_data = np.roll(brth_buffer_copy[0], -brth_buffer_index)
                (line,) = ax.plot(t_brth, y_data, color=color, linewidth=0.8)
                self.brth_line = line
                ax.tick_params(axis='both', labelsize=7)
                ax.ticklabel_format(axis='y', style='plain', useOffset=False)
                ax.set_xlim(-BIO_BUFFER_SECONDS, 0)
                ax.set_ylabel("BRTH", fontsize=8, color=color, rotation=0, va='center', ha='left', labelpad=10)
                ax.yaxis.set_label_position("right")
                for spine in ax.spines.values():
                    spine.set_color(color)
                ax.set_visible(True)
            else:
                ax.set_visible(False)

        self.axes_eeg[-1].set_xlabel("Time (s)", fontsize=8)
        fft_axes_used = [fax for fax in self.axes_bio_fft if fax is not None]
        if fft_axes_used:
            fft_axes_used[-1].set_xlabel("Frequency (Hz)", fontsize=8)
        self._update_page_label()
        self._update_page_buttons()
        self.canvas_eeg.draw_idle()

    def _rebuild_emg_plot(self, state: DeviceDataState):
        emg_available = False
        emg_buffer_copy = None
        emg_idx_buf_copy = None
        emg_buffer_index = 0
        display_channels = 0
        mag_angle_available = False
        mag_angle_buffer_copy = None
        mag_angle_idx_buf_copy = None
        mag_angle_buffer_index = 0

        state.emg_buffer_lock.lock()
        state.mag_angle_buffer_lock.lock()
        try:
            emg_available = state.emg_buffer is not None and state.emg_sample_index_buffer is not None
            if emg_available:
                emg_buffer_copy = state.emg_buffer.copy()
                emg_idx_buf_copy = state.emg_sample_index_buffer.copy()
                emg_buffer_index = state.emg_buffer_index
                display_channels = state.emg_display_channels
            mag_angle_available = (state.has_mag_angle and state.mag_angle_buffer is not None
                                   and state.mag_angle_sample_index_buffer is not None)
            if mag_angle_available:
                mag_angle_buffer_copy = state.mag_angle_buffer.copy()
                mag_angle_idx_buf_copy = state.mag_angle_sample_index_buffer.copy()
                mag_angle_buffer_index = state.mag_angle_buffer_index
        finally:
            state.mag_angle_buffer_lock.unlock()
            state.emg_buffer_lock.unlock()

        self.eeg_lines = []
        self.ecg_line = None
        self.brth_line = None
        self.mag_angle_line = None
        self.bio_lines = []
        self._eeg_display_channels = 0
        self._last_plotted_sample_indices.pop(DataType.NTF_EEG, None)
        self._last_plotted_sample_indices.pop(DataType.NTF_ECG, None)
        self._last_plotted_sample_indices.pop(DataType.NTF_BRTH, None)

        if not emg_available and not mag_angle_available:
            self._reset_eeg_axes(EEG_AXIS_COUNT, set())
            self.bio_fft_lines = []
            for ax in self.axes_eeg:
                ax.cla()
                ax.set_visible(True)
            self.bio_title_label.setText("EMG Waveform")
            self.axes_eeg[0].set_title("EMG (Device not supported or disabled)")
            self.canvas_eeg.draw_idle()
            self._last_plotted_sample_indices.pop(DataType.NTF_EMG, None)
            self._last_plotted_sample_indices.pop(DataType.NTF_MAG_ANGLE_DATA, None)
            self.emg_lines = []
            self._update_page_label()
            self._update_page_buttons()
            return

        self.bio_title_label.setText(
            "EMG + Angle Waveform" if emg_available and mag_angle_available
            else ("EMG Waveform" if emg_available else "Angle Waveform"))
        if emg_available:
            self._last_plotted_sample_indices[DataType.NTF_EMG] = int(emg_idx_buf_copy.max())
        if mag_angle_available:
            self._last_plotted_sample_indices[DataType.NTF_MAG_ANGLE_DATA] = int(mag_angle_idx_buf_copy.max())

        if display_channels == 0 and emg_available:
            display_channels = min(emg_buffer_copy.shape[0], EEG_AXIS_COUNT - int(mag_angle_available))

        # EMG channel rows and the Angle row get the left FFT + right waveform split
        fft_rows = set(range(display_channels))
        if mag_angle_available:
            fft_rows.add(display_channels)
        self._reset_eeg_axes(EEG_AXIS_COUNT, fft_rows)
        self.bio_fft_lines = [None] * len(self.axes_eeg)

        self.emg_lines = []
        t = np.linspace(-BIO_BUFFER_SECONDS, 0, emg_buffer_copy.shape[1]) if emg_available else None
        t_mag_angle = (np.linspace(-BIO_BUFFER_SECONDS, 0, mag_angle_buffer_copy.shape[1])
                       if mag_angle_available else None)
        for ch, ax in enumerate(self.axes_eeg):
            ax.cla()
            if ch < display_channels:
                color = EEG_CHANNEL_COLORS[ch % len(EEG_CHANNEL_COLORS)]
                y_data = np.roll(emg_buffer_copy[ch], -emg_buffer_index)
                (line,) = ax.plot(t, y_data, color=color, linewidth=0.8)
                self.emg_lines.append(line)
                ax.tick_params(axis='both', labelsize=7)
                ax.ticklabel_format(axis='y', style='plain', useOffset=False)
                ax.set_xlim(-BIO_BUFFER_SECONDS, 0)
                ax.set_ylabel(f"EMG-{ch + 1}", fontsize=8, color=color, rotation=0, va='center', ha='left', labelpad=10)
                ax.yaxis.set_label_position("right")
                for spine in ax.spines.values():
                    spine.set_color(color)
                ax.set_visible(True)
                fax = self.axes_bio_fft[ch]
                if fax is not None:
                    fax.cla()
                    (fft_line,) = fax.plot([], [], color=color, linewidth=0.8)
                    self.bio_fft_lines[ch] = fft_line
                    fax.tick_params(axis='both', labelsize=7)
                    fax.set_ylabel(f"EMG-{ch + 1}", fontsize=8, color=color,
                                   rotation=0, va='center', ha='right', labelpad=10)
                    for spine in fax.spines.values():
                        spine.set_color(color)
            elif ch == display_channels and mag_angle_available:
                color = plt.cm.tab10(8)
                y_data = np.roll(mag_angle_buffer_copy[0], -mag_angle_buffer_index)
                (line,) = ax.plot(t_mag_angle, y_data, color=color, linewidth=0.8)
                self.mag_angle_line = line
                ax.tick_params(axis='both', labelsize=7)
                ax.ticklabel_format(axis='y', style='plain', useOffset=False)
                ax.set_xlim(-BIO_BUFFER_SECONDS, 0)
                ax.set_ylim(0, 180)
                ax.set_ylabel("Angle", fontsize=8, color=color, rotation=0, va='center', ha='left', labelpad=10)
                ax.yaxis.set_label_position("right")
                for spine in ax.spines.values():
                    spine.set_color(color)
                ax.set_visible(True)
                fax = self.axes_bio_fft[ch]
                if fax is not None:
                    fax.cla()
                    (fft_line,) = fax.plot([], [], color=color, linewidth=0.8)
                    self.bio_fft_lines[ch] = fft_line
                    fax.tick_params(axis='both', labelsize=7)
                    fax.set_ylabel("Angle", fontsize=8, color=color,
                                   rotation=0, va='center', ha='right', labelpad=10)
                    for spine in fax.spines.values():
                        spine.set_color(color)
            else:
                ax.set_visible(False)

        self.axes_eeg[-1].set_xlabel("Time (s)", fontsize=8)
        fft_axes_used = [fax for fax in self.axes_bio_fft if fax is not None]
        if fft_axes_used:
            fft_axes_used[-1].set_xlabel("Frequency (Hz)", fontsize=8)
        self._update_page_label()
        self._update_page_buttons()
        self.canvas_eeg.draw_idle()

    def _rebuild_ppg_plot(self, state: DeviceDataState):
        # EEG fp1/fp2 and PPG red/ir rows get the left FFT + right waveform split
        fft_rows = {i for i, (dt, _, _, _) in enumerate(BIO_PLOT_CONFIG)
                    if dt in (DataType.NTF_EEG, DataType.NTF_PPG)}
        self._reset_eeg_axes(PPG_AXIS_COUNT, fft_rows)
        self.bio_fft_lines = [None] * len(self.axes_eeg)
        buffers_copy = {}
        idx_buffers_copy = {}
        has_any_data = False
        state.bio_buffer_lock.lock()
        try:
            for dt in (DataType.NTF_EEG, DataType.NTF_PPG, DataType.NTF_SPO2):
                buf = state.bio_buffers.get(dt)
                idx_buf = state.bio_sample_index_buffers.get(dt)
                if buf is not None and idx_buf is not None:
                    buf_idx = state.bio_buffer_indices.get(dt, 0)
                    buffers_copy[dt] = np.roll(buf.copy(), -buf_idx, axis=1)
                    idx_buffers_copy[dt] = idx_buf.copy()
                    has_any_data = True
        finally:
            state.bio_buffer_lock.unlock()

        self.eeg_lines = []
        self.ecg_line = None
        self.brth_line = None
        self.mag_angle_line = None
        self.emg_lines = []
        self._eeg_display_channels = 0
        self._last_plotted_sample_indices.pop(DataType.NTF_MAG_ANGLE_DATA, None)

        if not has_any_data:
            for ax in self.axes_eeg:
                ax.cla()
            for fax in self.axes_bio_fft:
                if fax is not None:
                    fax.cla()
            self.bio_title_label.setText("EEG + PPG + SpO2 Waveform")
            self.axes_eeg[0].set_title("EEG + PPG + SpO2 (Device not supported or disabled)")
            self.canvas_eeg.draw_idle()
            for dt in (DataType.NTF_EEG, DataType.NTF_PPG, DataType.NTF_SPO2):
                self._last_plotted_sample_indices.pop(dt, None)
            self.bio_lines = []
            self._update_page_label()
            self._update_page_buttons()
            return

        self.bio_title_label.setText("EEG + PPG + SpO2 Waveform")
        for dt, idx_buf in idx_buffers_copy.items():
            self._last_plotted_sample_indices[dt] = int(idx_buf.max())

        self.bio_lines = []
        for plot_idx, (dt, ch_idx, title, color) in enumerate(BIO_PLOT_CONFIG):
            ax = self.axes_eeg[plot_idx]
            ax.cla()

            buf_copy = buffers_copy.get(dt)
            if buf_copy is not None and ch_idx < buf_copy.shape[0]:
                t = np.linspace(-BUFFER_SECONDS, 0, buf_copy.shape[1])
                (line,) = ax.plot(t, buf_copy[ch_idx], color=color, linewidth=0.8)
            else:
                (line,) = ax.plot([], [], color=color, linewidth=0.8)

            self.bio_lines.append(line)
            ax.tick_params(axis='both', labelsize=7)
            ax.ticklabel_format(axis='y', style='plain', useOffset=False)
            ax.set_xlim(-BUFFER_SECONDS, 0)
            ax.set_ylabel(title, fontsize=8, color=color, rotation=0, va='center', ha='left', labelpad=10)
            ax.yaxis.set_label_position("right")
            for spine in ax.spines.values():
                spine.set_color(color)
            fax = self.axes_bio_fft[plot_idx]
            if fax is not None:
                fax.cla()
                (fft_line,) = fax.plot([], [], color=color, linewidth=0.8)
                self.bio_fft_lines[plot_idx] = fft_line
                fax.tick_params(axis='both', labelsize=7)
                fax.set_ylabel(title, fontsize=8, color=color,
                               rotation=0, va='center', ha='right', labelpad=10)
                for spine in fax.spines.values():
                    spine.set_color(color)

        self.axes_eeg[-1].set_xlabel("Time (s)", fontsize=8)
        fft_axes_used = [fax for fax in self.axes_bio_fft if fax is not None]
        if fft_axes_used:
            fft_axes_used[-1].set_xlabel("Frequency (Hz)", fontsize=8)
        self._update_page_label()
        self._update_page_buttons()
        self.canvas_eeg.draw_idle()

    # -- 3D quaternion -----------------------------------------------------------------------

    def _setup_3d_plot(self):
        self.ax_3d.clear()
        self.ax_3d.set_xlim([-2, 2])
        self.ax_3d.set_ylim([-2, 2])
        self.ax_3d.set_zlim([-2, 2])
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.set_title('IMU Quaternion Visualization (3D Cube)')
        self._create_cube()

        face_colors = ['cyan', 'magenta', 'yellow', 'red', 'green', 'blue']
        self._cube_collection = Poly3DCollection(self.cube_faces, facecolors=face_colors,
                                                linewidths=1, edgecolors='black', alpha=1.0)
        self.ax_3d.add_collection3d(self._cube_collection)

        axis_length = 1.5
        self._quiver_arrows = (
            self.ax_3d.quiver(0, 0, 0, axis_length, 0, 0,
                              color='r', arrow_length_ratio=0.1, linewidth=2),
            self.ax_3d.quiver(0, 0, 0, 0, axis_length, 0,
                              color='g', arrow_length_ratio=0.1, linewidth=2),
            self.ax_3d.quiver(0, 0, 0, 0, 0, axis_length,
                              color='b', arrow_length_ratio=0.1, linewidth=2),
        )
        self._draw_cube([1.0, 0.0, 0.0, 0.0])

    def _create_cube(self):
        vertices = np.array([
            [-1, -1, -1],
            [ 1, -1, -1],
            [ 1,  1, -1],
            [-1,  1, -1],
            [-1, -1,  1],
            [ 1, -1,  1],
            [ 1,  1,  1],
            [-1,  1,  1]
        ])
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],
            [vertices[4], vertices[5], vertices[6], vertices[7]],
            [vertices[0], vertices[1], vertices[5], vertices[4]],
            [vertices[2], vertices[3], vertices[7], vertices[6]],
            [vertices[0], vertices[3], vertices[7], vertices[4]],
            [vertices[1], vertices[2], vertices[6], vertices[5]]
        ]
        self.cube_vertices = vertices
        self.cube_faces = faces

    def _quaternion_to_rotation_matrix(self, q):
        w, x, y, z = q
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        if norm > 0:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])

    def _draw_cube(self, quaternion):
        R = self._quaternion_to_rotation_matrix(quaternion)
        rotated_vertices = np.dot(self.cube_vertices, R.T)
        rotated_faces = [
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[2], rotated_vertices[3]],
            [rotated_vertices[4], rotated_vertices[5], rotated_vertices[6], rotated_vertices[7]],
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[5], rotated_vertices[4]],
            [rotated_vertices[2], rotated_vertices[3], rotated_vertices[7], rotated_vertices[6]],
            [rotated_vertices[0], rotated_vertices[3], rotated_vertices[7], rotated_vertices[4]],
            [rotated_vertices[1], rotated_vertices[2], rotated_vertices[6], rotated_vertices[5]]
        ]
        self._cube_collection.set_verts(rotated_faces)

    def _update_quaternion(self, state: DeviceDataState, data: SensorData):
        try:
            if data.dataType == DataType.NTF_QUATERNION:
                if data.channelCount == 4 and data.sampleCount > 0:
                    quaternion = [data.getData(ch, 0) for ch in range(4)]
                    state.quaternion_lock.lock()
                    state.quaternion = quaternion
                    state.quaternion_lock.unlock()
        except Exception as e:
            print(f"Quaternion update exception: {e}")

    # -- Periodic refresh -----------------------------------------------------------------

    def _update_plots(self):
        if self.windowState() & QtCore.Qt.WindowMinimized:
            return

        self._poll_fft_results()
        self._refresh_sdk_label()

        state = self._current_state()
        if state is None:
            return

        now = time.time()
        if now - self._rate_last_refresh >= 1.0:
            self._rate_last_refresh = now
            state.refresh_actual_rates(now)
            self.status_label.setText(state.build_status_text())
            self.rate_label.setText(state.build_rate_text())

        dt  = self.active_data_type
        buf_copy = None
        idx_buf_copy = None
        buffer_index = 0
        lock = state.get_buffer_lock(dt)
        lock.lock()
        try:
            buf = state.buffers.get(dt)
            idx_buf = state.sample_index_buffers.get(dt)
            if buf is not None and idx_buf is not None:
                buf_copy = buf.copy()
                idx_buf_copy = idx_buf.copy()
                buffer_index = state.buffer_indices.get(dt, 0)
        finally:
            lock.unlock()

        if buf_copy is not None and idx_buf_copy is not None and self.lines_2d:
            current_last_idx = int(idx_buf_copy.max())
            last_plotted_idx = self._last_plotted_sample_indices.get(dt, -1)
            if current_last_idx != last_plotted_idx:
                buf_copy = np.roll(buf_copy, -buffer_index, axis=1)
                now_fft = time.time()
                if (not self._fft_pending
                        and now_fft - self._fft_last_submit >= FFT_UPDATE_INTERVAL):
                    sr = state.sample_rates.get(dt) or state.nominal_rates.get(dt) or 0
                    if sr > 0:
                        self._fft_last_submit = now_fft
                        self._submit_fft(dt, sr, buf_copy)
                labels = CHANNEL_LABELS.get(dt, [])
                for ch, line in enumerate(self.lines_2d):
                    if ch < buf_copy.shape[0]:
                        line.set_ydata(buf_copy[ch])
                        label = labels[ch] if ch < len(labels) else f"ch{ch}"
                        lbl_widget = self.value_labels.get(label)
                        if lbl_widget:
                            latest = buf_copy[ch, -1]
                            lbl_widget.setText(f"{label}: {latest:+.4f}")

                fixed_range = FIXED_Y_RANGES.get(dt)
                if fixed_range is not None:
                    self.ax_2d.set_ylim(fixed_range)
                else:
                    all_data = buf_copy.flatten()
                    mn, mx = all_data.min(), all_data.max()
                    margin = max((mx - mn) * 0.1, 0.01)
                    new_ylim = (mn - margin, mx + margin)
                    cur_ylim = self.ax_2d.get_ylim()
                    y_range = cur_ylim[1] - cur_ylim[0]
                    if (abs(new_ylim[0] - cur_ylim[0]) > 0.05 * y_range or
                            abs(new_ylim[1] - cur_ylim[1]) > 0.05 * y_range):
                        self.ax_2d.set_ylim(new_ylim)
                self.canvas_2d.draw_idle()
                self._last_plotted_sample_indices[dt] = current_last_idx

        # PPG mode: refresh the fixed 6 plots
        if state.bio_kind == "ppg":
            state.bio_buffer_lock.lock()
            try:
                bio_buffers_copy = {}
                bio_idx_buffers_copy = {}
                bio_impedance_copy = {}
                for dt in (DataType.NTF_EEG, DataType.NTF_PPG, DataType.NTF_SPO2):
                    buf = state.bio_buffers.get(dt)
                    idx_buf = state.bio_sample_index_buffers.get(dt)
                    if buf is not None and idx_buf is not None:
                        buf_idx = state.bio_buffer_indices.get(dt, 0)
                        bio_buffers_copy[dt] = np.roll(buf.copy(), -buf_idx, axis=1)
                        bio_idx_buffers_copy[dt] = idx_buf.copy()
                        if dt in state.bio_impedance:
                            bio_impedance_copy[dt] = state.bio_impedance[dt][:]
            finally:
                state.bio_buffer_lock.unlock()

            if bio_buffers_copy and self.bio_lines:
                any_updated = False
                for plot_idx, (dt, ch_idx, title, color) in enumerate(BIO_PLOT_CONFIG):
                    buf_copy = bio_buffers_copy.get(dt)
                    idx_buf_copy = bio_idx_buffers_copy.get(dt)
                    if buf_copy is None or idx_buf_copy is None:
                        continue
                    if plot_idx >= len(self.bio_lines) or plot_idx >= len(self.axes_eeg):
                        continue

                    current_last_idx = int(idx_buf_copy.max())
                    last_plotted_idx = self._last_plotted_sample_indices.get(dt, -1)
                    if current_last_idx == last_plotted_idx:
                        continue

                    any_updated = True
                    line = self.bio_lines[plot_idx]
                    ax = self.axes_eeg[plot_idx]
                    if ch_idx < buf_copy.shape[0]:
                        line.set_ydata(buf_copy[ch_idx])
                        ch_data = buf_copy[ch_idx]
                        mn, mx = ch_data.min(), ch_data.max()
                        margin = max((mx - mn) * 0.1, 0.01)
                        if mn == mx:
                            mn -= 1
                            mx += 1
                        ax.set_ylim(mn - margin, mx + margin)

                        if dt == DataType.NTF_EEG:
                            imp_list = bio_impedance_copy.get(dt, [])
                            if ch_idx < len(imp_list) and isinstance(imp_list[ch_idx], (int, float)):
                                current_impedance = imp_list[ch_idx] / 1000.0
                                if current_impedance <= 500:
                                    imp_color = "green"
                                elif 500 < current_impedance <= 999:
                                    imp_color = "orange"
                                else:
                                    imp_color = "red"
                                ax.set_ylabel(
                                    f"{title}\n{current_impedance:.2f} KOhm",
                                    fontsize=8, color=imp_color, rotation=0,
                                    va='center', ha='left', labelpad=10
                                )
                            else:
                                ax.set_ylabel(
                                    title,
                                    fontsize=8, color=color, rotation=0,
                                    va='center', ha='left', labelpad=10
                                )
                        else:
                            ax.set_ylabel(
                                title,
                                fontsize=8, color=color, rotation=0,
                                va='center', ha='left', labelpad=10
                            )

                # Per-row FFT snapshots for the EEG/PPG rows
                now_fft = time.time()
                if (not self._bio_fft_pending
                        and now_fft - self._bio_fft_last_submit >= FFT_UPDATE_INTERVAL):
                    row_specs = []
                    for plot_idx, (dt, ch_idx, _title, _color) in enumerate(BIO_PLOT_CONFIG):
                        if dt not in (DataType.NTF_EEG, DataType.NTF_PPG):
                            continue
                        fft_buf_copy = bio_buffers_copy.get(dt)
                        if fft_buf_copy is None or ch_idx >= fft_buf_copy.shape[0]:
                            continue
                        if (plot_idx >= len(self.bio_fft_lines)
                                or self.bio_fft_lines[plot_idx] is None):
                            continue
                        sr = state.bio_sample_rates.get(dt) or state.nominal_rates.get(dt) or 0
                        if sr <= 0:
                            continue
                        row_specs.append((plot_idx, fft_buf_copy[ch_idx], sr))
                    if row_specs:
                        self._bio_fft_last_submit = now_fft
                        self._submit_bio_fft(DataType.NTF_PPG, row_specs)

                if any_updated:
                    self.canvas_eeg.draw_idle()
                    for dt, idx_buf in bio_idx_buffers_copy.items():
                        self._last_plotted_sample_indices[dt] = int(idx_buf.max())

        eeg_buffer_copy = None
        eeg_idx_buf_copy = None
        eeg_impedance_copy = None
        eeg_buffer_index = 0
        ecg_buffer_copy = None
        ecg_idx_buf_copy = None
        ecg_impedance_copy = None
        ecg_buffer_index = 0
        brth_buffer_copy = None
        brth_idx_buf_copy = None
        brth_impedance_copy = None
        brth_buffer_index = 0
        mag_angle_buffer_copy = None
        mag_angle_idx_buf_copy = None
        mag_angle_buffer_index = 0
        state.eeg_buffer_lock.lock()
        state.ecg_buffer_lock.lock()
        state.brth_buffer_lock.lock()
        state.mag_angle_buffer_lock.lock()
        try:
            if state.eeg_buffer is not None and state.eeg_sample_index_buffer is not None:
                eeg_buffer_copy = state.eeg_buffer.copy()
                eeg_idx_buf_copy = state.eeg_sample_index_buffer.copy()
                eeg_impedance_copy = list(state.eeg_impedance)
                eeg_buffer_index = state.eeg_buffer_index
            if state.has_ecg and state.ecg_buffer is not None and state.ecg_sample_index_buffer is not None:
                ecg_buffer_copy = state.ecg_buffer.copy()
                ecg_idx_buf_copy = state.ecg_sample_index_buffer.copy()
                ecg_impedance_copy = list(state.ecg_impedance)
                ecg_buffer_index = state.ecg_buffer_index
            if state.has_brth and state.brth_buffer is not None and state.brth_sample_index_buffer is not None:
                brth_buffer_copy = state.brth_buffer.copy()
                brth_idx_buf_copy = state.brth_sample_index_buffer.copy()
                brth_impedance_copy = list(state.brth_impedance)
                brth_buffer_index = state.brth_buffer_index
            if (state.has_mag_angle and state.mag_angle_buffer is not None
                    and state.mag_angle_sample_index_buffer is not None):
                mag_angle_buffer_copy = state.mag_angle_buffer.copy()
                mag_angle_idx_buf_copy = state.mag_angle_sample_index_buffer.copy()
                mag_angle_buffer_index = state.mag_angle_buffer_index
        finally:
            state.mag_angle_buffer_lock.unlock()
            state.brth_buffer_lock.unlock()
            state.ecg_buffer_lock.unlock()
            state.eeg_buffer_lock.unlock()

        brth_axis_index = len(self.axes_eeg) - 1 if state.has_brth else None
        ecg_axis_index = len(self.axes_eeg) - 1 - int(state.has_brth) if state.has_ecg else None
        # Angle row matches the rebuild layout: after the EMG channel rows
        # in EMG mode, above ECG/BRTH in EEG mode (no row in PPG mode)
        if not state.has_mag_angle or state.bio_kind == "ppg":
            mag_angle_axis_index = None
        elif state.bio_kind == "emg":
            mag_angle_axis_index = len(self.emg_lines)
        else:
            mag_angle_axis_index = len(self.axes_eeg) - 1 - int(state.has_brth) - int(state.has_ecg)
        start_ch, _ = self._eeg_page_range()

        if eeg_buffer_copy is not None and eeg_idx_buf_copy is not None and self.eeg_lines:
            current_last_idx = int(eeg_idx_buf_copy.max())
            last_plotted_idx = self._last_plotted_sample_indices.get(DataType.NTF_EEG, -1)
            if current_last_idx != last_plotted_idx:
                for ch, line in enumerate(self.eeg_lines):
                    if ch >= self._eeg_display_channels:
                        continue
                    eeg_ch = start_ch + ch
                    if eeg_ch >= eeg_buffer_copy.shape[0]:
                        continue
                    y_data = np.roll(eeg_buffer_copy[eeg_ch], -eeg_buffer_index)
                    line.set_ydata(y_data)
                    ax = self.axes_eeg[ch]
                    if not ax.get_visible():
                        ax.set_visible(True)
                    ch_data = y_data
                    mn, mx = ch_data.min(), ch_data.max()
                    margin = max((mx - mn) * 0.1, 0.01)
                    if mn == mx:
                        mn -= 1
                        mx += 1
                    ax.set_ylim(mn - margin, mx + margin)

                    if eeg_ch < len(eeg_impedance_copy) and isinstance(eeg_impedance_copy[eeg_ch], (int, float)):
                        current_impedance = eeg_impedance_copy[eeg_ch] / 1000.0
                        if current_impedance <= 500:
                            color = "green"
                        elif 500 < current_impedance <= 999:
                            color = "orange"
                        else:
                            color = "red"
                        ax.set_ylabel(
                            f"EEG-{eeg_ch + 1}\n{current_impedance:.2f} KOhm",
                            fontsize=8, color=color, rotation=0,
                            va='center', ha='left', labelpad=10
                        )
                        ax.yaxis.set_label_position("right")

                # Per-row FFT snapshots for this EEG page and the ECG/Angle rows
                now_fft = time.time()
                if (not self._bio_fft_pending
                        and now_fft - self._bio_fft_last_submit >= FFT_UPDATE_INTERVAL):
                    row_specs = []
                    sr = state.eeg_sample_rate or state.nominal_rates.get(DataType.NTF_EEG) or 0
                    if sr > 0 and self._eeg_display_channels > 0:
                        snapshot = np.roll(
                            eeg_buffer_copy[start_ch:start_ch + self._eeg_display_channels],
                            -eeg_buffer_index, axis=1)
                        for row in range(snapshot.shape[0]):
                            row_specs.append((row, snapshot[row], sr))
                    if (ecg_axis_index is not None and ecg_buffer_copy is not None
                            and self.ecg_line is not None):
                        sr_ecg = (state.ecg_sample_rate
                                  or state.nominal_rates.get(DataType.NTF_ECG) or 0)
                        if sr_ecg > 0:
                            row_specs.append((
                                ecg_axis_index,
                                np.roll(ecg_buffer_copy[0], -ecg_buffer_index),
                                sr_ecg))
                    if (mag_angle_axis_index is not None and mag_angle_buffer_copy is not None
                            and self.mag_angle_line is not None):
                        sr_mag = (state.mag_angle_sample_rate
                                  or state.nominal_rates.get(DataType.NTF_MAG_ANGLE_DATA) or 0)
                        if sr_mag > 0:
                            row_specs.append((
                                mag_angle_axis_index,
                                np.roll(mag_angle_buffer_copy[0], -mag_angle_buffer_index),
                                sr_mag))
                    if row_specs:
                        self._bio_fft_last_submit = now_fft
                        self._submit_bio_fft(DataType.NTF_EEG, row_specs)

                self.canvas_eeg.draw_idle()
                self._last_plotted_sample_indices[DataType.NTF_EEG] = current_last_idx

        if brth_buffer_copy is not None and brth_idx_buf_copy is not None and self.brth_line is not None:
            current_last_idx = int(brth_idx_buf_copy.max())
            last_plotted_idx = self._last_plotted_sample_indices.get(DataType.NTF_BRTH, -1)
            if current_last_idx != last_plotted_idx:
                y_data = np.roll(brth_buffer_copy[0], -brth_buffer_index)
                self.brth_line.set_ydata(y_data)
                ax = self.axes_eeg[brth_axis_index]
                if not ax.get_visible():
                    ax.set_visible(True)
                ch_data = y_data
                mn, mx = ch_data.min(), ch_data.max()
                margin = max((mx - mn) * 0.1, 0.01)
                if mn == mx:
                    mn -= 1
                    mx += 1
                ax.set_ylim(mn - margin, mx + margin)
                self.canvas_eeg.draw_idle()
                self._last_plotted_sample_indices[DataType.NTF_BRTH] = current_last_idx

        if ecg_buffer_copy is not None and ecg_idx_buf_copy is not None and self.ecg_line is not None:
            current_last_idx = int(ecg_idx_buf_copy.max())
            last_plotted_idx = self._last_plotted_sample_indices.get(DataType.NTF_ECG, -1)
            if current_last_idx != last_plotted_idx:
                y_data = np.roll(ecg_buffer_copy[0], -ecg_buffer_index)
                self.ecg_line.set_ydata(y_data)
                ax = self.axes_eeg[ecg_axis_index]
                if not ax.get_visible():
                    ax.set_visible(True)
                ch_data = y_data
                mn, mx = ch_data.min(), ch_data.max()
                margin = max((mx - mn) * 0.1, 0.01)
                if mn == mx:
                    mn -= 1
                    mx += 1
                ax.set_ylim(mn - margin, mx + margin)

                if ecg_impedance_copy and len(ecg_impedance_copy) > 0 and isinstance(ecg_impedance_copy[0], (int, float)):
                    current_impedance = ecg_impedance_copy[0] / 1000.0
                    if current_impedance <= 500:
                        color = "green"
                    elif 500 < current_impedance <= 999:
                        color = "orange"
                    else:
                        color = "red"
                    ax.set_ylabel(
                        f"ECG\n{current_impedance:.2f} KOhm",
                        fontsize=8, color=color, rotation=0,
                        va='center', ha='left', labelpad=10
                    )
                    ax.yaxis.set_label_position("right")

                self.canvas_eeg.draw_idle()
                self._last_plotted_sample_indices[DataType.NTF_ECG] = current_last_idx

        emg_buffer_copy = None
        emg_idx_buf_copy = None
        emg_impedance_copy = None
        emg_buffer_index = 0
        state.emg_buffer_lock.lock()
        try:
            if state.emg_buffer is not None and state.emg_sample_index_buffer is not None:
                emg_buffer_copy = state.emg_buffer.copy()
                emg_idx_buf_copy = state.emg_sample_index_buffer.copy()
                emg_impedance_copy = list(state.emg_impedance)
                emg_buffer_index = state.emg_buffer_index
        finally:
            state.emg_buffer_lock.unlock()

        if emg_buffer_copy is not None and emg_idx_buf_copy is not None and self.emg_lines:
            current_last_idx = int(emg_idx_buf_copy.max())
            last_plotted_idx = self._last_plotted_sample_indices.get(DataType.NTF_EMG, -1)
            if current_last_idx != last_plotted_idx:
                for ch, line in enumerate(self.emg_lines):
                    if ch >= emg_buffer_copy.shape[0]:
                        continue
                    y_data = np.roll(emg_buffer_copy[ch], -emg_buffer_index)
                    line.set_ydata(y_data)
                    ax = self.axes_eeg[ch]
                    if not ax.get_visible():
                        ax.set_visible(True)
                    ch_data = y_data
                    mn, mx = ch_data.min(), ch_data.max()
                    margin = max((mx - mn) * 0.1, 0.01)
                    if mn == mx:
                        mn -= 1
                        mx += 1
                    ax.set_ylim(mn - margin, mx + margin)

                    if ch < len(emg_impedance_copy) and isinstance(emg_impedance_copy[ch], (int, float)):
                        current_impedance = emg_impedance_copy[ch] / 1000.0
                        if current_impedance <= 500:
                            color = "green"
                        elif 500 < current_impedance <= 999:
                            color = "orange"
                        else:
                            color = "red"
                        ax.set_ylabel(
                            f"EMG-{ch + 1}\n{current_impedance:.2f} KOhm",
                            fontsize=8, color=color, rotation=0,
                            va='center', ha='left', labelpad=10
                        )
                        ax.yaxis.set_label_position("right")

                # Per-channel FFT snapshots for the displayed EMG channels and the Angle row
                now_fft = time.time()
                if (not self._bio_fft_pending
                        and now_fft - self._bio_fft_last_submit >= FFT_UPDATE_INTERVAL):
                    sr = state.emg_sample_rate or state.nominal_rates.get(DataType.NTF_EMG) or 0
                    row_specs = []
                    if sr > 0 and self.emg_lines:
                        snapshot = np.roll(
                            emg_buffer_copy[:len(self.emg_lines)],
                            -emg_buffer_index, axis=1)
                        row_specs = [(row, snapshot[row], sr) for row in range(snapshot.shape[0])]
                    if self.mag_angle_line is not None and mag_angle_buffer_copy is not None:
                        sr_mag = (state.mag_angle_sample_rate
                                  or state.nominal_rates.get(DataType.NTF_MAG_ANGLE_DATA) or 0)
                        if sr_mag > 0:
                            row_specs.append((
                                len(self.emg_lines),
                                np.roll(mag_angle_buffer_copy[0], -mag_angle_buffer_index),
                                sr_mag))
                    if row_specs:
                        self._bio_fft_last_submit = now_fft
                        self._submit_bio_fft(DataType.NTF_EMG, row_specs)

                self.canvas_eeg.draw_idle()
                self._last_plotted_sample_indices[DataType.NTF_EMG] = current_last_idx

        if (mag_angle_buffer_copy is not None and mag_angle_idx_buf_copy is not None
                and self.mag_angle_line is not None and mag_angle_axis_index is not None):
            current_last_idx = int(mag_angle_idx_buf_copy.max())
            last_plotted_idx = self._last_plotted_sample_indices.get(DataType.NTF_MAG_ANGLE_DATA, -1)
            if current_last_idx != last_plotted_idx:
                y_data = np.roll(mag_angle_buffer_copy[0], -mag_angle_buffer_index)
                self.mag_angle_line.set_ydata(y_data)
                ax = self.axes_eeg[mag_angle_axis_index]
                if not ax.get_visible():
                    ax.set_visible(True)
                ax.set_ylim(0, 180)
                self.canvas_eeg.draw_idle()
                self._last_plotted_sample_indices[DataType.NTF_MAG_ANGLE_DATA] = current_last_idx

                # Angle row FFT, submitted here when the EEG/EMG block above
                # did not submit this round (its row specs already include this row)
                now_fft = time.time()
                if (not self._bio_fft_pending
                        and now_fft - self._bio_fft_last_submit >= FFT_UPDATE_INTERVAL):
                    sr_mag = (state.mag_angle_sample_rate
                              or state.nominal_rates.get(DataType.NTF_MAG_ANGLE_DATA) or 0)
                    if (sr_mag > 0 and mag_angle_axis_index < len(self.bio_fft_lines)
                            and self.bio_fft_lines[mag_angle_axis_index] is not None):
                        self._bio_fft_last_submit = now_fft
                        self._submit_bio_fft(
                            DataType.NTF_EMG if state.bio_kind == "emg" else DataType.NTF_EEG,
                            [(mag_angle_axis_index, y_data, sr_mag)])

        try:
            state.quaternion_lock.lock()
            current_quaternion = state.quaternion[:]
            state.quaternion_lock.unlock()

            now = time.time()
            elapsed_ms = (now - self._last_3d_update_time) * 1000
            quaternion_changed = current_quaternion != self._last_drawn_quaternion

            if elapsed_ms >= PLOT_UPDATE_INTERVAL and quaternion_changed:
                self._draw_cube(current_quaternion)
                self._last_drawn_quaternion = current_quaternion[:]
                self._last_3d_update_time = now

            self.canvas_3d.draw_idle()
        except Exception as e:
            print(f"3D update exception: {e}")

    def _refresh_sdk_label(self):
        now = time.time()
        if self._backend_query_pending or now - self._backend_query_ms < 1.0:
            return
        self._backend_query_pending = True
        self._backend_query_ms = now

        def work():
            try:
                result = str(self.sensor_controller.getParam("BACK_END"))
            except Exception as e:
                result = f"Error: {e}"

            def apply():
                self._backend_query_pending = False
                if not result or result.startswith("Error") or result == self._shown_backend:
                    return
                self._shown_backend = result
                self.sdk_label.setText(
                    f"SDK: {self.sensor_controller.getVersion()} | Backend: {result}")

            self._ui(apply)

        threading.Thread(target=work, daemon=True).start()

    # -- Callbacks ---------------------------------------------------------------------------

    def _app_log(self, message: str, level: str = "I", sensor=None):
        target = sensor if sensor is not None else self.current_sensor
        if target is not None:
            target.log(message, level)
        else:
            self.sensor_controller.log(message, level)

    def _on_state_changed(self, sensor: SensorProfile, state: DeviceStateEx):
        print(f"[State] {sensor.BLEDevice.Name}: {state}")
        if state == DeviceStateEx.Disconnected:
            self.device_disconnected_sig.emit(sensor.BLEDevice.Address)

    def _on_auto_reconnect(self, sensor: SensorProfile, restore: bool, answer):
        sensor.log(f"App: auto reconnect callback received, restore={restore}")
        self.auto_reconnect_sig.emit(sensor.BLEDevice.Address, restore)
        submit(self._auto_reconnect_recovery, sensor.BLEDevice.Address, restore)
        self._app_log("App: auto reconnect recovery submitted via sensor.submit", sensor=sensor)
        answer(True)

    def _auto_reconnect_recovery(self, addr: str, restore: bool):
        sensor = self.sensor_controller.getSensor(addr)
        if sensor is None:
            self._app_log(f"App: auto reconnect recovery failed, unknown device: {addr}", "E")
            return
        device = sensor.BLEDevice
        self._connect_device(device, self._auto_reconnect_enabled, select_current=True)
        saved = dict(self._saved_params_by_addr.get(addr, {})) if restore else {}
        if saved and addr in self.device_states:
            sensor = self.device_states[addr].sensor
            for key, value in saved.items():
                result = sensor.setParam(key, value)
                print(f"[AutoReconnect] restore setParam({key}, {value}) -> {result}")
                sensor.log(f"App: restore setParam({key}, {value}) -> {result}")
            self._ui(lambda: self._refresh_control_states(sensor))

    def _press_connect_for_address(self, addr: str, restore: bool = True):
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            if f"Address: {addr}" in item.text():
                self.device_list.setCurrentItem(item)
                break

    def _record_saved_param(self, sensor: SensorProfile, key: str, value: str, result: str):
        if sensor is None or str(result).startswith("Error"):
            return
        self._saved_params_by_addr.setdefault(sensor.BLEDevice.Address, {})[key] = value

    def _on_device_disconnected(self, addr: str):
        self._app_log(f"App: device disconnected, removed from UI: {addr}")
        state = self.device_states.pop(addr, None)
        self._update_device_item_text(addr, connected=False)
        if self.current_sensor is not None and self.current_sensor.BLEDevice.Address == addr:
            self.current_sensor = None
            self._refresh_display_for_state(None)
            self.status_label.setText("Disconnected (device)")
            self.rate_label.setText("")
        self._update_button_states()

    def _on_error(self, sensor: SensorProfile, reason: str):
        print(f"[Error] {sensor.BLEDevice.Name}: {reason}")
        sensor.log(f"App: error callback: {reason}", "E")

    def _update_lost_packet_display(self, addr: str, lost_type: str, count: int):
        state = self.device_states.get(addr)
        if state is None:
            return
        state.lost_counts[lost_type] = count
        if self.current_sensor is not None and self.current_sensor.BLEDevice.Address == addr:
            text = "  ".join(f"{k}: {v}" for k, v in sorted(state.lost_counts.items()))
            self.lost_packet_label.setText("Packet Loss Stats: " + text)

    def _handle_gesture_data(self, addr: str, data: SensorData):
        if data.channelCount == 0 or data.sampleCount == 0:
            return
        sample = data.getChannelSample(0, data.sampleCount - 1)
        gesture = int(sample.data)
        raw_gesture = int(sample.rawData)
        possiblity = int(sample.impedance)
        strength = int(sample.saturation)
        self.gesture_signal.emit(addr, gesture, raw_gesture, possiblity, strength)

    @staticmethod
    def _gesture_text(gesture_tuple) -> str:
        if gesture_tuple is None:
            return GESTURE_DEFAULT_TEXT
        gesture, raw_gesture, possiblity, strength = gesture_tuple
        return (
            "Gesture:\n"
            f"  gesture: {gesture} (0-8)\n"
            f"  raw gesture: {raw_gesture} (0-8)\n"
            f"  possiblity: {possiblity} (0-100)\n"
            f"  strength: {strength} (0-100)"
        )

    def _update_gesture_display(self, addr: str, gesture: int, raw_gesture: int, possiblity: int, strength: int):
        state = self.device_states.get(addr)
        if state is not None:
            state.gesture = (gesture, raw_gesture, possiblity, strength)
        if self.current_sensor is not None and self.current_sensor.BLEDevice.Address == addr:
            self.gesture_label.setText(self._gesture_text((gesture, raw_gesture, possiblity, strength)))

    def _on_power_changed(self, sensor: SensorProfile, power: int):
        print(f"[Power] {sensor.BLEDevice.Name}: {power}%")
        self.power_changed_sig.emit(sensor, power)

    def _on_device_info_update(self, sensor: SensorProfile, info: DeviceInfo):
        print(f"[Link] {sensor.BLEDevice.Name}: "
              f"interval={info.ConnectionIntervalMs}ms latency={info.PeripheralLatency} "
              f"timeout={info.SupervisionTimeoutMs}ms mtu={info.MTUSize}")
        self.device_info_sig.emit(sensor, info)

    @staticmethod
    def _link_text(info: Optional[DeviceInfo]) -> str:
        backend = getattr(info, "Backend", "") if info is not None else ""
        backend_part = f" | backend {backend}" if backend else ""
        if info is None or info.PeripheralLatency < 0 or info.ConnectionIntervalMs <= 0:
            return "Link: --" + backend_part
        return (f"Link: {info.ConnectionIntervalMs}ms / "
                f"latency {info.PeripheralLatency} / "
                f"timeout {info.SupervisionTimeoutMs}ms") + backend_part

    @staticmethod
    def _mtu_text(info: Optional[DeviceInfo]) -> str:
        if info is None or info.MTUSize <= 0:
            return "MTU: --"
        return f"MTU: {info.MTUSize}"

    def _update_link_info_display(self, sensor: SensorProfile, info: Optional[DeviceInfo] = None):
        if info is None:
            info = sensor.getDeviceInfo()
        state = self.device_states.get(sensor.BLEDevice.Address)
        if state is not None and info is not None and state.status_parts:
            rate_map = {DataType.NTF_EEG: info.EegSampleRate, DataType.NTF_ECG: info.EcgSampleRate}
            state.status_parts = [
                (label, ch, rate_map.get(dt) or sr, dt)
                for label, ch, sr, dt in state.status_parts
            ]
        if state is not None and info is not None:
            if state.sync_bio_sample_rates(info) and self.current_sensor == sensor:
                self._rebuild_eeg_plot()
            imu_changed = state.sync_imu_sample_rates(info)
            if (imu_changed and self.current_sensor == sensor
                    and self.active_data_type in imu_changed):
                self._rebuild_2d_plot()
            if info.EegSampleRate > 0:
                rate = int(info.EegSampleRate)
                options, cur = state.sample_rate_state
                if cur != rate:
                    state.sample_rate_state = (options, rate)
                    if self.current_sensor == sensor:
                        self._set_sample_rate_checked(rate)
            if info.EmgSampleRate > 0:
                rate = int(info.EmgSampleRate)
                options, cur = state.emg_sample_rate_state
                if cur != rate:
                    state.emg_sample_rate_state = (options, rate)
                    if self.current_sensor == sensor:
                        self._set_emg_sample_rate_checked(rate)
            if info.AccSampleRate > 0:
                rate = int(info.AccSampleRate)
                options, cur = state.imu_sample_rate_state
                if cur != rate:
                    state.imu_sample_rate_state = (options, rate)
                    if self.current_sensor == sensor:
                        self._set_imu_sample_rate_checked(rate)
            if info.PpgSampleRate > 0:
                rate = int(info.PpgSampleRate)
                options, cur = state.ppg_sample_rate_state
                if cur != rate:
                    state.ppg_sample_rate_state = (options, rate)
                    if self.current_sensor == sensor:
                        self._set_ppg_sample_rate_checked(rate)
        if self.current_sensor == sensor:
            self.link_label.setText(self._link_text(info))
            self.mtu_label.setText(self._mtu_text(info))
            if state is not None and state.status_parts:
                self.status_label.setText(state.build_status_text())
                self.rate_label.setText(state.build_rate_text())

    def _update_power_display(self, sensor: SensorProfile, power: int):
        if power < 0:
            return
        state = self.device_states.get(sensor.BLEDevice.Address)
        if state is not None:
            state.last_power = power
        if self.current_sensor == sensor:
            self.power_label.setText(f"Power: {power}%")

    def _check_set_param_result(self, key: str, result: str) -> bool:
        if str(result).startswith("Error"):
            QtWidgets.QMessageBox.warning(self, "Set Parameter Failed", f"Failed to set {key}:\n{result}")
            return False
        return True

    def _apply_sdk_debug_log(self):
        log_dir = os.path.join(
            str(Path.home() / "Documents" / "sensorsdklog"),
            f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{self.sensor_controller.getVersion().replace('.', '_')}",
        )
        self.sensor_controller.setParam("LOG_PATH", log_dir)
        print(f"[Debug Log] LOG_PATH -> {log_dir}")
        self.sensor_controller.setParam("DEBUG_ENABLED", "True")

    def _on_debug_log_toggled(self, state: int):
        enabled = (state == QtCore.Qt.Checked)
        self._app_log(f"User: SDK debug log {'ON' if enabled else 'OFF'}")
        self._debug_log_enabled = enabled
        if enabled:
            self._apply_sdk_debug_log()
        else:
            self.sensor_controller.setParam("DEBUG_ENABLED", "False")
        value = "True" if enabled else "False"
        for sensor in self.sensor_controller.getConnectedSensors():
            if sensor.isReady and sensor.hasInited:
                result = sensor.setParam("DEBUG_LOG_PATH", value)
                print(f"[Debug Log] setParam({sensor.BLEDevice.Address}, DEBUG_LOG_PATH, {value}) -> {result}")
                sensor.log(f"App: setParam(DEBUG_LOG_PATH, {value}) -> {result}")
                self._check_set_param_result("DEBUG_LOG_PATH", result)

    def _apply_dongle_debug(self):
        value = "True" if self._dongle_debug_enabled else "False"
        result = self.sensor_controller.setParam("BLE_TRACE_ENABLED", value)
        print(f"[Dongle Debug] setParam(BLE_TRACE_ENABLED, {value}) -> {result}")

    def _on_dongle_debug_toggled(self, state: int):
        enabled = (state == QtCore.Qt.Checked)
        self._app_log(f"User: dongle debug {'ON' if enabled else 'OFF'}")
        self._dongle_debug_enabled = enabled
        self._apply_dongle_debug()

    def _on_data_debug_log_toggled(self, state: int):
        enabled = (state == QtCore.Qt.Checked)
        self._app_log(f"User: data debug log {'ON' if enabled else 'OFF'}")
        self._data_debug_log_enabled = enabled
        value = "True" if enabled else "False"
        for sensor in self.sensor_controller.getConnectedSensors():
            if sensor.isReady and sensor.hasInited:
                result = sensor.setParam("DEBUG_BLE_DATA_PATH", value)
                print(f"[Data Debug Log] setParam({sensor.BLEDevice.Address}, DEBUG_BLE_DATA_PATH, {value}) -> {result}")
                sensor.log(f"App: setParam(DEBUG_BLE_DATA_PATH, {value}) -> {result}")
                self._check_set_param_result("DEBUG_BLE_DATA_PATH", result)

    def _on_ntf_toggled(self, key: str):
        if self.current_sensor is None or not self.current_sensor.isReady:
            return
        if self._updating_ntf_controls:
            return
        cb = self._ntf_checkboxes.get(key)
        if cb is None:
            return
        value = "ON" if cb.isChecked() else "OFF"
        print(f"[NTF] setParam({key}, {value}) ...")
        result = self.current_sensor.setParam(key, value)
        print(f"[NTF] setParam({key}, {value}) -> {result}")
        self._app_log(f"User: setParam({key}, {value}) -> {result}")
        self._record_saved_param(self.current_sensor, key, value, result)
        if self._check_set_param_result(key, result):
            self._refresh_control_states(self.current_sensor)
            self._clear_ui_data()

    def _refresh_control_states(self, sensor: SensorProfile):
        info = sensor.getDeviceInfo()
        channel_map = {
            "NTF_EMG":   info.EmgChannelCount if info else 0,
            "NTF_GEST":  info.GestChannelCount if info else 0,
            "NTF_EEG":   info.EegChannelCount if info else 0,
            "NTF_ECG":   info.EcgChannelCount if info else 0,
            "NTF_PPG":   info.PpgChannelCount if info else 0,
            "NTF_SPO2":  info.Spo2ChannelCount if info else 0,
            "NTF_IMU":   max(info.AccChannelCount, info.GyroChannelCount) if info else 0,
            "NTF_BRTH":  info.BrthChannelCount if info else 0,
            "NTF_IMPEDANCE": info.ImpeChannelCount if info else 0,
            "NTF_MAG_ANGLE": info.MagAngleChannelCount if info else 0,
            "NTF_GFORCE_EULER": info.EulerChannelCount if info else 0,
            "NTF_GFORCE_QUAT": info.QuatChannelCount if info else 0,
            "NTF_GFORCE_ACC": info.AccChannelCount if info else 0,
            "NTF_GFORCE_GYRO": info.GyroChannelCount if info else 0,
        }

        ntf_states = {}
        ntf_result = sensor.getParam("NTF")
        print(f"[Refresh] getParam(NTF) -> {ntf_result}")
        if not str(ntf_result).startswith("Error"):
            items = str(ntf_result).split("|")
            for i in range(0, len(items) - 1, 2):
                key = items[i]
                value = items[i + 1]
                count = channel_map.get(key, 0)
                ntf_states[key] = (count > 0, value == "ON" if count > 0 else False)

        filter_states = {}
        filter_result = sensor.getParam("FILTER")
        print(f"[Refresh] getParam(FILTER) -> {filter_result}")
        has_filter = bool(filter_result) and not str(filter_result).startswith("Error")
        if has_filter:
            items = str(filter_result).split("|")
            parsed = {items[i]: items[i + 1] for i in range(0, len(items) - 1, 2)}
            for key in self._filter_checkboxes:
                filter_states[key] = (True, parsed.get(key) == "ON")
        else:
            for key in self._filter_checkboxes:
                filter_states[key] = (False, False)

        sample_rate_options = []
        options_result = sensor.getParam("EEG_SAMPLE_RATE_LIST")
        print(f"[Refresh] getParam(EEG_SAMPLE_RATE_LIST) -> {options_result}")
        if not str(options_result).startswith("Error"):
            for item in str(options_result).split("|"):
                try:
                    sample_rate_options.append(int(item))
                except ValueError:
                    pass

        current_sample_rate = 0
        rate_result = sensor.getParam("EEG_SAMPLE_RATE")
        print(f"[Refresh] getParam(EEG_SAMPLE_RATE) -> {rate_result}")
        if not str(rate_result).startswith("Error"):
            try:
                current_sample_rate = int(rate_result)
            except ValueError:
                pass
        sample_rate_state = (sample_rate_options, current_sample_rate)

        emg_sample_rate_options = []
        emg_options_result = sensor.getParam("EMG_SAMPLE_RATE_LIST")
        print(f"[Refresh] getParam(EMG_SAMPLE_RATE_LIST) -> {emg_options_result}")
        if not str(emg_options_result).startswith("Error"):
            for item in str(emg_options_result).split("|"):
                try:
                    emg_sample_rate_options.append(int(item))
                except ValueError:
                    pass

        current_emg_sample_rate = 0
        emg_rate_result = sensor.getParam("EMG_SAMPLE_RATE")
        print(f"[Refresh] getParam(EMG_SAMPLE_RATE) -> {emg_rate_result}")
        if not str(emg_rate_result).startswith("Error"):
            try:
                current_emg_sample_rate = int(emg_rate_result)
            except ValueError:
                pass
        emg_sample_rate_state = (emg_sample_rate_options, current_emg_sample_rate)

        imu_sample_rate_options = []
        imu_options_result = sensor.getParam("IMU_SAMPLE_RATE_LIST")
        print(f"[Refresh] getParam(IMU_SAMPLE_RATE_LIST) -> {imu_options_result}")
        if not str(imu_options_result).startswith("Error"):
            for item in str(imu_options_result).split("|"):
                try:
                    imu_sample_rate_options.append(int(item))
                except ValueError:
                    pass

        current_imu_sample_rate = 0
        imu_rate_result = sensor.getParam("IMU_SAMPLE_RATE")
        print(f"[Refresh] getParam(IMU_SAMPLE_RATE) -> {imu_rate_result}")
        if not str(imu_rate_result).startswith("Error"):
            try:
                current_imu_sample_rate = int(imu_rate_result)
            except ValueError:
                pass
        imu_sample_rate_state = (imu_sample_rate_options, current_imu_sample_rate)

        ppg_sample_rate_options = []
        ppg_options_result = sensor.getParam("PPG_SAMPLE_RATE_LIST")
        print(f"[Refresh] getParam(PPG_SAMPLE_RATE_LIST) -> {ppg_options_result}")
        if not str(ppg_options_result).startswith("Error"):
            for item in str(ppg_options_result).split("|"):
                try:
                    ppg_sample_rate_options.append(int(item))
                except ValueError:
                    pass

        current_ppg_sample_rate = 0
        ppg_rate_result = sensor.getParam("PPG_SAMPLE_RATE")
        print(f"[Refresh] getParam(PPG_SAMPLE_RATE) -> {ppg_rate_result}")
        if not str(ppg_rate_result).startswith("Error"):
            try:
                current_ppg_sample_rate = int(ppg_rate_result)
            except ValueError:
                pass
        ppg_sample_rate_state = (ppg_sample_rate_options, current_ppg_sample_rate)

        state = self.device_states.get(sensor.BLEDevice.Address)
        if state is not None:
            state.ntf_states = ntf_states
            state.filter_states = filter_states
            state.sample_rate_state = sample_rate_state
            state.emg_sample_rate_state = emg_sample_rate_state
            state.imu_sample_rate_state = imu_sample_rate_state
            state.ppg_sample_rate_state = ppg_sample_rate_state

        if self.current_sensor == sensor:
            self.gesture_box.setVisible(info is None or info.GestChannelCount > 0)
            self._apply_control_states(ntf_states, filter_states, sample_rate_state,
                                       emg_sample_rate_state, imu_sample_rate_state,
                                       ppg_sample_rate_state)

    def _apply_control_states(self, ntf_states: dict, filter_states: dict, sample_rate_state: tuple = ([], 0),
                              emg_sample_rate_state: tuple = ([], 0), imu_sample_rate_state: tuple = ([], 0),
                              ppg_sample_rate_state: tuple = ([], 0)):
        self._updating_ntf_controls = True
        try:
            for key, cb in self._ntf_checkboxes.items():
                enabled, checked = ntf_states.get(key, (False, False))
                cb.setVisible(enabled or not ntf_states)
                cb.setEnabled(enabled)
                cb.setChecked(checked)
        finally:
            self._updating_ntf_controls = False
        self._updating_filter_controls = True
        try:
            for key, cb in self._filter_checkboxes.items():
                enabled, checked = filter_states.get(key, (False, False))
                cb.setEnabled(enabled)
                cb.setChecked(checked)
        finally:
            self._updating_filter_controls = False
        options, current_rate = sample_rate_state
        # A group with no options is hidden entirely; an unsupported radio is hidden too
        self._sample_rate_group.setVisible(bool(options))
        self._updating_sample_rate_controls = True
        try:
            if current_rate not in self._sample_rate_radios:
                self._sample_rate_button_group.setExclusive(False)
            for rate, rb in self._sample_rate_radios.items():
                rb.setVisible(rate in options)
                rb.setEnabled(rate in options)
                rb.setChecked(rate == current_rate)
            self._sample_rate_button_group.setExclusive(True)
        finally:
            self._updating_sample_rate_controls = False
        emg_options, current_emg_rate = emg_sample_rate_state
        self._emg_sample_rate_group.setVisible(bool(emg_options))
        self._updating_emg_sample_rate_controls = True
        try:
            if current_emg_rate not in self._emg_sample_rate_radios:
                self._emg_sample_rate_button_group.setExclusive(False)
            for rate, rb in self._emg_sample_rate_radios.items():
                rb.setVisible(rate in emg_options)
                rb.setEnabled(rate in emg_options)
                rb.setChecked(rate == current_emg_rate)
            self._emg_sample_rate_button_group.setExclusive(True)
        finally:
            self._updating_emg_sample_rate_controls = False
        imu_options, current_imu_rate = imu_sample_rate_state
        self._imu_sample_rate_group.setVisible(bool(imu_options))
        self._updating_imu_sample_rate_controls = True
        try:
            if current_imu_rate not in self._imu_sample_rate_radios:
                self._imu_sample_rate_button_group.setExclusive(False)
            for rate, rb in self._imu_sample_rate_radios.items():
                rb.setVisible(rate in imu_options)
                rb.setEnabled(rate in imu_options)
                rb.setChecked(rate == current_imu_rate)
            self._imu_sample_rate_button_group.setExclusive(True)
        finally:
            self._updating_imu_sample_rate_controls = False
        ppg_options, current_ppg_rate = ppg_sample_rate_state
        self._ppg_sample_rate_group.setVisible(bool(ppg_options))
        self._updating_ppg_sample_rate_controls = True
        try:
            if current_ppg_rate not in self._ppg_sample_rate_radios:
                self._ppg_sample_rate_button_group.setExclusive(False)
            for rate, rb in self._ppg_sample_rate_radios.items():
                rb.setVisible(rate in ppg_options)
                rb.setEnabled(rate in ppg_options)
                rb.setChecked(rate == current_ppg_rate)
            self._ppg_sample_rate_button_group.setExclusive(True)
        finally:
            self._updating_ppg_sample_rate_controls = False

    def _set_sample_rate_checked(self, rate: int):
        self._updating_sample_rate_controls = True
        try:
            if rate not in self._sample_rate_radios:
                self._sample_rate_button_group.setExclusive(False)
            for r, rb in self._sample_rate_radios.items():
                rb.setChecked(r == rate)
            self._sample_rate_button_group.setExclusive(True)
        finally:
            self._updating_sample_rate_controls = False

    def _set_emg_sample_rate_checked(self, rate: int):
        self._updating_emg_sample_rate_controls = True
        try:
            if rate not in self._emg_sample_rate_radios:
                self._emg_sample_rate_button_group.setExclusive(False)
            for r, rb in self._emg_sample_rate_radios.items():
                rb.setChecked(r == rate)
            self._emg_sample_rate_button_group.setExclusive(True)
        finally:
            self._updating_emg_sample_rate_controls = False

    def _set_imu_sample_rate_checked(self, rate: int):
        self._updating_imu_sample_rate_controls = True
        try:
            if rate not in self._imu_sample_rate_radios:
                self._imu_sample_rate_button_group.setExclusive(False)
            for r, rb in self._imu_sample_rate_radios.items():
                rb.setChecked(r == rate)
            self._imu_sample_rate_button_group.setExclusive(True)
        finally:
            self._updating_imu_sample_rate_controls = False

    def _set_ppg_sample_rate_checked(self, rate: int):
        self._updating_ppg_sample_rate_controls = True
        try:
            if rate not in self._ppg_sample_rate_radios:
                self._ppg_sample_rate_button_group.setExclusive(False)
            for r, rb in self._ppg_sample_rate_radios.items():
                rb.setChecked(r == rate)
            self._ppg_sample_rate_button_group.setExclusive(True)
        finally:
            self._updating_ppg_sample_rate_controls = False

    def _on_filter_combo_changed(self, _index: int):
        self._filter_band = self.filter_combo.currentData()
        self._app_log(f"User: live filter -> {self.filter_combo.currentText()}")
        for state in self.device_states.values():
            state.set_live_filter_band(self._filter_band)

    def _on_filter_toggled(self, key: str):
        if self.current_sensor is None or not self.current_sensor.isReady:
            return
        if self._updating_filter_controls:
            return
        cb = self._filter_checkboxes.get(key)
        if cb is None:
            return
        value = "ON" if cb.isChecked() else "OFF"
        print(f"[Filter] setParam({key}, {value}) ...")
        result = self.current_sensor.setParam(key, value)
        print(f"[Filter] setParam({key}, {value}) -> {result}")
        self._app_log(f"User: setParam({key}, {value}) -> {result}")
        self._record_saved_param(self.current_sensor, key, value, result)
        if self._check_set_param_result(key, result):
            self._refresh_control_states(self.current_sensor)
            self._clear_ui_data()

    def _on_sample_rate_toggled(self, rate: int, checked: bool):
        if not checked:
            return
        if self.current_sensor is None or not self.current_sensor.isReady:
            return
        if self._updating_sample_rate_controls:
            return
        self._set_sample_rate_checked(rate)
        QtCore.QTimer.singleShot(0, lambda r=rate: self._apply_sample_rate(r))

    def _apply_sample_rate(self, rate: int):
        sensor = self.current_sensor
        if sensor is None or not sensor.isReady:
            return
        value = str(rate)
        print(f"[Sample Rate] setParam(EEG_SAMPLE_RATE, {value}) ...")
        result = sensor.setParam("EEG_SAMPLE_RATE", value)
        print(f"[Sample Rate] setParam(EEG_SAMPLE_RATE, {value}) -> {result}")
        self._app_log(f"User: setParam(EEG_SAMPLE_RATE, {value}) -> {result}")
        self._record_saved_param(sensor, "EEG_SAMPLE_RATE", value, result)
        self._check_set_param_result("EEG_SAMPLE_RATE", result)
        self._refresh_control_states(sensor)
        if not str(result).startswith("Error"):
            self._clear_ui_data()

    def _on_emg_sample_rate_toggled(self, rate: int, checked: bool):
        if not checked:
            return
        if self.current_sensor is None or not self.current_sensor.isReady:
            return
        if self._updating_emg_sample_rate_controls:
            return
        self._set_emg_sample_rate_checked(rate)
        QtCore.QTimer.singleShot(0, lambda r=rate: self._apply_emg_sample_rate(r))

    def _apply_emg_sample_rate(self, rate: int):
        sensor = self.current_sensor
        if sensor is None or not sensor.isReady:
            return
        value = str(rate)
        print(f"[Sample Rate] setParam(EMG_SAMPLE_RATE, {value}) ...")
        result = sensor.setParam("EMG_SAMPLE_RATE", value)
        print(f"[Sample Rate] setParam(EMG_SAMPLE_RATE, {value}) -> {result}")
        self._app_log(f"User: setParam(EMG_SAMPLE_RATE, {value}) -> {result}")
        self._record_saved_param(sensor, "EMG_SAMPLE_RATE", value, result)
        self._check_set_param_result("EMG_SAMPLE_RATE", result)
        self._refresh_control_states(sensor)
        if not str(result).startswith("Error"):
            self._clear_ui_data()

    def _on_imu_sample_rate_toggled(self, rate: int, checked: bool):
        if not checked:
            return
        if self.current_sensor is None or not self.current_sensor.isReady:
            return
        if self._updating_imu_sample_rate_controls:
            return
        self._set_imu_sample_rate_checked(rate)
        QtCore.QTimer.singleShot(0, lambda r=rate: self._apply_imu_sample_rate(r))

    def _apply_imu_sample_rate(self, rate: int):
        sensor = self.current_sensor
        if sensor is None or not sensor.isReady:
            return
        value = str(rate)
        print(f"[Sample Rate] setParam(IMU_SAMPLE_RATE, {value}) ...")
        result = sensor.setParam("IMU_SAMPLE_RATE", value)
        print(f"[Sample Rate] setParam(IMU_SAMPLE_RATE, {value}) -> {result}")
        self._app_log(f"User: setParam(IMU_SAMPLE_RATE, {value}) -> {result}")
        self._record_saved_param(sensor, "IMU_SAMPLE_RATE", value, result)
        self._check_set_param_result("IMU_SAMPLE_RATE", result)
        self._refresh_control_states(sensor)
        if not str(result).startswith("Error"):
            self._clear_ui_data()

    def _on_ppg_sample_rate_toggled(self, rate: int, checked: bool):
        if not checked:
            return
        if self.current_sensor is None or not self.current_sensor.isReady:
            return
        if self._updating_ppg_sample_rate_controls:
            return
        self._set_ppg_sample_rate_checked(rate)
        QtCore.QTimer.singleShot(0, lambda r=rate: self._apply_ppg_sample_rate(r))

    def _apply_ppg_sample_rate(self, rate: int):
        sensor = self.current_sensor
        if sensor is None or not sensor.isReady:
            return
        value = str(rate)
        print(f"[Sample Rate] setParam(PPG_SAMPLE_RATE, {value}) ...")
        result = sensor.setParam("PPG_SAMPLE_RATE", value)
        print(f"[Sample Rate] setParam(PPG_SAMPLE_RATE, {value}) -> {result}")
        self._app_log(f"User: setParam(PPG_SAMPLE_RATE, {value}) -> {result}")
        self._record_saved_param(sensor, "PPG_SAMPLE_RATE", value, result)
        self._check_set_param_result("PPG_SAMPLE_RATE", result)
        self._refresh_control_states(sensor)
        if not str(result).startswith("Error"):
            self._clear_ui_data()

    def _refresh_display_for_state(self, state: Optional[DeviceDataState]):
        self._last_plotted_sample_indices.clear()
        self._last_drawn_quaternion = None

        if state is not None and state.info is not None:
            info = state.info
            self.model_label.setText(f"Model: {info.ModelName}")
            self.hw_version_label.setText(f"HW Version: {info.HardwareVersion}")
            self.fw_version_label.setText(f"FW Version: {info.FirmwareVersion}")
            self.link_label.setText(self._link_text(info))
            self.mtu_label.setText(self._mtu_text(info))
        else:
            self.model_label.setText("Model: --")
            self.hw_version_label.setText("HW Version: --")
            self.fw_version_label.setText("FW Version: --")
            self.link_label.setText("Link: --")
            self.mtu_label.setText("MTU: --")

        if state is not None and state.last_power is not None:
            self.power_label.setText(f"Power: {state.last_power}%")
        else:
            self.power_label.setText("Power: --%")

        if state is not None and state.status_parts:
            self.status_label.setText(state.build_status_text())
            self.rate_label.setText(state.build_rate_text())
        elif state is not None and state.status_text:
            self.status_label.setText(state.status_text)
            self.rate_label.setText("")
        else:
            self.status_label.setText("Not Connected")
            self.rate_label.setText("")

        if state is not None and state.lost_counts:
            text = "  ".join(f"{k}: {v}" for k, v in sorted(state.lost_counts.items()))
            self.lost_packet_label.setText("Packet Loss Stats: " + text)
        else:
            self.lost_packet_label.setText("Packet Loss Stats: None")

        self.gesture_label.setText(self._gesture_text(state.gesture if state is not None else None))

        ntf_states = state.ntf_states if state is not None else {}
        filter_states = state.filter_states if state is not None else {}
        sample_rate_state = state.sample_rate_state if state is not None else ([], 0)
        emg_sample_rate_state = state.emg_sample_rate_state if state is not None else ([], 0)
        imu_sample_rate_state = state.imu_sample_rate_state if state is not None else ([], 0)
        ppg_sample_rate_state = state.ppg_sample_rate_state if state is not None else ([], 0)
        self._apply_control_states(ntf_states, filter_states, sample_rate_state,
                                   emg_sample_rate_state, imu_sample_rate_state,
                                   ppg_sample_rate_state)

        self._rebuild_2d_plot()
        self._rebuild_eeg_plot()

    def _clear_ui_data(self):
        self._last_plotted_sample_indices.clear()

        state = self._current_state()
        if state is not None:
            state.clear_buffers()

        self._rebuild_2d_plot()
        self._rebuild_eeg_plot()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = IMUQuaternionEMGEEGDemo()

    def _sigint(sig, frame):
        app.quit()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)
    app.aboutToQuit.connect(lambda: window.sensor_controller.terminate())
    sys.exit(app.exec_())
