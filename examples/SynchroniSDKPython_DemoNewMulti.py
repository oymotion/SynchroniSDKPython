import sys
import signal
import time
import subprocess
import multiprocessing
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
    scipy_signal = None    # 无 scipy 时实时滤波单选框禁用，其余功能不受影响

from PyQt5 import QtWidgets, QtCore

from sensor import *


SCAN_DEVICE_PERIOD_IN_MS   = 3000
PACKAGE_COUNT              = 32
POWER_REFRESH_PERIOD_IN_MS = 60000
PLOT_UPDATE_INTERVAL       = 50
FFT_UPDATE_INTERVAL        = 0.5   # 秒，工作线程 FFT 频谱的计算间隔
DEMO_VERSION               = "0.0.9"  # Demo 自身版本号：每次修改本 Demo 时 +0.0.1
BUFFER_SECONDS             = 5
BIO_BUFFER_SECONDS         = 1

matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'Arial Unicode MS', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False
matplotlib.rcParams['lines.antialiased'] = False
matplotlib.rcParams['agg.path.chunksize'] = 10000

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

# 实时频段滤波选项（右侧 EMG/EEG/ECG/BRTH/PPG 生物电波形）：(显示名, (下限, 上限))，None = 关闭
FILTER_BANDS = (
    ("Off", None),
    ("δ 0.5-4Hz", (0.5, 4.0)),
    ("θ 4-8Hz", (4.0, 8.0)),
    ("α 8-13Hz", (8.0, 13.0)),
    ("β 13-30Hz", (13.0, 30.0)),
    ("γ 30-45Hz", (30.0, 45.0)),
)

EEG_CHANNEL_COLORS = plt.cm.tab10(np.linspace(0, 1, 8))

FIXED_Y_RANGES = {
    DataType.NTF_ACC: (-8, 8),
    DataType.NTF_GYRO: (-2000, 2000),
    DataType.NTF_EULER_DATA: (-180, 180),
    DataType.NTF_QUATERNION: (-1, 1),
}

# NTF_IMU 聚合批（新 EMG 设备）的固定通道布局：acc 0-2 / gyro 3-5 / euler 6-8 / quat 9-12
_IMU_AGGREGATE_SLICES = (
    (DataType.NTF_ACC,        0, 3),
    (DataType.NTF_GYRO,       3, 6),
    (DataType.NTF_EULER_DATA, 6, 9),
    (DataType.NTF_QUATERNION, 9, 13),
)


def split_imu_aggregate(data: SensorData) -> List[SensorData]:
    """把 NTF_IMU 聚合批按固定通道布局拆成四路独立 SensorData 视图
    （channelSamples 直接切片共享 Sample 引用，不复制）；
    通道不足的类型跳过（非 QAT6 设备聚合流只有 acc+gyro 6 通道）。
    lostPackageCount 只挂在第一路上，避免丢包统计重复上报。"""
    n_ch = len(data.channelSamples)
    subs = []
    for dt, start, end in _IMU_AGGREGATE_SLICES:
        if n_ch < end:
            continue
        sub = SensorData()
        # 拆分视图是应用侧构造的 SensorData，公有接口无写入口，只能写内部字段
        sub._deviceMac = data.getDeviceMac()
        sub._dataType = dt
        sub._sampleRate = data.getSampleRate()
        sub._channelCount = end - start
        sub._packageSampleCount = data.getSampleCount()
        sub._channelSamples = data.channelSamples[start:end]
        sub._lostPackageCount = data.getLostPackageCount() if not subs else 0
        subs.append(sub)
    return subs

GESTURE_DEFAULT_TEXT = (
    "Gesture:\n"
    "  gesture: -- (0-8)\n"
    "  raw gesture: -- (0-8)\n"
    "  possiblity: -- (0-100)\n"
    "  strength: -- (0-100)"
)

# PPG 设备右侧共享图表配置：(数据类型, 通道索引, 标题, 颜色)：
# 2×EEG fp1/fp2 + 2×PPG red/ir LED + 2×SpO2 spo2/heart_rate
BIO_PLOT_CONFIG = [
    (DataType.NTF_EEG,  0, "fp1",   plt.cm.tab10(0)),
    (DataType.NTF_EEG,  1, "fp2",   plt.cm.tab10(1)),
    (DataType.NTF_PPG,  0, "red_led", plt.cm.tab10(2)),
    (DataType.NTF_PPG,  1, "ir_led",  plt.cm.tab10(3)),
    (DataType.NTF_SPO2, 0, "spo2",    plt.cm.tab10(4)),
    (DataType.NTF_SPO2, 1, "heart_rate", plt.cm.tab10(5)),
]

EEG_AXIS_COUNT = 8                     # EEG/EMG 模式右侧子图数
PPG_AXIS_COUNT = len(BIO_PLOT_CONFIG)  # PPG 模式右侧子图数

SAMPLE_RATE_CANDIDATES = (250, 500, 1000, 2000)


class DeviceDataState:
    """单个已连接设备的数据缓冲与显示状态。多设备连接时每个设备各持有一份。"""

    def __init__(self, sensor: SensorProfile):
        self.sensor = sensor
        self.info: Optional[DeviceInfo] = None
        self.last_power: Optional[int] = None
        self.status_text = ""
        self.lost_counts: dict = {}
        self.ntf_states: dict = {}     # key -> (enabled, checked)
        self.filter_states: dict = {}  # key -> (enabled, checked)
        self.sample_rate_state: tuple = ([], 0)  # (可选采样率列表, 当前采样率)
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

        self.has_emg = False
        self.emg_buffer = None
        self.emg_sample_index_buffer = None
        self.emg_buffer_index = 0
        self.emg_sample_rate = 0
        self.emg_display_channels = 0
        self.emg_impedance: list = []
        self.emg_buffer_lock = QtCore.QMutex()

        # PPG 设备的生物电缓冲（EEG fp1/fp2 + PPG + SpO2，5s 环形缓冲，共用一把锁）；
        # 仅 bio_kind == "ppg" 时由 init_buffers 建立
        self.bio_buffers: dict = {}
        self.bio_sample_index_buffers: dict = {}
        self.bio_buffer_indices: dict = {}
        self.bio_sample_rates: dict = {}
        self.bio_impedance: dict = {}
        self.bio_buffer_lock = QtCore.QMutex()

        # 实时频段滤波（Live Filter 单选框）：SDK 数据回调线程内在样本写入
        # 环形缓冲前做因果带通（sosfilt + 跨批延续的 zi 状态），缓冲里直接
        # 存滤波后数据，绘图路径无需改动；live_filter_band 由 UI 线程切换
        self.live_filter_band = None           # (lo, hi) 或 None
        self._filter_sos = None                # 当前 (band, 采样率) 对应的 sos
        self._filter_sos_key = None
        self._filter_zi = {}                   # DataType -> (n_sections, n_ch, 2) 滤波器状态

        # 右侧生物电显示区类型：根据设备能力在 "eeg" / "emg" / "ppg" 间切换
        self.bio_kind: Optional[str] = None

        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.quaternion_lock = QtCore.QMutex()

        # 状态行显示项：(label, 通道数, 标称采样率, 数据类型)；数据批自带的
        # sampleRate/通道数优先，未收到数据的项显示 "--"
        self.status_parts = None
        # 实际采样率收集：rate_counts 为当前统计窗口的样本数，actual_rates 为
        # 每秒结算后的实测速率；nominal_rates/nominal_channels 记录批次自带标称值
        self.rate_lock = threading.Lock()
        self.rate_counts: dict = {}
        self.rate_window_start = time.time()
        self.actual_rates: dict = {}
        self.nominal_rates: dict = {}
        self.nominal_channels: dict = {}
        # 本次起流的首包 delay（毫秒，数据批自带 getDelay()，0=未上报）
        # 与起流墙钟时刻（Unix 秒，数据批自带 getStartTimeSec()，0=未知）
        self.stream_delay_ms = 0
        self.stream_start_time_sec = 0.0

    def note_data_received(self, data: SensorData):
        """统计每种数据类型实际收到的样本数（不含丢包占位样本），
        并记录数据批携带的标称采样率/通道数。"""
        if not data.channelSamples:
            return
        delay = data.getDelay()
        if delay:
            self.stream_delay_ms = delay
        start_sec = data.getStartTimeSec()
        if start_sec > 0:
            self.stream_start_time_sec = start_sec
        if data.getDataType() == DataType.NTF_IMU:
            # 聚合批：按拆分后的子类型分别计数，状态栏实测速率与四路独立流一致
            for sub in split_imu_aggregate(data):
                self.note_data_received(sub)
            return
        n = sum(1 for s in data.channelSamples[0] if not getattr(s, "isLost", False))
        with self.rate_lock:
            if n > 0:
                self.rate_counts[data.getDataType()] = self.rate_counts.get(data.getDataType(), 0) + n
            if data.getSampleRate() and data.getSampleRate() > 0:
                self.nominal_rates[data.getDataType()] = data.getSampleRate()
            ch = len(data.channelSamples)
            if ch > 0:
                self.nominal_channels[data.getDataType()] = ch

    def update_actual_rates(self):
        """每秒由 UI 定时器调用：结算上一窗口的实测速率并重置计数。"""
        now = time.time()
        with self.rate_lock:
            elapsed = now - self.rate_window_start
            if elapsed <= 0:
                return
            self.actual_rates = {dt: c / elapsed for dt, c in self.rate_counts.items()}
            self.rate_counts = {}
            self.rate_window_start = now

    def build_status_text(self) -> str:
        """组合状态行：连接名与各数据项的通道数、标称采样率。"""
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
        """组合实测采样率行（显示在状态行下一行）：各数据项每秒实测速率，
        标称值作对照；尚无实测数据时返回空串。"""
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

        self.has_emg = info.EmgSampleRate > 0 and info.EmgChannelCount > 0
        if self.has_emg:
            self.emg_sample_rate = info.EmgSampleRate
            self.emg_display_channels = min(info.EmgChannelCount, eeg_axis_count)
            buf_len = max(info.EmgSampleRate * BIO_BUFFER_SECONDS, 1)
            self.emg_buffer = np.zeros((self.emg_display_channels, buf_len))
            self.emg_sample_index_buffer = np.zeros((self.emg_display_channels, buf_len), dtype=np.int64)
            self.emg_buffer_index = 0
            self.emg_impedance = [None] * self.emg_display_channels

        # PPG 设备（含 EEG fp1/fp2，在 PPG 布局内显示）优先按 PPG 显示；
        # 否则有 EEG 能力优先按 EEG 显示，否则按 EMG 显示
        if info.PpgSampleRate > 0:
            self.bio_kind = "ppg"
        elif self.eeg_buffer is not None:
            self.bio_kind = "eeg"
        elif self.emg_buffer is not None:
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

        extra_axes = int(self.has_ecg) + int(self.has_brth)
        self.eeg_channels_per_page = eeg_axis_count - extra_axes

    def sync_bio_sample_rates(self, info: DeviceInfo) -> bool:
        """采样率变更（setParam("EEG_SAMPLE_RATE") 生效后由 device_info_update 推送）
        时按新速率重建生物电环形缓冲：缓冲长度 = 采样率 × 窗口秒数，不重建的话
        横轴时间窗与实际数据速率不一致，波形会被拉伸/压缩（如 10Hz 信号在
        250→500 后显示成 5Hz）。返回是否有缓冲被重建（调用方需重建图表横轴）。"""
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

        # PPG 模式下 EEG fp1/fp2 走 5s 的 bio_buffers
        if (self.bio_kind == "ppg" and info.EegSampleRate > 0
                and self.bio_sample_rates.get(DataType.NTF_EEG) not in (None, info.EegSampleRate)):
            buf = self.bio_buffers.get(DataType.NTF_EEG)
            if buf is not None:
                ch = buf.shape[0]
                buf_len = max(info.EegSampleRate * BUFFER_SECONDS, 1)
                self.bio_buffer_lock.lock()
                try:
                    self.bio_buffers[DataType.NTF_EEG] = np.zeros((ch, buf_len))
                    self.bio_sample_index_buffers[DataType.NTF_EEG] = np.zeros((ch, buf_len), dtype=np.int64)
                    self.bio_buffer_indices[DataType.NTF_EEG] = 0
                    self.bio_sample_rates[DataType.NTF_EEG] = info.EegSampleRate
                finally:
                    self.bio_buffer_lock.unlock()
                changed = True
        return changed

    def sync_imu_sample_rates(self, info: DeviceInfo) -> list:
        """IMU 采样率变化时（device_info_update 推送）按新速率重建四路 IMU 环形
        缓冲（长度 = 采样率 × BUFFER_SECONDS），与 sync_bio_sample_rates 同理：
        不重建则横轴时间窗与实际数据速率不一致，波形被拉伸/压缩。
        返回被重建的数据类型列表（调用方按需重建 2D 图表横轴）。"""
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
        """UI 线程切换频段：更新波段并让回调线程在下个数据批重建滤波器状态
        （环形缓冲中的旧数据随新写入自然滚动替换，无需清空）。"""
        self.live_filter_band = band
        self._filter_sos_key = None
        self._filter_zi = {}

    def apply_live_filter(self, dt, ch_idx: int, vals, sample_rate: int):
        """SDK 数据回调线程内实时滤波：按当前选中频段对单通道样本批做因果带通
        （4 阶 Butterworth，sosfilt），zi 状态跨数据批延续保证批间连续；
        未开启/参数无效/计算出错时原样返回。"""
        band = self.live_filter_band
        if band is None or vals is None or len(vals) == 0:
            return vals
        if scipy_signal is None or not sample_rate or sample_rate <= 0:
            return vals
        lo, hi = band
        if hi >= sample_rate / 2:
            return vals  # 频段超过奈奎斯特频率
        try:
            key = (band, int(sample_rate))
            if self._filter_sos_key != key:
                self._filter_sos = scipy_signal.butter(
                    4, [lo, hi], btype="band", fs=sample_rate, output="sos")
                self._filter_sos_key = key
                self._filter_zi = {}   # 频段/采样率变化：滤波器状态整体重置
            sos = self._filter_sos
            zi = self._filter_zi.get(dt)
            if zi is None or zi.shape[1] <= ch_idx:
                # 新数据类型/通道数变化：按当前通道数重建初始状态
                zi0 = scipy_signal.sosfilt_zi(sos)      # (n_sections, 2)
                zi = np.repeat(zi0[:, None, :], ch_idx + 1, axis=1)
                self._filter_zi[dt] = zi
            out, zi[:, ch_idx, :] = scipy_signal.sosfilt(sos, vals, zi=zi[:, ch_idx, :])
            return out.astype(np.float32)
        except Exception:
            return vals

    def filter_sensor_data(self, data: SensorData):
        """onData 回调线程内的实时滤波（direct / queue 两种模式共用，queue 模式
        在入队前完成）：对生物电批次逐通道带通滤波并把结果写回样本值，
        之后的分发/写缓冲路径不再滤波；非生物电类型不处理。"""
        if self.live_filter_band is None:
            return
        dt = data.getDataType()
        if not (dt in (DataType.NTF_EMG, DataType.NTF_EEG, DataType.NTF_ECG, DataType.NTF_BRTH)
                or (self.bio_buffers and dt in self.bio_buffers)):
            return
        for ch_idx, ch_samples in enumerate(data.channelSamples):
            if not ch_samples:
                continue
            vals = np.array([s.data for s in ch_samples], dtype=np.float32)
            vals = self.apply_live_filter(dt, ch_idx, vals, data.getSampleRate())
            for s, v in zip(ch_samples, vals):
                s._data = float(v)

    def append_data(self, data: SensorData):
        # PPG 设备：EEG/PPG/SpO2 数据写入 5s 生物电环形缓冲（右侧 6 子图显示），
        # 不再写入 1s 的 eeg 缓冲
        if self.bio_buffers and data.getDataType() in self.bio_buffers:
            self.bio_buffer_lock.lock()
            try:
                buf = self.bio_buffers.get(data.getDataType())
                idx_buf = self.bio_sample_index_buffers.get(data.getDataType())
                if buf is None or idx_buf is None or not data.channelSamples:
                    return
                buffer_size = buf.shape[1]
                n = len(data.channelSamples[0])
                if n == 0:
                    return
                if n > buffer_size:
                    n = buffer_size
                buf_idx = self.bio_buffer_indices.get(data.getDataType(), 0)
                write_start = buf_idx
                write_end = buf_idx + n

                # Circular-buffer write: avoid rolling the whole buffer on every packet.
                for ch_idx, ch_samples in enumerate(data.channelSamples):
                    if ch_idx >= buf.shape[0]:
                        break
                    new_vals = np.array([s.data for s in ch_samples], dtype=np.float32)
                    new_indices = np.array([s.sampleIndex for s in ch_samples], dtype=np.int64)
                    if len(new_vals) == 0:
                        continue
                    if len(new_vals) > n:
                        new_vals = new_vals[-n:]
                        new_indices = new_indices[-n:]

                    if write_end <= buffer_size:
                        buf[ch_idx, write_start:write_end] = new_vals
                        idx_buf[ch_idx, write_start:write_end] = new_indices
                    else:
                        first_part = buffer_size - write_start
                        buf[ch_idx, write_start:] = new_vals[:first_part]
                        buf[ch_idx, :n - first_part] = new_vals[first_part:]
                        idx_buf[ch_idx, write_start:] = new_indices[:first_part]
                        idx_buf[ch_idx, :n - first_part] = new_indices[first_part:]

                    if data.getDataType() == DataType.NTF_EEG:
                        while len(self.bio_impedance[data.getDataType()]) <= ch_idx:
                            self.bio_impedance[data.getDataType()].append(0)
                        self.bio_impedance[data.getDataType()][ch_idx] = ch_samples[-1].impedance

                self.bio_buffer_indices[data.getDataType()] = (buf_idx + n) % buffer_size
            finally:
                self.bio_buffer_lock.unlock()
            return

        if data.getDataType() == DataType.NTF_EMG:
            self.emg_buffer_lock.lock()
            try:
                buf = self.emg_buffer
                idx_buf = self.emg_sample_index_buffer
                if buf is None or idx_buf is None:
                    return
                buf_len = buf.shape[1]
                n = 0
                for ch_idx, ch_samples in enumerate(data.channelSamples):
                    if ch_idx >= buf.shape[0]:
                        break
                    new_vals = np.array([s.data for s in ch_samples], dtype=np.float32)
                    new_indices = np.array([s.sampleIndex for s in ch_samples], dtype=np.int64)
                    n = min(len(new_vals), buf_len)
                    if n == 0:
                        continue
                    write_start = self.emg_buffer_index
                    write_end = write_start + n
                    new_vals = new_vals[-n:]
                    new_indices = new_indices[-n:]
                    if write_end <= buf_len:
                        buf[ch_idx, write_start:write_end] = new_vals
                        idx_buf[ch_idx, write_start:write_end] = new_indices
                    else:
                        first_part = buf_len - write_start
                        buf[ch_idx, write_start:] = new_vals[:first_part]
                        buf[ch_idx, :n - first_part] = new_vals[first_part:]
                        idx_buf[ch_idx, write_start:] = new_indices[:first_part]
                        idx_buf[ch_idx, :n - first_part] = new_indices[first_part:]
                    while len(self.emg_impedance) <= ch_idx:
                        self.emg_impedance.append(None)
                    if ch_samples:
                        self.emg_impedance[ch_idx] = ch_samples[-1].impedance
                self.emg_buffer_index = (self.emg_buffer_index + n) % buf_len
            finally:
                self.emg_buffer_lock.unlock()
            return

        if data.getDataType() == DataType.NTF_EEG:
            self.eeg_buffer_lock.lock()
            try:
                buf = self.eeg_buffer
                idx_buf = self.eeg_sample_index_buffer
                if buf is None or idx_buf is None:
                    return
                buf_len = buf.shape[1]
                n = 0
                for ch_idx, ch_samples in enumerate(data.channelSamples):
                    if ch_idx >= buf.shape[0]:
                        break
                    new_vals = np.array([s.data for s in ch_samples], dtype=np.float32)
                    new_indices = np.array([s.sampleIndex for s in ch_samples], dtype=np.int64)
                    n = min(len(new_vals), buf_len)
                    if n == 0:
                        continue
                    write_start = self.eeg_buffer_index
                    write_end = write_start + n
                    new_vals = new_vals[-n:]
                    new_indices = new_indices[-n:]
                    if write_end <= buf_len:
                        buf[ch_idx, write_start:write_end] = new_vals
                        idx_buf[ch_idx, write_start:write_end] = new_indices
                    else:
                        first_part = buf_len - write_start
                        buf[ch_idx, write_start:] = new_vals[:first_part]
                        buf[ch_idx, :n - first_part] = new_vals[first_part:]
                        idx_buf[ch_idx, write_start:] = new_indices[:first_part]
                        idx_buf[ch_idx, :n - first_part] = new_indices[first_part:]
                    while len(self.eeg_impedance) <= ch_idx:
                        self.eeg_impedance.append(0)
                    if ch_samples:
                        self.eeg_impedance[ch_idx] = ch_samples[-1].impedance
                self.eeg_buffer_index = (self.eeg_buffer_index + n) % buf_len
            finally:
                self.eeg_buffer_lock.unlock()
            return

        if data.getDataType() == DataType.NTF_ECG:
            self.ecg_buffer_lock.lock()
            try:
                buf = self.ecg_buffer
                idx_buf = self.ecg_sample_index_buffer
                if buf is None or idx_buf is None:
                    return
                buf_len = buf.shape[1]
                n = 0
                for ch_idx, ch_samples in enumerate(data.channelSamples):
                    if ch_idx >= buf.shape[0]:
                        break
                    new_vals = np.array([s.data for s in ch_samples], dtype=np.float32)
                    new_indices = np.array([s.sampleIndex for s in ch_samples], dtype=np.int64)
                    n = min(len(new_vals), buf_len)
                    if n == 0:
                        continue
                    write_start = self.ecg_buffer_index
                    write_end = write_start + n
                    new_vals = new_vals[-n:]
                    new_indices = new_indices[-n:]
                    if write_end <= buf_len:
                        buf[ch_idx, write_start:write_end] = new_vals
                        idx_buf[ch_idx, write_start:write_end] = new_indices
                    else:
                        first_part = buf_len - write_start
                        buf[ch_idx, write_start:] = new_vals[:first_part]
                        buf[ch_idx, :n - first_part] = new_vals[first_part:]
                        idx_buf[ch_idx, write_start:] = new_indices[:first_part]
                        idx_buf[ch_idx, :n - first_part] = new_indices[first_part:]
                    while len(self.ecg_impedance) <= ch_idx:
                        self.ecg_impedance.append(0)
                    if ch_samples:
                        self.ecg_impedance[ch_idx] = ch_samples[-1].impedance
                self.ecg_buffer_index = (self.ecg_buffer_index + n) % buf_len
            finally:
                self.ecg_buffer_lock.unlock()
            return

        if data.getDataType() == DataType.NTF_BRTH:
            self.brth_buffer_lock.lock()
            try:
                buf = self.brth_buffer
                idx_buf = self.brth_sample_index_buffer
                if buf is None or idx_buf is None:
                    return
                buf_len = buf.shape[1]
                n = 0
                for ch_idx, ch_samples in enumerate(data.channelSamples):
                    if ch_idx >= buf.shape[0]:
                        break
                    new_vals = np.array([s.data for s in ch_samples], dtype=np.float32)
                    new_indices = np.array([s.sampleIndex for s in ch_samples], dtype=np.int64)
                    n = min(len(new_vals), buf_len)
                    if n == 0:
                        continue
                    write_start = self.brth_buffer_index
                    write_end = write_start + n
                    new_vals = new_vals[-n:]
                    new_indices = new_indices[-n:]
                    if write_end <= buf_len:
                        buf[ch_idx, write_start:write_end] = new_vals
                        idx_buf[ch_idx, write_start:write_end] = new_indices
                    else:
                        first_part = buf_len - write_start
                        buf[ch_idx, write_start:] = new_vals[:first_part]
                        buf[ch_idx, :n - first_part] = new_vals[first_part:]
                        idx_buf[ch_idx, write_start:] = new_indices[:first_part]
                        idx_buf[ch_idx, :n - first_part] = new_indices[first_part:]
                    while len(self.brth_impedance) <= ch_idx:
                        self.brth_impedance.append(0)
                    if ch_samples:
                        self.brth_impedance[ch_idx] = ch_samples[-1].impedance
                self.brth_buffer_index = (self.brth_buffer_index + n) % buf_len
            finally:
                self.brth_buffer_lock.unlock()
            return

        lock = self.get_buffer_lock(data.getDataType())
        lock.lock()
        try:
            buf = self.buffers.get(data.getDataType())
            idx_buf = self.sample_index_buffers.get(data.getDataType())
            if buf is None or idx_buf is None:
                return
            buf_len = buf.shape[1]
            n = 0
            buffer_index = self.buffer_indices.get(data.getDataType(), 0)
            for ch_idx, ch_samples in enumerate(data.channelSamples):
                if ch_idx >= buf.shape[0]:
                    break
                new_vals = np.array([s.data for s in ch_samples], dtype=np.float32)
                new_indices = np.array([s.sampleIndex for s in ch_samples], dtype=np.int64)
                n = min(len(new_vals), buf_len)
                if n == 0:
                    continue
                write_start = buffer_index
                write_end = write_start + n
                new_vals = new_vals[-n:]
                new_indices = new_indices[-n:]
                if write_end <= buf_len:
                    buf[ch_idx, write_start:write_end] = new_vals
                    idx_buf[ch_idx, write_start:write_end] = new_indices
                else:
                    first_part = buf_len - write_start
                    buf[ch_idx, write_start:] = new_vals[:first_part]
                    buf[ch_idx, :n - first_part] = new_vals[first_part:]
                    idx_buf[ch_idx, write_start:] = new_indices[:first_part]
                    idx_buf[ch_idx, :n - first_part] = new_indices[first_part:]
            self.buffer_indices[data.getDataType()] = (buffer_index + n) % buf_len
        finally:
            lock.unlock()

    def clear_buffers(self):
        """清空本设备所有数据缓冲区，等待新数据。"""
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
        finally:
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
    device_disconnected_sig = QtCore.pyqtSignal(str)         # address
    auto_reconnect_sig = QtCore.pyqtSignal(str, bool)        # (address, restore)
    replay_done_sig = QtCore.pyqtSignal(str)
    replay_member_done_sig = QtCore.pyqtSignal(object)       # (sensor) 多文件回放单个成员结束
    analyze_done_sig = QtCore.pyqtSignal(str, str)
    dongle_check_sig = QtCore.pyqtSignal(str)
    fft_done_sig = QtCore.pyqtSignal(int, object, object)   # (data_type, freqs, mags)，工作线程→UI 线程
    bio_fft_done_sig = QtCore.pyqtSignal(int, object, object)   # 右侧生物电每行 FFT：(data_type, {行号: freqs}, {行号: mags})

    def __init__(self):
        super().__init__()
        self.discovered_devices = []
        self.current_sensor: SensorProfile = None   # 当前在列表中选中、正在显示的设备
        self.device_states: dict = {}               # Address -> DeviceDataState（已连接设备）
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
        self.emg_lines = []
        self.bio_lines = []
        self._eeg_display_channels = 0

        self._updating_ntf_controls = False
        self._updating_filter_controls = False
        self._updating_sample_rate_controls = False
        self._debug_log_checkbox = None
        self._data_debug_log_checkbox = None
        self._ntf_checkboxes: dict = {}
        self._filter_checkboxes: dict = {}
        self._sample_rate_radios: dict = {}
        self._sample_rate_button_group = None
        self._debug_log_enabled = True
        self._data_debug_log_enabled = True
        # 每设备上次会话的日志/bin 导出路径：重连时优先续用上一条，而不是另起新文件
        self._last_log_paths: dict = {}
        self._last_data_log_paths: dict = {}
        self._replay_thread = None
        self._replay_sensor = None
        # 多文件同步回放的全部成员（单文件回放时为空，仅 _replay_sensor）
        self._replay_sensors = []
        # 回放进行中标志：由 SDK 的 onDataTransferStateChange 事件驱动
        # （数据流真正开始/结束），取代回放线程存活判断
        self._replay_active = False
        self._replay_paused = False
        self._replay_stop_requested = False

        # FFT 频谱：UI 定时器把波形快照打包成闭包提交到工作线程计算，
        # 结果经 fft_done_sig 回 UI 线程更新频谱子图；_fft_pending 防止任务堆积
        self.fft_lines = []
        self._fft_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="FFT")
        self._fft_pending = False
        self._fft_last_submit = 0.0

        # 右侧生物电每行 FFT（EEG/EMG/ECG/PPG 行：左 50% 频谱 + 右 50% 时域波形）：
        # axes_bio_fft 与 axes_eeg 等长（无频谱的行为 None），bio_fft_lines 同理；
        # 与 2D FFT 共用同一个单工作线程执行器，_bio_fft_pending 独立防堆积
        self.axes_bio_fft = []
        self.bio_fft_lines = []
        self._bio_fft_pending = False
        self._bio_fft_last_submit = 0.0
        self._eeg_axes_signature = None   # (行数, fft 行号元组或 None)，布局不变时不重建子图

        # 实时频段滤波选中项（Live Filter 单选框）：(lo, hi) 或 None；
        # 经 _on_data 懒同步到各设备状态的回调线程滤波路径
        self._filter_band = None

        # Use Queue Data 模式（对齐 C++ Qt demo，默认关）：onData 回调线程只做
        # 实时滤波 + clone 入队，分发处理（写缓冲/丢包统计/四元数/手势）由
        # 数据 worker 线程从有界队列取出后执行；direct 模式则全部在回调线程内联完成
        self._use_clone_data = False
        self._data_queue = collections.deque()
        self._data_queue_lock = threading.Lock()
        self._data_queue_event = threading.Event()
        self._data_worker_stop = False
        self._data_worker = threading.Thread(
            target=self._drain_data_queue, daemon=True, name="DataQueueDrain")
        self._data_worker.start()

        self._init_ui()

        # Debug Log 复选框默认勾选但启动时不触发 stateChanged，
        # 这里按当前开关状态主动应用一次（建时间戳子目录 + 开启调试日志），
        # 须在首次扫描/连接（BLE 子进程启动）之前执行
        if self._debug_log_enabled:
            self._apply_sdk_debug_log()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(PLOT_UPDATE_INTERVAL)
        self._rate_last_refresh = 0.0   # 实测采样率每秒刷新的时间戳

        self.add_device_sig.connect(self._add_device_item)
        self.update_device_sig.connect(self._update_device_rssi)
        self.power_changed_sig.connect(self._update_power_display)
        self.device_info_sig.connect(self._update_link_info_display)
        self.lost_packet_signal.connect(self._update_lost_packet_display)
        self.gesture_signal.connect(self._update_gesture_display)
        self.device_disconnected_sig.connect(self._on_device_disconnected)
        self.auto_reconnect_sig.connect(self._press_connect_for_address)
        self.replay_done_sig.connect(self._on_replay_done)
        self.replay_member_done_sig.connect(self._finish_replay_member)
        self.analyze_done_sig.connect(self._on_analyze_done)
        self.dongle_check_sig.connect(self._on_dongle_check_result)
        self.fft_done_sig.connect(self._on_fft_done)
        self.bio_fft_done_sig.connect(self._on_bio_fft_done)

        if not self.sensor_controller.hasDeviceFoundCallback:
            self.sensor_controller.onDeviceFoundCallback = self._on_device_found

    # ── UI ────────────────────────────────────────────────────────────────────

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
        # 上：时域波形；下：工作线程 FFT 结果驱动的频谱
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

        self.btn_check_dongle = QtWidgets.QPushButton("Check Setup Dongle")
        self.btn_check_dongle.clicked.connect(self._check_setup_dongle)
        _bold_font = self.btn_check_dongle.font()
        _bold_font.setBold(True)
        self.btn_check_dongle.setFont(_bold_font)

        self.btn_multi_start = QtWidgets.QPushButton("Multi Start")
        self.btn_multi_start.clicked.connect(self._multi_start)
        self.btn_multi_start.setEnabled(False)

        self.btn_multi_replay = QtWidgets.QPushButton("Multi Replay Bin")
        self.btn_multi_replay.clicked.connect(self._multi_replay_bin)

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

        button_layout = QtWidgets.QVBoxLayout()
        button_layout.addWidget(self.btn_scan)
        button_layout.addWidget(self.btn_stop_scan)
        button_layout.addWidget(self.btn_connect)
        button_layout.addWidget(self.btn_disconnect)
        button_layout.addStretch()

        # 回放按钮放在控制行最右边
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
        # Auto Reconnect 总开关：选中则所有 SensorProfile 的 autoReconnect 为 True
        self.chk_auto_reconnect = QtWidgets.QCheckBox("Auto Reconnect")
        self.chk_auto_reconnect.setChecked(True)   # SensorProfile.autoReconnect 默认 True
        self.chk_auto_reconnect.toggled.connect(self._on_auto_reconnect_toggled)
        device_header_layout.addWidget(self.chk_auto_reconnect)
        # Use Clone Data 开关（对齐 C++ Qt demo，默认不勾 = 不Clone,高性能）：
        self.chk_use_clone_data = QtWidgets.QCheckBox("Use Clone Data")
        self.chk_use_clone_data.setChecked(False)
        self.chk_use_clone_data.toggled.connect(self._on_use_clone_data_toggled)
        device_header_layout.addWidget(self.chk_use_clone_data)
        device_header_layout.addWidget(QtWidgets.QLabel("Discovered Devices:"))
        device_header_layout.addStretch()
        device_header_layout.addWidget(self.btn_multi_start)
        device_header_layout.addWidget(self.btn_multi_replay)
        device_header_layout.addWidget(self.btn_check_dongle)
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

        # 实时频段滤波下拉框（作用于右侧 EMG/EEG/ECG/BRTH/PPG 波形，
        # 在 SDK 数据回调线程内滤波后写入显示缓冲）
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

        options_layout = QtWidgets.QHBoxLayout()
        options_layout.addWidget(debug_log_group, stretch=1)
        options_layout.addWidget(ntf_group, stretch=1)
        options_layout.addWidget(filter_group, stretch=1)
        options_layout.addWidget(sample_rate_group, stretch=1)
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
        self.setWindowTitle(f"SynchroniSDKPython IMU + Quaternion + EMG + EEG Demo (sensor-sdk v{self.sensor_controller.getVersion()}, demo v{DEMO_VERSION})")
        self.resize(1600, 900)
        self.show()

    # ── Scan / Connect ────────────────────────────────────────────────────────

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
        # 检查/安装 dongle 驱动（Windows 弹 UAC，Linux 需终端 sudo 密码），
        # 后台线程执行避免阻塞 UI，结果经信号回到主线程提示
        self._app_log("User: check setup dongle")
        self.btn_check_dongle.setEnabled(False)
        self.btn_check_dongle.setText("Checking Dongle...")

        # Linux 非 root 时安装 udev 规则需要 sudo：密码要输到启动本程序的
        # shell（或 SDK 弹出的终端窗口），先弹提示避免用户以为程序卡死
        if sys.platform.startswith("linux") and hasattr(os, "geteuid") and os.geteuid() != 0:
            shell = os.path.basename(os.environ.get("SHELL") or "") or "shell"
            QtWidgets.QMessageBox.information(
                self, "Check Dongle",
                f"Please input sudo password in !!{shell}!!")

        def work():
            try:
                result = checkSetupDongle()
            except Exception as e:
                result = f"Error: {e}"
            self.dongle_check_sig.emit(result)

        threading.Thread(target=work, daemon=True).start()

    def _multi_start(self):
        # 停掉所有已连接设备的传输，再用 controller 的同步起流对齐启动：
        sensors = [state.sensor for state in self.device_states.values()
                   if state.sensor.isReady
                   and state.sensor.hasInited]
        if not sensors:
            self._app_log("User: multi start rejected (no connected device)", "W")
            self.status_label.setText("No connected device to sync-start")
            return
        self._app_log(f"User: multi start on {len(sensors)} device(s)")
        self.btn_multi_start.setEnabled(False)
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

    def _start_with_model_params(self, sensors):
        # 同型号设备用默认对齐参数，混合型号不做首包时差校验、重试 5 次
        model_names = set()
        for s in sensors:
            info = s.getDeviceInfo()
            model_names.add(info.ModelName if info else None)
        if len(model_names) == 1 and None not in model_names:
            return self.sensor_controller.multiStartDataNotification(sensors)
        return self.sensor_controller.multiStartDataNotification(
            sensors, timeout=60.0, maxDelayDispersionMs=-1, maxAttempts=5)

    def _on_auto_reconnect_toggled(self, checked: bool):
        self._app_log(f"User: auto reconnect {'ON' if checked else 'OFF'}")
        # 同步到所有已创建的 SensorProfile（含回放虚拟设备）
        for state in self.device_states.values():
            state.sensor.autoReconnect = checked

    def _on_use_clone_data_toggled(self, checked: bool):
        # direct / queue 模式切换：队列中残留批次继续由 worker 消费，
        # 两模式的缓冲写入路径相同，无需清空
        self._use_clone_data = checked
        self._app_log(f"User: use queue data {'ON' if checked else 'OFF'}")

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

    def _on_device_found(self, device_list: List[BLEDevice]):
        self._stop_scan()
        # 不过滤设备名，扫描到的所有设备都列入列表
        for d in device_list:
            existing = next((x for x in self.discovered_devices if x.Address == d.Address), None)
            if existing is None:
                self.discovered_devices.append(d)
                self.add_device_sig.emit(f"RSSI: {d.RSSI}, Name: {d.Name}, Address: {d.Address}")
            else:
                # SDK 在回调前已更新共享 BLEDevice 的 RSSI（同一对象，比较无意义），
                # 每次扫描返回都刷新一次列表项显示并重排
                existing.RSSI = d.RSSI
                self.update_device_sig.emit(d.Address, d.RSSI)

    def _add_device_item(self, text: str):
        item = QtWidgets.QListWidgetItem(text)
        # 记录 RSSI 到 UserRole，插入后按信号强度从大到小排序
        try:
            rssi = int(text.split("RSSI: ")[1].split(",")[0])
        except (IndexError, ValueError):
            rssi = None
        item.setData(QtCore.Qt.UserRole, rssi)
        self.device_list.addItem(item)
        self._sort_device_list()

    def _sort_device_list(self):
        """按 RSSI 从大到小重排设备列表项（未记录 RSSI 的排最后）。"""

        def rssi_of(it):
            rssi = it.data(QtCore.Qt.UserRole)
            return rssi if isinstance(rssi, int) else -999

        items = []
        while self.device_list.count():
            items.append(self.device_list.takeItem(0))
        items.sort(key=rssi_of, reverse=True)
        for it in items:
            self.device_list.addItem(it)

    def _update_device_rssi(self, addr: str, rssi: int):
        """扫描到新 RSSI 时更新列表项文本（保留 [Connected] 前缀）并重排。"""
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            text = item.text()
            if f"Address: {addr}" not in text:
                continue
            d = next((x for x in self.discovered_devices if x.Address == addr), None)
            if d is None:
                return
            new_text = f"RSSI: {rssi}, Name: {d.Name}, Address: {d.Address}"
            if text.startswith("[Connected] "):
                new_text = "[Connected] " + new_text
            item.setText(new_text)
            item.setData(QtCore.Qt.UserRole, rssi)
            break
        self._sort_device_list()

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
        """当前显示设备对应的 DeviceDataState，未连接时返回 None。"""
        if self.current_sensor is None:
            return None
        return self.device_states.get(self.current_sensor.BLEDevice.Address)

    def _on_device_selected(self, item):
        """单击设备列表只切换当前显示的设备，不触发连接。"""
        addr = item.text().split("Address: ")[1].strip() if "Address: " in item.text() else None
        state = self.device_states.get(addr) if addr else None
        self.current_sensor = state.sensor if state is not None else None
        self._refresh_display_for_state(state)
        self._update_button_states()

    def _update_button_states(self):
        addr = self._selected_address()
        connected = addr is not None and addr in self.device_states
        self.btn_connect.setEnabled(addr is not None and not connected)
        self.btn_disconnect.setEnabled(connected)
        self.btn_multi_start.setEnabled(len(self.device_states) >= 1)

    def _update_device_item_text(self, addr: str, connected: bool):
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            text = item.text()
            if f"Address: {addr}" not in text:
                continue
            if text.startswith("[Connected] "):
                text = text[len("[Connected] "):]
            if connected:
                text = "[Connected] " + text
            item.setText(text)
            break

    def _connect_selected_device(self):
        """连接列表中选中的设备，支持同时连接多台设备。"""
        device = self._selected_list_device()
        if device is None:
            self._app_log("User: connect rejected (no device selected)", "W")
            self.status_label.setText("Please select a device in the list first")
            return
        addr = device.Address
        if addr in self.device_states:
            return

        self._app_log(f"User: connect {device.Name} ({addr})")
        sensor = self.sensor_controller.requireSensor(device)
        if sensor is None:
            self._app_log(f"App: failed to create SensorProfile for {addr}", "E")
            self.status_label.setText("Failed to create SensorProfile")
            return

        sensor.onDataCallback  = self._on_data
        sensor.onStateChanged  = self._on_state_changed
        sensor.onErrorCallback = self._on_error
        sensor.onPowerChanged  = self._on_power_changed
        sensor.onDeviceInfoUpdate = self._on_device_info_update
        # 自动重连找回设备时：等效于按下 Connect 按钮（走本方法完整流程）
        sensor.onAutoReconnect = self._on_auto_reconnect
        sensor.autoReconnect = self.chk_auto_reconnect.isChecked()

        self.status_label.setText(f"Connecting: {device.Name} ...")
        self.btn_connect.setEnabled(False)

        if not sensor.isReady:
            if not sensor.connect():
                self._app_log(f"App: failed to connect to {device.Name} ({addr})", "E", sensor)
                self.status_label.setText(f"Failed to connect to {device.Name}")
                self._update_button_states()
                return

        state = DeviceDataState(sensor)

        if not sensor.hasInited:
            if not sensor.init(PACKAGE_COUNT, POWER_REFRESH_PERIOD_IN_MS):
                self._app_log(f"App: failed to initialize {device.Name} ({addr})", "E", sensor)
                self.status_label.setText(f"Failed to initialize {device.Name}")
                self._update_button_states()
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
            # GEST 的标称值 DeviceInfo 不提供，首个数据批到达时从批次自带的
            # sampleRate/通道数补齐
            state.status_parts.append(("GEST", 0, 0, DataType.NTF_GEST))
            state.status_text = state.build_status_text()

        if not sensor.isDataTransfering:
            if not sensor.startDataNotification():
                self._app_log(f"App: failed to start data stream on {addr}", "E", sensor)
                self.status_label.setText("Failed to start data stream")
                self._update_button_states()
                return

        self._app_log(f"App: device connected and streaming: {device.Name} ({addr})", sensor=sensor)
        self.device_states[addr] = state
        self._update_device_item_text(addr, connected=True)

        # 初始电量发布发生在 state 注册之前（子进程 init 成功后立即发布），
        # 这里显式补取一次，避免切换显示时电量显示 "--"
        try:
            power = sensor.getBatteryLevel()
            if power is not None and power >= 0:
                state.last_power = power
        except Exception:
            pass

        # 若连接的是当前选中的设备，将其设为当前显示设备
        if self._selected_address() == addr:
            self.current_sensor = sensor

        # 根据全局 Debug Log 开关状态初始化新设备的日志设置：
        # 重连时优先续用上次的日志/bin 文件（默认追加，而不是另起新文件）
        if self._debug_log_enabled:
            log_path = self._last_log_paths.get(addr) or "True"
            sensor.setParam("DEBUG_LOG_PATH", log_path)
            current = sensor.getParam("DEBUG_LOG_PATH")
            if current:
                self._last_log_paths[addr] = current
        if self._data_debug_log_enabled:
            data_path = self._last_data_log_paths.get(addr) or "True"
            sensor.setParam("DEBUG_BLE_DATA_PATH", data_path)
            current = sensor.getParam("DEBUG_BLE_DATA_PATH")
            if current:
                self._last_data_log_paths[addr] = current

        # 查询并缓存设备 NTF/FILTER 状态
        self._refresh_control_states(sensor)

        if self.current_sensor == sensor:
            self._refresh_display_for_state(state)

        self._update_button_states()

    def _disconnect_selected_device(self):
        """断开当前选中（显示）的设备，其余已连接设备不受影响。"""
        sensor = self.current_sensor
        if sensor is None:
            return
        self._app_log(f"User: disconnect {sensor.BLEDevice.Address}", sensor=sensor)
        self.btn_disconnect.setEnabled(False)
        self.btn_connect.setEnabled(False)
        for cb in self._ntf_checkboxes.values():
            cb.setEnabled(False)
        for cb in self._filter_checkboxes.values():
            cb.setEnabled(False)
        self.status_label.setText("Disconnecting...")
        sensor.disconnect()
        # 后续清理由 onStateChanged -> _on_device_disconnected 完成

    # ── Bin 文件回放 ────────────────────────────────────────────────────────────

    def _set_replay_mode_ui(self, replaying: bool):
        """回放期间禁用扫描 / 连接设备 / 调试日志等实时设备控件，回放结束后恢复。
        已连接的设备不受影响（回放开始前会拒绝在有设备连接时进入回放）。"""
        if replaying:
            # 回放不经过实时链路：停掉进行中的扫描
            if self.sensor_controller.isScanning:
                self.sensor_controller.stopScan()
            self.btn_stop_scan.setEnabled(False)
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(False)
            self.btn_multi_start.setEnabled(False)
            self.btn_multi_replay.setEnabled(False)
        else:
            self._update_button_states()
            self.btn_multi_replay.setEnabled(True)
        self.btn_scan.setEnabled(not replaying)
        self.device_list.setEnabled(not replaying)
        self._debug_log_checkbox.setEnabled(not replaying)
        self._data_debug_log_checkbox.setEnabled(not replaying)

    def _replay_bin_file(self):
        """选择一个 bin 文件并按原始时间节奏回放，数据显示流程与实时数据一致。"""
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

        self._app_log(f"User: replay bin file: {path}")
        config = self.sensor_controller.getBinFileInfo(path)
        if config is None:
            self._app_log(f"App: invalid bin file (no config record): {path}", "W")
            self.status_label.setText("Invalid bin file: no config record found")
            return

        mac = config.get("device_mac")
        if not mac:
            self.status_label.setText("Invalid bin file: config missing device_mac")
            return
        name = config.get("device_name") or ""

        # 用 bin 配置记录中的设备信息初始化显示缓冲区
        info = DeviceInfo()
        for key, value in (config.get("device_info") or {}).items():
            if hasattr(info, key):
                try:
                    setattr(info, key, value)
                except Exception:
                    pass

        sensor = self.sensor_controller.requireSensor(BLEDevice(name, mac, 0))
        if sensor is None:
            self.status_label.setText("Failed to create SensorProfile for replay")
            return
        sensor.onDataCallback = self._on_data
        sensor.onErrorCallback = self._on_error
        # 回放中配置记录切换可能改变采样率，SDK 会推 device_info_update，
        # 据此重建生物电/IMU 缓冲与横轴
        sensor.onDeviceInfoUpdate = self._on_device_info_update
        # 回放的数据流开关事件：回放开始/结束（含异常终止）时驱动 _replay_active
        sensor.onDataTransferStateChange = self._on_replay_transfer_state
        sensor.autoReconnect = self.chk_auto_reconnect.isChecked()

        self._replay_sensor = sensor
        # 单文件回放不属于任何组：清空组成员表，
        # transfer OFF 事件走单回放分支直接复位 _replay_active
        self._replay_sensors = []

        # 回放传感器作为一台虚拟设备进入 device_states：
        # _on_data 按地址路由到它的 DeviceDataState，显示流程与实时设备一致
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
        # 回放起始速率来自 bin 首条配置记录，单选框选中态同步（保持禁用态不变）
        if info.EegSampleRate > 0:
            state.sample_rate_state = ([], int(info.EegSampleRate))
            self._set_sample_rate_checked(int(info.EegSampleRate))

        self._replay_paused = False
        self._replay_stop_requested = False
        # 先在点击路径上置位，挡住事件到达前的重复点击；之后由
        # onDataTransferStateChange 事件与 _on_replay_done 维护
        self._replay_active = True
        self.btn_replay.setEnabled(False)
        self.btn_multi_replay.setEnabled(False)
        self.btn_replay_pause.setEnabled(True)
        self.btn_replay_pause.setText("Pause Replay")
        self.btn_replay_stop.setEnabled(True)
        self._set_replay_mode_ui(True)

        def _do_replay():
            try:
                result = self.sensor_controller.replayBinFile(
                    path, sensor, realtime=True)
                if result is None:
                    self.replay_done_sig.emit("Replay failed to start")
                elif self._replay_stop_requested:
                    self.replay_done_sig.emit("Replay stopped")
                else:
                    self.replay_done_sig.emit(f"Replay finished: {Path(path).name}")
            except Exception as e:
                self.replay_done_sig.emit(f"Replay error: {e}")

        self._replay_thread = threading.Thread(target=_do_replay, daemon=True, name="BinReplay")
        self._replay_thread.start()

    def _multi_replay_bin(self):
        """选择多个 bin 文件，按共享时钟对齐同步回放（SDK multiReplayBinFile）：
        全组最早首条数据记录为 t=0，保留采集时的相对偏移；暂停任一成员整组暂停。"""
        if self.device_states:
            self.status_label.setText("Please disconnect all devices before replaying bin files")
            return
        if self._replay_active:
            return

        default_dir = Path.home() / "Documents" / "sensorsdklog"
        start_dir = str(default_dir) if default_dir.exists() else str(Path.home())
        paths, _ = QtWidgets.QFileDialog.getOpenFileNames(
            self, "Select Bin Files (Multi Replay)", start_dir, "Bin Files (*.bin)"
        )
        if not paths:
            return
        self._app_log(f"User: multi replay bin files: {paths}")

        members = []  # (path, sensor, state)
        seen_macs = set()
        for path in paths:
            config = self.sensor_controller.getBinFileInfo(path)
            if config is None:
                self._app_log(f"App: invalid bin file (no config record): {path}", "W")
                continue
            mac = config.get("device_mac")
            if not mac:
                self._app_log(f"App: invalid bin file (config missing device_mac): {path}", "W")
                continue
            if mac in seen_macs:
                self._app_log(f"App: duplicate device mac skipped: {mac}", "W")
                continue
            seen_macs.add(mac)
            name = config.get("device_name") or ""

            # 用 bin 配置记录中的设备信息初始化显示缓冲区
            info = DeviceInfo()
            for key, value in (config.get("device_info") or {}).items():
                if hasattr(info, key):
                    try:
                        setattr(info, key, value)
                    except Exception:
                        pass

            sensor = self.sensor_controller.requireSensor(BLEDevice(name, mac, 0))
            if sensor is None:
                self._app_log(f"App: failed to create SensorProfile for {mac}", "E")
                continue
            sensor.onDataCallback = self._on_data
            sensor.onErrorCallback = self._on_error
            sensor.onDeviceInfoUpdate = self._on_device_info_update
            sensor.onDataTransferStateChange = self._on_replay_transfer_state
            sensor.autoReconnect = self.chk_auto_reconnect.isChecked()

            state = DeviceDataState(sensor)
            state.info = info
            state.init_buffers(info, EEG_AXIS_COUNT)
            duration = config.get("replay_duration", 0.0)
            state.status_text = (
                f"Replaying: {Path(path).name} (duration {duration:.1f}s, multi-sync) ...")
            members.append((path, sensor, state))
        if len(members) < 2:
            self.status_label.setText("Multi replay needs at least 2 valid bin files")
            return

        # 回放成员作为虚拟设备进入 device_states：_on_data 按地址路由到各自的
        # DeviceDataState，显示流程与实时设备一致
        self._replay_sensors = [sensor for _, sensor, _ in members]
        # 组回放没有“主”成员：Pause/Stop 对全部成员下发（组时钟语义下幂等），
        # 成员级收尾按各自的 transfer OFF 事件逐个进行
        self._replay_sensor = None
        for _, sensor, state in members:
            self.device_states[sensor.BLEDevice.Address] = state
            if state.info.EegSampleRate > 0:
                state.sample_rate_state = ([], int(state.info.EegSampleRate))
        self.current_sensor = members[0][1]
        self._refresh_display_for_state(members[0][2])
        if members[0][2].info.EegSampleRate > 0:
            self._set_sample_rate_checked(int(members[0][2].info.EegSampleRate))

        self._replay_paused = False
        self._replay_stop_requested = False
        # 先在点击路径上置位，挡住事件到达前的重复点击；之后由
        # onDataTransferStateChange 事件与 _on_replay_done 维护
        self._replay_active = True
        self.btn_replay.setEnabled(False)
        self.btn_multi_replay.setEnabled(False)
        self.btn_replay_pause.setEnabled(True)
        self.btn_replay_pause.setText("Pause Replay")
        self.btn_replay_stop.setEnabled(True)
        self._set_replay_mode_ui(True)

        member_paths = [p for p, _, _ in members]
        member_sensors = [s for _, s, _ in members]

        def _do_replay():
            try:
                results = self.sensor_controller.multiReplayBinFile(
                    member_paths, sensors=member_sensors, realtime=True)
                ok_count = sum(1 for r in results if r is not None)
                if self._replay_stop_requested:
                    self.replay_done_sig.emit("Multi replay stopped")
                elif ok_count == 0:
                    self.replay_done_sig.emit("Multi replay failed to start")
                else:
                    self.replay_done_sig.emit(
                        f"Multi replay finished: {ok_count}/{len(results)} device(s) ok")
            except Exception as e:
                self.replay_done_sig.emit(f"Multi replay error: {e}")

        self._replay_thread = threading.Thread(target=_do_replay, daemon=True, name="BinMultiReplay")
        self._replay_thread.start()

    def _on_replay_transfer_state(self, sensor: SensorProfile, is_transferring: bool):
        """回放会话的数据流开关事件（SDK 回调线程，对齐 C++ demo_multi）：
        开始只刷新 _replay_active；结束时多文件回放按成员逐个收尾
        （_finish_replay_member），单文件回放直接复位 _replay_active——
        最终结果消息统一由回放线程的 replay_done_sig 携带。"""
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

    def _finish_replay_member(self, sensor: SensorProfile):
        """多文件回放的单个成员结束（UI 线程）：立即移除它的虚拟设备状态，
        显示切到剩余成员；全部成员结束后复位 _replay_active，
        最终收尾仍由回放线程的 replay_done_sig 完成。"""
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

    def _toggle_replay_pause(self):
        """暂停/恢复当前回放（多文件回放对每个成员下发——组时钟语义下幂等）。"""
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
        """停止当前回放（多文件回放时逐个停止所有成员）。"""
        sensors = list(self._replay_sensors) if self._replay_sensors else (
            [self._replay_sensor] if self._replay_sensor is not None else [])
        if not sensors:
            return
        self._replay_stop_requested = True
        self.btn_replay_stop.setEnabled(False)
        self.btn_replay_pause.setEnabled(False)
        results = []
        for sensor in sensors:
            result = self.sensor_controller.stopBinReplay(sensor)
            results.append(result)
            self._app_log(f"User: stop replay -> {result}",
                          "I" if result == "OK" else "W", sensor)
        if all(r != "OK" for r in results):
            self.status_label.setText(f"Stop replay failed: {results[0]}")
            return
        self.status_label.setText("Stopping replay ...")

    def _on_replay_done(self, message: str):
        self._app_log(f"App: replay done: {message}")
        # 回放结束：移除回放用的虚拟设备状态（成员级收尾可能已移除一部分），
        # 恢复实时设备控件
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
        self.btn_multi_replay.setEnabled(True)
        self.btn_replay_pause.setEnabled(False)
        self.btn_replay_pause.setText("Pause Replay")
        self.btn_replay_stop.setEnabled(False)
        self._set_replay_mode_ui(False)
        self._replay_paused = False
        self._replay_stop_requested = False
        # 回放未成功启动时不会有数据流事件，这里兜底复位
        self._replay_active = False

    # ── Bin 文件离线解析 ──────────────────────────────────────────────────────

    def _analyze_bin_file(self):
        """选择 bin 文件并在同目录解析为 CSV，完成后用系统默认编辑器打开。"""
        if getattr(self, "_analyze_thread", None) is not None and self._analyze_thread.is_alive():
            return
        default_dir = Path.home() / "Documents" / "sensorsdklog"
        start_dir = str(default_dir) if default_dir.exists() else str(Path.home())
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Bin File to Analyze", start_dir, "Bin Files (*.bin)"
        )
        if not path:
            return

        self._app_log(f"User: analyze bin file: {path}")
        self.btn_analyze.setEnabled(False)
        self.status_label.setText(f"Analyzing: {Path(path).name} ...")

        def _do_analyze():
            try:
                csv_path = self.sensor_controller.parseBinToCsv(path)
                self.analyze_done_sig.emit(csv_path, "")
            except Exception as e:
                self.analyze_done_sig.emit("", str(e))

        self._analyze_thread = threading.Thread(target=_do_analyze, daemon=True, name="BinAnalyze")
        self._analyze_thread.start()

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
        """用系统默认应用打开文件（macOS open / Windows startfile / Linux xdg-open）。"""
        try:
            if sys.platform == "darwin":
                subprocess.Popen(["open", path])
            elif sys.platform.startswith("win"):
                os.startfile(path)  # type: ignore[attr-defined]
            else:
                subprocess.Popen(["xdg-open", path])
        except Exception:
            pass

    # ── Data Routing ──────────────────────────────────────────────────────────

    def _on_data(self, sensor: SensorProfile, data_list: list):
        """SDK 数据回调（回调线程）：一批 SensorData 列表一次性交付。

        实时滤波一律在本回调线程完成（queue 模式下也在入队前）；之后
        direct 模式（Use Queue Data 未勾选）内联分发处理，queue 模式
        clone 入队、由数据 worker 线程分发。缓冲区写入有各缓冲区锁保护，
        Qt 界面更新经信号/状态锁。
        """
        addr = sensor.BLEDevice.Address
        state = self.device_states.get(addr)
        if state is None:
            return
        use_clone = self._use_clone_data
        for data in data_list:
            if not (data and data.channelSamples):
                continue
            # 实际采样率收集：覆盖所有收到的数据类型
            state.note_data_received(data)
            # 实时滤波频段懒同步：UI 切换后下个数据批把新频段带到回调线程
            # （引用比较即可——切换时 _filter_band 整体替换，identity 必然变化）
            if state.live_filter_band is not self._filter_band:
                state.set_live_filter_band(self._filter_band)
            # 实时滤波必须在 onData 中完成：queue 模式入队的数据已是滤波结果
            state.filter_sensor_data(data)
            if use_clone:
                # 回调返回后 SDK 对象池会复用这批 SensorData，入队必须深拷贝
                self._enqueue_data(addr, data.clone())
            else:
                # 回调返回后 SDK 对象池会复用这批 SensorData，处理必须要快,否则会丢数据,表现为isDataValid()返回False
                self._enqueue_data(addr, data) 

    def _dispatch_sensor_data(self, addr: str, data: SensorData):
        """单批数据的分发处理（写环形缓冲/丢包统计/四元数/手势）：
        queue 模式由数据 worker 线程调用。"""
        state = self.device_states.get(addr)
        if state is None:
            return
        if not data.isDataValid():
            self._app_log(f"App: Your data process runs too slow: {data}", "W", data.getDeviceName())
            return

        #获得数据批的绝对时间戳（秒为单位），用于计算数据批间隔、绘图横轴等,与LSL标准相同
        LSLTimeStamp = data.getAbsTimeStampInSec(0,0)
        
        if data.getDataType() == DataType.NTF_IMU:
            # 新 EMG 设备的 IMU 聚合批：拆成四路独立批走原有分发/显示路径
            for sub in split_imu_aggregate(data):
                if sub.getDataType() in state.buffers:
                    self._append_sensor_data(addr, sub)
                if sub.getDataType() == DataType.NTF_QUATERNION:
                    self._update_quaternion(state, sub)
            return
        if (data.getDataType() in state.buffers
                or data.getDataType() in (DataType.NTF_EEG, DataType.NTF_ECG, DataType.NTF_BRTH, DataType.NTF_EMG)
                or (state.bio_buffers and data.getDataType() in state.bio_buffers)):
            self._append_sensor_data(addr, data)
        if data.getDataType() == DataType.NTF_QUATERNION:
            self._update_quaternion(state, data)
        if data.getDataType() == DataType.NTF_GEST:
            self._handle_gesture_data(addr, data)

    # queue 模式数据队列上限：消费跟不上时丢最旧的批次，避免内存无界增长
    # （对齐 C++ Qt demo 的 1000 批上限）
    DATA_QUEUE_MAX_BATCHES = 1000

    def _enqueue_data(self, addr: str, data: SensorData):
        """queue 模式入队（onData 回调线程）：有界队列，满了丢最旧。"""
        with self._data_queue_lock:
            while len(self._data_queue) >= self.DATA_QUEUE_MAX_BATCHES:
                self._data_queue.popleft()
            self._data_queue.append((addr, data))
        self._data_queue_event.set()

    def _drain_data_queue(self):
        """数据 worker 线程：批量取出队列数据并分发到显示缓冲；
        收到停止标记且队列排空后退出。"""
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
        """丢包统计上报 + 写环形缓冲区（append_data 内部有各缓冲区锁）。"""
        state = self.device_states.get(addr)
        if state is None:
            return
        if data.getLostPackageCount() > 0:
            type_name = DataType(data.getDataType()).name if data.getDataType() is not None else "Unknown"
            self.lost_packet_signal.emit(addr, type_name, data.getLostPackageCount())
        state.append_data(data)

    def closeEvent(self, event):
        self._app_log("App: demo window closing")
        self.timer.stop()
        # 停数据 worker：置停止标记并唤醒，等其排空队列后退出
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

    # ── Bottom-left 2D Waveform ───────────────────────────────────────────────

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

        # 频谱子图：通道线与波形一致，数据由工作线程 FFT 闭包的结果异步填充
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

    # ── FFT 频谱（工作线程闭包计算，信号回 UI 线程更新）─────────────────────────

    def _submit_fft(self, dt, sample_rate: int, buf_snapshot):
        """UI 线程调用：把 FFT 计算连同数据快照打包成闭包提交到工作线程，
        计算不占用 UI 线程；快照为已按时间序重排的副本，闭包运行期间不被改写。"""
        self._fft_pending = True

        def _compute_fft():
            try:
                # Hann 窗抑制频谱泄漏，幅值按窗增益归一化（峰值≈真实幅值）
                window = np.hanning(buf_snapshot.shape[1])
                windowed = buf_snapshot * window
                mags = np.abs(np.fft.rfft(windowed, axis=1)) / max(window.sum(), 1e-12) * 2
                freqs = np.fft.rfftfreq(buf_snapshot.shape[1], d=1.0 / sample_rate)
                self.fft_done_sig.emit(int(dt), freqs, mags)
            except Exception as e:
                print(f"[FFT] compute error: {e}")
            finally:
                self._fft_pending = False

        try:
            self._fft_executor.submit(_compute_fft)
        except RuntimeError:
            # 窗口关闭后执行器已 shutdown，退出途中的最后一次定时器触发
            self._fft_pending = False

    def _on_fft_done(self, dt_value: int, freqs, mags):
        """工作线程 FFT 结果（经 fft_done_sig 回到 UI 线程）：刷新频谱子图。
        期间用户可能已切换数据类型，类型不匹配的结果直接丢弃。"""
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
        """右侧生物电每行频谱：row_specs 为 [(行号, 已按时间序重排的 1-D 波形快照,
        采样率), ...]，各行采样率可不同（ECG/PPG 行与 EEG 行混排）。
        打包成闭包提交到工作线程（与 2D FFT 共用同一个单线程执行器，任务串行执行），
        计算不占用 UI 线程；结果（按行号索引的字典）经 bio_fft_done_sig 回 UI 线程。"""
        self._bio_fft_pending = True

        def _compute_bio_fft():
            try:
                freqs_map = {}
                mags_map = {}
                for row, data, sr in row_specs:
                    if sr <= 0 or data.size == 0:
                        continue
                    # Hann 窗抑制频谱泄漏，幅值按窗增益归一化（峰值≈真实幅值）
                    window = np.hanning(data.size)
                    windowed = data * window
                    mags_map[row] = np.abs(np.fft.rfft(windowed)) / max(window.sum(), 1e-12) * 2
                    freqs_map[row] = np.fft.rfftfreq(data.size, d=1.0 / sr)
                self.bio_fft_done_sig.emit(int(dt), freqs_map, mags_map)
            except Exception as e:
                print(f"[FFT] bio compute error: {e}")
            finally:
                self._bio_fft_pending = False

        try:
            self._fft_executor.submit(_compute_bio_fft)
        except RuntimeError:
            # 窗口关闭后执行器已 shutdown，退出途中的最后一次定时器触发
            self._bio_fft_pending = False

    def _on_bio_fft_done(self, dt_value: int, freqs_map, mags_map):
        """工作线程生物电 FFT 结果（经 bio_fft_done_sig 回到 UI 线程）：
        按行号刷新各行左侧的频谱子图。结果到达期间显示模式可能已切换
        （EMG↔EEG↔PPG、翻页），类型或行号不匹配的结果直接丢弃。"""
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

    # ── Right-side EMG / EEG (+ ECG + BRTH) Waveform ──────────────────────────

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
        """按显示模式重建右侧共享图表的子图。
        fft_rows 为 None：单列布局，每行一个通宽波形子图；
        否则 fft_rows 为「左 FFT 频谱 + 右时域波形」各 50% 宽度的行号集合
        （EEG/EMG 模式的通道行、EEG 页的 ECG 行、PPG 页的 EEG/PPG 行），
        其余行（BRTH/SpO2/未用）为通宽波形。
        布局签名与当前一致时不做任何事，否则清空 figure 重新创建。"""
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
        eeg_buffer_copy = None
        eeg_idx_buf_copy = None
        eeg_buffer_index = 0
        ecg_buffer_copy = None
        ecg_idx_buf_copy = None
        ecg_buffer_index = 0
        brth_buffer_copy = None
        brth_idx_buf_copy = None
        brth_buffer_index = 0

        if state is not None:
            state.eeg_buffer_lock.lock()
            state.ecg_buffer_lock.lock()
            state.brth_buffer_lock.lock()
            try:
                eeg_available = state.eeg_buffer is not None and state.eeg_sample_index_buffer is not None
                ecg_available = state.has_ecg and state.ecg_buffer is not None and state.ecg_sample_index_buffer is not None
                brth_available = state.has_brth and state.brth_buffer is not None and state.brth_sample_index_buffer is not None
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
            finally:
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
            self.eeg_lines = []
            self.ecg_line = None
            self.brth_line = None
            self.emg_lines = []
            self.bio_lines = []
            self._eeg_display_channels = 0
            self._update_page_label()
            self._update_page_buttons()
            return

        self.bio_title_label.setText("EEG + ECG + BRTH Waveform")
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

        start_ch, end_ch = self._eeg_page_range()
        page_eeg_count = max(0, end_ch - start_ch)
        self._eeg_display_channels = page_eeg_count

        brth_axis_index = EEG_AXIS_COUNT - 1 if brth_available else None
        ecg_axis_index = EEG_AXIS_COUNT - 1 - int(brth_available) if ecg_available else None

        # EEG 通道行与 ECG 行：左 50% FFT 频谱 + 右 50% 时域波形；BRTH/未用行通宽
        fft_rows = set(range(page_eeg_count))
        if ecg_axis_index is not None:
            fft_rows.add(ecg_axis_index)
        self._reset_eeg_axes(EEG_AXIS_COUNT, fft_rows)
        self.bio_fft_lines = [None] * len(self.axes_eeg)

        self.eeg_lines = []
        t = np.linspace(-BIO_BUFFER_SECONDS, 0, eeg_buffer_copy.shape[1])
        t_ecg = np.linspace(-BIO_BUFFER_SECONDS, 0, ecg_buffer_copy.shape[1]) if ecg_available else None
        t_brth = np.linspace(-BIO_BUFFER_SECONDS, 0, brth_buffer_copy.shape[1]) if brth_available else None

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
                # 左半：该通道的 FFT 频谱子图（数据由工作线程结果异步填充）
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
                # 左半：ECG 的 FFT 频谱子图（数据由工作线程结果异步填充）
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
        """EMG 设备的右侧显示区：复用 EEG 的 8 行子图区，显示 EMG 通道（不分页），
        每个通道行左 50% 为 FFT 频谱、右 50% 为时域波形。"""
        emg_available = False
        emg_buffer_copy = None
        emg_idx_buf_copy = None
        emg_buffer_index = 0
        display_channels = 0

        state.emg_buffer_lock.lock()
        try:
            emg_available = state.emg_buffer is not None and state.emg_sample_index_buffer is not None
            if emg_available:
                emg_buffer_copy = state.emg_buffer.copy()
                emg_idx_buf_copy = state.emg_sample_index_buffer.copy()
                emg_buffer_index = state.emg_buffer_index
                display_channels = state.emg_display_channels
        finally:
            state.emg_buffer_lock.unlock()

        self.eeg_lines = []
        self.ecg_line = None
        self.brth_line = None
        self.bio_lines = []
        self._eeg_display_channels = 0
        self._last_plotted_sample_indices.pop(DataType.NTF_EEG, None)
        self._last_plotted_sample_indices.pop(DataType.NTF_ECG, None)
        self._last_plotted_sample_indices.pop(DataType.NTF_BRTH, None)

        if not emg_available:
            self._reset_eeg_axes(EEG_AXIS_COUNT, set())
            self.bio_fft_lines = []
            for ax in self.axes_eeg:
                ax.cla()
                ax.set_visible(True)
            self.bio_title_label.setText("EMG Waveform")
            self.axes_eeg[0].set_title("EMG (Device not supported or disabled)")
            self.canvas_eeg.draw_idle()
            self._last_plotted_sample_indices.pop(DataType.NTF_EMG, None)
            self.emg_lines = []
            self._update_page_label()
            self._update_page_buttons()
            return

        self.bio_title_label.setText("EMG Waveform")
        self._last_plotted_sample_indices[DataType.NTF_EMG] = int(emg_idx_buf_copy.max())

        if display_channels == 0:
            display_channels = min(emg_buffer_copy.shape[0], EEG_AXIS_COUNT)

        # 每个 EMG 通道行：左 50% FFT 频谱 + 右 50% 时域波形
        self._reset_eeg_axes(EEG_AXIS_COUNT, set(range(display_channels)))
        self.bio_fft_lines = [None] * len(self.axes_eeg)

        self.emg_lines = []
        t = np.linspace(-BIO_BUFFER_SECONDS, 0, emg_buffer_copy.shape[1])
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
                # 左半：该通道的 FFT 频谱子图（数据由工作线程结果异步填充）
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
        """PPG 设备的右侧显示区：6 行（2×EEG fp1/fp2 + 2×PPG red/ir + 2×SpO2），
        EEG/PPG 行左 50% 为 FFT 频谱、右 50% 为时域波形；SpO2/heart_rate 为
        低频派生量，保持通宽波形不做 FFT。"""
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
                    # Reassemble the circular buffer into chronological order for plotting.
                    buffers_copy[dt] = np.roll(buf.copy(), -buf_idx, axis=1)
                    idx_buffers_copy[dt] = idx_buf.copy()
                    has_any_data = True
        finally:
            state.bio_buffer_lock.unlock()

        # PPG 布局不使用 EEG 分页与 EMG 线条，全部置空
        self.eeg_lines = []
        self.ecg_line = None
        self.brth_line = None
        self.emg_lines = []
        self._eeg_display_channels = 0

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
            # 左半：该行的 FFT 频谱子图（数据由工作线程结果异步填充）
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

    # ── 3D Quaternion ─────────────────────────────────────────────────────────

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
            if data.getDataType() == DataType.NTF_QUATERNION:
                if len(data.channelSamples) == 4 and len(data.channelSamples[0]) > 0:
                    quaternion = [
                        data.channelSamples[0][0].data,
                        data.channelSamples[1][0].data,
                        data.channelSamples[2][0].data,
                        data.channelSamples[3][0].data,
                    ]
                    state.quaternion_lock.lock()
                    state.quaternion = quaternion
                    state.quaternion_lock.unlock()
        except Exception as e:
            print(f"Quaternion update exception: {e}")

    # ── Periodic Refresh ──────────────────────────────────────────────────────

    def _update_plots(self):
        if self.windowState() & QtCore.Qt.WindowMinimized:
            return

        state = self._current_state()
        if state is None:
            return

        # 每秒结算一次实测采样率：状态行显示标称值，实测速率单独显示在下一行
        now = time.time()
        if now - self._rate_last_refresh >= 1.0:
            self._rate_last_refresh = now
            state.update_actual_rates()
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
                # Reassemble the circular buffer once for all channels.
                buf_copy = np.roll(buf_copy, -buffer_index, axis=1)
                # FFT 频谱：重排后的快照打包成闭包提交工作线程，结果经信号回 UI 线程
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
                    # Avoid tiny y-limit changes that force a full redraw.
                    cur_ylim = self.ax_2d.get_ylim()
                    y_range = cur_ylim[1] - cur_ylim[0]
                    if (abs(new_ylim[0] - cur_ylim[0]) > 0.05 * y_range or
                            abs(new_ylim[1] - cur_ylim[1]) > 0.05 * y_range):
                        self.ax_2d.set_ylim(new_ylim)
                self.canvas_2d.draw_idle()
                self._last_plotted_sample_indices[dt] = current_last_idx

        # PPG 设备：右侧 6 子图（EEG fp1/fp2 + PPG + SpO2）按 BIO_PLOT_CONFIG 逐条刷新；
        # 下方 EEG/ECG/BRTH/EMG 分支在 ppg 模式下线条均为空，自动跳过
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
                        # Reassemble the circular buffer into chronological order for plotting.
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
                                    f"{title}\n{current_impedance:.2f} KΩ",
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

                # EEG/PPG 行 FFT：重排后的快照按行（各行采样率可不同）提交
                # 工作线程，结果经信号异步回填
                now_fft = time.time()
                if (not self._bio_fft_pending
                        and now_fft - self._bio_fft_last_submit >= FFT_UPDATE_INTERVAL):
                    row_specs = []
                    for plot_idx, (dt, ch_idx, _title, _color) in enumerate(BIO_PLOT_CONFIG):
                        if dt not in (DataType.NTF_EEG, DataType.NTF_PPG):
                            continue
                        buf_copy = bio_buffers_copy.get(dt)
                        if buf_copy is None or ch_idx >= buf_copy.shape[0]:
                            continue
                        if (plot_idx >= len(self.bio_fft_lines)
                                or self.bio_fft_lines[plot_idx] is None):
                            continue
                        sr = state.bio_sample_rates.get(dt) or state.nominal_rates.get(dt) or 0
                        if sr <= 0:
                            continue
                        row_specs.append((plot_idx, buf_copy[ch_idx], sr))
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
        state.eeg_buffer_lock.lock()
        state.ecg_buffer_lock.lock()
        state.brth_buffer_lock.lock()
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
        finally:
            state.brth_buffer_lock.unlock()
            state.ecg_buffer_lock.unlock()
            state.eeg_buffer_lock.unlock()

        brth_axis_index = len(self.axes_eeg) - 1 if state.has_brth else None
        ecg_axis_index = len(self.axes_eeg) - 1 - int(state.has_brth) if state.has_ecg else None
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
                            f"EEG-{eeg_ch + 1}\n{current_impedance:.2f} KΩ",
                            fontsize=8, color=color, rotation=0,
                            va='center', ha='left', labelpad=10
                        )
                        ax.yaxis.set_label_position("right")

                # 每行 FFT（EEG 本页通道 + ECG 行）：重排后的快照提交工作线程，
                # 结果经信号异步回填
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
                        f"ECG\n{current_impedance:.2f} KΩ",
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
                            f"EMG-{ch + 1}\n{current_impedance:.2f} KΩ",
                            fontsize=8, color=color, rotation=0,
                            va='center', ha='left', labelpad=10
                        )
                        ax.yaxis.set_label_position("right")

                # 每通道 FFT：显示通道重排后的快照提交工作线程，结果经信号异步回填
                now_fft = time.time()
                if (not self._bio_fft_pending
                        and now_fft - self._bio_fft_last_submit >= FFT_UPDATE_INTERVAL):
                    sr = state.emg_sample_rate or state.nominal_rates.get(DataType.NTF_EMG) or 0
                    if sr > 0 and self.emg_lines:
                        snapshot = np.roll(
                            emg_buffer_copy[:len(self.emg_lines)],
                            -emg_buffer_index, axis=1)
                        row_specs = [(row, snapshot[row], sr) for row in range(snapshot.shape[0])]
                        self._bio_fft_last_submit = now_fft
                        self._submit_bio_fft(DataType.NTF_EMG, row_specs)

                self.canvas_eeg.draw_idle()
                self._last_plotted_sample_indices[DataType.NTF_EMG] = current_last_idx

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

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _app_log(self, message: str, level: str = "I", sensor=None):
        """把一条应用事件写进 SDK 日志：给定（或当前显示）设备时进其 profile
        日志，否则进 controller 日志——与 SDK 内部日志共用同一时间线。"""
        target = sensor if sensor is not None else self.current_sensor
        if target is not None:
            target.log(message, level)
        else:
            self.sensor_controller.log(message, level)

    def _on_state_changed(self, sensor: SensorProfile, state: DeviceStateEx):
        print(f"[State] {sensor.BLEDevice.Name}: {state}")
        if state == DeviceStateEx.Disconnected:
            self.device_disconnected_sig.emit(sensor.BLEDevice.Address)

    def _on_auto_reconnect(self, sensor: SensorProfile, restore: bool, answer) -> None:
        """onAutoReconnect 回调（SDK 恢复线程）：自动重连找回设备时，
        等效于按下 Connect 按钮——转到 UI 线程执行完整连接流程。
        restore=True 时流程结束后回放上次会话的参数（保留和恢复原有设置）。
        异步应答 answer(True) 表示由本流程接管（SDK 不再执行默认的参数回放恢复）。"""
        sensor.log(f"App: auto reconnect callback received, restore={restore}")
        self.auto_reconnect_sig.emit(sensor.BLEDevice.Address, restore)
        answer(True)

    def _press_connect_for_address(self, addr: str, restore: bool = True):
        """在设备列表中选中该地址，并执行与按下 Connect 按钮完全相同的流程。
        restore=True 时，流程结束后把上次会话的 setParam 参数回放一遍。"""
        # 流程开始先快照上次会话参数（排除 demo 自管的 DEBUG 日志参数）
        sensor = self.sensor_controller.getSensor(addr)
        saved = {}
        if restore and sensor is not None:
            saved = {k: v for k, v in (sensor._saved_params or {}).items()
                     if k not in ("DEBUG_LOG_PATH", "DEBUG_BLE_DATA_PATH")}
        for i in range(self.device_list.count()):
            item = self.device_list.item(i)
            if f"Address: {addr}" in item.text():
                self.device_list.setCurrentItem(item)
                break
        self._connect_selected_device()
        # 回放上次会话参数，恢复原有设置
        if saved and addr in self.device_states:
            sensor = self.device_states[addr].sensor
            for key, value in saved.items():
                result = sensor.setParam(key, value)
                print(f"[AutoReconnect] restore setParam({key}, {value}) -> {result}")
                sensor.log(f"App: restore setParam({key}, {value}) -> {result}")
            # 参数回放后同步复选框显示
            self._refresh_control_states(sensor)

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
        if not data.channelSamples:
            return
        samples = data.channelSamples[0]
        if not samples:
            return
        sample = samples[-1]
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
        # SDK 回调线程：只做日志与信号转发，控件更新交给 GUI 线程（Qt 控件只允许在 GUI 线程访问）
        print(f"[Power] {sensor.BLEDevice.Name}: {power}%")
        self.power_changed_sig.emit(sensor, power)

    def _on_device_info_update(self, sensor: SensorProfile, info: DeviceInfo):
        print(f"[Link] {sensor.BLEDevice.Name}: "
              f"interval={info.ConnectionIntervalMs}ms latency={info.PeripheralLatency} "
              f"timeout={info.SupervisionTimeoutMs}ms mtu={info.MTUSize}")
        self.device_info_sig.emit(sensor, info)

    @staticmethod
    def _link_text(info: Optional[DeviceInfo]) -> str:
        if info is None or info.PeripheralLatency < 0 or info.ConnectionIntervalMs <= 0:
            return "Link: --"
        return (f"Link: {info.ConnectionIntervalMs}ms / "
                f"latency {info.PeripheralLatency} / "
                f"timeout {info.SupervisionTimeoutMs}ms")

    @staticmethod
    def _mtu_text(info: Optional[DeviceInfo]) -> str:
        if info is None or info.MTUSize <= 0:
            return "MTU: --"
        return f"MTU: {info.MTUSize}"

    def _update_link_info_display(self, sensor: SensorProfile, info: Optional[DeviceInfo] = None):
        # info 由 onDeviceInfoUpdate 携带（回放 profile 未 init，getDeviceInfo() 返回 None）
        if info is None:
            info = sensor.getDeviceInfo()
        state = self.device_states.get(sensor.BLEDevice.Address)
        if state is not None and info is not None and state.status_parts:
            rate_map = {DataType.NTF_EEG: info.EegSampleRate, DataType.NTF_ECG: info.EcgSampleRate}
            state.status_parts = [
                (label, ch, rate_map.get(dt) or sr, dt)
                for label, ch, sr, dt in state.status_parts
            ]
        # 采样率变更后重建生物电/IMU 缓冲（横轴时间窗才能与新速率一致），当前显示
        # 设备还需重建图表线的 x 数据；非当前设备只重建缓冲，切换过去时会整体重画
        if state is not None and info is not None:
            if state.sync_bio_sample_rates(info) and self.current_sensor == sensor:
                self._rebuild_eeg_plot()
            imu_changed = state.sync_imu_sample_rates(info)
            if (imu_changed and self.current_sensor == sensor
                    and self.active_data_type in imu_changed):
                self._rebuild_2d_plot()
            # 单选框选中态跟随当前速率（回放等不经 _refresh_control_states 的路径）
            if info.EegSampleRate > 0:
                rate = int(info.EegSampleRate)
                options, cur = state.sample_rate_state
                if cur != rate:
                    state.sample_rate_state = (options, rate)
                    if self.current_sensor == sensor:
                        self._set_sample_rate_checked(rate)
        if self.current_sensor == sensor:
            self.link_label.setText(self._link_text(info))
            self.mtu_label.setText(self._mtu_text(info))
            if state is not None and state.status_parts:
                self.status_label.setText(state.build_status_text())
                self.rate_label.setText(state.build_rate_text())

    def _update_power_display(self, sensor: SensorProfile, power: int):
        state = self.device_states.get(sensor.BLEDevice.Address)
        if state is not None:
            state.last_power = power
        if self.current_sensor == sensor:
            self.power_label.setText(f"Power: {power}%")

    def _check_set_param_result(self, key: str, result: str) -> bool:
        """检查 setParam 结果，若报错则弹出 QMessageBox。返回 True 表示成功。"""
        if str(result).startswith("Error"):
            QtWidgets.QMessageBox.warning(self, "Set Parameter Failed", f"Failed to set {key}:\n{result}")
            return False
        return True

    def _apply_sdk_debug_log(self):
        # 开启时把日志目录设为 sensorsdklog 下以「当前时间戳_SDK版本」命名的
        # 子目录：本次会话的 controller log、各设备 profile log 与 bin 导出
        # 都归到该子目录（须在 setDebugEnabled(True) 之前设置）
        log_dir = os.path.join(
            str(Path.home() / "Documents" / "sensorsdklog"),
            f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{self.sensor_controller.getVersion().replace('.', '_')}",
        )
        self.sensor_controller.setLogPath(True, log_dir)
        print(f"[Debug Log] setLogPath -> {log_dir}")
        self.sensor_controller.setDebugEnabled(True)

    def _on_debug_log_toggled(self, state: int):
        enabled = (state == QtCore.Qt.Checked)
        self._app_log(f"User: SDK debug log {'ON' if enabled else 'OFF'}")
        self._debug_log_enabled = enabled
        if enabled:
            self._apply_sdk_debug_log()
        else:
            self.sensor_controller.setDebugEnabled(False)
        value = "True" if enabled else "False"
        for sensor in self.sensor_controller.getConnectedSensors():
            if sensor.isReady and sensor.hasInited:
                result = sensor.setParam("DEBUG_LOG_PATH", value)
                print(f"[Debug Log] setParam({sensor.BLEDevice.Address}, DEBUG_LOG_PATH, {value}) -> {result}")
                sensor.log(f"App: setParam(DEBUG_LOG_PATH, {value}) -> {result}")
                self._check_set_param_result("DEBUG_LOG_PATH", result)

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
        if self._check_set_param_result(key, result):
            self._refresh_control_states(self.current_sensor)
            self._clear_ui_data()

    def _refresh_control_states(self, sensor: SensorProfile):
        """查询设备当前 NTF/FILTER 参数并缓存到对应设备状态；若为当前显示设备则同步刷新 UI。"""
        info = sensor.getDeviceInfo()
        channel_map = {
            "NTF_EMG":   info.EmgChannelCount if info else 0,
            "NTF_GEST":  info.EmgChannelCount if info else 0,
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

        state = self.device_states.get(sensor.BLEDevice.Address)
        if state is not None:
            state.ntf_states = ntf_states
            state.filter_states = filter_states
            state.sample_rate_state = sample_rate_state

        if self.current_sensor == sensor:
            self._apply_control_states(ntf_states, filter_states, sample_rate_state)

    def _apply_control_states(self, ntf_states: dict, filter_states: dict, sample_rate_state: tuple = ([], 0)):
        """把缓存的 NTF/FILTER 状态应用到 UI 复选框（不触发 setParam）；设备不支持的 NTF 选项直接隐藏。"""
        self._updating_ntf_controls = True
        try:
            for key, cb in self._ntf_checkboxes.items():
                enabled, checked = ntf_states.get(key, (False, False))
                # 无状态信息（未选设备/查询失败）时保持全部可见，仅置灰
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
        self._updating_sample_rate_controls = True
        try:
            if current_rate not in self._sample_rate_radios:
                self._sample_rate_button_group.setExclusive(False)
            for rate, rb in self._sample_rate_radios.items():
                rb.setEnabled(rate in options)
                rb.setChecked(rate == current_rate)
            self._sample_rate_button_group.setExclusive(True)
        finally:
            self._updating_sample_rate_controls = False

    def _set_sample_rate_checked(self, rate: int):
        """仅更新采样率单选框选中态（不触碰启用状态、不触发 setParam）。"""
        self._updating_sample_rate_controls = True
        try:
            if rate not in self._sample_rate_radios:
                self._sample_rate_button_group.setExclusive(False)
            for r, rb in self._sample_rate_radios.items():
                rb.setChecked(r == rate)
            self._sample_rate_button_group.setExclusive(True)
        finally:
            self._updating_sample_rate_controls = False

    def _on_filter_combo_changed(self, _index: int):
        """Live Filter 频段下拉框切换：更新选中项；各设备的回调线程滤波路径
        在下个数据批懒同步（set_live_filter_band 会重置滤波器状态）。"""
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
        value = str(rate)
        print(f"[Sample Rate] setParam(EEG_SAMPLE_RATE, {value}) ...")
        result = self.current_sensor.setParam("EEG_SAMPLE_RATE", value)
        print(f"[Sample Rate] setParam(EEG_SAMPLE_RATE, {value}) -> {result}")
        self._app_log(f"User: setParam(EEG_SAMPLE_RATE, {value}) -> {result}")
        self._check_set_param_result("EEG_SAMPLE_RATE", result)
        self._refresh_control_states(self.current_sensor)
        if not str(result).startswith("Error"):
            self._clear_ui_data()

    def _refresh_display_for_state(self, state: Optional[DeviceDataState]):
        """切换显示设备时，刷新设备信息、丢包统计、手势、开关状态与图表。"""
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
        self._apply_control_states(ntf_states, filter_states, sample_rate_state)

        self._rebuild_2d_plot()
        self._rebuild_eeg_plot()

    def _clear_ui_data(self):
        """清除当前显示设备的数据缓冲区并重建图表，等待新数据。"""
        self._last_plotted_sample_indices.clear()

        state = self._current_state()
        if state is not None:
            state.clear_buffers()

        self._rebuild_2d_plot()
        self._rebuild_eeg_plot()


if __name__ == "__main__":
    # PyInstaller 打包后，multiprocessing 子进程必须调用 freeze_support()
    multiprocessing.freeze_support()
    app = QtWidgets.QApplication(sys.argv)
    window = IMUQuaternionEMGEEGDemo()

    def _sigint(sig, frame):
        app.quit()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)
    app.aboutToQuit.connect(lambda: window.sensor_controller.terminate())
    sys.exit(app.exec_())
