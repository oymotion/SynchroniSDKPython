import sys
import signal
import time
import subprocess
import multiprocessing
import os
import threading
from pathlib import Path
from typing import List, Optional

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QRunnable, QThreadPool

from sensor import *
import sensor


SCAN_DEVICE_PERIOD_IN_MS   = 3000
PACKAGE_COUNT              = 32
POWER_REFRESH_PERIOD_IN_MS = 60000
PLOT_UPDATE_INTERVAL       = 50
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

EEG_CHANNEL_COLORS = plt.cm.tab10(np.linspace(0, 1, 8))

FIXED_Y_RANGES = {
    DataType.NTF_ACC: (-8, 8),
    DataType.NTF_GYRO: (-2000, 2000),
    DataType.NTF_EULER_DATA: (-180, 180),
    DataType.NTF_QUATERNION: (-1, 1),
}


class DataTask(QRunnable):
    def __init__(self, callback, data):
        super().__init__()
        self._cb = callback
        self._data = data

    def run(self):
        try:
            self._cb(self._data)
        except Exception as e:
            print(f"[DataTask] Exception: {e}")


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

    def note_data_received(self, data: SensorData):
        """统计每种数据类型实际收到的样本数（不含丢包占位样本），
        并记录数据批携带的标称采样率/通道数。"""
        if not data.channelSamples:
            return
        n = sum(1 for s in data.channelSamples[0] if not getattr(s, "isLost", False))
        with self.rate_lock:
            if n > 0:
                self.rate_counts[data.dataType] = self.rate_counts.get(data.dataType, 0) + n
            if data.sampleRate and data.sampleRate > 0:
                self.nominal_rates[data.dataType] = data.sampleRate
            ch = len(data.channelSamples)
            if ch > 0:
                self.nominal_channels[data.dataType] = ch

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

        extra_axes = int(self.has_ecg) + int(self.has_brth)
        self.eeg_channels_per_page = eeg_axis_count - extra_axes

    def append_data(self, data: SensorData):
        if data.dataType == DataType.NTF_EEG:
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

        if data.dataType == DataType.NTF_ECG:
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

        if data.dataType == DataType.NTF_BRTH:
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

        lock = self.get_buffer_lock(data.dataType)
        lock.lock()
        try:
            buf = self.buffers.get(data.dataType)
            idx_buf = self.sample_index_buffers.get(data.dataType)
            if buf is None or idx_buf is None:
                return
            buf_len = buf.shape[1]
            n = 0
            buffer_index = self.buffer_indices.get(data.dataType, 0)
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
            self.buffer_indices[data.dataType] = (buffer_index + n) % buf_len
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
        finally:
            self.brth_buffer_lock.unlock()
            self.ecg_buffer_lock.unlock()
            self.eeg_buffer_lock.unlock()


class IMUQuaternionEEGDemo(QtWidgets.QWidget):
    data_received  = QtCore.pyqtSignal(object, object)       # (address, SensorData)
    add_device_sig = QtCore.pyqtSignal(str)
    lost_packet_signal = QtCore.pyqtSignal(str, str, int)    # (address, type_name, count)
    device_disconnected_sig = QtCore.pyqtSignal(str)         # address
    device_disconnected_sig = QtCore.pyqtSignal(str)         # address
    auto_reconnect_sig = QtCore.pyqtSignal(str, bool)        # (address, restore)
    replay_done_sig = QtCore.pyqtSignal(str)
    analyze_done_sig = QtCore.pyqtSignal(str, str)
    dongle_check_sig = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.discovered_devices = []
        self.current_sensor: SensorProfile = None   # 当前在列表中选中、正在显示的设备
        self.device_states: dict = {}               # Address -> DeviceDataState（已连接设备）
        self.sensor_controller = SensorController()
        self.thread_pool = QThreadPool.globalInstance()
        self._data_type_pools = {}                  # (Address, DataType) -> QThreadPool

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
        self._eeg_display_channels = 0

        self._updating_ntf_controls = False
        self._updating_filter_controls = False
        self._debug_log_checkbox = None
        self._data_debug_log_checkbox = None
        self._ntf_checkboxes: dict = {}
        self._filter_checkboxes: dict = {}
        self._debug_log_enabled = True
        self._data_debug_log_enabled = True
        # 每设备上次会话的日志/bin 导出路径：重连时优先续用上一条，而不是另起新文件
        self._last_log_paths: dict = {}
        self._last_data_log_paths: dict = {}
        self._replay_thread = None
        self._replay_sensor = None
        self._replay_paused = False
        self._replay_stop_requested = False

        self._init_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(PLOT_UPDATE_INTERVAL)
        self._rate_last_refresh = 0.0   # 实测采样率每秒刷新的时间戳

        self.add_device_sig.connect(self._add_device_item)
        self.data_received.connect(self._dispatch_data, type=QtCore.Qt.DirectConnection)
        self.lost_packet_signal.connect(self._update_lost_packet_display)
        self.device_disconnected_sig.connect(self._on_device_disconnected)
        self.auto_reconnect_sig.connect(self._press_connect_for_address)
        self.replay_done_sig.connect(self._on_replay_done)
        self.analyze_done_sig.connect(self._on_analyze_done)
        self.dongle_check_sig.connect(self._on_dongle_check_result)

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
        self.figure_2d, self.ax_2d = plt.subplots()
        self.canvas_2d = FigureCanvas(self.figure_2d)
        bottom_left_layout.addWidget(QtWidgets.QLabel("2D Waveform (ACC/GYRO/Euler)"))
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
        device_header_layout.addWidget(QtWidgets.QLabel("Discovered Devices:"))
        device_header_layout.addStretch()
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
        type_layout.addStretch()

        self.value_labels: dict = {}
        self.value_box = QtWidgets.QGroupBox("Real-time Values")
        self.value_layout = QtWidgets.QHBoxLayout()
        self.value_box.setLayout(self.value_layout)

        self.lost_packet_label = QtWidgets.QLabel("Packet Loss Stats: None")
        self.lost_packet_label.setWordWrap(True)
        self.lost_packet_box = QtWidgets.QGroupBox("Packet Loss Stats")
        lost_packet_layout = QtWidgets.QVBoxLayout()
        lost_packet_layout.addWidget(self.lost_packet_label)
        self.lost_packet_box.setLayout(lost_packet_layout)

        status_layout = QtWidgets.QHBoxLayout()
        status_layout.addWidget(self.value_box, stretch=1)
        status_layout.addWidget(self.lost_packet_box, stretch=1)

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
        self.power_label = QtWidgets.QLabel("Power: --%")
        device_info_layout.addWidget(self.model_label)
        device_info_layout.addWidget(self.hw_version_label)
        device_info_layout.addWidget(self.fw_version_label)
        device_info_layout.addWidget(self.power_label)
        device_info_layout.addStretch()
        controls_layout.addLayout(device_info_layout)

        debug_log_group = QtWidgets.QGroupBox("Debug Log")
        debug_log_layout = QtWidgets.QHBoxLayout()
        self._debug_log_checkbox = QtWidgets.QCheckBox("Enable SDK Debug Log")
        self._debug_log_checkbox.setChecked(True)
        self._debug_log_checkbox.stateChanged.connect(self._on_debug_log_toggled)
        debug_log_layout.addWidget(self._debug_log_checkbox)
        self._data_debug_log_checkbox = QtWidgets.QCheckBox("Enable Data Debug Log")
        self._data_debug_log_checkbox.setChecked(True)
        self._data_debug_log_checkbox.stateChanged.connect(self._on_data_debug_log_toggled)
        debug_log_layout.addWidget(self._data_debug_log_checkbox)
        debug_log_group.setLayout(debug_log_layout)

        ntf_group = QtWidgets.QGroupBox("Data Notification")
        ntf_layout = QtWidgets.QHBoxLayout()
        self._ntf_checkboxes = {
            "NTF_EEG":  QtWidgets.QCheckBox("EEG"),
            "NTF_ECG":  QtWidgets.QCheckBox("ECG"),
            "NTF_BRTH": QtWidgets.QCheckBox("BRTH"),
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

        options_layout = QtWidgets.QHBoxLayout()
        options_layout.addWidget(debug_log_group, stretch=1)
        options_layout.addWidget(ntf_group, stretch=1)
        options_layout.addWidget(filter_group, stretch=1)
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

        self.figure_eeg, self.axes_eeg = plt.subplots(8, 1, sharex=True, figsize=(8, 12))
        self.figure_eeg.subplots_adjust(left=0.05, right=0.9, hspace=0.4)
        self.canvas_eeg = FigureCanvas(self.figure_eeg)
        eeg_layout.addWidget(QtWidgets.QLabel("EEG + ECG + BRTH Waveform"))
        eeg_layout.addWidget(self.canvas_eeg)

        right_layout.addLayout(controls_layout, stretch=1)
        right_layout.addLayout(eeg_layout, stretch=4)

        main_layout.addLayout(left_layout, stretch=3)
        main_layout.addLayout(right_layout, stretch=7)
        self.setLayout(main_layout)
        self.setWindowTitle(f"SynchroniSDKPython IMU + Quaternion + EEG + ECG + BRTH Demo (sensor-sdk v{sensor.__version__})")
        self.resize(1600, 900)
        self.show()

    # ── Scan / Connect ────────────────────────────────────────────────────────

    def _start_scan(self):
        if not self.sensor_controller.isEnable:
            self.status_label.setText("Please enable Bluetooth first")
            return
        if not self.sensor_controller.isScanning:
            self.sensor_controller.startScan(SCAN_DEVICE_PERIOD_IN_MS)
        self.btn_scan.setEnabled(False)
        self.btn_stop_scan.setEnabled(True)

    def _stop_scan(self):
        self.sensor_controller.stopScan()
        self.btn_scan.setEnabled(True)
        self.btn_stop_scan.setEnabled(False)

    def _check_setup_dongle(self):
        # 检查/安装 dongle 驱动（Windows 弹 UAC，Linux 需终端 sudo 密码），
        # 后台线程执行避免阻塞 UI，结果经信号回到主线程提示
        self.btn_check_dongle.setEnabled(False)
        self.btn_check_dongle.setText("Checking Dongle...")

        def work():
            try:
                result = self.sensor_controller.checkSetupDongle()
            except Exception as e:
                result = f"Error: {e}"
            self.dongle_check_sig.emit(result)

        threading.Thread(target=work, daemon=True).start()

    def _on_dongle_check_result(self, result: str):
        self.btn_check_dongle.setEnabled(True)
        self.btn_check_dongle.setText("Check Setup Dongle")
        if result == "OK":
            QtWidgets.QMessageBox.information(
                self, "Check Setup Dongle",
                "USB BLE dongle is ready (driver installed and usable by the SDK).")
        else:
            QtWidgets.QMessageBox.warning(self, "Check Setup Dongle", result)

    def _on_device_found(self, device_list: List[BLEDevice]):
        self._stop_scan()
        filtered = filter(lambda x: x.Name.startswith("OB") or x.Name.startswith("Sync") or x.Name.startswith("Orion"), device_list)
        for d in filtered:
            if d.Address not in [x.Address for x in self.discovered_devices]:
                self.discovered_devices.append(d)
                self.add_device_sig.emit(f"RSSI: {d.RSSI}, Name: {d.Name}, Address: {d.Address}")

    def _add_device_item(self, text: str):
        self.device_list.addItem(text)

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
            self.status_label.setText("Please select a device in the list first")
            return
        addr = device.Address
        if addr in self.device_states:
            return

        sensor = self.sensor_controller.requireSensor(device)
        if sensor is None:
            self.status_label.setText("Failed to create SensorProfile")
            return

        sensor.onDataCallback  = self._on_data
        sensor.onStateChanged  = self._on_state_changed
        sensor.onErrorCallback = self._on_error
        sensor.onPowerChanged  = self._on_power_changed
        # 自动重连找回设备时：等效于按下 Connect 按钮（走本方法完整流程）
        sensor.onAutoReconnect = self._on_auto_reconnect

        self.status_label.setText(f"Connecting: {device.Name} ...")
        self.btn_connect.setEnabled(False)

        if sensor.deviceState != DeviceStateEx.Ready:
            if not sensor.connect():
                self.status_label.setText(f"Failed to connect to {device.Name}")
                self._update_button_states()
                return

        state = DeviceDataState(sensor)

        if not sensor.hasInited:
            if not sensor.init(PACKAGE_COUNT, POWER_REFRESH_PERIOD_IN_MS):
                self.status_label.setText(f"Failed to initialize {device.Name}")
                self._update_button_states()
                return

            info = sensor.getDeviceInfo()
            state.info = info
            state.init_buffers(info, len(self.axes_eeg))
            state.status_parts = [
                ("ACC",   info.AccChannelCount,   info.AccSampleRate,   DataType.NTF_ACC),
                ("Euler", info.EulerChannelCount, info.EulerSampleRate, DataType.NTF_EULER_DATA),
                ("Quat",  info.QuatChannelCount,  info.QuatSampleRate,  DataType.NTF_QUATERNION),
                ("EEG",   info.EegChannelCount,   info.EegSampleRate,   DataType.NTF_EEG),
                ("ECG",   info.EcgChannelCount,   info.EcgSampleRate,   DataType.NTF_ECG),
                ("BRTH",  info.BrthChannelCount,  info.BrthSampleRate,  DataType.NTF_BRTH),
            ]
            state.status_text = state.build_status_text()

        if not sensor.isDataTransfering:
            if not sensor.startDataNotification():
                self.status_label.setText("Failed to start data stream")
                self._update_button_states()
                return

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
        else:
            self._update_button_states()
        self.btn_scan.setEnabled(not replaying)
        self.device_list.setEnabled(not replaying)
        self._debug_log_checkbox.setEnabled(not replaying)
        self._data_debug_log_checkbox.setEnabled(not replaying)

    def _replay_bin_file(self):
        """选择一个 bin 文件并按原始时间节奏回放，数据显示流程与实时数据一致。"""
        if self.device_states:
            self.status_label.setText("Please disconnect all devices before replaying a bin file")
            return
        if self._replay_thread is not None and self._replay_thread.is_alive():
            return

        default_dir = Path.home() / "Documents" / "sensorsdklog"
        start_dir = str(default_dir) if default_dir.exists() else str(Path.home())
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Bin File", start_dir, "Bin Files (*.bin)"
        )
        if not path:
            return

        config = self.sensor_controller.getBinFileInfo(path)
        if config is None:
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

        self._replay_sensor = sensor

        # 回放传感器作为一台虚拟设备进入 device_states：
        # _on_data 按地址路由到它的 DeviceDataState，显示流程与实时设备一致
        state = DeviceDataState(sensor)
        state.info = info
        state.init_buffers(info, len(self.axes_eeg))
        duration = config.get("replay_duration", 0.0)
        version = config.get("version", "?")
        state.status_text = (
            f"Replaying: {Path(path).name} (config v{version}, duration {duration:.1f}s, realtime) ...")
        self.device_states[mac] = state
        self.current_sensor = sensor
        self._refresh_display_for_state(state)

        self._replay_paused = False
        self._replay_stop_requested = False
        self.btn_replay.setEnabled(False)
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

    def _toggle_replay_pause(self):
        """暂停/恢复当前回放。"""
        sensor = self._replay_sensor
        if sensor is None:
            return
        if self._replay_paused:
            result = self.sensor_controller.resumeBinReplay(sensor)
        else:
            result = self.sensor_controller.pauseBinReplay(sensor)
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
        """停止当前回放。"""
        sensor = self._replay_sensor
        if sensor is None:
            return
        self._replay_stop_requested = True
        self.btn_replay_stop.setEnabled(False)
        self.btn_replay_pause.setEnabled(False)
        result = self.sensor_controller.stopBinReplay(sensor)
        if result != "OK":
            self.status_label.setText(f"Stop replay failed: {result}")
            return
        self.status_label.setText("Stopping replay ...")

    def _on_replay_done(self, message: str):
        # 回放结束：移除回放用的虚拟设备状态，恢复实时设备控件
        sensor = self._replay_sensor
        if sensor is not None:
            self.device_states.pop(sensor.BLEDevice.Address, None)
            if self.current_sensor is sensor:
                self.current_sensor = None
        self._refresh_display_for_state(None)
        self.status_label.setText(message)
        self.btn_replay.setEnabled(True)
        self.btn_replay_pause.setEnabled(False)
        self.btn_replay_pause.setText("Pause Replay")
        self.btn_replay_stop.setEnabled(False)
        self._set_replay_mode_ui(False)
        self._replay_paused = False
        self._replay_stop_requested = False

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
            self.status_label.setText(f"Analyze failed: {error}")
            return
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

    def _on_data(self, sensor: SensorProfile, data: SensorData):
        if not (data and data.channelSamples):
            return
        addr = sensor.BLEDevice.Address
        state = self.device_states.get(addr)
        if state is None:
            return
        # 实际采样率收集：覆盖所有收到的数据类型
        state.note_data_received(data)
        if data.dataType in state.buffers or data.dataType in (DataType.NTF_EEG, DataType.NTF_ECG, DataType.NTF_BRTH):
            self.data_received.emit(addr, data)
        if data.dataType == DataType.NTF_QUATERNION:
            self._update_quaternion(state, data)

    def _get_data_type_pool(self, key):
        pool = self._data_type_pools.get(key)
        if pool is None:
            pool = QThreadPool()
            pool.setMaxThreadCount(1)
            self._data_type_pools[key] = pool
        return pool

    def _dispatch_data(self, addr: str, data: SensorData):
        state = self.device_states.get(addr)
        if state is None:
            return
        if data.lostPackageCount > 0:
            type_name = DataType(data.dataType).name if data.dataType is not None else "Unknown"
            self.lost_packet_signal.emit(addr, type_name, data.lostPackageCount)

        task = DataTask(state.append_data, data)
        pool = self._get_data_type_pool((addr, data.dataType))
        pool.start(task)

    def closeEvent(self, event):
        for pool in self._data_type_pools.values():
            pool.waitForDone()
        try:
            self.sensor_controller.terminate()
        except Exception as e:
            print(f"[closeEvent] terminate error: {e}")
        event.accept()

    # ── Bottom-left 2D Waveform ───────────────────────────────────────────────

    def _on_type_changed(self, _):
        self.active_data_type = self.type_combo.currentData()
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
        self.canvas_2d.draw_idle()

    # ── Right-side EEG + ECG Waveform ─────────────────────────────────────────

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
            self._rebuild_eeg_plot()
            self._update_page_label()
            self._update_page_buttons()

    def _rebuild_eeg_plot(self):
        state = self._current_state()
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
            for ax in self.axes_eeg:
                ax.cla()
                ax.set_visible(True)
            suffix = "(Not connected)" if state is None else "(Device not supported or disabled)"
            self.axes_eeg[0].set_title(f"EEG + ECG + BRTH {suffix}")
            self.canvas_eeg.draw_idle()
            self._last_plotted_sample_indices.pop(DataType.NTF_EEG, None)
            self._last_plotted_sample_indices.pop(DataType.NTF_ECG, None)
            self._last_plotted_sample_indices.pop(DataType.NTF_BRTH, None)
            self.eeg_lines = []
            self.ecg_line = None
            self.brth_line = None
            self._eeg_display_channels = 0
            self._update_page_label()
            self._update_page_buttons()
            return

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

        brth_axis_index = len(self.axes_eeg) - 1 if brth_available else None
        ecg_axis_index = len(self.axes_eeg) - 1 - int(brth_available) if ecg_available else None

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
            if data.dataType == DataType.NTF_QUATERNION:
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

    def _on_state_changed(self, sensor: SensorProfile, state: DeviceStateEx):
        print(f"[State] {sensor.BLEDevice.Name}: {state}")
        if state == DeviceStateEx.Disconnected:
            self.device_disconnected_sig.emit(sensor.BLEDevice.Address)

    def _on_auto_reconnect(self, sensor: SensorProfile, restore: bool) -> bool:
        """onAutoReconnect 回调（SDK 恢复线程）：自动重连找回设备时，
        等效于按下 Connect 按钮——转到 UI 线程执行完整连接流程。
        restore=True 时流程结束后回放上次会话的参数（保留和恢复原有设置）。
        返回 True 表示由本流程接管（SDK 不再执行默认的参数回放恢复）。"""
        self.auto_reconnect_sig.emit(sensor.BLEDevice.Address, restore)
        return True

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
            # 参数回放后同步复选框显示
            self._refresh_control_states(sensor)

    def _on_device_disconnected(self, addr: str):
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

    def _update_lost_packet_display(self, addr: str, lost_type: str, count: int):
        state = self.device_states.get(addr)
        if state is None:
            return
        state.lost_counts[lost_type] = count
        if self.current_sensor is not None and self.current_sensor.BLEDevice.Address == addr:
            text = "  ".join(f"{k}: {v}" for k, v in sorted(state.lost_counts.items()))
            self.lost_packet_label.setText("Packet Loss Stats: " + text)

    def _on_power_changed(self, sensor: SensorProfile, power: int):
        print(f"[Power] {sensor.BLEDevice.Name}: {power}%")
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

    def _on_debug_log_toggled(self, state: int):
        enabled = (state == QtCore.Qt.Checked)
        self._debug_log_enabled = enabled
        self.sensor_controller.setDebugEnabled(enabled)
        value = "True" if enabled else "False"
        for sensor in self.sensor_controller.getConnectedSensors():
            if sensor.deviceState == DeviceStateEx.Ready and sensor.hasInited:
                result = sensor.setParam("DEBUG_LOG_PATH", value)
                print(f"[Debug Log] setParam({sensor.BLEDevice.Address}, DEBUG_LOG_PATH, {value}) -> {result}")
                self._check_set_param_result("DEBUG_LOG_PATH", result)

    def _on_data_debug_log_toggled(self, state: int):
        enabled = (state == QtCore.Qt.Checked)
        self._data_debug_log_enabled = enabled
        value = "True" if enabled else "False"
        for sensor in self.sensor_controller.getConnectedSensors():
            if sensor.deviceState == DeviceStateEx.Ready and sensor.hasInited:
                result = sensor.setParam("DEBUG_BLE_DATA_PATH", value)
                print(f"[Data Debug Log] setParam({sensor.BLEDevice.Address}, DEBUG_BLE_DATA_PATH, {value}) -> {result}")
                self._check_set_param_result("DEBUG_BLE_DATA_PATH", result)

    def _on_ntf_toggled(self, key: str):
        if self.current_sensor is None or self.current_sensor.deviceState != DeviceStateEx.Ready:
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

        state = self.device_states.get(sensor.BLEDevice.Address)
        if state is not None:
            state.ntf_states = ntf_states
            state.filter_states = filter_states

        if self.current_sensor == sensor:
            self._apply_control_states(ntf_states, filter_states)

    def _apply_control_states(self, ntf_states: dict, filter_states: dict):
        """把缓存的 NTF/FILTER 状态应用到 UI 复选框（不触发 setParam）。"""
        self._updating_ntf_controls = True
        try:
            for key, cb in self._ntf_checkboxes.items():
                enabled, checked = ntf_states.get(key, (False, False))
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

    def _on_filter_toggled(self, key: str):
        if self.current_sensor is None or self.current_sensor.deviceState != DeviceStateEx.Ready:
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
        if self._check_set_param_result(key, result):
            self._refresh_control_states(self.current_sensor)
            self._clear_ui_data()

    def _refresh_display_for_state(self, state: Optional[DeviceDataState]):
        """切换显示设备时，刷新设备信息、丢包统计、开关状态与图表。"""
        self._last_plotted_sample_indices.clear()
        self._last_drawn_quaternion = None

        if state is not None and state.info is not None:
            info = state.info
            self.model_label.setText(f"Model: {info.ModelName}")
            self.hw_version_label.setText(f"HW Version: {info.HardwareVersion}")
            self.fw_version_label.setText(f"FW Version: {info.FirmwareVersion}")
        else:
            self.model_label.setText("Model: --")
            self.hw_version_label.setText("HW Version: --")
            self.fw_version_label.setText("FW Version: --")

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

        ntf_states = state.ntf_states if state is not None else {}
        filter_states = state.filter_states if state is not None else {}
        self._apply_control_states(ntf_states, filter_states)

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
    window = IMUQuaternionEEGDemo()

    def _sigint(sig, frame):
        app.quit()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)
    app.aboutToQuit.connect(lambda: window.sensor_controller.terminate())
    sys.exit(app.exec_())
