import matplotlib.pyplot as plt

# plt.rcParams["font.family"] = "SimHei"
# plt.rcParams["axes.unicode_minus"] = False

import sys
import signal
import time
from typing import List
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QRunnable, QThreadPool
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT
import numpy as np


from sensor import *

SCAN_DEVICE_PERIOD_IN_MS = 3000
PACKAGE_COUNT = 32
POWER_REFRESH_PERIOD_IN_MS = 60000
PLOT_UPDATE_INTERVAL = 100


PERIOD_OPTIONS = {"500ms": 0.5, "1s": 1, "5s": 5, "10s": 10, "30s": 30, "60s": 60}


class DataProcessingTask(QRunnable):
    def __init__(self, parent, data):
        super().__init__()
        self.parent = parent
        self.data = data

    def run(self):
        try:
            self.parent.add_data_to_buffer(self.data)
        except Exception as e:
            print(f"Exception occurred in DataProcessingTask: {e}")


class BluetoothDeviceScanner(QtWidgets.QWidget):
    data_received = QtCore.pyqtSignal(object)
    add_device_signal = QtCore.pyqtSignal(str)
    update_plot_signal = QtCore.pyqtSignal()
    lost_packet_signal = QtCore.pyqtSignal(str, int)

    def __init__(self):
        super().__init__()
        self.discovered_devices = []
        self.connected_device = None
        self.SensorControllerInstance = SensorController()
        self.sensor_profiles = {}
        self.current_sensor = None
        self.sampling_rate = 250
        self.period = 1
        self.data_buffer = None
        self.buffer_index = 0
        self.line = None
        self.background = None
        self.thread_pool = QThreadPool.globalInstance()
        self.current_channel = 0
        self.EmgChannelCount = 0
        self.impedance = []
        self.initUI()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(PLOT_UPDATE_INTERVAL)

        self.add_device_signal.connect(self.add_device_to_list)
        self.update_plot_signal.connect(self.update_plot)
        self.data_received.connect(self.start_data_processing)
        self.lost_packet_signal.connect(self.update_lost_packet_display)

        self.lost_packet_counts = {}

        # Throttle plot updates triggered by data callbacks to keep the main thread responsive.
        self._last_plot_signal_time = 0.0
        self._min_plot_signal_interval = 0.05  # 50 ms

        if not self.SensorControllerInstance.hasDeviceFoundCallback:
            self.SensorControllerInstance.onDeviceFoundCallback = self.deviceFoundCallback

        self.init_blitting()

    def initUI(self):
        main_layout = QtWidgets.QHBoxLayout()

        left_layout = QtWidgets.QVBoxLayout()
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        left_layout.addWidget(self.canvas, stretch=18)
        toolbar = NavigationToolbar2QT(self.canvas, self)
        left_layout.addWidget(toolbar, stretch=1)

        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlim(0, self.period)
        self.ax.set_ylim(-1000, 1000)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Amplitude (uV)")
        self.ax.set_title("EMG Waveform (Real-time)")


        self.impedance_label = QtWidgets.QLabel("Impedance: 0 Ω")
        left_layout.addWidget(self.impedance_label, stretch=1)

        self.lost_packet_label = QtWidgets.QLabel("Lost Packet Statistics: None")
        self.lost_packet_label.setWordWrap(True)
        left_layout.addWidget(self.lost_packet_label, stretch=1)

        right_layout = QtWidgets.QVBoxLayout()
        self.scan_button = QtWidgets.QPushButton("Start Scanning Bluetooth Devices")
        self.scan_button.clicked.connect(self.start_scan)
        right_layout.addWidget(self.scan_button)

        self.stop_scan_button = QtWidgets.QPushButton("Stop Scanning Bluetooth Devices")
        self.stop_scan_button.clicked.connect(self.stop_scan)
        self.stop_scan_button.setEnabled(False)
        right_layout.addWidget(self.stop_scan_button)

        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self.disconnect_device)
        self.disconnect_button.setEnabled(False)
        right_layout.addWidget(self.disconnect_button)

        self.device_list = QtWidgets.QListWidget()
        self.device_list.itemClicked.connect(self.connect_device)
        right_layout.addWidget(self.device_list)


        self.period_combobox = QtWidgets.QComboBox()
        for option in PERIOD_OPTIONS.keys():
            self.period_combobox.addItem(option)
        self.period_combobox.setCurrentText("1s")
        self.period_combobox.currentTextChanged.connect(self.change_period)
        right_layout.addWidget(QtWidgets.QLabel("Select Period:"))
        right_layout.addWidget(self.period_combobox)


        self.channel_combobox = QtWidgets.QComboBox()
        self.channel_combobox.currentIndexChanged.connect(self.change_channel)
        right_layout.addWidget(QtWidgets.QLabel("Select Channel:"))
        right_layout.addWidget(self.channel_combobox)


        self.hpf_checkbox = QtWidgets.QCheckBox("HPF")
        self.lpf_checkbox = QtWidgets.QCheckBox("LPF")
        filter_layout1 = QtWidgets.QHBoxLayout()
        filter_layout1.addWidget(self.hpf_checkbox)
        filter_layout1.addWidget(self.lpf_checkbox)

        self.notch_filter_50_checkbox = QtWidgets.QCheckBox("50Hz Notch Filter")
        self.notch_filter_60_checkbox = QtWidgets.QCheckBox("60Hz Notch Filter")
        filter_layout2 = QtWidgets.QHBoxLayout()
        filter_layout2.addWidget(self.notch_filter_50_checkbox)
        filter_layout2.addWidget(self.notch_filter_60_checkbox)

        right_layout.addLayout(filter_layout1)
        right_layout.addLayout(filter_layout2)


        self.hpf_checkbox.stateChanged.connect(self.toggle_hpf)
        self.lpf_checkbox.stateChanged.connect(self.toggle_lpf)
        self.notch_filter_50_checkbox.stateChanged.connect(self.toggle_notch_50)
        self.notch_filter_60_checkbox.stateChanged.connect(self.toggle_notch_60)

        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)
        main_layout.setStretch(0, 7)
        main_layout.setStretch(1, 3)

        self.setLayout(main_layout)
        self.setWindowTitle("SynchroniSDKPython Demo")
        self.resize(1000, 600)
        self.show()

    def toggle_hpf(self, state):
        if self.current_sensor is None:
            print("No device currently connected")
            return
        if state == QtCore.Qt.Checked:
            print("High-pass filter (HPF) enabled")
            self.current_sensor.setParam("FILTER_HPF", "ON")
        else:
            print("High-pass filter (HPF) disabled")
            self.current_sensor.setParam("FILTER_HPF", "OFF")

    def toggle_lpf(self, state):
        if self.current_sensor is None:
            print("No device currently connected")
            return
        if state == QtCore.Qt.Checked:
            print("Low-pass filter (LPF) enabled")
            self.current_sensor.setParam("FILTER_LPF", "ON")
        else:
            print("Low-pass filter (LPF) disabled")
            self.current_sensor.setParam("FILTER_LPF", "OFF")

    def toggle_notch_50(self, state):
        if self.current_sensor is None:
            print("No device currently connected")
            return
        if state == QtCore.Qt.Checked:
            print("50Hz notch filter enabled")
            self.current_sensor.setParam("FILTER_50HZ", "ON")
        else:
            print("50Hz notch filter disabled")
            self.current_sensor.setParam("FILTER_50HZ", "OFF")

    def toggle_notch_60(self, state):
        if self.current_sensor is None:
            print("No device currently connected")
            return
        if state == QtCore.Qt.Checked:
            print("60Hz notch filter enabled")
            self.current_sensor.setParam("FILTER_60HZ", "ON")
        else:
            print("60Hz notch filter disabled")
            self.current_sensor.setParam("FILTER_60HZ", "OFF")

    def start_scan(self):
        try:
            self.device_list.clear()
            self.discovered_devices.clear()
            if not self.SensorControllerInstance.isEnable:
                print("please open bluetooth")
                return
            if not self.SensorControllerInstance.isScanning:
                if not self.SensorControllerInstance.startScan(SCAN_DEVICE_PERIOD_IN_MS):
                    print("please try scan later")
                self.scan_button.setEnabled(False)
                self.stop_scan_button.setEnabled(True)
        except Exception as e:
            print(f"Scan error: {e}")

    def stop_scan(self):
        try:
            self.SensorControllerInstance.stopScan()
            self.scan_button.setEnabled(True)
            self.stop_scan_button.setEnabled(False)
        except Exception as e:
            print(f"Stop scan error: {e}")

    def connect_device(self, item):
        item_text = item.text()
        address_start = item_text.find("Address: ") + len("Address: ")
        device_address = item_text[address_start:].strip()

        target_device = next((d for d in self.discovered_devices if d.Address == device_address), None)

        if target_device:
            try:
                self.current_sensor = self.SensorControllerInstance.requireSensor(target_device)
                if self.current_sensor is None:
                    print("Failed to create SensorProfile")
                    return
                
                if self.connected_device:
                    return

                self.current_sensor.onDataCallback = self.onDataCallback
                self.current_sensor.onPowerChanged = self.onPowerChanged
                self.current_sensor.onStateChanged = self.onStateChanged
                self.current_sensor.onErrorCallback = self.onErrorCallback

                if self.current_sensor.deviceState != DeviceStateEx.Ready:
                    if not self.current_sensor.connect():
                        print("connect device: " + self.current_sensor.BLEDevice.Name + " failed")
                        return

                if not self.current_sensor.hasInited:
                    # result = self.current_sensor.setParam('DEBUG_BLE_DATA_PATH', '/temp/test.csv')
                    if not self.current_sensor.init(PACKAGE_COUNT, POWER_REFRESH_PERIOD_IN_MS):
                        print("init device: " + self.current_sensor.BLEDevice.Name + " failed")
                        return
                    deviceInfo = self.current_sensor.getDeviceInfo()
                    self.sampling_rate = deviceInfo.EmgSampleRate
                    self.EmgChannelCount = deviceInfo.EmgChannelCount
                    self.update_buffer_size()

                    self.channel_combobox.clear()

                    for i in range(self.EmgChannelCount):
                        self.channel_combobox.addItem(f"Channel {i + 1}")
                    self.channel_combobox.setCurrentIndex(0)

                if not self.current_sensor.startDataNotification():
                    print("start data transfer with device: " + self.current_sensor.BLEDevice.Name + " failed")
                    return

                self.connected_device = target_device
                self.sensor_profiles[device_address] = self.current_sensor
                self.disconnect_button.setEnabled(True)

                self.init_blitting()

            except Exception as e:
                print(f"Device connection error: {e}")

    def add_device_to_list(self, item_text):
        self.device_list.addItem(item_text)

    def disconnect_device(self):
        if self.connected_device:
            device_address = self.connected_device.Address
            sensor = self.sensor_profiles.get(device_address)
            if sensor:
                try:
                    sensor.disconnect()
                    print(f"Disconnected from device {self.connected_device.Name}")
                    self.connected_device = None
                    self.disconnect_button.setEnabled(False)
                except Exception as e:
                    print(f"Exception occurred while disconnecting device: {e}")
            else:
                print("No sensor profile found for the connected device.")
        else:
            print("No device is currently connected.")

    def deviceFoundCallback(self, deviceList: List[BLEDevice]):
        try:
            self.SensorControllerInstance.stopScan()
            self.scan_button.setEnabled(True)
            self.stop_scan_button.setEnabled(False)

            filteredDevice = filter(lambda x: x.Name.startswith("OY") or x.Name.startswith("gForce"), deviceList)

            for device in filteredDevice:
                if device.Address not in [d.Address for d in self.discovered_devices]:
                    item_text = f"RSSI: {device.RSSI}, Name: {device.Name}, Address: {device.Address}"
                    self.add_device_signal.emit(item_text)
                    self.discovered_devices.append(device)
        except Exception as e:
            print(f"Exception occurred in device discovery callback: {e}")

    def onDataCallback(self, sensor: SensorProfile, data: SensorData):
        if data and data.channelSamples and data.dataType in [DataType.NTF_EMG]:
            lost_count = self.lost_packet_counts.get(data.dataType, 0)
            if data.lostPackageCount > lost_count:
                lost_count = data.lostPackageCount
                self.update_lost_packet_display(str(data.dataType), lost_count)
            self.data_received.emit(data)
            # print(data.channelSamples[0][0].sampleIndex)

    def onPowerChanged(self, sensor: SensorProfile, power: int):
        print("connected sensor: " + sensor.BLEDevice.Name + " power: " + str(power))
        # if not sensor.isDataTransfering:
        #     try:
        #         sensor.disconnect()
        #         self.SensorControllerInstance.startScan(SCAN_DEVICE_PERIOD_IN_MS)
        #     except Exception as e:
        #         print(f"电源变化时断开连接并重新扫描出现异常: {e}")

    def onStateChanged(self, sensor: SensorProfile, newstate: DeviceStateEx):
        print("device: " + sensor.BLEDevice.Name + str(newstate))

    def onErrorCallback(self, sensor: SensorProfile, reason: str):
        print("device: " + sensor.BLEDevice.Name + reason)

    def _lost_type_name(self, type_str: str) -> str:
        try:
            return DataType(int(type_str, 0)).name
        except (ValueError, KeyError):
            return f"TYPE {type_str}"

    def update_lost_packet_display(self, lost_type: str, count: int):
        type_name = self._lost_type_name(lost_type)
        self.lost_packet_counts[type_name] = count
        lines = [f"  {k}: {v}" for k, v in sorted(self.lost_packet_counts.items())]
        self.lost_packet_label.setText("Lost Packet Statistics:\n" + "\n".join(lines))

    def update_buffer_size(self):
        buffer_size = int(self.period * self.sampling_rate)
        self.data_buffer = np.zeros((self.EmgChannelCount, buffer_size))
        self.buffer_index = 0

    def add_data_to_buffer(self, data: SensorData):
        try:
            if data and data.channelSamples:
                # Circular-buffer write: avoids rolling the whole buffer on every packet.
                buffer_size = self.data_buffer.shape[1]
                for i, channel in enumerate(data.channelSamples):
                    if i >= len(self.impedance):
                        self.impedance.append(0)
                    new_data = np.array([sample.data for sample in channel], dtype=np.float32)
                    num_samples = min(len(new_data), buffer_size)

                    if num_samples > 0:
                        write_start = self.buffer_index
                        write_end = write_start + num_samples

                        if write_end <= buffer_size:
                            self.data_buffer[i, write_start:write_end] = new_data[-num_samples:]
                        else:
                            first_part = buffer_size - write_start
                            self.data_buffer[i, write_start:] = new_data[-num_samples:-num_samples + first_part]
                            self.data_buffer[i, :num_samples - first_part] = new_data[-num_samples + first_part:]

                    # Keep only the latest impedance value per channel.
                    if channel:
                        self.impedance[i] = channel[-1].impedance

                self.buffer_index = (self.buffer_index + num_samples) % buffer_size

                # Throttle plot update signals to avoid flooding the main thread.
                now = time.time()
                if now - self._last_plot_signal_time >= self._min_plot_signal_interval:
                    self._last_plot_signal_time = now
                    self.update_plot_signal.emit()
        except Exception as e:
            print(f"Exception occurred in add_data_to_buffer: {e}")

    # def add_data_to_buffer(self, data: SensorData):
    #     try:
    #         if data and data.channelSamples:
    #             for i, channel in enumerate(data.channelSamples):
    #                 # 确保阻抗列表长度足够
    #                 if i >= len(self.impedance):
    #                     self.impedance.append([])

    #                 # 获取新数据
    #                 new_data = np.array([sample.data for sample in channel])
    #                 impedance_values = [sample.impedance for sample in channel]

    #                 # 缓冲区大小
    #                 buffer_size = len(self.data_buffer[i])
    #                 num_samples = len(new_data)

    #                 # 处理缓冲区数据
    #                 if num_samples < buffer_size:
    #                     # 缓冲区有足够空间，将新数据添加到右侧
    #                     self.data_buffer[i] = np.roll(self.data_buffer[i], -num_samples)
    #                     self.data_buffer[i][-num_samples:] = new_data
    #                 else:
    #                     # 缓冲区空间不足，只保留最新的 buffer_size 个数据
    #                     self.data_buffer[i] = new_data[-buffer_size:]

    #                 # 更新阻抗值
    #                 self.impedance[i] = impedance_values

    #             # 发射信号更新图形
    #             self.update_plot_signal.emit()
    #     except IndexError as e:
    #         print(f"add_data_to_buffer 方法中索引错误: {e}")
    #     except ValueError as e:
    #         print(f"add_data_to_buffer 方法中值错误: {e}")
    #     except Exception as e:
    #         print(f"add_data_to_buffer 方法中出现未知异常: {e}")

    def init_blitting(self):
        self.canvas.draw()
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        (self.line,) = self.ax.plot([], [], label="Channel 1")
        self.ax.legend(handles=[self.line], loc="upper right")

    def update_plot(self):
        # Skip rendering while the window is minimized to avoid DWM issues on Windows.
        if self.windowState() & QtCore.Qt.WindowMinimized:
            return

        try:
            if self.data_buffer is not None:
                buffer_size = int(self.period * self.sampling_rate)
                time_axis = np.linspace(0, self.period, buffer_size)
                # Reassemble the circular buffer into chronological order for plotting.
                y_data = np.roll(self.data_buffer[self.current_channel], -self.buffer_index)

                min_val = np.min(y_data)
                max_val = np.max(y_data)

                if min_val == max_val:
                    min_val = min_val - 100
                    max_val = max_val + 100

                margin = 0.1 * (max_val - min_val)
                min_val -= margin
                max_val += margin

                self.ax.set_ylim(min_val, max_val)
                self.ax.set_xlim(0, self.period)

                self.line.set_data(time_axis, y_data)
                self.ax.draw_artist(self.line)

                # Use draw_idle instead of synchronous draw to reduce main-thread blocking.
                self.canvas.draw_idle()

            if (self.impedance and
                    self.current_channel < len(self.impedance) and
                    isinstance(self.impedance[self.current_channel], (int, float))):
                current_impedance = self.impedance[self.current_channel] / 1000.0
                if current_impedance <= 500:
                    color = "green"
                elif 500 < current_impedance <= 999:
                    color = "yellow"
                else:
                    color = "red"
                self.impedance_label.setText(f"Impedance: {current_impedance:.2f} kΩ")
                self.impedance_label.setStyleSheet(f"color: {color}")

        except Exception as e:
            print(f"Exception occurred in update_plot: {e}")

    def start_data_processing(self, data):
        task = DataProcessingTask(self, data)
        self.thread_pool.start(task)

    def change_period(self, period_text):
        self.period = PERIOD_OPTIONS[period_text]
        self.data_buffer = None
        self.update_buffer_size()
        self.ax.clear()

        self.ax.set_xlim(0, self.period)
        self.ax.set_ylim(-1000, 1000)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Amplitude (uV)")
        self.ax.set_title("EMG Waveform (Real-time)")

        (self.line,) = self.ax.plot([], [], label=f"Channel {self.current_channel + 1}")
        self.ax.legend(handles=[self.line], loc="upper right")

        self.canvas.draw()
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)

    def change_channel(self, index):
        self.current_channel = index
        self.data_buffer = None
        self.update_buffer_size()
        self.ax.clear()

        self.ax.set_xlim(0, self.period)
        self.ax.set_ylim(-1000, 1000)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Amplitude (uV)")
        self.ax.set_title("EMG Waveform (Real-time)")

        (self.line,) = self.ax.plot([], [], label=f"Channel {self.current_channel + 1}")
        self.ax.legend(handles=[self.line], loc="upper right")

        self.canvas.draw()
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.data_buffer = None
        self.update_buffer_size()
        self.ax.clear()

        self.ax.set_xlim(0, self.period)
        self.ax.set_ylim(-1000, 1000)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Amplitude (uV)")
        self.ax.set_title("EMG Waveform (Real-time)")

        (self.line,) = self.ax.plot([], [], label=f"Channel {self.current_channel + 1}")
        self.ax.legend(handles=[self.line], loc="upper right")

        self.canvas.draw()
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)


if __name__ == "__main__":
    try:
        app = QtWidgets.QApplication(sys.argv)
        scanner = BluetoothDeviceScanner()

        def sigint_handler(signal, frame):
            app.quit()
            sys.exit(0)

        signal.signal(signal.SIGINT, sigint_handler)
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Uncaught exception in application: {e}")
