# sensor-sdk

Synchroni sdk for Python

## Brief

Synchroni SDK is the software development kit for developers to access Synchroni products.

## Contributing

See the [contributing guide](CONTRIBUTING.md) to learn how to contribute to the repository and the development workflow.

## License

MIT

---

## Installation

```sh
pip install sensor-sdk 
```

### USB Bluetooth dongle (bumble backend)

On macOS, when a USB Bluetooth HCI dongle is plugged in (e.g. Actions `10d7:b012` or `33fa:0012`; several dongles can be used at once, each serving one connection), the SDK automatically drives it directly with a [bumble](https://github.com/google/bumble) host-mode backend instead of CoreBluetooth — no Bluetooth permission prompt, works even when the internal Bluetooth is off, and allows larger ATT MTU. The [bleak-bumble](https://github.com/ekspla/bleak-bumble_dev_host_mode) backend is vendored into the SDK (`sensor/bleak_bumble/`, MIT license), and the bumble stack + libusb are declared as regular dependencies — a plain `pip install sensor-sdk` is all that is needed.

Behavior and controls:

- With the dependencies installed and a dongle present, the backend is selected automatically on macOS when the SDK starts (native `bleak` is used otherwise). Check the active backend with `SensorController.getBLEBackendName()` (`"bumble"` / `"bleak"`).
- `SENSOR_SDK_BLE_BACKEND=bleak` forces the native backend; `SENSOR_SDK_BLE_BACKEND=bumble` forces the dongle backend on any platform; `SENSOR_SDK_BUMBLE_TRANSPORT` overrides the bumble transport spec (e.g. `usb:0`).
- USB access needs no sudo on macOS as long as the OS has not claimed the dongle (it normally doesn't when the internal Bluetooth works).
- With the bumble backend, each dongle serves one connection at a time: scanning uses a free dongle and is skipped while all dongles are connected; connections fail fast when no dongle is free. Connection timeout is raised to 25s automatically.

## 1. Permission

Application will obtain bluetooth permission by itself.

## 2. Import SDK

```python
from sensor import *
```

## SensorController methods

### 1. Initalize

```python
SensorControllerInstance = SensorController()

# register scan listener
if not SensorControllerInstance.hasDeviceFoundCallback:
    def on_device_callback(deviceList: List[BLEDevice]):
        # return all devices doesn't connected
        pass
    SensorControllerInstance.onDeviceFoundCallback = on_device_callback
```

### 2. Start scan

Use `def startScan(period_in_ms: int) -> bool` to start scan

```python
success = SensorControllerInstance.startScan(6000)
```

returns true if start scan success, periodInMS means onDeviceCallback will be called every periodInMS

Use `def scan(period_in_ms: int) -> list[BLEDevice]` to scan once time

```python
bleDevices = SensorControllerInstance.scan(6000)
```

### 3. Stop scan

Use `def stopScan() -> None` to stop scan

```python
SensorControllerInstance.stopScan()
```

### 4. Check scaning

Use `property isScanning: bool` to check scanning status

```python
isScanning = SensorControllerInstance.isScanning
```

### 5. Check if bluetooth is enabled

Use `property isEnable: bool` to check if bluetooth is enable

```python
isEnable = SensorControllerInstance.isEnable
```

### 6. Create SensorProfile

Use `def requireSensor(device: BLEDevice) -> SensorProfile | None` to create SensorProfile.

If bleDevice is invalid, result is None.

```python
sensorProfile = SensorControllerInstance.requireSensor(bleDevice)
```

### 7. Get SensorProfile

Use `def getSensor(deviceMac: str) -> SensorProfile | None` to get SensorProfile.

If SensorProfile didn't created, result is None.

```python
sensorProfile = SensorControllerInstance.getSensor(bleDevice.Address)
```

### 8. Get Connected SensorProfiles

Use `def getConnectedSensors() -> list[SensorProfile]` to get connected SensorProfiles.

```python
sensorProfiles = SensorControllerInstance.getConnectedSensors()
```

### 9. Get Connected BLE Devices

Use `def getConnectedDevices() -> list[BLEDevice]` to get connected BLE Devices.

```python
bleDevices = SensorControllerInstance.getConnectedDevices()
```

### 10. Terminate

Use `def terminate()` to terminate sdk

```python

def terminate():
    SensorControllerInstance.terminate()
    exit()
    
def main():
    signal.signal(signal.SIGINT, lambda signal, frame: terminate())
    time.sleep(30)
    SensorControllerInstance.terminate()
    
Please MAKE SURE to call terminate when exit main() or press Ctrl+C
```

## SensorProfile methods

### 11. Initalize

Please register callbacks for SensorProfile

```python
sensorProfile = SensorControllerInstance.requireSensor(bleDevice)

# register callbacks
def on_state_changed(sensor, newState):
    # please do logic when device disconnected unexpected
    pass

def on_error_callback(sensor, reason):
    # called when error occurs
    pass

def on_power_changed(sensor, power):
    # callback for get battery level of device, power from 0 - 100, -1 is invalid
    pass

def on_data_callback(sensor, data):
    # called after start data transfer
    pass

sensorProfile.onStateChanged = on_state_changed
sensorProfile.onErrorCallback = on_error_callback
sensorProfile.onPowerChanged = on_power_changed
sensorProfile.onDataCallback = on_data_callback
```

### 12. Connect device

Use `def connect() -> bool` to connect.

```python
success = sensorProfile.connect()
```

### 13. Disconnect

Use `def disconnect() -> bool` to disconnect.

If data notification is currently active, `disconnect()` will automatically stop it first before closing the BLE connection.

```python
success = sensorProfile.disconnect()
```

### 14. Get device status

Use `property deviceState: DeviceStateEx` to get device status.

Please send command in 'Ready' state, should be after connect() return True.

```python
deviceStateEx = sensorProfile.deviceState

# deviceStateEx has define:
# class DeviceStateEx(Enum):
#     Disconnected = 0
#     Connecting = 1
#     Connected = 2
#     Ready = 3
#     Disconnecting = 4
#     Invalid = 5
```

### 15. Get BLE device of SensorProfile

Use `property BLEDevice: BLEDevice` to get BLE device of SensorProfile.

```python
bleDevice = sensorProfile.BLEDevice
```

### 16. Get device info of SensorProfile

Use `def getDeviceInfo() -> DeviceInfo | None` to get device info of SensorProfile.

Please call after device in 'Ready' state and `init()` has succeeded, returns None otherwise.

```python
    deviceInfo = sensorProfile.getDeviceInfo()

# deviceInfo is a DeviceInfo object with attributes:
#   DeviceName, ModelName, HardwareVersion, FirmwareVersion, MTUSize
#   plus a ChannelCount / SampleRate attribute pair for each modality:
#   Ppg, Spo2, Impe, Emg, Eeg, Ecg, Acc, Gyro, Brth, MagAngle, Euler, Quat
# e.g. deviceInfo.EmgChannelCount, deviceInfo.EegSampleRate
```

### 17. Init data transfer

Use `def init(packageSampleCount: int, powerRefreshInterval: int) -> bool`.

Please call after device in 'Ready' state, return True if init succeed.

```python
success = sensorProfile.init(5, 60*1000)
```

packageSampleCount:   set sample counts of SensorData.channelSamples in onDataCallback()
powerRefreshInterval: callback period for onPowerChanged()

### 18. Check if init data transfer succeed

Use `property hasInited: bool` to check if init data transfer succeed.

```python
hasInited = sensorProfile.hasInited
```

### 19. DataNotify

Use `def startDataNotification() -> bool` to start data notification.

Please call if hasInited return True

#### 19.1 Start data transfer

```python
success = sensorProfile.startDataNotification()
```

Data type list：

```python
class DataType(Enum):
    NTF_ACC = 0x1  # unit is g
    NTF_GYRO = 0x2  # unit is degree/s
    NTF_EEG = 0x10  # unit is uV
    NTF_ECG = 0x11  # unit is uV
    NTF_BRTH = 0x15  # unit is uV
```

Process data in onDataCallback.

```python
def on_data_callback(sensor, data):
    if data.dataType == DataType.NTF_EEG:
        pass
    elif data.dataType == DataType.NTF_ECG:
        pass

    # process data as you wish
    for oneChannelSamples in data.channelSamples:
        for sample in oneChannelSamples:
            if sample.isLost:
                # do some logic
                pass
            else:
                # draw with sample.data & sample.channelIndex
                # print(f"{sample.channelIndex} | {sample.sampleIndex} | {sample.data} | {sample.impedance}")
                pass

sensorProfile.onDataCallback = on_data_callback
```

#### 19.2 Stop data transfer

Use `def stopDataNotification() -> bool` to stop data transfer.

```python
success = sensorProfile.stopDataNotification()
```

#### 19.3 Check if it's data transfering

Use `property isDataTransfering: bool` to check if it's data transfering.

```python
isDataTransfering = sensorProfile.isDataTransfering
```

### 20. Get battery level

Use `def getBatteryLevel() -> int` to get battery level. Please call after device in 'Ready' state.

```python
batteryPower = sensorProfile.getBatteryLevel()

# batteryPower is battery level returned, value ranges from 0 to 100, 0 means out of battery, while 100 means full.
```

Please check console.py in examples directory

### Async methods

all methods start with async is async methods, they has same params and return result as sync methods.

Please check async_console.py in examples directory

### setParam method

Use `def setParam(self, key: str, value: str) -> str` to set parameter of sensor profile. Please call after device in 'Ready' state.

The asynchronous variant is `asyncSetParam(self, key: str, value: str) -> str`.

If the device is already streaming when you change an `NTF_*` or `FILTER_*` key, the SDK will stop and restart the data notification so the new setting takes effect immediately.

Below is available key and value:

```python
# Data stream toggles
result = sensorProfile.setParam("NTF_GEST", "ON")
result = sensorProfile.setParam("NTF_EMG", "ON")
result = sensorProfile.setParam("NTF_EEG", "ON")
result = sensorProfile.setParam("NTF_ECG", "ON")
result = sensorProfile.setParam("NTF_IMU", "ON")
result = sensorProfile.setParam("NTF_BRTH", "ON")
result = sensorProfile.setParam("NTF_IMPEDANCE", "ON")
result = sensorProfile.setParam("NTF_MAG_ANGLE", "ON")
result = sensorProfile.setParam("NTF_PPG_RAW", "ON")
result = sensorProfile.setParam("NTF_GFORCE_EULER", "ON")
result = sensorProfile.setParam("NTF_GFORCE_QUAT", "ON")
result = sensorProfile.setParam("NTF_GFORCE_ACC", "ON")
result = sensorProfile.setParam("NTF_GFORCE_GYRO", "ON")
# set data stream to ON or OFF, result is "OK" if succeed
# Note: on legacy (non-new) EMG devices, NTF_GEST and NTF_EMG are mutually exclusive.

# Firmware filter toggles
result = sensorProfile.setParam("FILTER_50HZ", "ON")
# set 50Hz notch filter to ON or OFF, result is "OK" if succeed

result = sensorProfile.setParam("FILTER_60HZ", "ON")
# set 60Hz notch filter to ON or OFF, result is "OK" if succeed

result = sensorProfile.setParam("FILTER_HPF", "ON")
# set 0.5Hz hpf filter to ON or OFF, result is "OK" if succeed

result = sensorProfile.setParam("FILTER_LPF", "ON")
# set 80Hz lpf filter to ON or OFF, result is "OK" if succeed

result = sensorProfile.setParam("DEBUG_BLE_DATA_PATH", "d:/temp/test.bin")
# set the bin export path: the session's raw BLE capture is recorded in the system
# temp directory and copied to this location on stopDataNotification / disconnect;
# "True" exports to {DeviceName}_data_YYYYMMDD_HHMMSS.bin in the SDK log directory,
# "False" or "" disables export (the temp bin is just deleted).
# please give an absolute path and make sure it is valid and writeable by yourself

result = sensorProfile.setParam("DEBUG_LOG_PATH", "True")
# enable SDK file logging to a default file ({DeviceName}_log_YYYYMMDD_HHMMSS.txt),
# or pass an absolute custom path instead of "True"; "False" or "" disables it.
# getParam("DEBUG_LOG_PATH") returns the current log file path ("" when disabled).
```


### getParam method

Use `def getParam(self, key: str) -> str` to query the current parameter state of a sensor profile. Please call after the device reaches the 'Ready' state.

The asynchronous variant is `asyncGetParam(self, key: str) -> str`.

Supported aggregate query keys:

```python
result = sensorProfile.getParam("FILTER")
# Returns a pipe-separated string of all filter states, e.g.:
# "FILTER_50HZ|ON|FILTER_60HZ|ON|FILTER_HPF|ON|FILTER_LPF|ON"

result = sensorProfile.getParam("NTF")
# Returns a pipe-separated string of all notification states, e.g.:
# "NTF_BRTH|ON|NTF_ECG|ON|NTF_EEG|ON|NTF_EMG|ON|..."
```

If the key is not supported, the result starts with `"Error"`.

## Bin file recording and replay

On every successful connect, the SDK records all raw BLE packets of the session into a `.bin` file in the SDK log directory (`~/Documents/sensorsdklog` by default, or the directory of the configured log file), named `{DeviceName}_{MAC}_{YYYYMMDD_HHMMSS}.bin`. Recording is always on; it is skipped or stopped with a warning when free disk space is below 100MB or a disk error occurs, without affecting live streaming. Bin files can be replayed offline for debugging and packet-loss analysis.

### Get bin file info

Use `def getBinFileInfo(self, file_path: str) -> Optional[dict]` to read the config record of a bin file:

```python
info = SensorControllerInstance.getBinFileInfo("path/to/session.bin")
# Returns a dict:
#   device_mac, device_name, chip_type, is_universal_stream, feature_map,
#   device_info, sensor_datas (per data-type parse config),
#   replay_duration (recording seconds, written into the header record at close;
#                    older bins without a header fall back to a full-file estimate)
# Returns None if the file does not exist or has no config record.
```

### Replay a bin file

Use `def replayBinFile(self, file_path: str, sensor: Optional[SensorProfile] = None, realtime: bool = True, timeout: Optional[float] = None) -> Optional[SensorProfile]` to replay a bin file through the normal parsing pipeline. Parsed results arrive via `onDataCallback` on the returned SensorProfile, same as live data.

```python
# Simplest form: controller creates the profile from the bin config record
sensor = SensorControllerInstance.replayBinFile("path/to/session.bin")

# Recommended: reuse an existing profile with callbacks registered
sensor = SensorControllerInstance.requireSensor(device)
sensor.onDataCallback = on_data
SensorControllerInstance.replayBinFile("path/to/session.bin", sensor, realtime=True)
```

- `sensor`: an existing SensorProfile to replay through. When `None`, the controller creates (or reuses) a profile from the bin config record; in that mode the profile is returned only after replay finishes, so register callbacks on an existing profile to receive data.
- `realtime`: `True` replays at the recorded pace; `False` replays as fast as possible.
- `timeout`: seconds to wait for completion. `None` auto-estimates from the bin duration in realtime mode (duration + 30s, min 60s), or 600s otherwise. On timeout the call returns while replay may still be running in the background.
- Replay is rejected while the target sensor is streaming live data.

### Pause / resume / stop replay

```python
result = SensorControllerInstance.pauseBinReplay(sensor)   # pause feeding
result = SensorControllerInstance.resumeBinReplay(sensor)  # resume feeding
result = SensorControllerInstance.stopBinReplay(sensor)    # abort; the blocking replayBinFile call returns
# Each returns "OK" on success or an error string otherwise.
```

### Parse a bin file to CSV

Use `def parseBinToCsv(self, bin_path: str, csv_path: Optional[str] = None) -> str` to convert a recorded bin file to CSV offline (parsing runs through the real pipeline; row timestamps come from the bin records):

```python
csv_path = SensorControllerInstance.parseBinToCsv("d:/temp/test.bin")
# or with an explicit output path:
csv_path = SensorControllerInstance.parseBinToCsv("d:/temp/test.bin", "d:/temp/test.csv")
# Returns the CSV file path. Columns:
# timestamp, mac, type, raw_hex, data_type, sample_rate, channel_count, lost_count, samples_info, first_sample
```

## Logging controls

File logging is disabled by default. Records emitted before file logging is first enabled are held in a bounded memory buffer and replayed into the file on first enable, so early (scan/connect) logs are not lost. The log path is automatically shared with the BLE subprocess, so both sides write to the same file.

```python
SensorControllerInstance.setDebugEnabled(True)
# enable/disable SDK debug logs

SensorControllerInstance.setLogPath("d:/temp/sdk.log")
# enable file logging to a custom path; pass "" to disable

SensorControllerInstance.enableFileLog(True)
# enable file logging with the default path
# (~/Documents/sensorsdklog/log_YYYYMMDD_HHMMSS.txt)

SensorControllerInstance.setControllerLogPath("d:/temp/controller.log")
# SensorController-dedicated log file; pass "" to disable

SensorControllerInstance.enableControllerLog(True)
# enable the controller-dedicated log with the default path
# (~/Documents/sensorsdklog/sensor_controller_log_YYYYMMDD_HHMMSS.txt)
```
