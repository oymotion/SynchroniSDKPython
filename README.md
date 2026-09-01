# sensor-sdk

OYMotion sdk for Python

## Brief

OYMotion SDK is the software development kit for developers to access OYMotion products.

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

When a compatible USB Bluetooth dongle is plugged in and usable, the SDK automatically drives it directly with a host-mode backend instead of the OS Bluetooth stack — no Bluetooth permission prompt, and it works even when the internal Bluetooth is off. A plain `pip install sensor-sdk` installs everything needed.

Behavior and controls:

- With a usable dongle present, the backend is selected automatically when the SDK starts (the native OS stack is used otherwise). Check the active backend with `SensorController.getBLEBackendName()` (`"bumble"` / `"bleak"`).
- `SensorController.checkSetupDongle()` checks dongle readiness and, when no dongle is usable, runs the bundled one-time setup (driver binding on Windows, permission setup on Linux, with an elevation prompt) and re-checks. It returns `"OK"` on success or an `"Error: ..."` string on failure, and blocks while waiting for the elevation prompt. macOS needs no setup. The same helper is also exported at package top level as `sensor.checkSetupDongle()`.
- `SENSOR_SDK_BLE_BACKEND=bleak` forces the native backend; `SENSOR_SDK_BLE_BACKEND=bumble` forces the dongle backend.

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

Use `def getVersion() -> str` to get the SDK version string.

```python
version = SensorControllerInstance.getVersion()
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

The asynchronous variant is `asyncScan(period_in_ms: int) -> list[BLEDevice]`.

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

Use `onEnableCallback` to be notified when the bluetooth enable state changes:

```python
SensorControllerInstance.onEnableCallback = lambda enabled: print("bluetooth on" if enabled else "bluetooth off")
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
from typing import List
from sensor import SensorProfile, SensorData, DeviceStateEx

sensorProfile = SensorControllerInstance.requireSensor(bleDevice)

# register callbacks
def on_state_changed(sensor: SensorProfile, newState: DeviceStateEx):
    # device state transitions (Connecting/Connected/Ready/Disconnected/...)
    # please do logic when device disconnected unexpected
    pass

def on_error_callback(sensor: SensorProfile, reason: str):
    # called when error occurs
    pass

def on_power_changed(sensor: SensorProfile, power: int):
    # callback for get battery level of device, power from 0 - 100
    # (invalid -1 readings are never reported here; getBatteryLevel() may
    # still return -1 when no valid reading is available yet)
    pass

def on_data_callback(sensor: SensorProfile, data_list: List[SensorData]):
    # called after start data transfer; each invocation delivers the whole
    # batch of SensorData objects parsed together (loop over it to process
    # each one)
    for data in data_list:
        pass

sensorProfile.onStateChanged = on_state_changed
sensorProfile.onErrorCallback = on_error_callback
sensorProfile.onPowerChanged = on_power_changed
sensorProfile.onDataCallback = on_data_callback
```

Callbacks run on SDK background threads: keep them short or thread-safe, and update UI through your framework's main-thread mechanism (e.g. Qt signals).

A fifth callback, `onAutoReconnect`, customizes stream recovery after an abnormal disconnect — see section 15.1.

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

Use `property isReady: bool` as a shortcut for `deviceState == DeviceStateEx.Ready`.

```python
if sensorProfile.isReady:
    pass
```

### 15. Get BLE device of SensorProfile

Use `property BLEDevice: BLEDevice` to get BLE device of SensorProfile.

```python
bleDevice = sensorProfile.BLEDevice
```

### 15.1 Auto reconnect and resume data stream

Use `property autoReconnect: bool` (default `True`) to control automatic recovery. While `True` and the device is streaming, an abnormal disconnect is followed by automatic reconnect → `init()` with the previous init arguments → re-applying the `setParam` parameters from the previous streaming session → `startDataNotification()`. Recovery retries on the next successful reconnect if a step fails. Explicit user calls (`connect()`, `disconnect()`, `stopDataNotification()`) cancel the pending resume, and setting `autoReconnect = False` disables the behavior entirely.

```python
sensorProfile.autoReconnect = True   # default; set False to opt out
```

**Custom recovery via `onAutoReconnect`** (default `None`): when the auto-reconnect finds the disconnected device again (back in `Ready`, about to resume), this callback is invoked instead of the default flow.

```python
def on_reconnect(sensor, restore: bool, answer) -> None:
    # restore=True   -> a previous session exists (init args + setParam values can be
    #                   preserved and restored); restore=False -> fresh-init case
    # answer(True)   -> the app handled recovery itself (fresh init or custom flow);
    #                   the SDK skips the default recovery
    # answer(False)  -> fall back to the default flow (connect -> init -> replay
    #                   setParam values -> startDataNotification)
    sensor.init(32, 60000)
    sensor.startDataNotification()
    answer(True)   # answer exactly once, from any thread, at any later time

sensorProfile.onAutoReconnect = on_reconnect
```

The answer is **asynchronous**: the callback receives an `answer(handled)` callable and must invoke it exactly once — it may return immediately and answer later from any thread (e.g. after UI-thread work). If no answer arrives in time the SDK falls back to the default recovery. Callback exceptions are logged and treated as `answer(False)`. Blocking calls (`init()`, `setParam()`, `startDataNotification()`) are allowed inside the callback. The legacy two-argument form `callback(sensor, restore) -> bool` is still accepted; its return value is treated as the answer.

### 16. Get device info of SensorProfile

Use `def getDeviceInfo() -> DeviceInfo | None` to get device info of SensorProfile.

Please call after device in 'Ready' state and `init()` has succeeded, returns None otherwise.

```python
    deviceInfo = sensorProfile.getDeviceInfo()

# deviceInfo is a DeviceInfo object with attributes:
#   DeviceName, ModelName, HardwareVersion, FirmwareVersion, MTUSize
#   plus a ChannelCount / SampleRate attribute pair for each modality:
#   Ppg, Spo2, Impe, Emg, Eeg, Ecg, Acc, Gyro, Brth, MagAngle, Euler, Quat
#   plus EmgMaxSampleRate / EegMaxSampleRate / EcgMaxSampleRate: the maximum
#   sample rate reported by the device capability query (0 = not reported).
#   plus ImuChannelCount / ImuSampleRate: the aggregated NTF_IMU stream
#   (0 = no aggregated stream, use the four separate ACC/GYRO/EULER/QUAT
#   streams instead).
#   plus ConnectionIntervalMs / PeripheralLatency / SupervisionTimeoutMs:
#   the negotiated BLE link parameters (0 / -1 / 0 = unknown).
# e.g. deviceInfo.EmgChannelCount, deviceInfo.EegSampleRate
```

Use `onDeviceInfoUpdate` to get notified when the DeviceInfo changes after init — e.g. the link parameters are updated by the peripheral shortly after connect, or `EEG_SAMPLE_RATE` changes the reported rates.

```python
def on_device_info_update(sensor: SensorProfile, info: DeviceInfo):
    pass

sensorProfile.onDeviceInfoUpdate = on_device_info_update
```

### 17. Init data transfer

Use `def init(packageSampleCount: int, powerRefreshInterval: int) -> bool`.

Please call after device in 'Ready' state, return True if init succeed.

```python
success = sensorProfile.init(5, 60*1000)
```

packageSampleCount:   set sample counts of SensorData.channelSamples per batch in onDataCallback()
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

#### 19.2 Synchronized start on multiple devices

Use `def multiStartDataNotification(sensors: list[SensorProfile], timeout: float = 30.0, maxDelayDispersionMs: int = 5, maxAttempts: int = 3) -> dict[str, bool]` on `SensorController` (async variant: `asyncMultiStartDataNotification`) to start data notification on several devices at once. Every sensor must be `Ready` and `hasInited`; the result maps each device MAC to its success flag — devices that fail validation do not prevent the others from starting.

On the dongle backend the start commands of all devices are released at virtually the same time; on the native backend the starts are simply issued concurrently.

After each start round the SDK validates the dispersion (max − min) of the devices' first-packet delays; if it exceeds `maxDelayDispersionMs` (default 5 ms; pass `-1` to skip the dispersion check entirely — devices only need to start and produce a first packet), any device produces no first packet in time, or any start fails, all devices are stopped and the round retries (up to `maxAttempts`, default 3). If still not within tolerance, the streams are stopped and the call reports failure.

```python
results = SensorControllerInstance.multiStartDataNotification([sensor1, sensor2])
# {"AA-BB-CC-DD-EE-01": True, "AA-BB-CC-DD-EE-02": False}

# relax the first-packet delay dispersion tolerance to 10 ms:
results = SensorControllerInstance.multiStartDataNotification([sensor1, sensor2], maxDelayDispersionMs=10)
```

#### 19.3 Synchronized stop on multiple devices

`def multiStopDataNotification(sensors: list[SensorProfile], timeout: float = 10.0) -> dict[str, bool]` (async variant: `asyncMultiStopDataNotification`) is the stop counterpart of `multiStartDataNotification`: on the dongle backend the stop commands of all devices go out at virtually the same time; on the native backend the stops are issued concurrently. Devices that are not streaming count as successful (nothing to stop); invalid devices do not affect the others.

```python
results = SensorControllerInstance.multiStopDataNotification([sensor1, sensor2])
```

Data type list：

```python
class DataType(Enum):
    NTF_ACC = 0x1            # acceleration, unit is g
    NTF_GYRO = 0x2           # gyroscope, unit is degree/s
    NTF_EULER_DATA = 0x4     # euler angle, unit is degree
    NTF_QUATERNION = 0x5     # quaternion (w, x, y, z)
    NTF_GEST = 0x07          # gesture id
    NTF_EMG = 0x8            # unit is uV
    NTF_MAG_ANGLE_DATA = 0x0D
    NTF_EEG = 0x10           # unit is uV
    NTF_ECG = 0x11           # unit is uV
    NTF_IMPEDANCE = 0x12     # electrode impedance
    NTF_IMU = 0x13           # aggregated IMU batch (acc 0-2 / gyro 3-5 / euler 6-8 / quat 9-12;
                             # see DeviceInfo.ImuChannelCount)
    NTF_ADS = 0x14
    NTF_BRTH = 0x15          # respiration, unit is uV
    NTF_IMPEDANCE_EXT = 0x16
    NTF_SPO2 = 0x17          # SpO2 percentage
    NTF_PPG = 0x18           # PPG raw samples
```

Process data in onDataCallback. Each invocation delivers a list of SensorData batches parsed together; loop over the list to process each batch. SensorData's public interface:

- metadata: `getDeviceMac()` / `getDataType()` / `getSampleRate()` / `getChannelCount()` / `getSampleCount()` / `getChannelMask()` / `getLostPackageCount()` / `getStartTimeStamp()` / `getStartTimeSec()` (wall-clock stream-start anchor in Unix seconds, 0.0 when unknown) / `getDelay()` / `isDataValid()`
- whole batch: read-only `channelSamples` / `startSampleIndex` properties, `clone()` for a deep copy
- single-point accessors (`ci` = channel index, `si` = sample index): `getChannelSample(ci, si)` returns the `Sample`; `getData(ci, si)` / `getRawData(ci, si)` / `getImpedance(ci, si)` / `getSaturation(ci, si)` / `getSampleIndex(ci, si)` / `getTimeStampInMs(ci, si)` / `getAbsTimeStampInSec(ci, si)` (absolute timestamp in seconds, 0.0 when the stream-start anchor is unknown) / `isLost(ci, si)` return the individual field
- Sample fields (`data`, `rawData`, `impedance`, `saturation`, `sampleIndex`, `channelIndex`, `absTimeStampInSec`, `isLost`) are read-only properties.

```python
def on_data_callback(sensor: SensorProfile, data_list: List[SensorData]):
    for data in data_list:
        if data.getDataType() == DataType.NTF_EEG:
            pass
        elif data.getDataType() == DataType.NTF_ECG:
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

# batteryPower is battery level returned, value ranges from 0 to 100, 0 means out of battery, while 100 means full;
# -1 means no valid reading is available yet (onPowerChanged never reports -1).
```

Please check console.py in examples directory

### Async methods

All methods starting with `async` are async methods; they have the same params and return result as the sync methods:

- `SensorController`: `asyncScan`, `asyncMultiStartDataNotification`, `asyncMultiStopDataNotification`
- `SensorProfile`: `asyncConnect`, `asyncDisconnect`, `asyncInit`, `asyncStartDataNotification`, `asyncStopDataNotification`, `asyncSetParam`, `asyncGetParam`, `asyncGetBatteryLevel`

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
result = sensorProfile.setParam("NTF_PPG", "ON")
result = sensorProfile.setParam("NTF_PPG_RAW", "ON")   # alias of NTF_PPG
result = sensorProfile.setParam("NTF_SPO2", "ON")
result = sensorProfile.setParam("NTF_GFORCE_EULER", "ON")
result = sensorProfile.setParam("NTF_GFORCE_QUAT", "ON")
result = sensorProfile.setParam("NTF_GFORCE_ACC", "ON")
result = sensorProfile.setParam("NTF_GFORCE_GYRO", "ON")
# set data stream to ON or OFF, result is "OK" if succeed
# NTF_IMU is the master switch of the four NTF_GFORCE_* streams: toggling it
# updates all four, and toggling any of the four updates the aggregated NTF_IMU state.
# Note: on legacy EMG devices, NTF_GEST and NTF_EMG are mutually exclusive.

# Firmware filter toggles
result = sensorProfile.setParam("FILTER_50HZ", "ON")
# set 50Hz notch filter to ON or OFF, result is "OK" if succeed

# EEG/ECG sample rate (bound together on devices that have both)
result = sensorProfile.setParam("EEG_SAMPLE_RATE", "500")
# The value is validated against the device-reported capability list (see
# getParam("EEG_SAMPLE_RATE_LIST")); an unsupported value returns
# "Error: unsupported sample rate ...". While streaming, the stream is
# restarted so the new rate takes effect.

result = sensorProfile.setParam("FILTER_60HZ", "ON")
# set 60Hz notch filter to ON or OFF, result is "OK" if succeed

result = sensorProfile.setParam("FILTER_HPF", "ON")
# set 0.5Hz hpf filter to ON or OFF, result is "OK" if succeed

result = sensorProfile.setParam("FILTER_LPF", "ON")
# set 80Hz lpf filter to ON or OFF, result is "OK" if succeed

# NeuCir remote control (NeuCir devices only)
result = sensorProfile.setParam("NEUCIR_SET_MODE", "APP_REMOTE")
result = sensorProfile.setParam("NEUCIR_APP_CONTROL", "OPEN")   # OPEN / CLOSE / STOP

result = sensorProfile.setParam("DEBUG_BLE_DATA_PATH", "d:/temp/test.bin")
# set the bin export path: the session's raw BLE capture (bin) is exported to
# this location on stopDataNotification / disconnect.
# "True" exports to a default bin file in the SDK log directory
# (see setLogPath; disabled when file output is off), "False" or "" disables
# export.
# please give an absolute path and make sure it is valid and writeable by yourself

result = sensorProfile.setParam("DEBUG_LOG_PATH", "True")
# enable this profile's log file in the SDK log directory (see setLogPath),
# or pass an absolute custom path instead of "True";
# "False" or "" disables it. The profile log contains only this profile's logs;
# all other (common) logs go to the controller log.
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

result = sensorProfile.getParam("EEG_SAMPLE_RATE")
# Returns the current EEG/ECG sample rate in Hz, e.g. "250"

result = sensorProfile.getParam("EEG_SAMPLE_RATE_LIST")
# Returns the device-reported selectable sample rates (EEG/ECG bound, pipe-
# separated), e.g. "250|500"; "Error: Not supported" when the device did not
# report a capability
```

If the key is not supported, the result starts with `"Error"`.

## Bin file recording and replay

On every successful connect, the SDK records the raw BLE packets of the session into a `.bin` file. Bin files can be replayed offline for debugging and packet-loss analysis.

### Get bin file info

Use `def getBinFileInfo(self, file_path: str) -> Optional[dict]` to read the config record of a bin file:

```python
info = SensorControllerInstance.getBinFileInfo("path/to/session.bin")
# Returns a dict:
#   device_mac, device_name, chip_type, is_universal_stream, feature_map,
#   device_info, sensor_datas (per data-type parse config),
#   replay_duration (recording seconds)
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
- `timeout`: seconds to wait for completion. `None` auto-estimates from the bin duration. On timeout the call returns while replay may still be running in the background.
- Replay is rejected while the target sensor is streaming live data.

### Pause / resume / stop replay

```python
result = SensorControllerInstance.pauseBinReplay(sensor)   # pause feeding
result = SensorControllerInstance.resumeBinReplay(sensor)  # resume feeding
result = SensorControllerInstance.stopBinReplay(sensor)    # abort; the blocking replayBinFile call returns
# Each returns "OK" on success or an error string otherwise.
```

### Replay multiple bin files in sync (shared clock)

Use `def multiReplayBinFile(self, file_paths: List[str], sensors: Optional[List[Optional[SensorProfile]]] = None, realtime: bool = True, timeout: Optional[float] = None) -> List[Optional[SensorProfile]]` to replay several bin captures on one shared clock aligned by record timestamps, so concurrently recorded captures keep their original relative offsets (a device that started streaming later delivers its first data correspondingly later).

```python
sensors = [SensorControllerInstance.requireSensor(device1),
           SensorControllerInstance.requireSensor(device2)]
for s in sensors:
    s.onDataCallback = on_data
results = SensorControllerInstance.multiReplayBinFile(
    ["path/to/dev1.bin", "path/to/dev2.bin"], sensors=sensors, realtime=True)
# results is aligned with file_paths; a None entry marks a member that failed
# (file missing / no config record / duplicate MAC / device streaming or
#  already replaying).
```

- `sensors`: optional list aligned with `file_paths`; a `None` entry (or omitting the parameter) makes the controller create (or reuse) the profile from that bin's config record — register callbacks on existing profiles to receive data.
- `realtime`: `True` replays on the aligned group clock; `False` replays every member as fast as possible (no alignment).
- `timeout`: seconds to wait for the whole group. `None` auto-estimates from the group's duration.
- `pauseBinReplay` / `resumeBinReplay` on any member pauses/resumes the **whole group** (alignment is preserved); `stopBinReplay` still works per device.
- The call blocks until all members finish; run it on a background thread to keep the UI responsive.

### Parse a bin file to CSV

Use `def parseBinToCsv(self, bin_path: str, csv_path: Optional[str] = None) -> str` to convert a recorded bin file to CSV offline:

```python
csv_path = SensorControllerInstance.parseBinToCsv("d:/temp/test.bin")
# or with an explicit output path:
csv_path = SensorControllerInstance.parseBinToCsv("d:/temp/test.bin", "d:/temp/test.csv")
# Returns the CSV file path.
```

**CSV format** — header row:

```
timestamp,mac,type,raw_hex,data_type,sample_rate,channel_count,lost_count,samples_info,first_sample
```

Row kinds (a bin without a config record yields `raw` rows only):

- `raw` rows — one per data record: `timestamp` = ISO 8601 (local time), `raw_hex` = raw packet bytes as hex; remaining columns empty.
- `cmd_send` / `cmd_recv` rows — one per command record: `raw_hex` = command bytes as hex, `data_type` = decoded command name (e.g. `NTF_DATA_START` / `GET_FEATURE_MAP:SUCCESS`); analysis-only, not fed into parsing.
- `event` rows — one per BLE event record: `data_type` = event name (`connect` / `disconnect` / `stream_start` / `stream_stop`).
- `parsed` rows — one per parsed batch (`raw_hex` empty):

| Column | Meaning |
|--------|---------|
| `timestamp` | ISO 8601 (local time) |
| `mac` | Device MAC |
| `type` | `parsed` |
| `data_type` | `DataType` enum name of the batch, e.g. `NTF_EMG`, `NTF_EEG`, `NTF_ACC` |
| `sample_rate` | Batch sample rate (Hz) |
| `channel_count` | Batch channel count |
| `lost_count` | `lostPackageCount` of the batch |
| `samples_info` | Per-channel sample counts as a Python list string, e.g. `[32, 32, 32]` |
| `first_sample` | First sample of the first non-empty channel: `data=<v>\|raw=<raw>\|imp=<impedance>\|sat=<saturation>\|idx=<sampleIndex>\|ts=<ms timestamp>\|ch=<channelIndex>\|lost=<isLost>` |

## Logging controls

`setLogPath` sets the SDK log **directory** (it must be a directory). All default file outputs live in it: the controller log, the default per-profile logs (`DEBUG_LOG_PATH=True`) and the default bin exports (`DEBUG_BLE_DATA_PATH=True`).

The **controller log** holds all common logs (scan, connection management, backend, and logs of profiles whose profile log is not enabled). It is created automatically when `setDebugEnabled(True)` is called, and closed on `setDebugEnabled(False)`. Each **profile log** (enabled per profile via `setParam("DEBUG_LOG_PATH", ...)`) contains only that profile's logs.

```python
SensorControllerInstance.setDebugEnabled(True)
# enable SDK debug logs; automatically creates the controller log in the log
# directory. setDebugEnabled(False) closes it.

SensorControllerInstance.setLogPath(True, "d:/temp/sdklogs")
# set the log directory (must be a directory; created if missing, rejected if
# it points to an existing file). setLogPath(False) disables file output
# (controller log, default profile logs and default bin exports).

SensorControllerInstance.setLogPath(True)
# reset to the default log directory (~/Documents/sensorsdklog)
```

### Application log entries

Applications can write their own events (user actions, business state, etc.) into the same SDK log files, keeping one shared timeline with the SDK's internal logs:

```python
SensorControllerInstance.log("User clicked start", "I")
# written to the controller log (common channel)

sensor.log("User toggled filter 50Hz", "I")
# SensorProfile.log: routed to that profile's log file when enabled,
# otherwise falls back to the controller log
```

`level` accepts `"D"` / `"I"` / `"W"` / `"E"` (case-insensitive, default `"I"`); `"D"` follows the `setDebugEnabled` switch, the other levels are always emitted. Entries are tagged `[App]` in the log files.
