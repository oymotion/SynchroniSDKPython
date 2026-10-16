# -*- coding: utf-8 -*-
"""gForceMax 能力探测脚本。

连接目标设备，探测并固化 gForceMax 的能力基线，重点确认「四元数
（NTF_GFORCE_QUAT / QuatChannelCount）」这一相对 gForceUltra 的新增能力。

步骤：
  1) dump DeviceInfo 全字段（C 扩展类，dir()/反射拿不到字段，逐个 getattr 试探），
     重点看 QuatChannelCount / QuatSampleRate / EulerChannelCount
  2) dump getParam：聚合 NTF/FILTER + 逐个 NTF_* / 采样率参数 key（Error 开头 = 不支持）
  3) 起流 10s，统计实际收到的 DataType（确认 NTF_QUATERNION 等流确有数据）

用法：
    python probe_device_info.py           # 用 config.TARGET_IDENTITY
    python probe_device_info.py 9A91      # 显式指定 identity

前置条件：设备已开机、在范围内、config.py 已配置。
"""

import os
import sys
import time
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(BASE_DIR)
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import scan_and_match

# DeviceInfo 候选字段（来自 README 与 examples/pyqt_demo.py）
DEVICE_INFO_FIELDS = [
    "DeviceName", "ModelName", "HardwareVersion", "FirmwareVersion", "MTUSize",
    "PpgChannelCount", "PpgSampleRate",
    "Spo2ChannelCount", "Spo2SampleRate",
    "ImpeChannelCount", "ImpeSampleRate",
    "EmgChannelCount", "EmgSampleRate",
    "EegChannelCount", "EegSampleRate",
    "EcgChannelCount", "EcgSampleRate",
    "AccChannelCount", "AccSampleRate",
    "GyroChannelCount", "GyroSampleRate",
    "BrthChannelCount", "BrthSampleRate",
    "MagAngleChannelCount", "MagAngleSampleRate",
    "EulerChannelCount", "EulerSampleRate",
    "QuatChannelCount", "QuatSampleRate",
    "EmgMaxSampleRate", "EegMaxSampleRate", "EcgMaxSampleRate",
    "ImuChannelCount", "ImuSampleRate",
    "ConnectionIntervalMs", "PeripheralLatency", "SupervisionTimeoutMs",
]

NTF_KEYS = [
    "NTF_GEST", "NTF_EMG", "NTF_EEG", "NTF_ECG", "NTF_IMU",
    "NTF_BRTH", "NTF_IMPEDANCE", "NTF_MAG_ANGLE",
    "NTF_PPG", "NTF_PPG_RAW", "NTF_SPO2",
    "NTF_GFORCE_EULER", "NTF_GFORCE_QUAT", "NTF_GFORCE_ACC", "NTF_GFORCE_GYRO",
]

SAMPLE_RATE_KEYS = [
    "EEG_SAMPLE_RATE", "EEG_SAMPLE_RATE_LIST",
    "EMG_SAMPLE_RATE", "EMG_SAMPLE_RATE_LIST",
    "IMU_SAMPLE_RATE", "IMU_SAMPLE_RATE_LIST",
    "PPG_SAMPLE_RATE", "PPG_SAMPLE_RATE_LIST",
]


def dump_device_info(info):
    """逐个访问候选字段，区分「字段不存在」与「字段值」。"""
    lines = []
    for f in DEVICE_INFO_FIELDS:
        try:
            v = getattr(info, f)
            lines.append(f"{f} = {v!r}")
        except AttributeError:
            lines.append(f"{f} = <字段不存在>")
        except Exception as e:
            lines.append(f"{f} = <{type(e).__name__}: {e}>")
    return "\n".join(lines)


def dump_params(sensor):
    """dump getParam 聚合状态 + 逐个 NTF_* / 采样率参数 key 探测。"""
    lines = []
    for agg in ["NTF", "FILTER"]:
        try:
            v = sensor.getParam(agg)
        except Exception as e:
            v = f"<{type(e).__name__}: {e}>"
        lines.append(f"getParam({agg!r}) = {v!r}")

    lines.append("")
    lines.append("--- 逐个 NTF key 探测（Error 开头 = 不支持）---")
    for key in NTF_KEYS:
        try:
            v = sensor.getParam(key)
        except Exception as e:
            v = f"<{type(e).__name__}: {e}>"
        supported = not (isinstance(v, str) and v.startswith("Error"))
        lines.append(f"getParam({key!r}) = {v!r}  ->  {'支持' if supported else '不支持'}")

    lines.append("")
    lines.append("--- 采样率参数探测（Error 开头 = 不支持）---")
    for key in SAMPLE_RATE_KEYS:
        try:
            v = sensor.getParam(key)
        except Exception as e:
            v = f"<{type(e).__name__}: {e}>"
        supported = not (isinstance(v, str) and v.startswith("Error"))
        lines.append(f"getParam({key!r}) = {v!r}  ->  {'支持' if supported else '不支持'}")
    return "\n".join(lines)


class DataTypeCounter:
    """起流期间统计各 DataType 的累计样本数（确认哪些流确有数据）。"""
    def __init__(self):
        self.dt_samples = {}
        self.lock = threading.Lock()

    def __call__(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        with self.lock:
            for it in items:
                try:
                    n = it.getChannelCount() * it.getSampleCount()
                    if n <= 0:
                        continue
                    dt = it.getDataType()
                    name = dt.name if isinstance(dt, DataType) else str(dt)
                    self.dt_samples[name] = self.dt_samples.get(name, 0) + n
                except Exception:
                    continue

    def stats(self):
        with self.lock:
            return dict(self.dt_samples)


def main():
    ctrl = SensorControllerInstance
    target_identity = sys.argv[1].strip().upper() if len(sys.argv) > 1 else None

    print("=" * 60, flush=True)
    print("gForceMax 能力探测（重点：四元数 NTF_GFORCE_QUAT）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    is_enable = ctrl.isEnable
    print(f"[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[失败] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    print(f"\n[扫描] 目标 identity: {target_identity or common.TARGET_IDENTITIES}", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, target_identity=target_identity)
    if target is None:
        print("[失败] 未匹配到目标设备", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[失败] requireSensor 返回 None", flush=True)
        ctrl.terminate()
        return

    print("[连接] SensorProfile.connect() ...", flush=True)
    sensor.connect()
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[失败] 未到达 Ready，state={sensor.deviceState}", flush=True)
        ctrl.terminate()
        return

    print(f"[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)

    info = sensor.getDeviceInfo()
    if info is None:
        print("[失败] getDeviceInfo() 返回 None（需 init 成功且 Ready）", flush=True)
        sensor.disconnect()
        ctrl.terminate()
        return

    print("\n=== DeviceInfo 全字段 dump ===", flush=True)
    print(dump_device_info(info), flush=True)
    print("=" * 30, flush=True)

    # 四元数重点结论
    try:
        quat_ch = info.QuatChannelCount
        quat_rate = info.QuatSampleRate
    except Exception as e:
        quat_ch = quat_rate = f"<{type(e).__name__}>"
    print(f"\n[四元数结论] QuatChannelCount={quat_ch!r}  QuatSampleRate={quat_rate!r}  ->  "
          f"{'支持' if isinstance(quat_ch, int) and quat_ch > 0 else '不支持/待确认'}", flush=True)

    print("\n=== getParam 探测 ===", flush=True)
    print(dump_params(sensor), flush=True)
    print("=" * 30, flush=True)

    # 起流统计实际收到的 DataType（确认四元数等流有数据）
    print("\n[起流] 开启待测流并统计 DataType（10s）...", flush=True)
    for key in NTF_KEYS:
        try:
            sensor.setParam(key, "ON")
        except Exception:
            pass
    counter = DataTypeCounter()
    sensor.onDataCallback = counter
    try:
        sensor.startDataNotification()
    except Exception as e:
        print(f"[起流] 抛异常 {type(e).__name__}: {e}", flush=True)
        sensor.disconnect()
        ctrl.terminate()
        return
    time.sleep(10)
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    print("[起流] 实际收到的 DataType 样本统计：", flush=True)
    for dt, n in sorted(counter.stats().items()):
        print(f"    {dt}: {n} 样本", flush=True)

    sensor.disconnect()
    ctrl.terminate()


if __name__ == "__main__":
    main()
