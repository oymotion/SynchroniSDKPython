# -*- coding: utf-8 -*-
"""probe_device_info.py：快速连接目标设备并 dump DeviceInfo 全字段，验证字段名。

用途：
  DeviceInfo 是 C 扩展类，字段走 C 层动态解析，dir()/反射拿不到字段列表。
  本脚本连接设备后 getDeviceInfo()，逐个访问候选字段，区分「字段不存在」与「字段值」。

用法：
    python probe_device_info.py

设备信息见 config.py（匹配 config 中 enabled 的设备）。
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor import *
import config
import common
from common import scan_and_match

# DeviceInfo 候选字段（来自 README L313-323 与 example）
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


def dump_device_info(info):
    """逐个访问候选字段，区分「字段不存在」与「字段值」。"""
    lines = []
    for f in DEVICE_INFO_FIELDS:
        try:
            v = getattr(info, f)  # 不带默认值：字段不存在会抛 AttributeError
            lines.append(f"{f} = {v!r}")
        except AttributeError:
            lines.append(f"{f} = <字段不存在>")
        except Exception as e:
            lines.append(f"{f} = <{type(e).__name__}: {e}>")
    return "\n".join(lines)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("probe_device_info：DeviceInfo 全字段探测", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    is_enable = ctrl.isEnable
    print(f"[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[失败] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[失败] 未匹配到 config 中启用的设备", flush=True)
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
    print(f"[连接] 到达 Ready，state={sensor.deviceState}", flush=True)

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

    sensor.disconnect()
    ctrl.terminate()


if __name__ == "__main__":
    main()
