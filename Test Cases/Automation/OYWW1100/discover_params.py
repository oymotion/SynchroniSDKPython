# -*- coding: utf-8 -*-
"""探测设备支持的参数 key。

连接设备后：
1) 打印 getParam("NTF") 和 getParam("FILTER") 当前值
2) 逐个尝试 setParam(NTF_*_KEY, "ON")，输出返回值
3) 最后恢复原状态

前置条件：设备已开机、在范围内、config.py 已配置。
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(BASE_DIR)
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import _identity_of, scan_and_match

# 所有 NTF_* key，来自 README
NTF_KEYS = [
    "NTF_ACC", "NTF_GYRO", "NTF_EULER_DATA", "NTF_QUATERNION",
    "NTF_GEST", "NTF_EMG", "NTF_MAG_ANGLE_DATA",
    "NTF_EEG", "NTF_ECG", "NTF_IMPEDANCE",
    "NTF_IMU", "NTF_ADS", "NTF_BRTH",
    "NTF_IMPEDANCE_EXT", "NTF_SPO2", "NTF_PPG",
]

FILTER_KEYS = ["FILTER_50HZ", "FILTER_60HZ", "FILTER_HPF", "FILTER_LPF"]


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("设备参数探测", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"目标 identity: {common.TARGET_IDENTITIES}", flush=True)

    if not ctrl.isEnable:
        print("[跳过] 蓝牙未开启", flush=True)
        ctrl.terminate()
        return

    # scan
    print(f"\n[扫描] ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if devices:
        for d in devices:
            n = getattr(d, 'Name', '?')
            a = getattr(d, 'Address', '?')
            print(f"  {n} {a} identity={_identity_of(n)}", flush=True)
    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标: {name} {addr}", flush=True)

    # requireSensor
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] requireSensor 返回 None", flush=True)
        ctrl.terminate()
        return

    # connect
    ok = sensor.connect()
    print(f"[连接] connect() -> {ok}", flush=True)
    if not ok:
        print("[FAIL] 连接失败", flush=True)
        ctrl.terminate()
        return

    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[FAIL] 未到达 Ready, state={sensor.deviceState}", flush=True)
        sensor.disconnect()
        ctrl.terminate()
        return

    # init
    iret = sensor.init(20, 1000)
    print(f"[init] init(20, 1000) -> {iret}", flush=True)

    # ---- 当前状态 ----
    print("\n" + "=" * 60, flush=True)
    print("当前参数状态", flush=True)
    print("=" * 60, flush=True)

    for key in ["NTF", "FILTER", "EEG_SAMPLE_RATE", "IMU_SAMPLE_RATE"]:
        try:
            val = sensor.getParam(key)
            print(f"  getParam({key!r}) -> {val!r}", flush=True)
        except Exception as e:
            print(f"  getParam({key!r}) -> 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 逐个探测 NTF_* ----
    print("\n" + "=" * 60, flush=True)
    print("逐个探测 NTF_* setParam", flush=True)
    print("=" * 60, flush=True)

    supported = []
    unsupported = []

    for key in NTF_KEYS:
        try:
            r = sensor.setParam(key, "ON")
            print(f"  setParam({key!r}, 'ON') -> {r!r}", flush=True)
            if r == "OK":
                supported.append(key)
            else:
                unsupported.append(key)
        except Exception as e:
            print(f"  setParam({key!r}, 'ON') -> 抛异常 {type(e).__name__}: {e}", flush=True)
            unsupported.append(key)

    # ---- 逐个探测 FILTER_* ----
    print("\n" + "=" * 60, flush=True)
    print("逐个探测 FILTER_* setParam", flush=True)
    print("=" * 60, flush=True)

    for key in FILTER_KEYS:
        try:
            r = sensor.setParam(key, "ON")
            print(f"  setParam({key!r}, 'ON') -> {r!r}", flush=True)
            if r == "OK":
                supported.append(key)
            else:
                unsupported.append(key)
        except Exception as e:
            print(f"  setParam({key!r}, 'ON') -> 抛异常 {type(e).__name__}: {e}", flush=True)
            unsupported.append(key)

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("探测结果", flush=True)
    print("=" * 60, flush=True)
    print(f"  支持的 key ({len(supported)}): {supported}", flush=True)
    print(f"  不支持的 key ({len(unsupported)}): {unsupported}", flush=True)

    # 断开
    try:
        sensor.disconnect()
    except Exception:
        pass
    ctrl.terminate()


if __name__ == "__main__":
    main()