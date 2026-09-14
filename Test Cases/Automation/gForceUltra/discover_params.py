# -*- coding: utf-8 -*-
"""探测设备支持的参数 key。

连接设备后：
1) 打印 getParam("NTF") / getParam("FILTER") 当前值
2) 探测采样率参数：getParam(EMG_SAMPLE_RATE / EEG_SAMPLE_RATE / IMU_SAMPLE_RATE 及其 _LIST)
3) 逐个尝试 setParam(NTF_*_KEY / FILTER_*_KEY, "ON")，输出返回值，汇总支持/不支持

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

# 所有 NTF_* key，来自 README（语义命名，非底层 DataType 常量）
NTF_KEYS = [
    "NTF_GEST", "NTF_EMG", "NTF_EEG", "NTF_ECG",
    "NTF_IMU", "NTF_BRTH", "NTF_IMPEDANCE",
    "NTF_MAG_ANGLE", "NTF_PPG", "NTF_PPG_RAW", "NTF_SPO2",
    "NTF_GFORCE_EULER", "NTF_GFORCE_QUAT",
    "NTF_GFORCE_ACC", "NTF_GFORCE_GYRO",
]

FILTER_KEYS = ["FILTER_50HZ", "FILTER_60HZ", "FILTER_HPF", "FILTER_LPF"]

# 采样率参数 key（README 仅文档化 EEG_SAMPLE_RATE；EMG/IMU 采样率需运行时确认）
SAMPLE_RATE_KEYS = ["EMG_SAMPLE_RATE", "EEG_SAMPLE_RATE", "IMU_SAMPLE_RATE"]


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

    for key in ["NTF", "FILTER"]:
        try:
            val = sensor.getParam(key)
            print(f"  getParam({key!r}) -> {val!r}", flush=True)
        except Exception as e:
            print(f"  getParam({key!r}) -> 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 采样率参数探测（getParam 安全探测，不实际改参） ----
    print("\n" + "=" * 60, flush=True)
    print("采样率参数探测", flush=True)
    print("=" * 60, flush=True)

    for key in SAMPLE_RATE_KEYS:
        for suffix in ["", "_LIST"]:
            k = key + suffix
            try:
                val = sensor.getParam(k)
                print(f"  getParam({k!r}) -> {val!r}", flush=True)
            except Exception as e:
                print(f"  getParam({k!r}) -> 抛异常 {type(e).__name__}: {e}", flush=True)

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