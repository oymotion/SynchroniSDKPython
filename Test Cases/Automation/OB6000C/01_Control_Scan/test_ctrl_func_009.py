# -*- coding: utf-8 -*-
"""CTRL-FUNC-009：getConnectedSensors/getConnectedDevices。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-009
可自动化：semi-auto（运行前需人工确认设备在范围内）

前置条件（运行前请人工准备好）：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 人工确认待测设备已开机 -> 按回车
  2) 检查 SensorController.isEnable == True
  3) scan 发现 OB6000C -> requireSensor -> connect
  4) 连接后 getConnectedSensors/getConnectedDevices 应包含该 mac
  5) disconnect 后两者应不含该 mac
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record


def _find_ob6000c(devices):
    for d in devices or []:
        name = getattr(d, 'Name', None) or ''
        if name.upper().startswith('OB6000C'):
            return d
    return None


def _mac_of_sensor(s):
    dev = getattr(s, 'BLEDevice', None)
    return (getattr(dev, 'Address', '') or '').upper()


def _mac_of_device(d):
    return (getattr(d, 'Address', '') or '').upper()


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-009 getConnectedSensors/getConnectedDevices", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描发现设备
    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    try:
        devices = ctrl.scan(config.SCAN_TIMEOUT_MS)
    except Exception as e:
        devices = None
        print(f"[扫描] 抛异常 {type(e).__name__}: {e}", flush=True)
    target = _find_ob6000c(devices)

    if target is None:
        print("[检查] 未发现 OB6000C 设备", flush=True)
        record(results, "连接后 getConnectedSensors 含该 mac", False,
               "getConnectedSensors 含目标 mac", "未发现 OB6000C 设备")
        record(results, "连接后 getConnectedDevices 含该 mac", False,
               "getConnectedDevices 含目标 mac", "未发现 OB6000C 设备")
    else:
        target_mac = _mac_of_device(target)
        name = getattr(target, 'Name', '?')
        print(f"[扫描] 目标设备: {name} {target_mac}", flush=True)

        sensor = ctrl.requireSensor(target)
        if sensor is None:
            print("[连接] requireSensor 返回 None", flush=True)
            record(results, "连接后 getConnectedSensors 含该 mac", False,
                   "getConnectedSensors 含目标 mac", "requireSensor 返回 None")
            record(results, "连接后 getConnectedDevices 含该 mac", False,
                   "getConnectedDevices 含目标 mac", "requireSensor 返回 None")
        else:
            sensor.onStateChanged = lambda s, st: print(f"  [SensorProfile.onStateChanged] {s.BLEDevice.Name} -> {st}", flush=True)
            try:
                ok = sensor.connect()
            except Exception as e:
                ok = False
                print(f"[连接] SensorProfile.connect 抛异常 {type(e).__name__}: {e}", flush=True)
            state = sensor.deviceState
            print(f"[连接] SensorProfile.connect() -> {ok}, deviceState = {state}", flush=True)

            if not ok:
                print("[检查] 连接失败，跳过已连接集合检查", flush=True)
                record(results, "连接后 getConnectedSensors 含该 mac", False,
                       "getConnectedSensors 含目标 mac", f"connect 返回 {ok}")
                record(results, "连接后 getConnectedDevices 含该 mac", False,
                       "getConnectedDevices 含目标 mac", f"connect 返回 {ok}")
            else:
                time.sleep(1)  # 等状态/集合刷新

                # 检查1：getConnectedSensors 含该 mac
                try:
                    sensors = ctrl.getConnectedSensors() or []
                except Exception as e:
                    sensors = []
                    print(f"[检查1] SensorController.getConnectedSensors 抛异常 {type(e).__name__}: {e}", flush=True)
                sensor_macs = [_mac_of_sensor(s) for s in sensors]
                print(f"[检查1] getConnectedSensors 返回 {len(sensors)} 个, macs={sensor_macs}", flush=True)
                record(results, "连接后 getConnectedSensors 含该 mac", target_mac in sensor_macs,
                       f"getConnectedSensors 含 {target_mac}", f"macs={sensor_macs}")

                # 检查2：getConnectedDevices 含该 mac
                try:
                    conn_devices = ctrl.getConnectedDevices() or []
                except Exception as e:
                    conn_devices = []
                    print(f"[检查2] SensorController.getConnectedDevices 抛异常 {type(e).__name__}: {e}", flush=True)
                device_macs = [_mac_of_device(d) for d in conn_devices]
                print(f"[检查2] getConnectedDevices 返回 {len(conn_devices)} 个, macs={device_macs}", flush=True)
                record(results, "连接后 getConnectedDevices 含该 mac", target_mac in device_macs,
                       f"getConnectedDevices 含 {target_mac}", f"macs={device_macs}")

                # 断开
                try:
                    sensor.disconnect()
                except Exception as e:
                    print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)
                time.sleep(1)

                # 检查3：断开后 getConnectedSensors 不含该 mac
                try:
                    sensors_after = ctrl.getConnectedSensors() or []
                except Exception as e:
                    sensors_after = []
                    print(f"[检查3] getConnectedSensors 抛异常 {type(e).__name__}: {e}", flush=True)
                sensor_macs_after = [_mac_of_sensor(s) for s in sensors_after]
                print(f"[检查3] 断开后 getConnectedSensors macs={sensor_macs_after}", flush=True)
                record(results, "断开后 getConnectedSensors 不含该 mac", target_mac not in sensor_macs_after,
                       f"getConnectedSensors 不含 {target_mac}", f"macs={sensor_macs_after}")

                # 检查4：断开后 getConnectedDevices 不含该 mac
                try:
                    conn_devices_after = ctrl.getConnectedDevices() or []
                except Exception as e:
                    conn_devices_after = []
                    print(f"[检查4] getConnectedDevices 抛异常 {type(e).__name__}: {e}", flush=True)
                device_macs_after = [_mac_of_device(d) for d in conn_devices_after]
                print(f"[检查4] 断开后 getConnectedDevices macs={device_macs_after}", flush=True)
                record(results, "断开后 getConnectedDevices 不含该 mac", target_mac not in device_macs_after,
                       f"getConnectedDevices 不含 {target_mac}", f"macs={device_macs_after}")

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for name, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {name}（实际: {actual}）", flush=True)
        else:
            print(f"  [FAIL] {name}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()
