# -*- coding: utf-8 -*-
"""CTRL-FUNC-008：getSensor(mac) 命中/未命中。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-008
可自动化：semi-auto（运行前需人工确认设备在范围内）

前置条件（运行前请人工准备好）：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 人工确认待测设备已开机 -> 按回车
  2) 检查 SensorController.isEnable == True
  3) scan 发现 OB6000C -> requireSensor 创建 profile
  4) getSensor(正确 mac) 返回该 profile（命中）
  5) getSensor(错误 mac) 返回 None（未命中）
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record

BAD_MAC = "00:00:00:00:00:00"  # 用于"未命中"的错误 mac


def _find_ob6000c(devices):
    for d in devices or []:
        name = getattr(d, 'Name', None) or ''
        if name.upper().startswith('OB6000C'):
            return d
    return None


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-008 getSensor(mac) 命中/未命中", flush=True)
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

    created_sensor = None
    correct_mac = None

    if target is None:
        print("[检查1] 未发现 OB6000C 设备", flush=True)
        record(results, "requireSensor(有效设备) 返回 SensorProfile", False,
               "requireSensor(有效 BLEDevice) 返回 SensorProfile", "未发现 OB6000C 设备")
        print("[检查2] 未发现 OB6000C 设备，无法验证命中", flush=True)
        record(results, "getSensor(正确 mac) 返回 profile", False,
               "getSensor(正确 mac) 返回 SensorProfile", "未发现 OB6000C 设备")
    else:
        name = getattr(target, 'Name', '?')
        correct_mac = getattr(target, 'Address', None)
        print(f"[扫描] 目标设备: {name} {correct_mac}", flush=True)

        # 检查1：requireSensor 创建 profile
        try:
            created_sensor = ctrl.requireSensor(target)
        except Exception as e:
            created_sensor = None
            print(f"[检查1] SensorController.requireSensor 抛异常 {type(e).__name__}: {e}", flush=True)
        is_profile = isinstance(created_sensor, SensorProfile)
        print(f"[检查1] requireSensor -> {type(created_sensor).__name__}, 是 SensorProfile = {is_profile}", flush=True)
        record(results, "requireSensor(有效设备) 返回 SensorProfile", is_profile,
               "requireSensor(有效 BLEDevice) 返回 SensorProfile", f"返回 {type(created_sensor).__name__}")

        # 检查2：getSensor(正确 mac) 命中
        if not correct_mac:
            print("[检查2] 目标设备 Address 为空，无法验证命中", flush=True)
            record(results, "getSensor(正确 mac) 返回 profile", False,
                   "getSensor(正确 mac) 返回 SensorProfile", "Address 为空")
        else:
            try:
                sensor_by_mac = ctrl.getSensor(correct_mac)
            except Exception as e:
                sensor_by_mac = None
                print(f"[检查2] SensorController.getSensor 抛异常 {type(e).__name__}: {e}", flush=True)
            is_profile_hit = isinstance(sensor_by_mac, SensorProfile)
            same = (sensor_by_mac is created_sensor) if (is_profile_hit and created_sensor is not None) else None
            print(f"[检查2] getSensor({correct_mac}) -> {type(sensor_by_mac).__name__}, "
                  f"是 SensorProfile = {is_profile_hit}, 同一对象 = {same}", flush=True)
            record(results, "getSensor(正确 mac) 返回 profile", is_profile_hit,
                   "getSensor(正确 mac) 返回 SensorProfile", f"返回 {type(sensor_by_mac).__name__}")

    # 检查3：getSensor(错误 mac) 未命中
    try:
        sensor_bad = ctrl.getSensor(BAD_MAC)
    except Exception as e:
        sensor_bad = None
        print(f"[检查3] SensorController.getSensor({BAD_MAC}) 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[检查3] getSensor({BAD_MAC}) -> {sensor_bad!r}", flush=True)
    record(results, "getSensor(错误 mac) 返回 None", sensor_bad is None,
           "getSensor(错误 mac) 返回 None", f"返回 {sensor_bad!r}")

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
