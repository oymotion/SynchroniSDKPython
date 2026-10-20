# -*- coding: utf-8 -*-
"""PARAM-FUNC-003：NTF_IMU 联动四个 NTF_GFORCE_*（不适用 gForceUltra）。

对应用例：04_参数.md -> PARAM-FUNC-003
可自动化：auto（能力判定后执行/跳过；设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 能力判定：DeviceInfo.ImuChannelCount 用于确认设备有 NTF_IMU 聚合流
  3) gForceUltra 最新固件（与 gForceMax 同源）无 NTF_GFORCE_ACC/GYRO/EULER/QUAT
     独立开关（getParam 返回 Error），acc/gyro/euler/quat 打包进 NTF_IMU 聚合流，
     故「NTF_IMU 联动四路 NTF_GFORCE_*」双向联动本用例不适用，记录"不适用，跳过"。

说明：
  旧固件存在四个 NTF_GFORCE_* 独立开关时才测双向联动；新固件已无独立开关，
  无主/子开关可联动，故本用例在 gForceUltra 恒为"不适用"。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：gForceUltra 上电、在范围内
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-003 NTF_IMU 联动四个 NTF_GFORCE_*", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：gForceUltra 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 gForceUltra 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含目标设备", f"匹配到 {name} {addr}")

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    print("\n[连接] SensorProfile.connect() ...", flush=True)
    try:
        ok = sensor.connect()
        connect_txt = f"返回 {ok}"
    except Exception as e:
        ok = None
        connect_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[连接] SensorProfile.connect() -> {connect_txt}  state={sensor.deviceState}", flush=True)
    record(results, "SensorProfile.connect 返回 True", ok is True,
           "connect() 返回 True", f"connect() -> {connect_txt}")

    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    ready = (sensor.deviceState == DeviceStateEx.Ready)
    record(results, "connect 后到达 Ready", ready, "deviceState==Ready", f"state={sensor.deviceState}")

    if not ready:
        print("[FAIL] 未到达 Ready，无法继续", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] SensorProfile.init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    # getDeviceInfo -> ImuChannelCount
    info = sensor.getDeviceInfo()
    record(results, "getDeviceInfo() 返回 DeviceInfo", info is not None,
           "getDeviceInfo() 返回 DeviceInfo（非 None）",
           f"返回 {type(info).__name__ if info is not None else None}")
    if info is None:
        print("[FAIL] getDeviceInfo() 返回 None，无法进行能力判定", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    try:
        imu_channels = int(getattr(info, 'ImuChannelCount', 0) or 0)
    except Exception as e:
        imu_channels = 0
        print(f"[info] 读取 ImuChannelCount 抛异常 {type(e).__name__}: {e}，按 0 处理", flush=True)
    print(f"\n[能力] DeviceInfo.ImuChannelCount = {imu_channels}", flush=True)

    # gForceUltra 最新固件无 NTF_GFORCE_* 独立开关，双向联动不适用
    record(results, "NTF_IMU 与四个 NTF_GFORCE_* 双向联动", None,
           "存在四个 NTF_GFORCE_* 独立开关时校验联动",
           f"无 NTF_GFORCE_* 独立开关（ImuChannelCount={imu_channels}），不适用")
    print("[SKIP] gForceUltra 无 NTF_GFORCE_* 独立开关，双向联动不适用，跳过", flush=True)

    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for rname, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {rname}（实际: {actual}）", flush=True)
        elif status == "SKIP":
            print(f"  [SKIP] {rname}（{actual}）", flush=True)
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status == "FAIL":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()
