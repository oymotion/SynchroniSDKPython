# -*- coding: utf-8 -*-
"""PARAM-FUNC-003：NTF_IMU 联动四个 NTF_GFORCE_*。

对应用例：04_参数.md -> PARAM-FUNC-003
可自动化：auto（能力判定后执行/跳过；设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 能力判定：DeviceInfo.ImuChannelCount>0 才测（新 EMG 设备才有聚合流主开关）
  3) 主方向：setParam("NTF_IMU","ON") 后读 getParam("NTF")，四个 NTF_GFORCE_* 应全 ON；
     setParam("NTF_IMU","OFF") 后四个应全 OFF
  4) 反方向：单独切一个 NTF_GFORCE_*（如 ACC）为 OFF，读 NTF_IMU 状态；
     再切回 ON，读 NTF_IMU 状态

说明：
  README：NTF_IMU 是四个 NTF_GFORCE_* 的主开关，切 IMU 会更新四路，
  切任意一路会更新聚合的 NTF_IMU 状态。
  本用例把"一致"定义为：NTF_IMU 的 ON 等价于"四个 NTF_GFORCE_* 全 ON"，
  即四路不全 ON 时 NTF_IMU 应为 OFF。这样可证伪且覆盖双向联动。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内（实测 ImuChannelCount=13）
"""

import os
import re
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match

GFORCE_KEYS = ["NTF_GFORCE_ACC", "NTF_GFORCE_GYRO", "NTF_GFORCE_EULER", "NTF_GFORCE_QUAT"]


def _parse_pipe(s):
    out = {}
    if not s:
        return out
    parts = s.split("|")
    for i in range(0, len(parts) - 1, 2):
        out[parts[i]] = parts[i + 1]
    return out


def _get_ntf(sensor):
    return _parse_pipe(sensor.getParam("NTF"))


def _gforce_states(ntf):
    return {k: ntf.get(k, "<缺失>") for k in GFORCE_KEYS}


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-003 NTF_IMU 联动四个 NTF_GFORCE_*", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，"
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

    if imu_channels <= 0:
        record(results, "NTF_IMU 与四个 NTF_GFORCE_* 双向联动", None,
               "ImuChannelCount>0 时校验联动", "ImuChannelCount==0，无聚合流主开关")
        print("[SKIP] ImuChannelCount==0，设备无 NTF_IMU 聚合流，跳过联动校验", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    # 主方向：IMU ON -> 四 GFORCE 全 ON
    print("\n[联动] 主方向：NTF_IMU ON 后检查四路 GFORCE ...", flush=True)
    try:
        r = sensor.setParam("NTF_IMU", "ON")
    except Exception as e:
        r = f"抛异常 {type(e).__name__}: {e}"
    ntf1 = _get_ntf(sensor)
    g1 = _gforce_states(ntf1)
    all_on_1 = all(v == "ON" for v in g1.values())
    print(f"[联动] setParam('NTF_IMU','ON')->{r!r}，四路 GFORCE={g1}", flush=True)
    record(results, "NTF_IMU ON 后四个 NTF_GFORCE_* 全 ON",
           (r == "OK" and all_on_1),
           "setParam 返回 'OK' 且四路 GFORCE 全 ON",
           f"setParam->{r!r} 四路={g1}")

    # 反方向：切 ACC OFF -> NTF_IMU 应变 OFF（四路不全 ON）
    print("[联动] 反方向：单独切 NTF_GFORCE_ACC 为 OFF，检查 NTF_IMU ...", flush=True)
    try:
        r2 = sensor.setParam("NTF_GFORCE_ACC", "OFF")
    except Exception as e:
        r2 = f"抛异常 {type(e).__name__}: {e}"
    ntf2 = _get_ntf(sensor)
    g2 = _gforce_states(ntf2)
    imu2 = ntf2.get("NTF_IMU", "<缺失>")
    all_on_2 = all(v == "ON" for v in g2.values())
    expect_imu2_off = (imu2 == "OFF")
    print(f"[联动] setParam('NTF_GFORCE_ACC','OFF')->{r2!r}，四路={g2}，NTF_IMU={imu2}", flush=True)
    record(results, "单独切一路 GFORCE OFF 后 NTF_IMU 变为 OFF",
           (r2 == "OK" and not all_on_2 and expect_imu2_off),
           "四路不全 ON 时 NTF_IMU 应为 OFF",
           f"setParam->{r2!r} 四路={g2} NTF_IMU={imu2}")

    # 反方向：切 ACC 回 ON -> NTF_IMU 应变回 ON（四路全 ON）
    print("[联动] 反方向：切 NTF_GFORCE_ACC 回 ON，检查 NTF_IMU ...", flush=True)
    try:
        r3 = sensor.setParam("NTF_GFORCE_ACC", "ON")
    except Exception as e:
        r3 = f"抛异常 {type(e).__name__}: {e}"
    ntf3 = _get_ntf(sensor)
    g3 = _gforce_states(ntf3)
    imu3 = ntf3.get("NTF_IMU", "<缺失>")
    all_on_3 = all(v == "ON" for v in g3.values())
    expect_imu3_on = (imu3 == "ON")
    print(f"[联动] setParam('NTF_GFORCE_ACC','ON')->{r3!r}，四路={g3}，NTF_IMU={imu3}", flush=True)
    record(results, "单独切一路 GFORCE 回 ON 后 NTF_IMU 变为 ON",
           (r3 == "OK" and all_on_3 and expect_imu3_on),
           "四路全 ON 时 NTF_IMU 应为 ON",
           f"setParam->{r3!r} 四路={g3} NTF_IMU={imu3}")

    # 主方向：IMU OFF -> 四 GFORCE 全 OFF
    print("[联动] 主方向：NTF_IMU OFF 后检查四路 GFORCE ...", flush=True)
    try:
        r4 = sensor.setParam("NTF_IMU", "OFF")
    except Exception as e:
        r4 = f"抛异常 {type(e).__name__}: {e}"
    ntf4 = _get_ntf(sensor)
    g4 = _gforce_states(ntf4)
    all_off_4 = all(v == "OFF" for v in g4.values())
    print(f"[联动] setParam('NTF_IMU','OFF')->{r4!r}，四路 GFORCE={g4}", flush=True)
    record(results, "NTF_IMU OFF 后四个 NTF_GFORCE_* 全 OFF",
           (r4 == "OK" and all_off_4),
           "setParam 返回 'OK' 且四路 GFORCE 全 OFF",
           f"setParam->{r4!r} 四路={g4}")

    # 清理
    try:
        sensor.setParam("NTF_IMU", "OFF")
    except Exception:
        pass

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
