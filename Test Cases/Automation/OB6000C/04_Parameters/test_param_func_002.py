# -*- coding: utf-8 -*-
"""PARAM-FUNC-002：FILTER_* 键 ON/OFF 返回 OK，getParam 一致。

对应用例：04_参数.md -> PARAM-FUNC-002
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 对 FILTER_50HZ / FILTER_60HZ / FILTER_HPF / FILTER_LPF 逐一 ON/OFF
  3) 校验返回 "OK"，且 getParam("FILTER") 中对应键状态一致；
     返回以 "Error" 开头则记录"设备不支持，跳过"

说明：
  getParam("FILTER") 返回 pipe 分隔串，如
  "FILTER_50HZ|ON|FILTER_60HZ|ON|FILTER_HPF|ON|FILTER_LPF|ON"。
  README：setParam 成功返回 "OK"。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
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


def _parse_pipe(s):
    out = {}
    if not s:
        return out
    parts = s.split("|")
    for i in range(0, len(parts) - 1, 2):
        out[parts[i]] = parts[i + 1]
    return out


FILTER_KEYS = ["FILTER_50HZ", "FILTER_60HZ", "FILTER_HPF", "FILTER_LPF"]


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-002 FILTER_* 键 ON/OFF 返回 OK，getParam 一致", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

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

    # 逐个 FILTER 键测试
    print("\n[参数] 逐个 FILTER_* 键 ON/OFF 并核对 getParam('FILTER') ...", flush=True)
    for key in FILTER_KEYS:
        try:
            r_on = sensor.setParam(key, "ON")
        except Exception as e:
            r_on = f"抛异常 {type(e).__name__}: {e}"
        filt_on = _parse_pipe(sensor.getParam("FILTER"))
        v_on = filt_on.get(key, "<缺失>")

        try:
            r_off = sensor.setParam(key, "OFF")
        except Exception as e:
            r_off = f"抛异常 {type(e).__name__}: {e}"
        filt_off = _parse_pipe(sensor.getParam("FILTER"))
        v_off = filt_off.get(key, "<缺失>")

        # 返回以 "Error" 开头 → 不支持
        if isinstance(r_on, str) and r_on.startswith("Error"):
            print(f"[SKIP] {key}: setParam 返回 {r_on!r}，设备不支持", flush=True)
            record(results, f"{key} ON/OFF 返回 OK 且 getParam 一致", None,
                   f"setParam ON/OFF 返回 'OK'，getParam('FILTER') 中 {key} 为 ON/OFF",
                   f"setParam 返回 {r_on!r}（不支持）")
            continue

        ok_on = (r_on == "OK")
        ok_off = (r_off == "OK")
        match_on = (v_on == "ON")
        match_off = (v_off == "OFF")
        all_ok = ok_on and ok_off and match_on and match_off

        actual = (f"ON->{r_on!r}/getParam={v_on}，OFF->{r_off!r}/getParam={v_off}")
        print(f"[参数] {key}: ON->{r_on!r}(getParam={v_on})，OFF->{r_off!r}(getParam={v_off})", flush=True)
        record(results, f"{key} ON/OFF 返回 OK 且 getParam 一致", all_ok,
               f"setParam ON/OFF 均返回 'OK'，getParam('FILTER') 中 {key} 为 ON/OFF", actual)

    # 清理：关闭所有 filter
    for key in FILTER_KEYS:
        try:
            sensor.setParam(key, "OFF")
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
