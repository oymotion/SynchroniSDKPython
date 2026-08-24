# -*- coding: utf-8 -*-
"""PARAM-FUNC-010：DEBUG_BLE_DATA_PATH / DEBUG_LOG_PATH。

对应用例：04_参数.md -> PARAM-FUNC-010
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) DEBUG_LOG_PATH：
     - setParam("DEBUG_LOG_PATH", "True") 返回 "OK"，getParam 返回非空路径
     - setParam("DEBUG_LOG_PATH", "False") 返回 "OK"，getParam 返回 ""
  3) DEBUG_BLE_DATA_PATH：
     - setParam("DEBUG_BLE_DATA_PATH", "True") 返回 "OK"
     - setParam("DEBUG_BLE_DATA_PATH", "False") 返回 "OK"（清理）

说明：
  README：getParam("DEBUG_LOG_PATH") 返回当前 log 路径（"" when disabled）。
  README 未列出 getParam("DEBUG_BLE_DATA_PATH")，故该键只校验 setParam 返回
  "OK"，不判定读回。DEBUG_BLE_DATA_PATH=True 会在 disconnect 时导出 bin，
  故测试后立即设回 "False" 避免产生多余文件。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内
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


def _set(sensor, key, value):
    try:
        return sensor.setParam(key, value)
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def _get(sensor, key):
    try:
        return sensor.getParam(key)
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-010 DEBUG_BLE_DATA_PATH / DEBUG_LOG_PATH", flush=True)
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

    # DEBUG_LOG_PATH：开
    print("\n[参数] DEBUG_LOG_PATH 开启 ...", flush=True)
    r_on = _set(sensor, "DEBUG_LOG_PATH", "True")
    log_on = _get(sensor, "DEBUG_LOG_PATH")
    print(f"[参数] setParam('DEBUG_LOG_PATH','True')->{r_on!r}  getParam={log_on!r}", flush=True)
    ok_log_on = (r_on == "OK") and isinstance(log_on, str) and len(log_on) > 0
    record(results, "DEBUG_LOG_PATH 开启返回 OK 且路径非空", ok_log_on,
           "setParam 返回 'OK' 且 getParam 返回非空路径",
           f"setParam->{r_on!r} getParam->{log_on!r}")

    # DEBUG_LOG_PATH：关
    print("[参数] DEBUG_LOG_PATH 关闭 ...", flush=True)
    r_off = _set(sensor, "DEBUG_LOG_PATH", "False")
    log_off = _get(sensor, "DEBUG_LOG_PATH")
    print(f"[参数] setParam('DEBUG_LOG_PATH','False')->{r_off!r}  getParam={log_off!r}", flush=True)
    ok_log_off = (r_off == "OK") and isinstance(log_off, str) and log_off == ""
    record(results, "DEBUG_LOG_PATH 关闭返回 OK 且路径为空", ok_log_off,
           "setParam 返回 'OK' 且 getParam 返回 ''",
           f"setParam->{r_off!r} getParam->{log_off!r}")

    # DEBUG_BLE_DATA_PATH：开/关（README 未列 getParam，只校验 setParam 返回）
    print("[参数] DEBUG_BLE_DATA_PATH 开/关 ...", flush=True)
    b_on = _set(sensor, "DEBUG_BLE_DATA_PATH", "True")
    print(f"[参数] setParam('DEBUG_BLE_DATA_PATH','True')->{b_on!r}", flush=True)
    b_off = _set(sensor, "DEBUG_BLE_DATA_PATH", "False")
    print(f"[参数] setParam('DEBUG_BLE_DATA_PATH','False')->{b_off!r}", flush=True)
    ok_ble = (b_on == "OK") and (b_off == "OK")
    record(results, "DEBUG_BLE_DATA_PATH 开/关返回 OK", ok_ble,
           "setParam('True'/'False') 均返回 'OK'",
           f"True->{b_on!r} False->{b_off!r}")

    # 清理：确保 log 关闭
    try:
        sensor.setParam("DEBUG_LOG_PATH", "False")
    except Exception:
        pass
    try:
        sensor.setParam("DEBUG_BLE_DATA_PATH", "False")
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
