# -*- coding: utf-8 -*-
"""PARAM-FUNC-004：OYWW1100 EMG_SAMPLE_RATE 正反向（500/1000 设置 + 非法值 750）。

对应用例：04_参数.md -> PARAM-FUNC-004
可自动化：auto（设备上电、在范围内为运行前置）

流程（正向）：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 能力判定：getParam("EMG_SAMPLE_RATE") 返回以 "Error" 开头 → 不支持，SKIP
  3) 默认值校验：初始 getParam("EMG_SAMPLE_RATE") 应为 "1000"
  4) 可选列表校验：getParam("EMG_SAMPLE_RATE_LIST") 非 Error 时校验含 500/1000
  5) 逐一 setParam("EMG_SAMPLE_RATE", value)，getParam("EMG_SAMPLE_RATE") 核对

流程（反向）：
  6) setParam("EMG_SAMPLE_RATE", "750") 应返回 Error（列表外值）
  7) setParam("EMG_SAMPLE_RATE", 空串) 应返回 Error

说明：
  OYWW1100 腕带 EMG 采样率支持 500Hz 与 1000Hz 两档，默认 1000Hz。
  本脚本合并正反向：正向校验默认值 + 两档逐一设置读回一致；反向校验
  列表外值（750）与空串被拒绝（返回 Error 或抛异常）。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内
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

EXPECTED_RATES = ["500", "1000"]  # OYWW1100 EMG 采样率两档
DEFAULT_RATE = "1000"             # 默认采样率
INVALID_VALUES = ["750", ""]      # 反向：列表外值 + 空串


def _read_param(sensor, key):
    """读取参数，异常时返回带前缀的字符串，避免中断。"""
    try:
        return sensor.getParam(key)
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def _is_error(txt):
    """参数返回是否为 Error（字符串以 Error 开头）。"""
    return not isinstance(txt, str) or txt.startswith("Error")


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-004 OYWW1100 EMG_SAMPLE_RATE 正反向（500/1000 + 非法值）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    print(f"[扫描] 扫描到 {len(devices) if devices else 0} 台设备:", flush=True)
    if devices:
        for d in devices:
            n = getattr(d, 'Name', '?')
            a = getattr(d, 'Address', '?')
            print(f"  {n} {a} identity={common._identity_of(n)}", flush=True)

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

    # requireSensor
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    # connect
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

    # 到达 Ready
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

    # init
    print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] SensorProfile.init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    # 上电默认值 + 能力判定：未做任何 setParam 前，首次读取 EMG 采样率
    cur_init = _read_param(sensor, "EMG_SAMPLE_RATE")
    list_str = _read_param(sensor, "EMG_SAMPLE_RATE_LIST")
    print(f"\n[上电默认值/能力] getParam('EMG_SAMPLE_RATE') = {cur_init!r}（未 set 前首次读取）", flush=True)
    print(f"[能力] getParam('EMG_SAMPLE_RATE_LIST') = {list_str!r}", flush=True)

    if _is_error(cur_init) and _is_error(list_str):
        record(results, "EMG_SAMPLE_RATE 能力判定", None,
               "getParam('EMG_SAMPLE_RATE')/LIST 至少一个返回有效值",
               f"EMG_SAMPLE_RATE={cur_init!r} LIST={list_str!r}，设备未上报能力")
        print("[SKIP] 设备未上报 EMG 采样率能力，跳过", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    # 确定可选档位：优先用 LIST，LIST 无效时回退到硬编码档位
    if not _is_error(list_str):
        rates = [r.strip() for r in list_str.split("|") if r.strip()]
    else:
        rates = EXPECTED_RATES
    print(f"[参数] 支持采样率: {rates}", flush=True)

    # 上电默认值校验（未 set 前首次读取）
    record(results, "EMG_SAMPLE_RATE 上电默认值 = 1000",
           str(cur_init) == DEFAULT_RATE,
           f"首次(未 set 前) getParam('EMG_SAMPLE_RATE') == '{DEFAULT_RATE}'",
           f"getParam('EMG_SAMPLE_RATE') = {cur_init!r}")

    # 可选列表校验（仅当 LIST 可用时）
    if not _is_error(list_str):
        missing = [r for r in EXPECTED_RATES if r not in rates]
        list_ok = (len(missing) == 0)
        record(results, "EMG_SAMPLE_RATE_LIST 含 500 与 1000",
               list_ok,
               f"EMG_SAMPLE_RATE_LIST 含 {EXPECTED_RATES}",
               f"列表={rates}，缺失={missing if missing else '无'}")
        if not list_ok:
            print(f"[FAIL] EMG_SAMPLE_RATE_LIST 缺少期望采样率: {missing}", flush=True)
            try:
                sensor.disconnect()
            except Exception:
                pass
            print("\n结论: FAIL", flush=True)
            ctrl.terminate()
            return

    # ---- 正向：逐一设置读回一致 ----
    for rate in EXPECTED_RATES:
        print(f"\n[测试] EMG_SAMPLE_RATE = {rate}", flush=True)

        # setParam 单独处理（返回值为 "OK"/"Error" 字符串）
        try:
            set_ret = sensor.setParam("EMG_SAMPLE_RATE", rate)
            set_txt = f"返回 {set_ret!r}"
        except Exception as e:
            set_ret = None
            set_txt = f"抛异常 {type(e).__name__}: {e}"
        print(f"[setParam] EMG_SAMPLE_RATE={rate} -> {set_txt}", flush=True)

        cur = _read_param(sensor, "EMG_SAMPLE_RATE")
        print(f"[getParam] EMG_SAMPLE_RATE = {cur!r}", flush=True)

        ok_val = (set_ret == "OK") and (str(cur) == rate)
        record(results, f"EMG_SAMPLE_RATE={rate} 设置并读回一致",
               ok_val,
               f"setParam 返回 OK，getParam 返回 {rate}",
               f"setParam={set_txt}, getParam={cur!r}")

    # ---- 反向：列表外值与空串报错 ----
    print(f"\n[反向] 不支持的值: {INVALID_VALUES}", flush=True)
    for val in INVALID_VALUES:
        label = f"EMG_SAMPLE_RATE={val!r}" if val != "" else "EMG_SAMPLE_RATE=空串"
        print(f"\n[setParam] {label} ...", flush=True)
        try:
            r = sensor.setParam("EMG_SAMPLE_RATE", val)
            ret_txt = f"返回 {r!r}"
        except Exception as e:
            r = f"抛异常 {type(e).__name__}: {e}"
            ret_txt = f"抛异常 {type(e).__name__}: {e}"
        print(f"[setParam] {label} -> {ret_txt}", flush=True)

        is_error = (isinstance(r, str) and r.startswith("Error")) or ("异常" in ret_txt)
        record(results, f"setParam({label}) 返回 Error",
               is_error,
               f"setParam({label}) 返回 Error 或抛异常",
               ret_txt)

    # 清理
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 汇总 ----
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
