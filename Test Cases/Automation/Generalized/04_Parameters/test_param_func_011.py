# -*- coding: utf-8 -*-
"""PARAM-FUNC-011：getParam 采样率参数与 legacy 参数（spec 能力门控）。

对应用例：04_参数.md -> PARAM-FUNC-011
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 载入设备 spec，遍历 spec["sample_rates"] 的键
  3) 对每个参数键 k 构造 (k, k + "_LIST") 查询对，逐对 getParam：
     - 硬断言：每次查询返回 str（不崩溃），且不以 "抛异常" / "<" 开头
     - 返回形态（有效值 / Error 开头 / 空列表）另作 informational（None）记录
  4) EMG_RESOLUTION / EMG_RESOLUTION_LIST 为 legacy 参数（不在 spec sample_rates），
     额外作为 informational（None）查询保留，不硬断言

说明：
  能力来源只认 spec["sample_rates"]，不写死具体型号/档位：spec 里有 IMU/PPG 等
  采样率参数键才有对应键可测，没有就不测。getParam 返回 str（不崩溃）是唯一硬
  断言；"Error 开头" 属于查询接口正确返回的合法形态，不判 FAIL。SDK 1.4.3 出错
  可能返回以 "<" 开头的字符串或抛异常，均视为不满足"不崩溃"。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：已上电、在范围内
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


def _load_spec(target):
    identity = common.identity_of_device(target)
    if identity is None:
        print("[FAIL] 无法从广播名提取 identity，无法载入设备 spec", flush=True)
        return None
    try:
        return common.spec_for_identity(identity)
    except Exception as e:
        print(f"[FAIL] 载入设备 spec 失败（identity={identity}）: {type(e).__name__}: {e}", flush=True)
        return None


def _get(sensor, key):
    try:
        return sensor.getParam(key)
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-011 getParam 采样率参数与 legacy 参数（spec 能力门控）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：已上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备已【开机】且在范围内，"
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

    spec = _load_spec(target)
    if spec is None:
        record(results, "载入设备 spec", False, "能从广播名提取 identity 并载入对应 spec", "spec 载入失败")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "载入设备 spec", True, "能从广播名提取 identity 并载入对应 spec", f"model={spec.get('model')}")

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

    # 逐一查询（能力门控，来源 spec["sample_rates"]，不写死具体键）
    sample_rates = spec.get("sample_rates") or {}
    for k in sample_rates:
        single_key = k
        list_key = k + "_LIST"
        v_single = _get(sensor, single_key)
        v_list = _get(sensor, list_key)
        print(f"\n[getParam] {single_key} = {v_single!r}", flush=True)
        print(f"[getParam] {list_key} = {v_list!r}", flush=True)

        single_ok = isinstance(v_single, str) and not v_single.startswith("抛异常") and not v_single.startswith("<")
        list_ok = isinstance(v_list, str) and not v_list.startswith("抛异常") and not v_list.startswith("<")

        record(results, f"getParam('{single_key}') 不崩溃", single_ok,
               f"getParam('{single_key}') 返回 str（不崩溃）",
               f"{single_key}={v_single!r}")
        record(results, f"getParam('{list_key}') 不崩溃", list_ok,
               f"getParam('{list_key}') 返回 str（不崩溃）",
               f"{list_key}={v_list!r}")

        # informational：能力形态（有效值 / Error 开头 / 空），供人工与后续校准
        record(results, f"{single_key} 能力形态（informational）", None,
               "有效值或 Error/空 均为合法形态",
               f"{single_key}={v_single!r} {list_key}={v_list!r}")

    # legacy 参数（不在 spec sample_rates 中），仅 informational 查询，不硬断言
    for legacy_key in ("EMG_RESOLUTION", "EMG_RESOLUTION_LIST"):
        v = _get(sensor, legacy_key)
        print(f"\n[getParam] {legacy_key} = {v!r}", flush=True)
        record(results, f"{legacy_key} 能力形态（informational）", None,
               "legacy 参数，仅记录返回形态",
               f"{legacy_key}={v!r}")

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
