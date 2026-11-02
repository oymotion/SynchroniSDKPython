# -*- coding: utf-8 -*-
"""PARAM-FUNC-006：getParam EEG_SAMPLE_RATE / EEG_SAMPLE_RATE_LIST（spec 驱动）。

对应用例：04_参数.md -> PARAM-FUNC-006
可自动化：auto（能力判定后执行/跳过）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 载入设备 spec；spec["sample_rates"].get("EEG_SAMPLE_RATE") 为 None
     → 设备无 EEG_SAMPLE_RATE 参数，SKIP
  3) 查询 getParam("EEG_SAMPLE_RATE")（应为数字字符串，且属于 spec rates）
     与 getParam("EEG_SAMPLE_RATE_LIST")（应为非空 pipe 列表，且与 spec rates 一致）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：上电、在范围内
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


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-006 getParam EEG_SAMPLE_RATE / EEG_SAMPLE_RATE_LIST（spec 驱动）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备已【开机】且在范围内，"
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

    spec = _load_spec(target)
    if spec is None:
        record(results, "载入设备 spec", False, "能从广播名提取 identity 并载入对应 spec", "spec 载入失败")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "载入设备 spec", True, "能从广播名提取 identity 并载入对应 spec", f"model={spec.get('model')}")

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

    # 能力判定（spec 驱动）：EEG_SAMPLE_RATE 参数存在才继续
    sr = (spec.get("sample_rates") or {}).get("EEG_SAMPLE_RATE")
    if sr is None:
        record(results, "设备无 EEG_SAMPLE_RATE 参数", None,
               "spec[\"sample_rates\"].get('EEG_SAMPLE_RATE') 存在",
               f"EEG_SAMPLE_RATE spec={sr!r}，设备无该参数")
        print("[SKIP] 设备无 EEG_SAMPLE_RATE 参数，跳过", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    spec_rates = sr["rates"]
    print(f"[能力] EEG 支持采样率: {spec_rates}", flush=True)

    # 读两个查询
    try:
        list_txt = sensor.getParam("EEG_SAMPLE_RATE_LIST")
    except Exception as e:
        list_txt = f"抛异常 {type(e).__name__}: {e}"
    try:
        rate_txt = sensor.getParam("EEG_SAMPLE_RATE")
    except Exception as e:
        rate_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"\n[查询] getParam('EEG_SAMPLE_RATE_LIST') = {list_txt!r}", flush=True)
    print(f"[查询] getParam('EEG_SAMPLE_RATE') = {rate_txt!r}", flush=True)

    # RATE：应为数字字符串，且属于 spec 档位
    ok_rate_digit = isinstance(rate_txt, str) and re.fullmatch(r"\d+", rate_txt) is not None
    record(results, "getParam('EEG_SAMPLE_RATE') 返回数字字符串", ok_rate_digit,
           "返回纯数字字符串（如 '250'）", f"getParam('EEG_SAMPLE_RATE')={rate_txt!r}")

    ok_rate_in_rates = isinstance(rate_txt, str) and rate_txt in spec_rates
    record(results, "getParam('EEG_SAMPLE_RATE') 值属于 spec 档位", ok_rate_in_rates,
           f"值 ∈ {spec_rates}", f"getParam('EEG_SAMPLE_RATE')={rate_txt!r}")

    # LIST：应为非空 pipe 列表，且与 spec 档位一致
    list_rates = [x.strip() for x in list_txt.split("|") if x.strip()] if isinstance(list_txt, str) else []
    ok_list_nonempty = len(list_rates) > 0
    record(results, "getParam('EEG_SAMPLE_RATE_LIST') 返回非空列表", ok_list_nonempty,
           "返回 pipe 分隔的非空采样率列表", f"getParam('EEG_SAMPLE_RATE_LIST')={list_txt!r}")

    ok_list_match = (sorted(list_rates) == sorted(spec_rates))
    record(results, "getParam('EEG_SAMPLE_RATE_LIST') 与 spec 档位一致", ok_list_match,
           f"pipe 列表 == {spec_rates}", f"列表={list_rates}")

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
