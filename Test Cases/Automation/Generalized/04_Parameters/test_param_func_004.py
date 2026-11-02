# -*- coding: utf-8 -*-
"""PARAM-FUNC-004：EMG_SAMPLE_RATE 正反向（spec 驱动）。

对应用例：04_参数.md -> PARAM-FUNC-004
可自动化：auto（设备上电、在范围内为运行前置）

流程（正向）：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 载入设备 spec；spec["sample_rates"].get("EMG_SAMPLE_RATE") 为 None 或
     settable != True → 设备无 EMG_SAMPLE_RATE 可设参数，SKIP
  3) 默认值校验：初始 getParam("EMG_SAMPLE_RATE") 应为 spec 的 default
  4) 逐一 setParam("EMG_SAMPLE_RATE", value)，getParam("EMG_SAMPLE_RATE") 核对（档位取自 spec rates）

流程（反向）：
  5) 动态构造非法值（一个不在 rates 里的值 + 空串），逐一 setParam 校验被拒绝
     （返回 Error / 抛异常 / 以 '<' 开头）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：上电、在范围内
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


def _read_param(sensor, key):
    """读取参数，异常时返回带前缀的字符串，避免中断。"""
    try:
        return sensor.getParam(key)
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def _is_rejected(txt):
    """参数返回是否表示「被拒绝」：Error 开头 / 抛异常开头 / '<' 开头。"""
    if not isinstance(txt, str):
        return False
    return txt.startswith("Error") or txt.startswith("抛异常") or txt.startswith("<")


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-004 EMG_SAMPLE_RATE 正反向（spec 驱动）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：上电、在范围内", flush=True)

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

    # 能力判定（spec 驱动）：EMG_SAMPLE_RATE 可设才继续
    sr = (spec.get("sample_rates") or {}).get("EMG_SAMPLE_RATE")
    if sr is None or sr.get("settable") is not True:
        record(results, "设备无 EMG_SAMPLE_RATE 可设参数", None,
               "spec[\"sample_rates\"].get('EMG_SAMPLE_RATE') 存在且 settable=True",
               f"EMG_SAMPLE_RATE spec={sr!r}，设备不支持该可设参数")
        print("[SKIP] 设备无 EMG_SAMPLE_RATE 可设参数，跳过正反向", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    rates = sr["rates"]
    default = sr["default"]
    print(f"[参数] EMG 支持采样率: {rates}，默认: {default}", flush=True)

    # ---- 正向：上电默认值校验（未 set 前首次读取）----
    cur_init = _read_param(sensor, "EMG_SAMPLE_RATE")
    print(f"[上电默认值] getParam('EMG_SAMPLE_RATE') = {cur_init!r}（未 set 前首次读取）", flush=True)
    record(results, "EMG_SAMPLE_RATE 上电默认值 = spec default",
           str(cur_init) == str(default),
           f"首次(未 set 前) getParam('EMG_SAMPLE_RATE') == '{default}'",
           f"getParam('EMG_SAMPLE_RATE') = {cur_init!r}")

    # ---- 正向：逐一设置读回一致 ----
    for rate in rates:
        print(f"\n[测试] EMG_SAMPLE_RATE = {rate}", flush=True)

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

    # ---- 反向：动态构造非法值（不在 rates 里的值 + 空串）----
    invalid_value = "999999"
    while invalid_value in rates:
        invalid_value = str(int(invalid_value) + 1)
    invalid_values = [invalid_value, ""]
    print(f"\n[反向] 不支持的值: {invalid_values}", flush=True)
    for val in invalid_values:
        label = f"EMG_SAMPLE_RATE={val!r}" if val != "" else "EMG_SAMPLE_RATE=空串"
        print(f"\n[setParam] {label} ...", flush=True)
        try:
            r = sensor.setParam("EMG_SAMPLE_RATE", val)
            ret_txt = f"返回 {r!r}"
        except Exception as e:
            r = f"抛异常 {type(e).__name__}: {e}"
            ret_txt = f"抛异常 {type(e).__name__}: {e}"
        print(f"[setParam] {label} -> {ret_txt}", flush=True)

        rejected = _is_rejected(r)
        record(results, f"setParam({label}) 被拒绝",
               rejected,
               f"setParam({label}) 返回 Error / 抛异常 / 以 '<' 开头",
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
