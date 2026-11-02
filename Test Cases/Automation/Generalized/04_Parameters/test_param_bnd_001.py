# -*- coding: utf-8 -*-
"""PARAM-BND-001：setParam 非法 value 被拒绝，不崩溃。

对应用例：04_参数.md -> PARAM-BND-001
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 载入设备 spec，动态取代表键：spec["streams"] 中 supported=True 的流键
     （最多 3 个）+ spec["filters"] 的键（最多 2 个）
  3) 对每个代表键逐一传非法 value：
     - 空串 ""
     - 非 ON/OFF（"INVALID"）
     - 小写 "on" / "off"
     - None
  4) 校验：非法值被拒绝（返回以 "Error"/"抛异常"/"<" 开头，或抛异常但进程不崩溃）；
     若返回 "OK"（接受非法值）则判 FAIL（校验缺失）
  5) spec 无可测参数键时，记录"无可用参数键"（SKIP）

说明：
  本用例为输入校验边界；"抛异常但进程不崩溃"同样视为"被拒绝"（PASS）。
  若脚本进程崩溃则无法记录结论，属严重缺陷，需单独上报。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：已上电、在范围内
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

# 非法 value 列表：(显示名, 实际值)
INVALID_VALUES = [
    ("空串", ""),
    ("非ON/OFF", "INVALID"),
    ("小写on", "on"),
    ("小写off", "off"),
    ("None", None),
]


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-BND-001 setParam 非法 value 被拒绝，不崩溃", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：已上电、在范围内", flush=True)

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

    # 动态取代表键：spec["streams"] 中 supported=True 的流键（最多 3）+ filters 键（最多 2）
    stream_keys = [k for k, cap in (spec.get("streams") or {}).items()
                   if cap.get("supported") is True and (cap.get("channels") or 0) > 0][:3]
    filter_keys = list((spec.get("filters") or {}).keys())[:2]
    test_keys = stream_keys + filter_keys

    if not test_keys:
        print("[SKIP] 无可用参数键（spec streams 与 filters 均无可测键），跳过", flush=True)
        record(results, "setParam 非法 value 被拒绝", None,
               "存在可用参数键时校验非法 value 被拒绝", "spec 无可测参数键")
    else:
        # 逐 key × 逐非法 value 测试
        print("\n[边界] 逐 key 传非法 value ...", flush=True)
        for key in test_keys:
            for label, value in INVALID_VALUES:
                try:
                    r = sensor.setParam(key, value)
                except Exception as e:
                    r = f"抛异常 {type(e).__name__}: {e}"

                # 判定：被拒绝 = 返回 Error/抛异常/< 开头（不崩溃）
                if isinstance(r, str) and (r.startswith("Error") or r.startswith("抛异常") or r.startswith("<")):
                    ok = True
                elif r == "OK":
                    ok = False  # 接受了非法值，校验缺失
                else:
                    # 其它返回（如非 OK 非 Error 的字符串）
                    ok = False

                print(f"[边界] setParam({key!r}, {value!r}) -> {r!r}", flush=True)
                record(results, f"{key} 非法 value({label}) 被拒绝",
                       ok,
                       "返回以 'Error'/'抛异常'/'<' 开头（不崩溃），不返回 'OK'",
                       f"setParam({key!r}, {value!r})->{r!r}")

    # 清理：恢复正常状态
    for key in test_keys:
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
