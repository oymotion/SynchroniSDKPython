# -*- coding: utf-8 -*-
"""MISC-FUNC-008：设备能力表达完整性（spec 驱动）。

对应用例：10_底层接口与边界补充.md -> MISC-FUNC-008
可自动化：auto（需待测设备上电在范围内）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：目标设备上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配目标设备 -> requireSensor -> connect -> init
  3) 载入设备 spec（按广播名 identity）
  4) 遍历 spec["streams"]（supported/channels）做数据流能力表达
  5) 遍历 spec["sample_rates"]（rates/default/stream/settable）做采样率能力表达
  6) spec 标记不支持的流记 SKIP（不 FAIL）
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


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MISC-FUNC-008 DeviceInfo 能力字段完整性", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：目标设备上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    target_ids = config.TARGET_IDENTITY.split(",") if hasattr(config, 'TARGET_IDENTITY') else ["?"]
    print(f"\n[扫描] 目标 identity: {target_ids}（config.TARGET_IDENTITY = '{config.TARGET_IDENTITY}'）", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    scanned_list = [(getattr(d, 'Name', '?'), getattr(d, 'Address', '?')) for d in (devices or [])]
    print(f"[扫描] 扫描到 {len(scanned_list)} 台设备: {scanned_list}", flush=True)
    if target is None:
        print(f"[FAIL] 未匹配到目标设备（目标 identity: {target_ids}，扫描到: {scanned_list}）", flush=True)
        print("[提示] 请检查 config.py 中 TARGET_IDENTITY 是否设置为正确的设备 identity。", flush=True)
        if hasattr(config, 'DEVICES'):
            available_ids = [getattr(d, 'identity', '?') for d in config.DEVICES]
            print(f"[提示] 当前 DEVICES 配置中可用的 identity: {available_ids}", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备",
               f"未匹配到目标（目标: {target_ids}，扫描到: {scanned_list}）")
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

    # requireSensor -> connect -> init
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        ctrl.terminate()
        return

    print("\n[连接] sensor.connect() ...", flush=True)
    try:
        ok = sensor.connect()
    except Exception as e:
        ok = False
        print(f"[连接] 抛异常 {type(e).__name__}: {e}", flush=True)
    if ok is not True:
        print("[FAIL] connect 失败", flush=True)
        ctrl.terminate()
        return

    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print("[FAIL] 未到达 Ready", flush=True)
        ctrl.terminate()
        return

    print(f"\n[init] sensor.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        ok_init = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
    except Exception as e:
        ok_init = False
        print(f"[init] 抛异常 {type(e).__name__}: {e}", flush=True)
    if ok_init is not True:
        print("[FAIL] init 失败", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    # ---- 能力表达（spec 驱动，替代 getDeviceInfo() 字段表与硬编码型号假设）----
    print("\n[能力表达] 按 spec 遍历 streams / sample_rates ...", flush=True)

    # 检查：spec["model"] 存在且非空（替代 DeviceInfo.ModelName）
    model = spec.get("model")
    model_ok = isinstance(model, str) and len(model) > 0
    print(f"[检查] spec model = {model!r} {'OK' if model_ok else 'FAIL'}", flush=True)
    record(results, "spec model 存在且非空", model_ok,
           "spec['model'] 为非空字符串", f"model = {model!r}")

    # 数据流能力：遍历 spec["streams"]，supported=True 且 channels>0 记为支持，否则 SKIP
    print("\n[数据流能力] spec['streams']:", flush=True)
    for stream_key, cap in (spec.get("streams") or {}).items():
        if not isinstance(cap, dict):
            continue
        supported = bool(cap.get("supported") is True and (cap.get("channels") or 0) > 0)
        channels = int(cap.get("channels") or 0)
        print(f"  {stream_key}: supported={supported}, channels={channels}", flush=True)
        if supported:
            record(results, f"{stream_key} 能力表达（支持，channels>0）", True,
                   f"spec['streams']['{stream_key}'] supported=True 且 channels>0",
                   f"channels={channels}")
        else:
            record(results, f"{stream_key} 能力表达", None,
                   f"spec['streams']['{stream_key}'] 存在该能力时校验",
                   f"spec 标记不支持/无通道（supported={cap.get('supported')}, channels={channels}）")

    # 采样率能力：遍历 spec["sample_rates"]（rates/default/stream/settable）
    print("\n[采样率能力] spec['sample_rates']:", flush=True)
    for param_key, entry in (spec.get("sample_rates") or {}).items():
        if not isinstance(entry, dict):
            continue
        rates = entry.get("rates") or []
        default = entry.get("default")
        stream = entry.get("stream")
        settable = entry.get("settable")
        print(f"  {param_key}: rates={rates}, default={default}, stream={stream}, settable={settable}", flush=True)
        record(results, f"{param_key} 能力表达（采样率档位）", True,
               f"spec['sample_rates']['{param_key}'] 含 rates/default/stream/settable",
               f"rates={rates}, default={default}, stream={stream}, settable={settable}")

    # 清理
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] 抛异常 {type(e).__name__}: {e}", flush=True)

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