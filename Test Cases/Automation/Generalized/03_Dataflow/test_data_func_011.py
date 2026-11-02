# -*- coding: utf-8 -*-
"""DATA-FUNC-011：传统 EMG 设备 NTF_GEST 与 NTF_EMG 互斥。

对应用例：03_数据流.md -> DATA-FUNC-011
可自动化：auto（能力判定后执行/跳过；设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 从 spec["streams"] 判定是否新 EMG 设备（是否有 NTF_IMU 聚合流）
  3) 有 NTF_IMU 聚合流：新 EMG 设备，NTF_GEST/NTF_EMG 不互斥 → 记录"不适用，跳过"（SKIP）
  4) 无 NTF_IMU 聚合流（传统 EMG 设备）且同时支持 NTF_GEST/NTF_EMG：验证互斥：
     - 清理：NTF_GEST OFF + NTF_EMG OFF
     - 同时 setParam("NTF_GEST","ON") + setParam("NTF_EMG","ON")
     - startDataNotification 起流，采集窗口统计实际收到的 DataType
     - 断言：两者只生效其一（EMG/GEST 不同时收到数据，且至少收到其一）
     - 不同时支持 NTF_GEST/NTF_EMG：不适用，SKIP

说明：
  README：on legacy (non-new) EMG devices, NTF_GEST and NTF_EMG are mutually exclusive.
  新 EMG 设备（有 NTF_IMU 聚合流）GEST 独立，不适用本用例。
  "互斥"以"实际收到的数据流"为准（setParam 返回 OK 不代表最终生效）。

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


def _dt_name(dt):
    try:
        if isinstance(dt, DataType):
            return dt.name
        return DataType(dt).name
    except Exception:
        return str(dt)


class StreamCollector:
    """按 DataType 统计收到的批次与样本数，用于判断 GEST/EMG 哪个生效。"""

    def __init__(self):
        self.by_type = {}

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            dt = d.getDataType()
            entry = self.by_type.setdefault(dt, {'batches': 0, 'samples': 0})
            entry['batches'] += 1
            try:
                n = d.getChannelCount() * d.getSampleCount()
            except Exception:
                n = 0
            entry['samples'] += n


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-011 传统 EMG 设备 NTF_GEST 与 NTF_EMG 互斥", flush=True)
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

    # spec 判定：是否新 EMG 设备（有 NTF_IMU 聚合流则 GEST 独立不互斥）
    streams = spec.get("streams") or {}

    def _supported(key):
        cap = streams.get(key) or {}
        return bool(cap.get("supported") is True and (cap.get("channels") or 0) > 0)

    emg_supported = _supported("NTF_EMG")
    gest_supported = _supported("NTF_GEST")
    imu_supported = _supported("NTF_IMU")
    print(f"\n[能力] spec.streams：NTF_EMG={emg_supported} NTF_GEST={gest_supported} NTF_IMU={imu_supported}", flush=True)

    # 设备须同时支持 NTF_EMG 与 NTF_GEST，本用例才有意义
    if not (emg_supported and gest_supported):
        record(results, "传统 EMG 设备 GEST/EMG 互斥判定", None,
               "spec 同时声明 NTF_EMG 与 NTF_GEST（supported 且 channels>0）时验证互斥",
               "spec 未同时声明 NTF_EMG 与 NTF_GEST，本用例不适用")
        print("[SKIP] spec 未同时声明 NTF_EMG 与 NTF_GEST，本用例不适用，跳过", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    # 能力判定：有 NTF_IMU 聚合流 = 新 EMG 设备（GEST 独立，无互斥），本用例仅针对传统设备
    if imu_supported:
        record(results, "传统 EMG 设备 GEST/EMG 互斥判定", None,
               "spec 无 NTF_IMU 聚合流（传统设备）时验证互斥",
               "spec 声明 NTF_IMU 聚合流，属新 EMG 设备，GEST 独立不互斥")
        print("[SKIP] 新 EMG 设备（spec 声明 NTF_IMU 聚合流），NTF_GEST/NTF_EMG 不互斥，本用例不适用，跳过", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    # ---- 传统 EMG 设备：验证 GEST/EMG 互斥 ----
    print("\n[互斥测试] 传统 EMG 设备，验证 NTF_GEST 与 NTF_EMG 互斥", flush=True)

    # 清理初始状态
    print("[清理] NTF_GEST OFF + NTF_EMG OFF ...", flush=True)
    try:
        sensor.setParam("NTF_GEST", "OFF")
    except Exception as e:
        print(f"  [清理] NTF_GEST OFF 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        sensor.setParam("NTF_EMG", "OFF")
    except Exception as e:
        print(f"  [清理] NTF_EMG OFF 抛异常 {type(e).__name__}: {e}", flush=True)

    # 同时 ON
    print("[setParam] NTF_GEST='ON' 与 NTF_EMG='ON' 同时设置 ...", flush=True)
    try:
        g_ret = sensor.setParam("NTF_GEST", "ON")
    except Exception as e:
        g_ret = f"抛异常 {type(e).__name__}: {e}"
    try:
        e_ret = sensor.setParam("NTF_EMG", "ON")
    except Exception as e:
        e_ret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[setParam] NTF_GEST ON -> {g_ret!r} ; NTF_EMG ON -> {e_ret!r}", flush=True)

    # 读回 NTF 状态（辅助信息，最终以实际数据流为准）
    try:
        ntf_state = sensor.getParam("NTF")
    except Exception as e:
        ntf_state = f"抛异常 {type(e).__name__}: {e}"
    print(f"[getParam] getParam('NTF') -> {ntf_state!r}", flush=True)

    collector = StreamCollector()
    sensor.onDataCallback = collector.on_data

    print("[起流] SensorProfile.startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        start_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        start_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] SensorProfile.startDataNotification() -> {start_txt}", flush=True)
    record(results, "SensorProfile.startDataNotification 返回 True", sret is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start_txt}")

    # 采集窗口
    print(f"\n[采集] 等待 {config.COLLECT_SECONDS}s 观察 GEST/EMG 实际生效流 ...", flush=True)
    time.sleep(config.COLLECT_SECONDS)

    emg_entry = collector.by_type.get(DataType.NTF_EMG, {'batches': 0, 'samples': 0})
    gest_entry = collector.by_type.get(DataType.NTF_GEST, {'batches': 0, 'samples': 0})
    got_types = {_dt_name(k): v['batches'] for k, v in collector.by_type.items()}
    print(f"[采集] 实际收到类型={got_types}", flush=True)
    print(f"[采集] EMG 批次={emg_entry['batches']} 样本={emg_entry['samples']} ; "
          f"GEST 批次={gest_entry['batches']} 样本={gest_entry['samples']}", flush=True)

    emg_on = emg_entry['samples'] > 0
    gest_on = gest_entry['samples'] > 0

    # 判定 1：至少一种流生效
    record(results, "GEST/EMG 至少一种流收到数据", emg_on or gest_on,
           "NTF_EMG 或 NTF_GEST 至少收到数据",
           f"EMG样本={emg_entry['samples']} GEST样本={gest_entry['samples']}")

    # 判定 2：互斥（不同时收到）
    record(results, "NTF_GEST 与 NTF_EMG 互斥（不同时收到数据）", not (emg_on and gest_on),
           "两者只生效其一（不同时收到 EMG 与 GEST 数据）",
           f"EMG={'收' if emg_on else '不收'} GEST={'收' if gest_on else '不收'}")

    # 停流 + 清理
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.setParam("NTF_GEST", "OFF")
    except Exception:
        pass
    try:
        sensor.setParam("NTF_EMG", "OFF")
    except Exception:
        pass

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
