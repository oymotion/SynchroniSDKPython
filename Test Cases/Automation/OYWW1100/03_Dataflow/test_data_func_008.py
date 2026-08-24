# -*- coding: utf-8 -*-
"""DATA-FUNC-008：isLost 与 lostPackageCount 一致。

对应用例：03_数据流.md -> DATA-FUNC-008
可自动化：semi-auto（需人工制造轻微信号干扰以触发丢包）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) setParam("NTF_EMG", "ON") 起 EMG 流
  3) startDataNotification 后，人工轻微遮挡/移远设备制造干扰（保持连接不断链）
  4) 采集 15s 收集所有批次，对每批统计 getLostPackageCount() 与 sample.isLost 置位样本数：
     - 无丢包计数（lostPackageCount==0）的批次，不应有 isLost=True 的样本
     - 有丢包计数（lostPackageCount>0）的批次，应存在 isLost=True 的样本

说明：
  丢包计数（package 级）与样本丢包标记（sample 级）应保持一致：
  计数为 0 时无丢包标记，计数 >0 时有丢包标记。
  OYWW1100 为新 EMG 设备，发生丢包时 lostPackageCount 会非 0；
  本用例通过人工制造轻微干扰触发丢包，以验证标记与计数一致。
  EMG 采样率 500Hz，15s 内可多批，覆盖多批统计。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内，且已佩戴（电极接触皮肤）
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

LOST_OBSERVE_SECONDS = 15  # 采集观察时长（秒），留足人工制造干扰的时间


class LostCollector:
    """收集所有批次，记录每批的 lostPackageCount 与 isLost 置位样本数。"""

    def __init__(self):
        self.batches = []  # 每项 (lost_count, lost_samples, batch_samples)
        self.total_samples = 0

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            cs = getattr(d, 'channelSamples', None)
            lost_samples = 0
            batch_samples = 0
            if cs:
                for ch in cs:
                    for s in ch:
                        batch_samples += 1
                        if getattr(s, 'isLost', False):
                            lost_samples += 1
            try:
                lost_count = d.getLostPackageCount()
            except Exception as e:
                lost_count = None  # 标记为调用异常
            self.batches.append((lost_count, lost_samples, batch_samples))
            self.total_samples += batch_samples


def check_lost_consistency(batches, results):
    """对收集到的所有批次校验 isLost 与 lostPackageCount 一致性。"""

    def add(name, ok, expect, actual):
        record(results, name, ok, expect, actual)

    # getLostPackageCount 可调用性
    err = next((f"批次{i} 抛异常" for i, (lc, _, _) in enumerate(batches) if lc is None), None)
    add("getLostPackageCount 可调用（所有批次）", err is None,
        "getLostPackageCount() 返回合法值", err if err else "全部可调用")

    # 判定 1：lostPackageCount==0 的批次，不应有 isLost=True 的样本
    bad_zero = None
    for i, (lc, ls, _) in enumerate(batches):
        if isinstance(lc, int) and lc == 0 and ls > 0:
            bad_zero = f"批次{i} lostPackageCount=0 但有 {ls} 个样本 isLost=True"
            break
    add("无丢包计数时无 isLost 标记（lostPackageCount==0 → isLost 全 False）", bad_zero is None,
        "lostPackageCount==0 的批次 isLost 全为 False",
        bad_zero if bad_zero else "全部一致")

    # 判定 2：lostPackageCount>0 的批次，应存在 isLost=True 的样本
    has_lost = any(isinstance(lc, int) and lc > 0 for lc, _, _ in batches)
    bad_pos = None
    for i, (lc, ls, _) in enumerate(batches):
        if isinstance(lc, int) and lc > 0 and ls == 0:
            bad_pos = f"批次{i} lostPackageCount={lc} 但没有样本 isLost=True"
            break
    add("有丢包计数时存在 isLost 标记（lostPackageCount>0 → 有 isLost=True 样本）", bad_pos is None,
        "lostPackageCount>0 的批次存在 isLost=True 样本",
        bad_pos if bad_pos else ("全部一致" if has_lost else "无丢包批次（本次未触发丢包）"))


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-008 isLost 与 lostPackageCount 一致", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内，且已佩戴（电极接触皮肤）", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】、在范围内且已【佩戴】，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配（未匹配到时自动重试，最多 3 次，间隔 10s）
    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS})，未匹配时最多重试 3 次（间隔 10s）...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    if target is None:
        print("[FAIL] 未匹配到目标设备（OYWW1100/80F3）", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含 OYWW1100", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含 OYWW1100", f"匹配到 {name} {addr}")

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

    # 起 EMG 流
    print("\n[起流] SensorProfile.setParam('NTF_EMG', 'ON') ...", flush=True)
    try:
        p_ret = sensor.setParam("NTF_EMG", "ON")
        p_txt = f"返回 {p_ret!r}"
    except Exception as e:
        p_ret = None
        p_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] setParam('NTF_EMG', 'ON') -> {p_txt}", flush=True)

    collector = LostCollector()
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

    # 采集窗口（semi-auto：人工制造轻微干扰触发丢包）
    input("\n>>> [人工操作] 请轻微遮挡/移远待测设备制造信号干扰（保持 deviceState==Ready，勿导致断链），"
          "用于触发丢包，完成后按回车开始采集 ...")
    print(f"\n[采集] 观察 {LOST_OBSERVE_SECONDS}s ...", flush=True)
    time.sleep(LOST_OBSERVE_SECONDS)
    print(f"[采集] 收到批数={len(collector.batches)} 样本数={collector.total_samples}", flush=True)

    # 统计本次是否真的发生丢包
    total_lost_count = sum(lc for lc, _, _ in collector.batches if isinstance(lc, int))
    total_lost_samples = sum(ls for _, ls, _ in collector.batches)
    if total_lost_count > 0 or total_lost_samples > 0:
        print(f"[丢包] 本次发生丢包：总 lostPackageCount={total_lost_count} 总 isLost 样本={total_lost_samples}", flush=True)
    else:
        print("[丢包] 本次未发生丢包（lostPackageCount 恒 0，isLost 全 False）。"
              "若需验证丢包一致性，请加强干扰后重试。", flush=True)

    if not collector.batches:
        record(results, "采集窗口内收到至少一批 SensorData", False,
               "onDataCallback 收到 >=1 批数据", f"批数={len(collector.batches)}")
        print("[FAIL] 未收到任何 SensorData 批次，无法校验丢包一致性", flush=True)
    else:
        record(results, "采集窗口内收到至少一批 SensorData", True,
               "onDataCallback 收到 >=1 批数据",
               f"批数={len(collector.batches)} 样本数={collector.total_samples}")
        print("\n[校验] 逐批统计 isLost 与 lostPackageCount 一致性 ...", flush=True)
        for i, (lc, ls, bs) in enumerate(collector.batches):
            print(f"  批次{i}: lostPackageCount={lc} isLost样本={ls} 批样本数={bs}", flush=True)
        check_lost_consistency(collector.batches, results)

    # 清理
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.setParam("NTF_EMG", "OFF")
    except Exception:
        pass
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
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()
