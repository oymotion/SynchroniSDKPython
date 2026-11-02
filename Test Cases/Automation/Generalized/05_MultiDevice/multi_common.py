# -*- coding: utf-8 -*-
"""多设备同步测试共享辅助函数。

供 05_MultiDevice 目录下各 MULTI-FUNC-* 脚本使用。
"""

import os
import sys
import time
import re

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match_all


def connect_and_init_all(ctrl, matched, results):
    """连接并初始化所有匹配到的设备，返回 SensorProfile 列表。

    matched 元素为 (identity, BLEDevice)。
    如果某台设备连接或初始化失败，记录 FAIL 并中断（返回空列表）。
    """
    sensors = []
    for tid, device in matched:
        sensor = ctrl.requireSensor(device)
        if sensor is None:
            record(results, f"requireSensor({tid})", False,
                   "返回 SensorProfile", "返回 None")
            return []

        name = getattr(device, 'Name', '?')
        addr = getattr(device, 'Address', '?')
        # 诊断：确认 requireSensor 返回的 profile 对应正确的设备
        profile_addr = getattr(getattr(sensor, 'BLEDevice', None), 'Address', '?')
        print(f"\n[连接] {name} ({tid}) {addr} ...", flush=True)
        print(f"  [requireSensor] profile.BLEDevice.Address = {profile_addr} (期望 {addr})", flush=True)
        print(f"  [连接前状态] deviceState = {sensor.deviceState}, isReady = {sensor.isReady}", flush=True)

        if profile_addr.upper() != addr.upper():
            record(results, f"requireSensor({tid}) 设备匹配", False,
                   f"profile 地址 == {addr}", f"profile 地址 = {profile_addr}")
            return []

        try:
            ok = sensor.connect()
        except Exception as e:
            ok = None
            print(f"[连接] 抛异常 {type(e).__name__}: {e}", flush=True)
        if ok is not True:
            record(results, f"connect({tid})", False,
                   "connect() 返回 True", f"connect() -> {ok}（连接后 state={sensor.deviceState}）")
            return []

        # 等待 Ready
        t0 = time.time()
        while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
            time.sleep(0.2)
        if sensor.deviceState != DeviceStateEx.Ready:
            record(results, f"到达 Ready({tid})", False,
                   "deviceState==Ready", f"state={sensor.deviceState}")
            return []

        print(f"[init] {tid} ...", flush=True)
        try:
            iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        except Exception as e:
            iret = None
            print(f"[init] 抛异常 {type(e).__name__}: {e}", flush=True)
        if iret is not True:
            record(results, f"init({tid})", False,
                   "init() 返回 True", f"init() -> {iret}")
            return []

        print(f"[就绪] {tid}: {name} {addr}", flush=True)
        sensors.append(sensor)

    return sensors


def disconnect_all(sensors):
    """断开所有设备连接。"""
    for s in sensors:
        try:
            s.disconnect()
        except Exception as e:
            print(f"[断开] 抛异常 {type(e).__name__}: {e}", flush=True)


def check_device_count(results, matched, required=2):
    """检查匹配到的设备数量是否满足要求，不足则记录 SKIP。"""
    if len(matched) < required:
        found_list = [(tid, getattr(d, 'Name', '?'), getattr(d, 'Address', '?'))
                      for tid, d in matched]
        msg = f"需要 {required} 台设备，实际匹配到 {len(matched)} 台: {found_list}"
        record(results, f"设备数量检查（需要≥{required}台）", None,
               f"匹配到 ≥{required} 台设备", msg)
        print(f"[SKIP] {msg}", flush=True)
        return False
    return True


class MultiDataCollector:
    """多设备数据回调收集器，记录每台设备的 startTimeStamp、delay 和批次计数。"""

    def __init__(self):
        self.first_batch = {}   # mac -> (startTimeStamp, delay)
        self.batch_counts = {}  # mac -> int
        self.lock = None

    def on_data(self, sensor, data_list):
        mac = sensor.BLEDevice.Address
        items = data_list if isinstance(data_list, list) else [data_list]
        self.batch_counts[mac] = self.batch_counts.get(mac, 0) + len(items)
        for data in items:
            ts = data.getStartTimeStamp()
            if ts is not None:
                self.first_batch[mac] = (ts, data.getDelay())

    def get_spread_ms(self):
        """计算跨设备 startTimeStamp 散布（ms），不足 2 台返回 None。"""
        ts_list = [(mac, v[0]) for mac, v in self.first_batch.items() if v[0] is not None]
        if len(ts_list) < 2:
            return None
        return (max(t for _, t in ts_list) - min(t for _, t in ts_list)) & 0xFFFFFFFF


def print_summary(results, all_pass):
    """打印测试结果汇总。"""
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    for rname, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {rname}（实际: {actual}）", flush=True)
        elif status == "SKIP":
            print(f"  [SKIP] {rname}（{actual}）", flush=True)
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)