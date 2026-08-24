# -*- coding: utf-8 -*-
"""DATA-FUNC-013：Sample 新增字段与单点访问器一致（0.9.0 新增）。

对应用例：03_数据流.md -> DATA-FUNC-013
可自动化：auto（设备上电、在范围内且已佩戴为运行前置，测试中无需人工动作）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) setParam("NTF_EMG", "ON") 起 EMG 流
  3) startDataNotification 后采集窗口内取第一个非空批次 SensorData
  4) 对 0.9.0 新增字段逐一校验"Sample 字段 == 单点访问器"：
       rawData         -> getRawData(ci,si)
       impedance       -> getImpedance(ci,si)
       saturation      -> getSaturation(ci,si)
       timeStampInMs   -> getTimeStampInMs(ci,si)
       absTimeStampInSec -> getAbsTimeStampInSec(ci,si)
     channelIndex 无直接单点访问器，通过 getChannelSample(ci,si).channelIndex 校验。

说明：
  0.9.0 新增了 Sample.rawData/impedance/saturation/channelIndex/timeStampInMs/
  absTimeStampInSec 等字段及对应单点访问器。本用例聚焦这些"新增字段"，逐字段
  给出独立结论，区别于 DATA-FUNC-007（全字段 + sampleIndex 单调）。
  channelIndex 语义为"样本所属通道"，getChannelSample(ci,si).channelIndex 应 == ci。

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


class BatchCollector:
    """保存第一个非空批次，用于新增字段校验。"""

    def __init__(self):
        self.first_batch = None
        self.first_batch_samples = 0
        self.batches = 0
        self.total_samples = 0

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            self.batches += 1
            cs = getattr(d, 'channelSamples', None)
            n = 0
            if cs:
                try:
                    n = sum(len(ch) for ch in cs)
                except TypeError:
                    n = len(cs)
            self.total_samples += n
            if self.first_batch is None and n > 0:
                self.first_batch = d
                self.first_batch_samples = n


# 0.9.0 新增字段 -> 对应单点访问器（channelIndex 单独处理，无直接访问器）
NEW_FIELD_ACCESSORS = [
    ("rawData", "getRawData"),
    ("impedance", "getImpedance"),
    ("saturation", "getSaturation"),
    ("timeStampInMs", "getTimeStampInMs"),
    ("absTimeStampInSec", "getAbsTimeStampInSec"),
]


def check_new_fields(data, results):
    """对一批 SensorData 逐一校验新增字段与访问器一致性。"""

    def add(name, ok, expect, actual):
        record(results, name, ok, expect, actual)

    cs = getattr(data, 'channelSamples', None)
    if not cs:
        add("channelSamples 结构合法", False, "通道数>0 且每通道样本数>0", "channelSamples 为空/无数据")
        return
    try:
        n_ch = len(cs)
        n_s = len(cs[0]) if n_ch else 0
    except TypeError:
        n_ch = n_s = 0
    if n_ch == 0 or n_s == 0:
        add("channelSamples 结构合法", False, "通道数>0 且每通道样本数>0",
            f"通道数={n_ch} 每通道样本数={n_s}")
        return
    add("channelSamples 结构合法", True, "通道数>0 且每通道样本数>0",
        f"通道数={n_ch} 每通道样本数={n_s}")

    # 遍历，返回首个不一致描述；None 表示全一致
    def scan(check_fn):
        for ci in range(n_ch):
            for si in range(n_s):
                try:
                    s = cs[ci][si]
                    bad = check_fn(ci, si, s)
                except Exception as e:
                    bad = f"抛异常 {type(e).__name__}: {e}"
                if bad is not None:
                    return f"ci={ci} si={si} {bad}"
        return None

    # 1) 各新增字段 -> 单点访问器
    for field, accessor in NEW_FIELD_ACCESSORS:
        def make(field, accessor):
            def inner(ci, si, s):
                v1 = getattr(s, field, None)
                v2 = getattr(data, accessor)(ci, si)
                if v2 != v1:
                    return f"{accessor}(ci,si)={v2!r} != sample.{field}={v1!r}"
                return None
            return inner

        bad = scan(make(field, accessor))
        add(f"{accessor}(ci,si) 与 sample.{field} 一致", bad is None,
            f"{accessor}(ci,si) == sample.{field}", bad if bad else "全部一致")

    # 2) channelIndex（无直接访问器，经 getChannelSample）
    def chk_channel_index(ci, si, s):
        s2 = data.getChannelSample(ci, si)
        v = getattr(s2, 'channelIndex', None)
        if v != ci:
            return f"getChannelSample(ci,si).channelIndex={v!r} != ci={ci}"
        return None

    bad = scan(chk_channel_index)
    add("getChannelSample(ci,si).channelIndex == ci", bad is None,
        "getChannelSample(ci,si).channelIndex 等于通道索引 ci", bad if bad else "全部一致")


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-013 Sample 新增字段与单点访问器一致", flush=True)
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

    collector = BatchCollector()
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
    print(f"\n[采集] 等待 {config.COLLECT_SECONDS}s 观察 onDataCallback ...", flush=True)
    time.sleep(config.COLLECT_SECONDS)
    print(f"[采集] 收到批次={collector.batches} 非空批次样本数={collector.first_batch_samples}", flush=True)

    # 停流
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.setParam("NTF_EMG", "OFF")
    except Exception:
        pass

    # 校验新增字段
    if collector.first_batch is None:
        print("[FAIL] 采集窗口内未收到非空数据，无法校验新增字段", flush=True)
        record(results, "采集窗口内收到非空数据", False,
               "收到至少一批非空 SensorData", f"批次数={collector.batches} 非空批次=0")
    else:
        record(results, "采集窗口内收到非空数据", True,
               "收到至少一批非空 SensorData",
               f"批次数={collector.batches} 非空样本数={collector.first_batch_samples}")
        check_new_fields(collector.first_batch, results)

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
