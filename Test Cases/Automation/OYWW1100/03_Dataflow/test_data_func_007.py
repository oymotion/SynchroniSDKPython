# -*- coding: utf-8 -*-
"""DATA-FUNC-007：channelSamples/单点访问器一致，sampleIndex 单调。

对应用例：03_数据流.md -> DATA-FUNC-007
可自动化：auto（设备上电、在范围内且已佩戴为运行前置，测试中无需人工动作）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) setParam("NTF_EMG", "ON") 起 EMG 流
  3) startDataNotification 后采集窗口内等回调，取第一个非空批次 SensorData
  4) 校验样本访问接口一致性：
     - channelSamples[ci][si] 为 Sample
     - getChannelSample(ci, si) 返回 Sample，各字段与 channelSamples 一致
     - getData/getRawData/getImpedance/getSaturation/getSampleIndex/
       getTimeStampInMs/getAbsTimeStampInSec/isLost(ci, si) 与 Sample 对应字段一致
     - 同一通道内 sampleIndex 严格递增

说明：
  单点访问器是上层按通道/样本随机访问的入口，须与 channelSamples 全量数据一致；
  sampleIndex 单调递增是时序正确性的基础。EMG 采样率 500Hz，数秒内多批，取第一批校验。

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


class MetaCollector:
    """保存第一个“非空”批次，用于访问器一致性校验。"""

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


SAMPLE_FIELDS = ["data", "rawData", "impedance", "saturation",
                 "sampleIndex", "channelIndex", "timeStampInMs", "absTimeStampInSec", "isLost"]


def check_accessors(data, results):
    """对一批 SensorData 校验 channelSamples 与各单点访问器一致性。"""

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

    # 全量遍历，返回首个不一致描述；None 表示全部一致
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

    # getChannelSample(ci, si) 返回 Sample，字段与 channelSamples 一致
    def chk_channel_sample(ci, si, s):
        s2 = data.getChannelSample(ci, si)
        for f in SAMPLE_FIELDS:
            if getattr(s2, f, None) != getattr(s, f, None):
                return (f"getChannelSample().{f}={getattr(s2, f, None)!r} "
                        f"!= channelSamples.{f}={getattr(s, f, None)!r}")
        return None

    bad = scan(chk_channel_sample)
    add("getChannelSample 字段与 channelSamples 一致", bad is None,
        "getChannelSample(ci,si) 各字段 == channelSamples[ci][si] 各字段",
        bad if bad else "全部一致")

    # 单点字段访问器与 Sample 字段一致
    acc_map = [
        ("data", "getData"),
        ("rawData", "getRawData"),
        ("impedance", "getImpedance"),
        ("saturation", "getSaturation"),
        ("sampleIndex", "getSampleIndex"),
        ("timeStampInMs", "getTimeStampInMs"),
        ("absTimeStampInSec", "getAbsTimeStampInSec"),
        ("isLost", "isLost"),
    ]
    for field, accessor in acc_map:
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

    # 同一通道内 sampleIndex 严格递增
    def chk_mono(ci, si, s):
        if si == 0:
            return None
        prev = cs[ci][si - 1].sampleIndex
        cur = s.sampleIndex
        if not (cur > prev):
            return f"sampleIndex 非递增：prev={prev!r} cur={cur!r}"
        return None

    bad = scan(chk_mono)
    add("sampleIndex 单调递增（通道内）", bad is None,
        "同一通道内 sampleIndex 严格递增", bad if bad else "全部递增")


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-007 channelSamples/单点访问器一致，sampleIndex 单调", flush=True)
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
        print("[FAIL] 未匹配到 config 中启用的设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含启用的目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含启用的目标设备", f"匹配到 {name} {addr}")

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

    collector = MetaCollector()
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

    # 采集窗口，等待第一批数据
    print(f"\n[采集] 等待 {config.COLLECT_SECONDS}s 观察 onDataCallback ...", flush=True)
    time.sleep(config.COLLECT_SECONDS)
    print(f"[采集] 收到批数={collector.batches} 样本数={collector.total_samples}", flush=True)

    first = collector.first_batch
    if first is None:
        record(results, "采集窗口内收到至少一批 SensorData", False,
               "onDataCallback 收到 >=1 批数据", f"批数={collector.batches} 样本数={collector.total_samples}")
        print("[FAIL] 未收到任何 SensorData 批次，无法校验访问器", flush=True)
    else:
        record(results, "采集窗口内收到至少一批 SensorData", True,
               "onDataCallback 收到 >=1 批数据",
               f"批数={collector.batches} 样本数={collector.total_samples}")
        print(f"\n[校验] 对第一批（非空）SensorData 校验访问器一致性 "
              f"(该批 channelSamples 展开样本数={collector.first_batch_samples}) ...", flush=True)
        check_accessors(first, results)

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
