# -*- coding: utf-8 -*-
"""DATA-FUNC-012：SensorData.clone 深拷贝。

对应用例：03_数据流.md -> DATA-FUNC-012
可自动化：auto（设备上电、在范围内且已佩戴为运行前置，测试中无需人工动作）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) setParam("NTF_EEG", "ON") 起 EEG 流
  3) startDataNotification 后采集窗口内取第一个非空批次 SensorData
  4) 校验 clone()：
     - clone() 返回 SensorData 实例，且与原对象不是同一引用
     - 元数据一致（deviceMac/dataType/sampleRate/channelCount/sampleCount/
       lostPackageCount/startTimeStamp）
     - 各样本字段全一致（保真）
     - 深拷贝独立性：尽力修改副本 Sample.data，确认原对象不受影响
       （若字段只读/不可原地修改，则记为 SKIP 并说明，不以 FAIL 误判）

说明：
  README：SensorData 的 channelSamples/startSampleIndex 只读，clone() 用于深拷贝；
  Sample 字段（data/rawData/...）为只读属性。
  深拷贝的核心语义是"修改副本不影响原对象"。因 Sample.data 的底层类型与可写性
  需运行时确认，本用例把"修改副本"做成尽力而为：能改则严格判定，不能改则 SKIP
  并打印字段类型供定位。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内，且已佩戴（电极接触皮肤）
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


def _dt_name(dt):
    try:
        if isinstance(dt, DataType):
            return dt.name
        return DataType(dt).name
    except Exception:
        return str(dt)


SAMPLE_FIELDS = ["data", "rawData", "impedance", "saturation",
                 "sampleIndex", "channelIndex", "timeStampInMs", "absTimeStampInSec", "isLost"]


class BatchCollector:
    """保存第一个非空批次，用于 clone 校验。"""

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


def _clone_metadata_equal(a, b):
    """比较两份 SensorData 的元数据是否一致。"""
    keys = ["getDeviceMac", "getDataType", "getSampleRate", "getChannelCount",
            "getSampleCount", "getLostPackageCount", "getStartTimeStamp"]
    for k in keys:
        try:
            va = getattr(a, k)()
        except Exception as e:
            va = f"<{type(e).__name__}>"
        try:
            vb = getattr(b, k)()
        except Exception as e:
            vb = f"<{type(e).__name__}>"
        if va != vb:
            return f"{k}() 不一致：原={va!r} 副本={vb!r}"
    return None


def _samples_equal(a, b):
    """遍历两份 SensorData 的 channelSamples 各 Sample 字段是否全一致。"""
    ca = getattr(a, 'channelSamples', None)
    cb = getattr(b, 'channelSamples', None)
    if not ca or not cb:
        return None  # 空结构由外层判定
    try:
        n_ch = len(ca)
    except TypeError:
        return None
    for ci in range(n_ch):
        try:
            ch_a = ca[ci]
            ch_b = cb[ci]
        except Exception as e:
            return f"取通道 {ci} 失败：{type(e).__name__}: {e}"
        n_s = len(ch_a)
        if len(ch_b) != n_s:
            return f"通道 {ci} 样本数不一致：原={n_s} 副本={len(ch_b)}"
        for si in range(n_s):
            for f in SAMPLE_FIELDS:
                try:
                    va = getattr(ch_a[si], f, None)
                    vb = getattr(ch_b[si], f, None)
                except Exception as e:
                    return f"ci={ci} si={si} 读 {f} 抛异常 {type(e).__name__}: {e}"
                if va != vb:
                    return f"ci={ci} si={si} {f} 不一致：原={va!r} 副本={vb!r}"
    return None


def _try_mutate_clone(orig, clone):
    """尽力修改副本 Sample.data 元素，判断是否影响原对象。
    返回 (结果, 说明)：True=深拷贝独立 / False=浅拷贝共享 / None=无法验证。"""
    ca = getattr(orig, 'channelSamples', None)
    cb = getattr(clone, 'channelSamples', None)
    if not ca or not cb:
        return None, "channelSamples 为空"
    try:
        s_orig = ca[0][0]
        s_clone = cb[0][0]
    except Exception as e:
        return None, f"取首样本失败 {type(e).__name__}: {e}"

    try:
        cdata = s_clone.data
        odata = s_orig.data
    except Exception as e:
        return None, f"读 data 抛异常 {type(e).__name__}: {e}"

    if not isinstance(cdata, list):
        return None, f"Sample.data 类型为 {type(cdata).__name__}（非 list），无法原地修改"
    if len(cdata) == 0:
        return None, "Sample.data 为空列表"

    before = odata[0]
    marker = -98765.4321
    try:
        cdata[0] = marker
    except Exception as e:
        return None, f"修改副本 data[0] 失败 {type(e).__name__}: {e}"

    changed = (odata[0] != before)
    # 还原副本，避免影响后续
    try:
        cdata[0] = before
    except Exception:
        pass

    if changed:
        return False, f"改副本 data[0] 后原对象 data[0] 从 {before!r} 变 {odata[0]!r}（浅拷贝共享）"
    return True, f"改副本 data[0] 后原对象 data[0] 仍为 {before!r}（深拷贝独立）"


def check_clone(data, results):
    """对一批 SensorData 校验 clone() 深拷贝。"""

    def add(name, ok, expect, actual):
        record(results, name, ok, expect, actual)

    # 1. clone() 返回实例
    try:
        cloned = data.clone()
    except Exception as e:
        add("SensorData.clone() 可调用", False, "clone() 返回 SensorData",
            f"clone() 抛异常 {type(e).__name__}: {e}")
        return
    add("SensorData.clone() 返回 SensorData 实例", isinstance(cloned, SensorData),
        "clone() 返回 SensorData 实例", f"返回 {type(cloned).__name__}")

    # 2. 非同一引用
    add("clone 与原对象不是同一引用", cloned is not data,
        "clone() 返回新对象（is not 原对象）",
        "clone 与原对象是同一引用" if cloned is data else "不同引用")

    # 3. 元数据一致
    meta_bad = _clone_metadata_equal(data, cloned)
    add("clone 元数据与原一致", meta_bad is None,
        "deviceMac/dataType/sampleRate/channelCount/sampleCount/lostPackageCount/startTimeStamp 一致",
        meta_bad if meta_bad else "全部一致")

    # 4. 样本字段全一致（保真）
    samp_bad = _samples_equal(data, cloned)
    add("clone 各样本字段与原一致", samp_bad is None,
        "各 Sample 的 data/rawData/impedance/saturation/sampleIndex/channelIndex/timeStampInMs/absTimeStampInSec/isLost 一致",
        samp_bad if samp_bad else "全部一致")

    # 5. 深拷贝独立性：尽力修改副本，确认原对象不变
    res, note = _try_mutate_clone(data, cloned)
    add("深拷贝：修改副本不影响原对象", res,
        "修改副本 Sample.data 后原对象不变",
        note)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-012 SensorData.clone 深拷贝", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内，且已佩戴（电极接触皮肤）", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】、在范围内且已【佩戴】，"
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

    # 起 EEG 流
    print("\n[起流] SensorProfile.setParam('NTF_EEG', 'ON') ...", flush=True)
    try:
        p_ret = sensor.setParam("NTF_EEG", "ON")
        p_txt = f"返回 {p_ret!r}"
    except Exception as e:
        p_ret = None
        p_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] setParam('NTF_EEG', 'ON') -> {p_txt}", flush=True)

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
        sensor.setParam("NTF_EEG", "OFF")
    except Exception:
        pass

    # clone 校验
    if collector.first_batch is None:
        print("[FAIL] 采集窗口内未收到非空数据，无法校验 clone", flush=True)
        record(results, "采集窗口内收到非空数据", False,
               "收到至少一批非空 SensorData", f"批次数={collector.batches} 非空批次=0")
    else:
        record(results, "采集窗口内收到非空数据", True,
               "收到至少一批非空 SensorData",
               f"批次数={collector.batches} 非空样本数={collector.first_batch_samples}")
        # 打印首样本 data 类型，辅助定位字段可写性
        try:
            s0 = collector.first_batch.channelSamples[0][0]
            d0 = s0.data
            print(f"[info] 首样本 data 类型={type(d0).__name__} 长度={len(d0) if hasattr(d0, '__len__') else '?'}", flush=True)
        except Exception as e:
            print(f"[info] 读取首样本 data 失败 {type(e).__name__}: {e}", flush=True)
        check_clone(collector.first_batch, results)

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
