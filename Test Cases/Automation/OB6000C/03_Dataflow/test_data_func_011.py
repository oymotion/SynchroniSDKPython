# -*- coding: utf-8 -*-
"""DATA-FUNC-011：EEG_SAMPLE_RATE 变更后 data rate 变化（EEG 设备核心）。

对应用例：03_数据流.md -> DATA-FUNC-011
可自动化：auto

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) getParam("EEG_SAMPLE_RATE_LIST") 获取可选值列表
  3) 对每个可选采样率，逐一 setParam("EEG_SAMPLE_RATE", value)，起流统计实际采样率
  4) 断言实际采样率与设置值一致（允许容差）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
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


class RateCollector:
    """统计 EEG 数据流样本数（sampleIndex 去重），用于计算实际采样率。"""

    def __init__(self):
        self.eeg_samples = 0        # 投递样本数（channelSamples 长度累加，含重复，仅观测）
        self.channel_count = 0
        self.first_ts = None
        self.min_sample_index = None  # 通道0 sampleIndex 最小值（跨批单调递增）
        self.max_sample_index = None  # 通道0 sampleIndex 最大值

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            dt = d.getDataType()
            if dt == DataType.NTF_EEG:
                cs = getattr(d, 'channelSamples', None)
                if cs:
                    if self.first_ts is None:
                        self.first_ts = time.time()
                    try:
                        n_ch = len(cs)
                        self.eeg_samples += sum(len(ch) for ch in cs)
                        if self.channel_count == 0:
                            self.channel_count = n_ch
                        # 用通道0 sampleIndex 记录全局首末（sampleIndex 跨批单调递增）
                        if n_ch:
                            for s in cs[0]:
                                idx = getattr(s, 'sampleIndex', None)
                                if idx is None:
                                    continue
                                if self.min_sample_index is None or idx < self.min_sample_index:
                                    self.min_sample_index = idx
                                if self.max_sample_index is None or idx > self.max_sample_index:
                                    self.max_sample_index = idx
                    except TypeError:
                        self.eeg_samples += len(cs)

    @property
    def unique_samples(self):
        """唯一样本数 = sampleIndex 范围跨度（去重，不含重复投递）。"""
        if self.min_sample_index is None or self.max_sample_index is None:
            return 0
        return self.max_sample_index - self.min_sample_index + 1


def wait_sample_rate_effective(sensor, target_rate, timeout=5.0):
    """等待采样率生效：注册 onDeviceInfoUpdate，等待 info.EegSampleRate == target_rate。

    EEG_SAMPLE_RATE 变更由设备异步生效，setParam 返回 "OK" 仅表示命令下发成功；
    生效后 SDK 通过 onDeviceInfoUpdate 推送新 DeviceInfo（缓存已就地更新）。
    超时返回 False 兜底（不阻塞测试，打印警告后继续）。
    """
    target = int(target_rate)
    state = {"matched": False, "updates": 0}

    def on_update(s, info):
        state["updates"] += 1
        sr = getattr(info, "EegSampleRate", None)
        print(f"  [onDeviceInfoUpdate] EegSampleRate={sr!r}（目标={target}）", flush=True)
        if sr is not None and int(sr) == target:
            state["matched"] = True

    # 先检查缓存 DeviceInfo（可能 setParam 已同步更新缓存，无需等回调）
    try:
        info = sensor.getDeviceInfo()
        sr = getattr(info, "EegSampleRate", None)
        if sr is not None and int(sr) == target:
            state["matched"] = True
    except Exception:
        pass

    old_cb = getattr(sensor, "onDeviceInfoUpdate", None)
    sensor.onDeviceInfoUpdate = on_update
    try:
        deadline = time.time() + timeout
        while not state["matched"] and time.time() < deadline:
            time.sleep(0.1)
    finally:
        sensor.onDeviceInfoUpdate = old_cb

    return state["matched"]


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-011 EEG_SAMPLE_RATE 变更后 data rate 变化", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
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

    # 获取 EEG_SAMPLE_RATE_LIST
    print("\n[参数] 获取 EEG_SAMPLE_RATE_LIST ...", flush=True)
    try:
        rate_list_str = sensor.getParam("EEG_SAMPLE_RATE_LIST")
        print(f"[参数] EEG_SAMPLE_RATE_LIST = {rate_list_str!r}", flush=True)
    except Exception as e:
        rate_list_str = None
        print(f"[参数] getParam('EEG_SAMPLE_RATE_LIST') 抛异常 {type(e).__name__}: {e}", flush=True)

    if not rate_list_str or rate_list_str.startswith("Error"):
        record(results, "EEG_SAMPLE_RATE 采样率验证", None,
               "EEG_SAMPLE_RATE_LIST 返回有效值列表",
               f"EEG_SAMPLE_RATE_LIST = {rate_list_str!r}，设备不支持或查询失败，跳过")
        print("[SKIP] 设备不支持 EEG_SAMPLE_RATE，跳过", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    rates = [r.strip() for r in rate_list_str.split("|") if r.strip()]
    print(f"[参数] 支持采样率: {rates}", flush=True)

    if not rates:
        record(results, "EEG_SAMPLE_RATE 采样率验证", None,
               "EEG_SAMPLE_RATE_LIST 包含至少一个值",
               f"EEG_SAMPLE_RATE_LIST = {rate_list_str!r}，列表为空")
        print("[SKIP] EEG_SAMPLE_RATE_LIST 为空，跳过", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    # 确保 NTF_EEG 开启
    try:
        sensor.setParam("NTF_EEG", "ON")
    except Exception as e:
        print(f"[setParam] NTF_EEG ON 抛异常 {type(e).__name__}: {e}", flush=True)

    # 对每个采样率逐一测试
    collector = RateCollector()
    sensor.onDataCallback = collector.on_data
    rate_results = []

    for rate in rates:
        print(f"\n{'=' * 40}", flush=True)
        print(f"[测试] EEG_SAMPLE_RATE = {rate}", flush=True)

        # 设置采样率
        try:
            r = sensor.setParam("EEG_SAMPLE_RATE", rate)
            print(f"[setParam] EEG_SAMPLE_RATE={rate} -> {r!r}", flush=True)
        except Exception as e:
            r = f"抛异常 {type(e).__name__}: {e}"
            print(f"[setParam] EEG_SAMPLE_RATE={rate} -> {r}", flush=True)

        if r != "OK":
            rate_results.append((rate, None, f"setParam 返回 {r!r}"))
            continue

        # 读回确认
        try:
            current_rate = sensor.getParam("EEG_SAMPLE_RATE")
            print(f"[getParam] EEG_SAMPLE_RATE = {current_rate!r}", flush=True)
        except Exception as e:
            current_rate = f"抛异常 {type(e).__name__}: {e}"
            print(f"[getParam] EEG_SAMPLE_RATE 抛异常: {e}", flush=True)

        # 等待采样率生效（onDeviceInfoUpdate 推送 EegSampleRate==rate），5s 超时兜底
        if not wait_sample_rate_effective(sensor, rate, timeout=5.0):
            print(f"[等待] EEG_SAMPLE_RATE={rate} 未在 5s 内确认生效，继续起流（兜底）", flush=True)

        # 起流采集
        collector.eeg_samples = 0
        collector.first_ts = None
        collector.min_sample_index = None
        collector.max_sample_index = None
        try:
            sret = sensor.startDataNotification()
        except Exception as e:
            sret = None
            print(f"[起流] 抛异常 {type(e).__name__}: {e}", flush=True)

        if sret is not True:
            rate_results.append((rate, None, "startDataNotification 失败"))
            try:
                sensor.stopDataNotification()
            except Exception:
                pass
            continue

        # 等待首批数据到达，排除起流建立延迟
        print("[采集] 等待首批 EEG 数据到达 ...", flush=True)
        t_wait0 = time.time()
        while collector.first_ts is None and time.time() - t_wait0 < 10:
            time.sleep(0.05)

        if collector.first_ts is None:
            print("[采集] 10s 内未收到任何 EEG 数据", flush=True)
            collect_duration = 0
        else:
            t_end = collector.first_ts + config.COLLECT_SECONDS
            print(f"[采集] 首批已到达，从首批起精确采集 {config.COLLECT_SECONDS}s ...", flush=True)
            while time.time() < t_end:
                time.sleep(0.05)
            collect_duration = time.time() - collector.first_ts

        try:
            sensor.stopDataNotification()
        except Exception as e:
            print(f"[停流] 抛异常 {type(e).__name__}: {e}", flush=True)

        unique = collector.unique_samples
        if collect_duration > 0 and unique > 0:
            actual_rate = unique / collect_duration
        else:
            actual_rate = 0
        expected_rate = int(rate)
        # 允许 10% 容差
        tolerance = expected_rate * 0.10
        rate_ok = abs(actual_rate - expected_rate) <= tolerance

        print(f"[结果] 期望={expected_rate}Hz, 实际≈{actual_rate:.1f}Hz "
              f"(唯一样本={unique} 投递样本={collector.eeg_samples} "
              f"通道={collector.channel_count} 时长={collect_duration:.1f}s)", flush=True)
        rate_results.append((rate, rate_ok, f"期望={expected_rate}Hz, 实际≈{actual_rate:.1f}Hz（唯一样本={unique}）"))

    all_rate_ok = all(ok for _, ok, _ in rate_results if ok is not None)
    for rate, ok, detail in rate_results:
        if ok is None:
            record(results, f"EEG_SAMPLE_RATE={rate} 数据率验证", None,
                   f"实际采样率 ≈ {rate}Hz", detail)
        else:
            record(results, f"EEG_SAMPLE_RATE={rate} 数据率验证", ok,
                   f"实际采样率 ≈ {rate}Hz（容差 10%）", detail)

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