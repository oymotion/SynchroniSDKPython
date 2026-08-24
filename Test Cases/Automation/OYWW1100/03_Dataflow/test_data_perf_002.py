# -*- coding: utf-8 -*-
"""DATA-PERF-002：连续起流 >=30min 无泄漏/掉速。

对应用例：03_数据流.md -> DATA-PERF-002
可自动化：semi-auto（人工值守确认设备供电/环境稳定，异常时终止）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) setParam("NTF_EMG", "ON") 起 EMG 流
  3) 连续采集 TOTAL_SECONDS（默认 30min），分窗口统计采样率与内存占用
  4) 校验：
     - 全程持续收到数据（无长时间中断）
     - 无掉速：末段平均采样率 相对 首段平均采样率 下降 <= 5%
     - 无内存泄漏：末窗口内存 相对 首窗口内存 增长 <= 阈值（默认 50MB）

说明：
  掉速检测用"前后段对比"而非绝对精度，因此即使 channelSamples 存在恒定比例的
  重复（DATA-PERF-001 已发现），也不影响"后期是否相对前期下降"的结论。
  内存用 Windows 进程 WorkingSetSize（RSS）观测，通过 ctypes 调用 psapi，无额外依赖。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内
  - 人工值守：测试约 30min，异常时可 Ctrl+C 终止（会输出已采集的中间统计）
"""

import os
import re
import sys
import time
import ctypes
from ctypes import wintypes

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match

TOTAL_SECONDS = 1800          # 连续起流总时长（秒），默认 30min
WINDOW_SECONDS = 60           # 分窗口统计间隔（秒）
DROP_TOLERANCE = 0.05         # 掉速容差：末段相对首段下降 >5% 判掉速
MEMORY_LEAK_THRESHOLD_MB = 50  # 内存泄漏阈值：末窗口相对首窗口增长超过该值判泄漏
EDGE_WINDOWS = 3              # 取首/末各 N 个窗口做平均对比


class _PROCESS_MEMORY_COUNTERS(ctypes.Structure):
    _fields_ = [
        ("cb", wintypes.DWORD),
        ("PageFaultCount", wintypes.DWORD),
        ("PeakWorkingSetSize", ctypes.c_size_t),
        ("WorkingSetSize", ctypes.c_size_t),
        ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
        ("QuotaPagedPoolUsage", ctypes.c_size_t),
        ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
        ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
        ("PagefileUsage", ctypes.c_size_t),
        ("PeakPagefileUsage", ctypes.c_size_t),
    ]


def get_rss_bytes():
    """当前进程 WorkingSetSize（字节），失败返回 None。"""
    try:
        psapi = ctypes.windll.psapi
        kernel32 = ctypes.windll.kernel32

        kernel32.GetCurrentProcess.restype = wintypes.HANDLE
        kernel32.GetCurrentProcess.argtypes = []
        psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
        psapi.GetProcessMemoryInfo.argtypes = [
            wintypes.HANDLE,
            ctypes.POINTER(_PROCESS_MEMORY_COUNTERS),
            wintypes.DWORD,
        ]

        h = kernel32.GetCurrentProcess()
        counters = _PROCESS_MEMORY_COUNTERS()
        counters.cb = ctypes.sizeof(_PROCESS_MEMORY_COUNTERS)
        if psapi.GetProcessMemoryInfo(h, ctypes.byref(counters), counters.cb):
            return counters.WorkingSetSize
    except Exception:
        pass
    return None


class LongRunCollector:
    """持续累计每通道样本数（时间维度），供分窗口快照。"""

    def __init__(self):
        self.total_samples = 0
        self.batches = 0

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            self.batches += 1
            cs = getattr(d, 'channelSamples', None)
            if cs:
                try:
                    n_ch = len(cs)
                    self.total_samples += len(cs[0]) if n_ch else 0
                except TypeError:
                    self.total_samples += len(cs)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-PERF-002 连续起流 >=30min 无泄漏/掉速", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"总时长 = {TOTAL_SECONDS}s（{TOTAL_SECONDS / 60:.0f}min），窗口 = {WINDOW_SECONDS}s，"
          f"掉速容差 = {DROP_TOLERANCE:.0%}，内存泄漏阈值 = {MEMORY_LEAK_THRESHOLD_MB}MB", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)
    print("  - 人工值守：全程约 30min，异常时 Ctrl+C 终止（会输出已采集统计）", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内、供电稳定，"
          "测试中请保持环境稳定，完成后按回车继续 ...")

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

    collector = LongRunCollector()
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

    # 等待首批数据到达（排除起流延迟）
    print("\n[采集] 等待首批数据到达 ...", flush=True)
    t_wait0 = time.time()
    while collector.total_samples == 0 and time.time() - t_wait0 < 15:
        time.sleep(0.1)
    if collector.total_samples == 0:
        print("[采集] 15s 内未收到任何数据，终止", flush=True)
        record(results, "全程持续收到数据", False, "采集期间持续收到数据", "未收到数据")
        print("\n结论: FAIL", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    # 分窗口长稳采集
    num_windows = int(TOTAL_SECONDS // WINDOW_SECONDS)
    window_rates = []
    window_rss_mb = []
    prev_total = 0

    print(f"\n[采集] 开始长稳采集，共 {num_windows} 个窗口，每个 {WINDOW_SECONDS}s ...", flush=True)
    print("[采集] 提示：异常时按 Ctrl+C 可提前终止并输出已采集统计", flush=True)

    try:
        for i in range(num_windows):
            win_start = time.time()
            while time.time() - win_start < WINDOW_SECONDS:
                time.sleep(0.5)

            cur_total = collector.total_samples
            window_samples = cur_total - prev_total
            prev_total = cur_total
            rate = window_samples / WINDOW_SECONDS

            rss = get_rss_bytes()
            rss_mb = (rss / (1024 * 1024)) if rss is not None else None
            window_rates.append(rate)
            window_rss_mb.append(rss_mb)

            rss_txt = f"{rss_mb:.1f}MB" if rss_mb is not None else "N/A"
            print(f"[窗口 {i + 1}/{num_windows}] 样本={window_samples} 采样率={rate:.1f}Hz 内存={rss_txt}", flush=True)
    except KeyboardInterrupt:
        print("\n[采集] 被用户中断，输出已采集的中间统计 ...", flush=True)

    # 停流
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.setParam("NTF_EMG", "OFF")
    except Exception:
        pass

    # ---- 判定 ----
    print("\n" + "=" * 60, flush=True)
    print("长稳统计", flush=True)
    print("=" * 60, flush=True)

    n_done = len(window_rates)
    print(f"[统计] 完成窗口数 = {n_done} / {num_windows}，总累计样本（每通道） = {collector.total_samples}，"
          f"总批次数 = {collector.batches}", flush=True)

    # 判定 1：全程持续收到数据（每个窗口样本数 > 0）
    all_windows_nonempty = n_done > 0 and all(s > 0 for s in [window_rates[i] * WINDOW_SECONDS for i in range(n_done)])
    record(results, "全程持续收到数据（各窗口样本数>0）", all_windows_nonempty,
           "每个窗口均收到数据", f"完成窗口数={n_done}")

    # 判定 2：无掉速（末段 vs 首段平均采样率）
    if n_done >= 2 * EDGE_WINDOWS:
        head = sum(window_rates[:EDGE_WINDOWS]) / EDGE_WINDOWS
        tail = sum(window_rates[-EDGE_WINDOWS:]) / EDGE_WINDOWS
        drop = (head - tail) / head if head > 0 else 0.0
        print(f"[掉速] 首 {EDGE_WINDOWS} 窗口平均={head:.1f}Hz，末 {EDGE_WINDOWS} 窗口平均={tail:.1f}Hz，"
              f"下降={drop:.2%}", flush=True)
        record(results, "无掉速（末段相对首段下降 <= 5%）", drop <= DROP_TOLERANCE,
               f"末段相对首段下降 <= {DROP_TOLERANCE:.0%}",
               f"首段={head:.1f}Hz 末段={tail:.1f}Hz 下降={drop:.2%}")
    else:
        record(results, "无掉速（末段相对首段下降 <= 5%）", False,
               f"需至少 {2 * EDGE_WINDOWS} 个窗口", f"完成窗口数={n_done}")

    # 判定 3：无内存泄漏（末窗口 vs 首窗口 RSS）
    valid_rss = [m for m in window_rss_mb if m is not None]
    if len(valid_rss) >= 2:
        head_rss = valid_rss[0]
        tail_rss = valid_rss[-1]
        growth = tail_rss - head_rss
        print(f"[内存] 首窗口={head_rss:.1f}MB，末窗口={tail_rss:.1f}MB，增长={growth:.1f}MB", flush=True)
        record(results, f"无内存泄漏（增长 <= {MEMORY_LEAK_THRESHOLD_MB}MB）",
               growth <= MEMORY_LEAK_THRESHOLD_MB,
               f"末窗口相对首窗口增长 <= {MEMORY_LEAK_THRESHOLD_MB}MB",
               f"首={head_rss:.1f}MB 末={tail_rss:.1f}MB 增长={growth:.1f}MB")
    else:
        record(results, f"无内存泄漏（增长 <= {MEMORY_LEAK_THRESHOLD_MB}MB）", False,
               "需要至少 2 个有效内存采样", f"有效内存采样数={len(valid_rss)}")

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
