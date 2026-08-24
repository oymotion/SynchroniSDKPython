# -*- coding: utf-8 -*-
"""PARAM-FUNC-007：起流中修改参数自动停流重起并生效。

对应用例：04_参数.md -> PARAM-FUNC-007
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) setParam("NTF_EEG","ON") 起 EEG 流，startDataNotification，确认收到数据
  3) 起流中 setParam("FILTER_50HZ","ON")（README：起流中改 NTF_*/FILTER_* 会
     自动停流重起），校验返回 "OK"
  4) getParam("FILTER") 反映新值（FILTER_50HZ=ON）
  5) 改参后继续观察窗口，确认数据流自动重起（样本数持续增长）
  6) 若设备有 EEG 能力，起流中改 EEG_SAMPLE_RATE 也自动停流重起（无能力则跳过）

说明：
  用 FILTER 键触发自动停流重起，避免改动 NTF 主数据流内容；判定"流重起"
  用 onDataCallback 样本计数（改参后仍增长），不依赖 isDataTransfering。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
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

OBSERVE_SECONDS = 3  # 改参后观察窗口（秒），确认流自动重起


def _parse_pipe(s):
    out = {}
    if not s:
        return out
    parts = s.split("|")
    for i in range(0, len(parts) - 1, 2):
        out[parts[i]] = parts[i + 1]
    return out


class SampleCounter:
    def __init__(self):
        self.batches = 0

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        self.batches += len(items)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-007 起流中修改参数自动停流重起并生效", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

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

    # 起 EEG 流
    print("\n[起流] SensorProfile.setParam('NTF_EEG', 'ON') ...", flush=True)
    try:
        p_ret = sensor.setParam("NTF_EEG", "ON")
        p_txt = f"返回 {p_ret!r}"
    except Exception as e:
        p_ret = None
        p_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] setParam('NTF_EEG', 'ON') -> {p_txt}", flush=True)

    counter = SampleCounter()
    sensor.onDataCallback = counter.on_data

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

    # 等待首批数据
    print("\n[采集] 等待首批数据到达 ...", flush=True)
    t_wait = time.time()
    while counter.batches == 0 and time.time() - t_wait < 15:
        time.sleep(0.1)
    before = counter.batches
    print(f"[采集] 起流后首批观察批次 = {before}", flush=True)
    if before == 0:
        record(results, "起流后收到数据", False, "起流后收到数据", "未收到数据")
        print("[FAIL] 起流后未收到数据，无法继续", flush=True)
        try:
            sensor.stopDataNotification()
        except Exception:
            pass
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "起流后收到数据", True, "起流后收到数据", f"批次={before}")

    # 起流中修改 FILTER 参数
    print("\n[改参] 起流中 setParam('FILTER_50HZ', 'ON') ...", flush=True)
    try:
        f_ret = sensor.setParam("FILTER_50HZ", "ON")
    except Exception as e:
        f_ret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[改参] setParam('FILTER_50HZ', 'ON') -> {f_ret!r}", flush=True)

    # 读回 FILTER 状态
    try:
        filt = _parse_pipe(sensor.getParam("FILTER"))
    except Exception as e:
        filt = {}
        print(f"[改参] getParam('FILTER') 抛异常 {type(e).__name__}: {e}", flush=True)
    f50 = filt.get("FILTER_50HZ", "<缺失>")
    print(f"[改参] getParam('FILTER') 中 FILTER_50HZ = {f50}", flush=True)

    ok_ret = (f_ret == "OK")
    ok_get = (f50 == "ON")
    record(results, "起流中改 FILTER 返回 OK 且 getParam 生效",
           ok_ret and ok_get,
           "setParam 返回 'OK' 且 getParam('FILTER') 中 FILTER_50HZ='ON'",
           f"setParam->{f_ret!r} FILTER_50HZ={f50}")

    # 改参后观察流是否自动重起（数据继续到达）
    print(f"[改参] 观察 {OBSERVE_SECONDS}s，确认流自动重起 ...", flush=True)
    time.sleep(OBSERVE_SECONDS)
    after = counter.batches
    grew = (after > before)
    print(f"[改参] 改参前批次={before}，改参后批次={after}，增长={after - before}", flush=True)
    record(results, "改参后数据流自动重起（样本继续到达）",
           grew,
           "改参后观察窗口内批次继续增长",
           f"改参前={before} 改参后={after}")

    # 起流中修改 EEG_SAMPLE_RATE（若设备有 EEG 能力）
    print("\n[改参] 检查 EEG 能力，测试起流中改 EEG_SAMPLE_RATE ...", flush=True)
    try:
        eeg_list = sensor.getParam("EEG_SAMPLE_RATE_LIST")
    except Exception as e:
        eeg_list = f"抛异常 {type(e).__name__}: {e}"
    print(f"[改参] getParam('EEG_SAMPLE_RATE_LIST') = {eeg_list!r}", flush=True)

    if isinstance(eeg_list, str) and not eeg_list.startswith("Error"):
        eeg_rates = [x.strip() for x in eeg_list.split("|") if x.strip()]
        if len(eeg_rates) >= 2:
            target_rate = eeg_rates[1]
            before_eeg = counter.batches
            try:
                e_ret = sensor.setParam("EEG_SAMPLE_RATE", target_rate)
            except Exception as e:
                e_ret = f"抛异常 {type(e).__name__}: {e}"
            print(f"[改参] 起流中 setParam('EEG_SAMPLE_RATE', {target_rate!r}) -> {e_ret!r}", flush=True)
            time.sleep(OBSERVE_SECONDS)
            after_eeg = counter.batches
            grew_eeg = (after_eeg > before_eeg)
            print(f"[改参] 改 EEG 前批次={before_eeg}，改后批次={after_eeg}", flush=True)
            record(results, "起流中改 EEG_SAMPLE_RATE 返回 OK 且流自动重起",
                   (e_ret == "OK" and grew_eeg),
                   "setParam 返回 'OK' 且改参后批次继续增长",
                   f"setParam->{e_ret!r} 改前={before_eeg} 改后={after_eeg}")
        else:
            record(results, "起流中改 EEG_SAMPLE_RATE 自动停流重起", None,
                   "列表需 ≥2 个值才能切换", f"列表={eeg_list!r}")
    else:
        record(results, "起流中改 EEG_SAMPLE_RATE 自动停流重起", None,
               "设备有 EEG 能力时校验", f"getParam('EEG_SAMPLE_RATE_LIST')={eeg_list!r}，无能力")

    # 清理
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.setParam("FILTER_50HZ", "OFF")
    except Exception:
        pass
    try:
        sensor.setParam("NTF_EEG", "OFF")
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
