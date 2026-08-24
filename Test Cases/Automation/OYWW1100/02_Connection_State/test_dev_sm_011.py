# -*- coding: utf-8 -*-
"""DEV-SM-011：多台设备同时 connect 互不干扰。

对应用例：02_连接与状态机.md -> DEV-SM-011
可自动化：auto（需 TARGET_IDENTITY 中配置 ≥2 台设备 identity，如 "80F9,80F3"）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 环境中 ≥2 台设备开机且在范围内
  - config.py 中 TARGET_IDENTITY 用逗号列出 ≥2 个 identity

流程：
  1) scan 匹配 common.TARGET_IDENTITIES 中所有设备（需 ≥2 台，否则 SKIP）
  2) 对每台 requireSensor -> connect -> 到达 Ready
  3) 全部连接后，断言每台仍各自 Ready（连接互不挤掉）
  4) 对每台 init -> startDataNotification 起流，各自 onDataCallback 收到数据
  5) 断开第一台，断言其余设备仍 Ready（断开互不影响）
  6) 清理：全部断开

互不干扰的可证伪点：
  - 连接第二台后第一台不被动断开（状态串扰）
  - 断开第一台后第二台不跟着断开（断开串扰）
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

READY_TIMEOUT = 15     # 单台连接后等待 Ready 超时（秒）
COLLECT_SECONDS = 3    # 起流后采集时长（秒）

from common import record, _identity_of, scan_and_match_all


def _tag_of(d):
    """设备短标识：优先广播名后四位，其次地址。"""
    name = getattr(d, 'Name', '') or ''
    ident = _identity_of(name)
    return ident or (getattr(d, 'Address', '') or '?').upper()


def _wait_until(cond, timeout, interval=0.5, what=""):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(interval)
    print(f"  [等待超时] {what}（{timeout}s）", flush=True)
    return False


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DEV-SM-011 多台设备同时 connect 互不干扰", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 环境中 ≥2 台设备开机且在范围内", flush=True)
    print(f"  - config.py 中 TARGET_IDENTITY = {config.TARGET_IDENTITY!r}（{len(common.TARGET_IDENTITIES)} 个目标）", flush=True)

    input("\n>>> [人工操作] 请确认 ≥2 台设备已【开机】且在范围内，"
          "且 config.py 中 TARGET_IDENTITY 已列出 ≥2 个 identity，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    matched, devices = scan_and_match_all(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, required=2)
    print(f"[扫描] 匹配到 {len(matched)} 台目标设备", flush=True)

    if len(matched) < 2:
        record(results, "环境中存在 ≥2 台目标设备", None,
               "scan 匹配到 ≥2 台目标设备", f"仅 {len(matched)} 台")
        print(f"\n[SKIP] 需要 ≥2 台设备。当前 TARGET_IDENTITY = {config.TARGET_IDENTITY!r}（{len(common.TARGET_IDENTITIES)} 个目标），"
              f"请确保 ≥2 台设备在范围内且 identity 已列入 TARGET_IDENTITY。", flush=True)

        print("\n" + "=" * 60, flush=True)
        print("测试结果汇总", flush=True)
        print("=" * 60, flush=True)
        for rname, status, expect, actual in results:
            print(f"  [SKIP] {rname}（{actual}）", flush=True)
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    record(results, "环境中存在 ≥2 台目标设备", True,
           "scan 匹配到 ≥2 台目标设备", f"{len(matched)} 台")

    # requireSensor
    sensors = []  # [(tag, sensor, device)]
    for tid, d in matched:
        tag = _tag_of(d)
        sensor = ctrl.requireSensor(d)
        ok = isinstance(sensor, SensorProfile)
        record(results, f"[{tag}] requireSensor 返回 SensorProfile", ok,
               "返回 SensorProfile", f"返回 {type(sensor).__name__}")
        if ok:
            sensors.append((tag, sensor, d))
        else:
            print(f"[FAIL] [{tag}] requireSensor 返回 {sensor!r}", flush=True)

    # connect 所有（串行，连接后各自等待 Ready）
    for tag, sensor, d in sensors:
        print(f"\n[连接] [{tag}] SensorProfile.connect() ...", flush=True)
        try:
            ok = sensor.connect()
            connect_txt = f"返回 {ok}"
        except Exception as e:
            ok = None
            connect_txt = f"抛异常 {type(e).__name__}: {e}"
        print(f"[连接] [{tag}] connect() -> {connect_txt}  state={sensor.deviceState}", flush=True)
        record(results, f"[{tag}] connect 返回 True", ok is True,
               "connect() 返回 True", f"connect() -> {connect_txt}")

        _wait_until(lambda s=sensor: s.deviceState == DeviceStateEx.Ready, READY_TIMEOUT, 0.2, f"[{tag}] 等待 Ready")
        record(results, f"[{tag}] 到达 Ready", sensor.deviceState == DeviceStateEx.Ready,
               "deviceState==Ready", f"state={sensor.deviceState}")

    # 互不干扰-连接：全部连接后，每台仍各自 Ready（连接后续设备不挤掉已连设备）
    all_ready = all(sensor.deviceState == DeviceStateEx.Ready for _, sensor, _ in sensors)
    ready_tags = [tag for tag, sensor, _ in sensors if sensor.deviceState == DeviceStateEx.Ready]
    record(results, "连接后每台设备各自保持 Ready（无串扰）", all_ready,
           "所有设备 deviceState==Ready", f"Ready: {ready_tags} / 全部: {[t for t,_,_ in sensors]}")

    # init + 起流 + 收数据
    data_counts = {}

    def make_on_data(tag):
        def on_data(sensor, data):
            items = data if isinstance(data, list) else [data]
            n = 0
            for dd in items:
                cs = getattr(dd, 'channelSamples', None)
                if cs:
                    try:
                        n += sum(len(ch) for ch in cs)
                    except TypeError:
                        n += len(cs)
            data_counts[tag] = data_counts.get(tag, 0) + n
        return on_data

    for tag, sensor, d in sensors:
        sensor.onDataCallback = make_on_data(tag)

        print(f"\n[init] [{tag}] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
        try:
            iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
            init_txt = f"返回 {iret}"
        except Exception as e:
            iret = None
            init_txt = f"抛异常 {type(e).__name__}: {e}"
        print(f"[init] [{tag}] init() -> {init_txt}", flush=True)
        record(results, f"[{tag}] init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

        print(f"[起流] [{tag}] SensorProfile.startDataNotification() ...", flush=True)
        try:
            sret = sensor.startDataNotification()
            start_txt = f"返回 {sret}"
        except Exception as e:
            sret = None
            start_txt = f"抛异常 {type(e).__name__}: {e}"
        print(f"[起流] [{tag}] startDataNotification() -> {start_txt}", flush=True)
        record(results, f"[{tag}] startDataNotification 返回 True", sret is True,
               "startDataNotification() 返回 True", f"startDataNotification() -> {start_txt}")

        record(results, f"[{tag}] 起流后 isDataTransfering==True", sensor.isDataTransfering is True,
               "isDataTransfering == True", f"isDataTransfering == {sensor.isDataTransfering}")

    # 采集数据
    print(f"\n[采集] 等待 {COLLECT_SECONDS}s 收集各设备数据 ...", flush=True)
    time.sleep(COLLECT_SECONDS)
    for tag, sensor, d in sensors:
        n = data_counts.get(tag, 0)
        record(results, f"[{tag}] 收到数据（样本数>0）", n > 0,
               "onDataCallback 收到样本", f"样本数={n}")

    # 互不干扰-断开：断开第一台，其余设备仍 Ready
    if len(sensors) >= 2:
        first_tag, first_sensor, _ = sensors[0]
        print(f"\n[断开干扰验证] 断开第一台 [{first_tag}]，观察其余设备状态 ...", flush=True)
        try:
            first_sensor.disconnect()
        except Exception as e:
            print(f"[断开] [{first_tag}] disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

        time.sleep(2)  # 给状态传播留时间
        others_ready = all(sensor.deviceState == DeviceStateEx.Ready for _, sensor, _ in sensors[1:])
        others_tags = [tag for tag, sensor, _ in sensors[1:] if sensor.deviceState == DeviceStateEx.Ready]
        record(results, "断开第一台后其余设备仍 Ready（无串扰）", others_ready,
               "其余设备 deviceState 仍 == Ready",
               f"断开 [{first_tag}] 后，其余 Ready: {others_tags} / 其余全部: {[t for t,_,_ in sensors[1:]]}")

    # 清理
    for tag, sensor, d in sensors:
        try:
            sensor.stopDataNotification()
        except Exception:
            pass
        try:
            sensor.disconnect()
        except Exception as e:
            print(f"[断开] [{tag}] disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

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