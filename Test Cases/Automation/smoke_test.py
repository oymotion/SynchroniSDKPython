# -*- coding: utf-8 -*-
"""Smoke test：对配置中每台设备执行 扫描->连接->init->起流->收数->停流->断开。

用法：
    python smoke_test.py

设备信息见 config.py（支持多台）。
要求 sensor-sdk 0.8.0（使用 getVersion/getBLEBackendName/isReady 等新接口）。
onDataCallback 兼容单个 SensorData 与 list 两种形式。
"""

import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor import *
import config


def _dt_name(dt):
    try:
        if isinstance(dt, DataType):
            return dt.name
        return DataType(dt).name
    except Exception:
        return str(dt)


def _identity_of(name):
    """从广播名（如 "OB6000C(6C6B)"）中提取括号内的后四位，无则返回 None。"""
    m = re.search(r"\(([0-9A-Fa-f]{4})\)", name or "")
    return m.group(1).upper() if m else None


class DeviceResult:
    def __init__(self):
        self.batches = 0
        self.total_samples = 0
        self.data_types = set()


def make_on_data(result):
    def on_data(sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            result.batches += 1
            cs = getattr(d, 'channelSamples', None)
            n = 0
            if cs:
                try:
                    n = sum(len(ch) for ch in cs)
                except TypeError:
                    n = len(cs)
            result.total_samples += n
            dt = d.getDataType()
            if dt is not None:
                result.data_types.add(_dt_name(dt))
    return on_data


def main():
    ctrl = SensorControllerInstance

    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"scanning {config.SCAN_TIMEOUT_MS} ms ...", flush=True)
    devices = ctrl.scan(config.SCAN_TIMEOUT_MS)
    print(f"found {len(devices)} device(s):", flush=True)
    for d in devices:
        print(f"  - {getattr(d, 'Name', '?')} {getattr(d, 'Address', '?')}", flush=True)

    matched = []
    for cfg in config.DEVICES:
        if not cfg.get("enabled", True):
            continue
        mac = (cfg.get("mac") or "").strip().upper()
        identity = (cfg.get("identity") or "").strip().upper()
        target = None
        for d in devices:
            addr = (getattr(d, 'Address', '') or '').upper()
            name = getattr(d, 'Name', '') or ''
            if mac and addr == mac:
                target = d
                break
            if identity and _identity_of(name) == identity:
                target = d
                break
        if target is None:
            print(f"[FAIL] 未匹配到设备: {cfg}", flush=True)
            continue
        matched.append((cfg, target))

    if not matched:
        print("no enabled device matched, abort", flush=True)
        ctrl.terminate()
        return

    summary = []
    for cfg, target in matched:
        name = getattr(target, 'Name', '?')
        addr = getattr(target, 'Address', '?')
        print(f"\n=== 测试设备: {name} {addr} ===", flush=True)

        result = DeviceResult()
        sensor = ctrl.requireSensor(target)
        if sensor is None:
            print("[FAIL] requireSensor 返回 None", flush=True)
            summary.append((name, "FAIL"))
            continue

        sensor.onDataCallback = make_on_data(result)
        sensor.onStateChanged = lambda s, st: print(f"  [state] {s.BLEDevice.Name} -> {st}", flush=True)
        sensor.onErrorCallback = lambda s, r: print(f"  [error] {s.BLEDevice.Name}: {r}", flush=True)

        ok = sensor.connect()
        print(f"  connect() -> {ok}  state={sensor.deviceState}", flush=True)
        if not ok:
            print("[FAIL] connect 返回 False", flush=True)
            summary.append((name, "FAIL"))
            sensor.disconnect()
            continue

        for _ in range(50):
            if sensor.deviceState == DeviceStateEx.Ready:
                break
            time.sleep(0.2)

        if sensor.deviceState != DeviceStateEx.Ready or not sensor.isReady:
            print(f"[FAIL] 未到 Ready: state={sensor.deviceState} isReady={sensor.isReady}", flush=True)
            summary.append((name, "FAIL"))
            sensor.disconnect()
            continue

        if not sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS):
            print("[FAIL] init 失败", flush=True)
            summary.append((name, "FAIL"))
            sensor.disconnect()
            continue

        if not sensor.startDataNotification():
            print("[FAIL] startDataNotification 失败", flush=True)
            summary.append((name, "FAIL"))
            sensor.disconnect()
            continue

        print(f"  采集 {config.COLLECT_SECONDS}s ...", flush=True)
        time.sleep(config.COLLECT_SECONDS)

        print(f"  batches={result.batches} samples={result.total_samples} types={sorted(result.data_types)}", flush=True)

        try:
            battery = sensor.getBatteryLevel()
        except Exception as e:
            print(f"[FAIL] 电池读取异常: {e}", flush=True)
            summary.append((name, "FAIL"))
            sensor.stopDataNotification()
            sensor.disconnect()
            continue

        print(f"  电池={battery}", flush=True)
        if battery < 0 or battery > 100:
            print(f"[FAIL] 电池读取异常: {battery}", flush=True)
            summary.append((name, "FAIL"))
            sensor.stopDataNotification()
            sensor.disconnect()
            continue

        sensor.stopDataNotification()
        sensor.disconnect()

        status = "PASS" if result.total_samples >= config.MIN_SAMPLES else "FAIL"
        summary.append((name, status))

    print("\n=== 汇总 ===", flush=True)
    for name, status in summary:
        print(f"  [{status}] {name}", flush=True)

    ctrl.terminate()
    print("DONE", flush=True)


if __name__ == '__main__':
    main()
