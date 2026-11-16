# -*- coding: utf-8 -*-
"""验证「关闭蓝牙断连 → 重开蓝牙 → 手动重连」的逻辑。

场景：
  1) 连接设备（不设置自动重连 autoReconnect=False）→ init → 起流，确认有数据
  2) 提示用户【关闭】系统蓝牙 → 等待 10 秒（期间设备应断开）
  3) 提示用户【开启】系统蓝牙 → 用户确认后
  4) 尝试重新连接设备 → init → 起流 → 采集 10 秒数据 → 结束

目的：验证关闭/重开系统蓝牙后，SDK 在【未开启自动重连】的情况下，
      能否通过手动 connect + init + startDataNotification 恢复正常数据流。

前置条件：
  - 电脑蓝牙开启；待测设备（config.TARGET_IDENTITY 默认）上电、在范围内
  - 测试过程中需手动关闭/开启系统蓝牙各一次（按提示操作）
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))            # .../reconnect after close bluetooth/
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))      # .../Automation/
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import scan_and_match

READY_TIMEOUT_S = 15                # 到达 Ready 的等待上限
INIT_TIMEOUT_MS = 10000             # init(..., timeoutInMS)
BT_OFF_WAIT_SECONDS = 10            # 关闭蓝牙后等待观察的秒数
POST_RECONNECT_COLLECT_SECONDS = 10  # 重连成功后采集数据的秒数


def _log_ts(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


class DataMonitor:
    """统计收到的数据回调，用于确认数据流是否真的恢复。"""

    def __init__(self):
        self.batches = 0
        self.samples = 0
        self.types = set()

    def on_data(self, sensor, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        self.batches += len(items)
        for d in items:
            try:
                dt = d.getDataType()
                self.types.add(dt)
                n_ch = d.getChannelCount()
                n_smp = d.getSampleCount()
                if n_ch > 0 and n_smp > 0:
                    self.samples += n_ch * n_smp
            except Exception:
                pass

    def summary(self):
        names = []
        for t in self.types:
            try:
                names.append(t.name)
            except Exception:
                names.append(str(t))
        return (f"batches={self.batches} samples={self.samples} "
                f"types={','.join(sorted(names)) if names else '-'}")


def _connect_init_start(sensor, label):
    """对已 requireSensor 的 profile 执行 connect → Ready → init → startDataNotification。"""
    sensor.autoReconnect = False
    print(f"\n[{label}] connect() ...", flush=True)
    try:
        sensor.connect()
    except Exception as e:
        print(f"[{label}] connect 抛异常 {type(e).__name__}: {e}", flush=True)
        return False

    t0 = time.time()
    while time.time() - t0 < READY_TIMEOUT_S and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[{label}] 未到达 Ready（state={sensor.deviceState}）", flush=True)
        return False
    print(f"[{label}] 到达 Ready", flush=True)

    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT,
                           config.POWER_REFRESH_INTERVAL_MS,
                           INIT_TIMEOUT_MS)
        print(f"[{label}] init -> {iret}", flush=True)
    except Exception as e:
        print(f"[{label}] init 抛异常 {type(e).__name__}: {e}", flush=True)
        return False

    try:
        sret = sensor.startDataNotification()
        print(f"[{label}] startDataNotification -> {sret}", flush=True)
    except Exception as e:
        print(f"[{label}] startDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)
        return False
    return True


def main():
    ctrl = SensorControllerInstance

    print("=" * 70, flush=True)
    print("验证：关闭蓝牙断连 → 重开蓝牙 → 手动重连", flush=True)
    print("=" * 70, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"目标设备 identity = {config.TARGET_IDENTITY}", flush=True)

    if ctrl.isEnable is not True:
        print("[失败] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    # 状态/错误回调（观察蓝牙关闭时的断连）
    def on_state(s, st):
        _log_ts(f"[onStateChanged] {getattr(s.BLEDevice, 'Name', '?')} -> {st}")

    def on_error(s, reason):
        _log_ts(f"[onErrorCallback] {getattr(s.BLEDevice, 'Name', '?')}: {reason!r}")

    # ---- 阶段 1：连接 + init + 起流 ----
    print(f"\n[阶段1] 扫描目标设备（identity={config.TARGET_IDENTITY}）...", flush=True)
    target, _ = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[失败] 未匹配到目标设备，退出", flush=True)
        ctrl.terminate()
        return
    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[阶段1] 目标设备: {name} {addr}", flush=True)

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[失败] requireSensor 返回 None，退出", flush=True)
        ctrl.terminate()
        return

    sensor.onStateChanged = on_state
    sensor.onErrorCallback = on_error

    monitor1 = DataMonitor()
    sensor.onDataCallback = monitor1.on_data

    if not _connect_init_start(sensor, "阶段1"):
        print("[失败] 阶段1 连接/init/起流失败，退出", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    time.sleep(3)
    print(f"[阶段1] 起流 3s 后数据: {monitor1.summary()}", flush=True)

    # ---- 阶段 2：关闭蓝牙 ----
    input("\n>>> [人工操作] 请【关闭】系统蓝牙，关闭完成后按回车继续 ...")

    print(f"\n[阶段2] 等待 {BT_OFF_WAIT_SECONDS}s 观察断开 ...", flush=True)
    for i in range(BT_OFF_WAIT_SECONDS):
        time.sleep(1)
        print(f"[阶段2] +{i + 1}s  deviceState={sensor.deviceState}", flush=True)

    # ---- 阶段 3：开启蓝牙 ----
    input("\n>>> [人工操作] 请【开启】系统蓝牙，开启完成后按回车继续 ...")
    # 给设备一点时间重新广播
    time.sleep(2)

    # ---- 阶段4 前清理旧状态：确保旧 profile 已停流/断开，避免残留状态影响重连 ----
    print("\n[清理] 重连前清理旧 profile 状态 ...", flush=True)
    try:
        sensor.stopDataNotification()
        print("[清理] stopDataNotification() 已调用", flush=True)
    except Exception as e:
        print(f"[清理] stopDataNotification() 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        sensor.disconnect()
        print("[清理] disconnect() 已调用", flush=True)
    except Exception as e:
        print(f"[清理] disconnect() 抛异常 {type(e).__name__}: {e}", flush=True)
    time.sleep(1)
    print(f"[清理] 清理后 deviceState={sensor.deviceState}", flush=True)

    # ---- 阶段 4：重新连接 + init + 起流 + 采集 10s ----
    print(f"\n[阶段4] 重新扫描并尝试连接 ...", flush=True)
    target2, _ = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target2 is not None:
        sensor2 = ctrl.requireSensor(target2)
        print(f"[阶段4] 重新扫描到设备: {getattr(target2, 'Name', '?')} {getattr(target2, 'Address', '?')}", flush=True)
    else:
        print("[阶段4] 重新扫描未匹配到设备，改用原 profile 直接重连", flush=True)
        sensor2 = sensor

    if sensor2 is None:
        print("[失败] 阶段4 requireSensor 返回 None，退出", flush=True)
        ctrl.terminate()
        return

    sensor2.onStateChanged = on_state
    sensor2.onErrorCallback = on_error
    monitor2 = DataMonitor()
    sensor2.onDataCallback = monitor2.on_data

    if not _connect_init_start(sensor2, "阶段4"):
        print("[失败] 阶段4 重新连接/init/起流失败，退出", flush=True)
        try:
            sensor2.disconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    print(f"[阶段4] 采集 {POST_RECONNECT_COLLECT_SECONDS}s 数据 ...", flush=True)
    time.sleep(POST_RECONNECT_COLLECT_SECONDS)

    # ---- 收尾 ----
    try:
        sensor2.stopDataNotification()
    except Exception:
        pass
    try:
        sensor2.disconnect()
    except Exception:
        pass
    ctrl.terminate()

    # ---- 结论 ----
    print("\n" + "=" * 70, flush=True)
    print("结论", flush=True)
    print("=" * 70, flush=True)
    print(f"阶段1 起流数据: {monitor1.summary()}", flush=True)
    print(f"阶段4 重连后数据: {monitor2.summary()}", flush=True)

    reconnected_ok = monitor2.batches > 0
    if reconnected_ok:
        print("\n[通过] 关闭/重开蓝牙后，未开启自动重连，手动 connect + init + "
              f"startDataNotification 成功恢复正常数据流（收到 {monitor2.batches} 批数据）。", flush=True)
    else:
        print("\n[未通过] 阶段4 未收到数据，重连或起流失败（需进一步排查）。", flush=True)


if __name__ == "__main__":
    main()
