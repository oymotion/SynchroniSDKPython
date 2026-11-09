# -*- coding: utf-8 -*-
"""探测角度流 setParam 流开关 key 的实际行为。

背景：ORehabArm 是纯运动/姿态设备。DataType 在 1.4.x 已拆分：
  - NTF_MAG(0x3)       磁力计，独立流
  - NTF_ANGLE_DATA(0x0D) 角度，独立流
README 的 setParam 章节仍写 `NTF_MAG_ANGLE`（旧版本遗留），需实测确认。

探测分三阶段：
  阶段1：验证 README 里的 NTF_MAG_ANGLE 是否有效（预期 Not supported）。
  阶段2：验证 getParam("NTF") 返回的开关列表中角度流 key `NTF_ANGLE`
        能否真正停/起角度流，以及是否影响磁力计/IMU。
  阶段3：验证磁力计是否有独立开关（NTF_MAG），或受 NTF_IMU 聚合开关控制。

用法：
    python probe_mag_angle.py          # 用 config.TARGET_IDENTITY（当前 1D0E）
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(BASE_DIR)
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *  # noqa: E402,F403
import config          # noqa: E402
from common import scan_and_match  # noqa: E402

OBSERVE_SECONDS = 3.0   # 每阶段观察时长（角度 250Hz / 磁 10Hz / IMU 50Hz 都能收到）


class BucketObserver:
    """按 DataType 分桶累计样本数，支持 clear() 分段观察。"""

    def __init__(self):
        self.by_type = {}  # dt_name -> 样本数累计

    def on_data(self, sensor, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        for d in items:
            try:
                dt = d.getDataType()
                name = dt.name if isinstance(dt, DataType) else f"0x{int(dt):X}"
                nch = d.getChannelCount()
                nsm = d.getSampleCount()
            except Exception:
                continue
            if nch <= 0 or nsm <= 0:
                continue
            self.by_type[name] = self.by_type.get(name, 0) + nch * nsm

    def clear(self):
        self.by_type = {}

    def snapshot(self):
        if not self.by_type:
            return "（无数据）"
        return ", ".join(f"{k}={v}" for k, v in sorted(self.by_type.items()))


def _try_get(sensor, key):
    try:
        return f"{sensor.getParam(key)!r}"
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def _try_set(sensor, key, val):
    try:
        return f"{sensor.setParam(key, val)!r}"
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def main():
    ctrl = SensorControllerInstance
    print("=" * 70, flush=True)
    print("探测 NTF_MAG_ANGLE setParam 流开关实际行为", flush=True)
    print("=" * 70, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)

    if ctrl.isEnable is not True:
        print("[失败] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    target, _devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[失败] 未匹配到目标设备（1D0E）。", flush=True)
        ctrl.terminate()
        return
    print(f"[扫描] {getattr(target, 'Name', '?')} {getattr(target, 'Address', '?')}", flush=True)

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[失败] requireSensor 返回 None。", flush=True)
        ctrl.terminate()
        return

    print("[连接] connect() ...", flush=True)
    sensor.connect()
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[失败] 未到 Ready, state={sensor.deviceState}", flush=True)
        ctrl.terminate()
        return
    print("[连接] Ready", flush=True)

    sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)

    obs = BucketObserver()
    sensor.onDataCallback = obs.on_data

    # 读当前开关状态
    print(f"[getParam] NTF = {_try_get(sensor, 'NTF')}", flush=True)
    print(f"[getParam] NTF_MAG_ANGLE = {_try_get(sensor, 'NTF_MAG_ANGLE')}", flush=True)

    # 起流
    try:
        sret = sensor.startDataNotification()
        print(f"[起流] startDataNotification() = {sret}", flush=True)
    except Exception as e:
        print(f"[起流] startDataNotification() 抛异常 {type(e).__name__}: {e}", flush=True)

    # 基线
    obs.clear()
    time.sleep(OBSERVE_SECONDS)
    print(f"[基线 {OBSERVE_SECONDS}s] {obs.snapshot()}", flush=True)

    # OFF
    print(f"[setParam] NTF_MAG_ANGLE=OFF -> {_try_set(sensor, 'NTF_MAG_ANGLE', 'OFF')}", flush=True)
    obs.clear()
    time.sleep(OBSERVE_SECONDS)
    print(f"[OFF 后 {OBSERVE_SECONDS}s] {obs.snapshot()}", flush=True)

    # 回读开关状态
    print(f"[getParam] NTF_MAG_ANGLE = {_try_get(sensor, 'NTF_MAG_ANGLE')}", flush=True)

    # ON
    print(f"[setParam] NTF_MAG_ANGLE=ON -> {_try_set(sensor, 'NTF_MAG_ANGLE', 'ON')}", flush=True)
    obs.clear()
    time.sleep(OBSERVE_SECONDS)
    print(f"[ON 后 {OBSERVE_SECONDS}s] {obs.snapshot()}", flush=True)

    # ---- 阶段 2：NTF_ANGLE ----
    # getParam("NTF") 返回的开关列表中角度流 key 为 NTF_ANGLE，
    # 而 NTF_MAG_ANGLE 该设备不支持；本阶段验证 NTF_ANGLE 的真实控制范围。
    print("\n" + "-" * 70, flush=True)
    print("[阶段2] NTF_ANGLE（来自 getParam('NTF') 的开关列表）", flush=True)
    print("-" * 70, flush=True)
    print(f"[getParam] NTF_ANGLE = {_try_get(sensor, 'NTF_ANGLE')}", flush=True)

    print(f"[setParam] NTF_ANGLE=OFF -> {_try_set(sensor, 'NTF_ANGLE', 'OFF')}", flush=True)
    obs.clear()
    time.sleep(OBSERVE_SECONDS)
    print(f"[NTF_ANGLE OFF 后 {OBSERVE_SECONDS}s] {obs.snapshot()}", flush=True)
    print(f"[getParam] NTF_ANGLE = {_try_get(sensor, 'NTF_ANGLE')}", flush=True)

    print(f"[setParam] NTF_ANGLE=ON -> {_try_set(sensor, 'NTF_ANGLE', 'ON')}", flush=True)
    obs.clear()
    time.sleep(OBSERVE_SECONDS)
    print(f"[NTF_ANGLE ON 后 {OBSERVE_SECONDS}s] {obs.snapshot()}", flush=True)

    # ---- 阶段 3：磁力计开关 ----
    # getParam("NTF") 的开关列表中没有 MAG，需确认磁力计是否有独立开关，
    # 或是否受 NTF_IMU 聚合开关控制。
    print("\n" + "-" * 70, flush=True)
    print("[阶段3] 磁力计开关：NTF_MAG / NTF_IMU", flush=True)
    print("-" * 70, flush=True)
    print(f"[getParam] NTF_MAG = {_try_get(sensor, 'NTF_MAG')}", flush=True)

    print(f"[setParam] NTF_MAG=OFF -> {_try_set(sensor, 'NTF_MAG', 'OFF')}", flush=True)
    obs.clear()
    time.sleep(OBSERVE_SECONDS)
    print(f"[NTF_MAG OFF 后 {OBSERVE_SECONDS}s] {obs.snapshot()}", flush=True)
    print(f"[getParam] NTF_MAG = {_try_get(sensor, 'NTF_MAG')}", flush=True)

    print(f"[setParam] NTF_MAG=ON -> {_try_set(sensor, 'NTF_MAG', 'ON')}", flush=True)
    obs.clear()
    time.sleep(OBSERVE_SECONDS)
    print(f"[NTF_MAG ON 后 {OBSERVE_SECONDS}s] {obs.snapshot()}", flush=True)

    print(f"[setParam] NTF_IMU=OFF -> {_try_set(sensor, 'NTF_IMU', 'OFF')}", flush=True)
    obs.clear()
    time.sleep(OBSERVE_SECONDS)
    print(f"[NTF_IMU OFF 后 {OBSERVE_SECONDS}s] {obs.snapshot()}", flush=True)
    print(f"[getParam] NTF_IMU = {_try_get(sensor, 'NTF_IMU')}", flush=True)

    print(f"[setParam] NTF_IMU=ON -> {_try_set(sensor, 'NTF_IMU', 'ON')}", flush=True)
    obs.clear()
    time.sleep(OBSERVE_SECONDS)
    print(f"[NTF_IMU ON 后 {OBSERVE_SECONDS}s] {obs.snapshot()}", flush=True)
    print(f"[getParam] NTF = {_try_get(sensor, 'NTF')}", flush=True)

    # 收尾
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.disconnect()
    except Exception:
        pass
    ctrl.terminate()

    print("\n" + "=" * 70, flush=True)
    print("探测完成：对比三段样本数，看 NTF_MAG_ANGLE OFF 影响了哪些流。", flush=True)
    print("=" * 70, flush=True)


if __name__ == "__main__":
    main()
