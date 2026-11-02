# -*- coding: utf-8 -*-
"""probe_mag_angle.py：探测 OYFM7020（80CE）是否支持「磁力计 / 角度」流。

背景：SDK 1.4.2 起，磁力计与角度被拆成两个独立流（相对 1.3.9 的合并
`NTF_MAG_ANGLE_DATA` 0x0D）：
    DataType.NTF_MAG        = 0x03   （磁力计）
    DataType.NTF_ANGLE_DATA = 0x0D   （角度）
    DeviceInfo 新增字段：MagChannelCount/MagSampleRate、AngleChannelCount/AngleSampleRate

本脚本做三层判定，避免单一信号误判：
    第 1 层：DeviceInfo 能力字段（权威声明，>0 表示支持）
    第 2 层：getParam 探测相关流开关 key（Error 开头 = 不支持）
    第 3 层：起流后实际观测到的 DataType（0x03 / 0x0D 是否真的来数据）

用法：
    python probe_mag_angle.py          # 用 config.TARGET_IDENTITY（当前 80CE）
    python probe_mag_angle.py 80CE     # 指定目标 identity（须在 config.DEVICES 中定义）
"""

import os
import sys
import threading
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(BASE_DIR)
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *  # noqa: E402,F403
import config          # noqa: E402
import common          # noqa: E402
from common import scan_and_match, resolve_target_identity  # noqa: E402

COLLECT_SECONDS = config.COLLECT_SECONDS  # 起流后观测时长（秒）


# DeviceInfo 中与磁力计/角度直接相关 + 邻近参考的字段
MAG_ANGLE_FIELDS = [
    "MagChannelCount", "MagSampleRate",        # 磁力计
    "AngleChannelCount", "AngleSampleRate",    # 角度
    "EulerChannelCount", "EulerSampleRate",    # 欧拉角（参考，常与角度/姿态相关）
    "QuatChannelCount", "QuatSampleRate",      # 四元数（参考）
    "AccChannelCount", "AccSampleRate",        # 加速度计（参考）
    "GyroChannelCount", "GyroSampleRate",      # 陀螺仪（参考）
    "ImuChannelCount", "ImuSampleRate",        # IMU 聚合（参考）
]


def _field(info, name):
    try:
        return getattr(info, name)
    except AttributeError:
        return "<字段不存在>"
    except Exception as e:
        return f"<{type(e).__name__}: {e}>"


def dump_info_fields(info):
    lines = []
    for f in MAG_ANGLE_FIELDS:
        lines.append(f"  {f} = {_field(info, f)!r}")
    return "\n".join(lines)


def _param_supported(v):
    """返回 (supported, value)：Error 开头或异常包装 '<...>' 视为不支持。"""
    if isinstance(v, str):
        return (not (v.startswith("Error") or v.startswith("<"))), v
    return False, repr(v)


class StreamObserver:
    """起流后按 DataType 分桶统计，专门找 NTF_MAG(0x03) 与 NTF_ANGLE_DATA(0x0D)。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.seen = {}  # dt_name -> {"channels": set, "rates": set, "samples": int, "batches": int}

    def __call__(self, profile, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        with self.lock:
            for it in items:
                try:
                    dt = it.getDataType()
                    name = dt.name if isinstance(dt, DataType) else f"0x{int(dt):X}"
                    n_ch = it.getChannelCount()
                    n_smp = it.getSampleCount()
                    rate = it.getSampleRate()
                    e = self.seen.setdefault(
                        name, {"channels": set(), "rates": set(), "samples": 0, "batches": 0})
                    e["channels"].add(n_ch)
                    e["rates"].add(rate)
                    e["samples"] += (n_ch * n_smp)
                    e["batches"] += 1
                except Exception:
                    continue

    def has(self, dt_name):
        with self.lock:
            return dt_name in self.seen

    def snapshot_lines(self):
        with self.lock:
            if not self.seen:
                return ["  (未收到任何数据)"]
            lines = []
            for name in sorted(self.seen):
                e = self.seen[name]
                lines.append(
                    f"  {name}: 批数={e['batches']} 样本={e['samples']} "
                    f"通道数={sorted(e['channels'])} 采样率={sorted(e['rates'])}")
            return lines


def main():
    ctrl = SensorControllerInstance
    target_identity = resolve_target_identity()

    print("=" * 60, flush=True)
    print("probe_mag_angle：OYFM7020 磁力计 / 角度 支持性探测", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    if target_identity:
        print(f"[目标] identity: {', '.join(target_identity)}（命令行指定）", flush=True)
    else:
        print(f"[目标] identity: {', '.join(common.TARGET_IDENTITIES)}（config 默认）", flush=True)

    if ctrl.isEnable is not True:
        print("[失败] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS,
                                     target_identity=target_identity)
    if target is None:
        print("[失败] 未匹配到目标设备。", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, "Name", "?")
    addr = getattr(target, "Address", "?")
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[失败] requireSensor 返回 None", flush=True)
        ctrl.terminate()
        return

    print("[连接] SensorProfile.connect() ...", flush=True)
    sensor.connect()
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[失败] 未到达 Ready，state={sensor.deviceState}", flush=True)
        ctrl.terminate()
        return
    print(f"[连接] 到达 Ready，state={sensor.deviceState}", flush=True)

    print(f"[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, "
          f"{config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)

    # getDeviceInfo() 是「读缓存/裸读」，不主动向设备请求，也不等待异步推送；
    # 设备信息经 onDeviceInfoUpdate 回调异步到达，init 返回后未必已就绪。
    # 因此先主动 fetchDeviceInfo（带超时），失败再轮询 getDeviceInfo 等异步推送。
    info = None
    try:
        info = sensor.fetchDeviceInfo(3000)
        print(f"[设备信息] fetchDeviceInfo(3000) -> "
              f"{type(info).__name__ if info is not None else None}", flush=True)
    except Exception as e:
        print(f"[设备信息] fetchDeviceInfo 抛异常 {type(e).__name__}: {e}", flush=True)
        info = None

    if info is None:
        for _ in range(20):
            info = sensor.getDeviceInfo()
            if info is not None:
                break
            time.sleep(0.25)

    if info is None:
        print("[失败] 未取得 DeviceInfo（fetchDeviceInfo 与 getDeviceInfo 均返回 None）", flush=True)
        sensor.disconnect()
        ctrl.terminate()
        return

    # ---- 第 1 层：DeviceInfo 能力字段 ----
    print("\n[第 1 层] DeviceInfo 能力字段", flush=True)
    print(dump_info_fields(info), flush=True)

    mag_info = False
    angle_info = False
    try:
        mag_info = (int(getattr(info, "MagChannelCount", 0)) > 0
                    or int(getattr(info, "MagSampleRate", 0)) > 0)
        angle_info = (int(getattr(info, "AngleChannelCount", 0)) > 0
                      or int(getattr(info, "AngleSampleRate", 0)) > 0)
    except Exception:
        pass
    print(f"  => DeviceInfo 判定: 磁力计={'支持' if mag_info else '不支持'}，"
          f"角度={'支持' if angle_info else '不支持'}", flush=True)

    # ---- 第 2 层：getParam 探测流开关 key ----
    print("\n[第 2 层] getParam 流开关 key 探测（Error 开头 = 不支持）", flush=True)
    param_support = {}
    for key in ["NTF_MAG", "NTF_ANGLE_DATA", "NTF_MAG_ANGLE"]:
        try:
            v = sensor.getParam(key)
        except Exception as e:
            v = f"<{type(e).__name__}: {e}>"
        supported, vv = _param_supported(v)
        param_support[key] = supported
        print(f"  getParam({key!r}) = {vv!r}  ->  {'支持' if supported else '不支持'}", flush=True)

    try:
        ntf_agg = sensor.getParam("NTF")
    except Exception as e:
        ntf_agg = f"<{type(e).__name__}: {e}>"
    print(f"  getParam('NTF') 聚合 = {ntf_agg!r}", flush=True)

    # 若 getParam 显示支持，尝试开启对应流（best-effort，失败不阻断探测）
    for key in ["NTF_MAG", "NTF_ANGLE_DATA"]:
        if param_support.get(key):
            try:
                r = sensor.setParam(key, "ON")
                print(f"  setParam({key!r}, 'ON') = {r!r}", flush=True)
            except Exception as e:
                print(f"  setParam({key!r}, 'ON') 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 第 3 层：起流观测实际 DataType ----
    observer = StreamObserver()
    sensor.onDataCallback = observer
    print(f"\n[第 3 层] 起流观测（{COLLECT_SECONDS}s）...", flush=True)
    try:
        sret = sensor.startDataNotification()
        print(f"  startDataNotification() = {sret}  isDataTransfering={sensor.isDataTransfering}",
              flush=True)
    except Exception as e:
        sret = None
        print(f"  startDataNotification() 抛异常 {type(e).__name__}: {e}", flush=True)

    time.sleep(COLLECT_SECONDS)

    print("  观测到的 DataType:", flush=True)
    for line in observer.snapshot_lines():
        print(line, flush=True)

    mag_stream = observer.has("NTF_MAG")
    angle_stream = observer.has("NTF_ANGLE_DATA")
    print(f"  => 起流观测: 磁力计(NTF_MAG)={'有数据' if mag_stream else '无数据'}，"
          f"角度(NTF_ANGLE_DATA)={'有数据' if angle_stream else '无数据'}", flush=True)

    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"  stopDataNotification() 抛异常 {type(e).__name__}: {e}", flush=True)

    try:
        sensor.disconnect()
    except Exception as e:
        print(f"  disconnect() 抛异常 {type(e).__name__}: {e}", flush=True)

    ctrl.terminate()

    # ---- 汇总 ----
    mag_supported = mag_info or param_support.get("NTF_MAG") or mag_stream
    angle_supported = (angle_info or param_support.get("NTF_ANGLE_DATA")
                       or param_support.get("NTF_MAG_ANGLE") or angle_stream)

    print("\n" + "=" * 60, flush=True)
    print("探测结论", flush=True)
    print("=" * 60, flush=True)
    print(f"  磁力计 : {'支持' if mag_supported else '不支持'}"
          f"（DeviceInfo={'是' if mag_info else '否'}，getParam={'是' if param_support.get('NTF_MAG') else '否'}，"
          f"起流={'有' if mag_stream else '无'}）", flush=True)
    print(f"  角度   : {'支持' if angle_supported else '不支持'}"
          f"（DeviceInfo={'是' if angle_info else '否'}，getParam={'是' if (param_support.get('NTF_ANGLE_DATA') or param_support.get('NTF_MAG_ANGLE')) else '否'}，"
          f"起流={'有' if angle_stream else '无'}）", flush=True)
    print("=" * 60, flush=True)


if __name__ == "__main__":
    main()
