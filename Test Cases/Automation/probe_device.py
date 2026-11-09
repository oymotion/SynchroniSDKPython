# -*- coding: utf-8 -*-
"""probe_device.py：通用设备能力全探测。

一次连上设备，摸清它到底支持哪些能力（流 / 通道数 / 采样率 / 参数），三层交叉验证：

    第 1 层：DeviceInfo 全字段 dump（权威能力声明，ChannelCount>0 表示该流支持）
    第 2 层：getParam 探测 NTF 流开关 / 采样率 / FILTER key（Error 开头 = 不支持）
    第 3 层：起流后实际观测到的 DataType（通道数 / 采样率 / 样本数 / 批数）
    第 3.5 层：起流后补测采样率 key（未起流读不到的 key）+ 停流后 setParam 探测多档位

注意：getDeviceInfo() 是「读缓存/裸读」，不主动向设备请求、也不等待异步推送；
脚本先 fetchDeviceInfo（带超时），失败再轮询 getDeviceInfo 等 onDeviceInfoUpdate 回调。

用法：
    python probe_device.py          # 用 config.TARGET_IDENTITY（逗号分隔，可多个）
    python probe_device.py 1D0E     # 指定目标 identity（须在 config.DEVICES 中定义）
"""

import os
import sys
import threading
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = BASE_DIR  # 通用版位于 Automation 根，config/common 就在本目录
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *  # noqa: E402,F403
import config          # noqa: E402
import common          # noqa: E402
from common import scan_and_match, resolve_target_identity  # noqa: E402

COLLECT_SECONDS = config.COLLECT_SECONDS  # 起流后观测时长（秒）


# DeviceInfo 全字段（来自 sensor.sensor_device.DeviceInfo.__init__）
DEVICE_INFO_FIELDS = [
    "DeviceName", "ModelName", "HardwareVersion", "FirmwareVersion",
    "PpgChannelCount", "PpgSampleRate",
    "Spo2ChannelCount", "Spo2SampleRate",
    "ImpeChannelCount", "ImpeSampleRate",
    "EmgChannelCount", "EmgSampleRate",
    "EegChannelCount", "EegSampleRate",
    "EcgChannelCount", "EcgSampleRate",
    "EmgMaxSampleRate", "EegMaxSampleRate", "EcgMaxSampleRate",
    "ConnectionIntervalMs", "PeripheralLatency", "SupervisionTimeoutMs",
    "Backend",
    "AccChannelCount", "AccSampleRate",
    "GyroChannelCount", "GyroSampleRate",
    "BrthChannelCount", "BrthSampleRate",
    "AngleChannelCount", "AngleSampleRate",
    "EulerChannelCount", "EulerSampleRate",
    "QuatChannelCount", "QuatSampleRate",
    "ImuChannelCount", "ImuSampleRate",
    "MTUSize", "IsMTUFine",
    "EmgGain", "EegGain", "EcgGain",
    "GestChannelCount", "GestSampleRate",
    "MagChannelCount", "MagSampleRate",
]

# 能力汇总：标签 -> (ChannelCount 字段, SampleRate 字段)
CAPABILITY_PAIRS = [
    ("PPG",     "PpgChannelCount",   "PpgSampleRate"),
    ("SpO2",    "Spo2ChannelCount",  "Spo2SampleRate"),
    ("阻抗",    "ImpeChannelCount",  "ImpeSampleRate"),
    ("EMG",     "EmgChannelCount",   "EmgSampleRate"),
    ("EEG",     "EegChannelCount",   "EegSampleRate"),
    ("ECG",     "EcgChannelCount",   "EcgSampleRate"),
    ("加速度计", "AccChannelCount",  "AccSampleRate"),
    ("陀螺仪",  "GyroChannelCount",  "GyroSampleRate"),
    ("呼吸",    "BrthChannelCount",  "BrthSampleRate"),
    ("角度",    "AngleChannelCount", "AngleSampleRate"),
    ("欧拉角",  "EulerChannelCount", "EulerSampleRate"),
    ("四元数",  "QuatChannelCount",  "QuatSampleRate"),
    ("IMU",     "ImuChannelCount",   "ImuSampleRate"),
    ("手势",    "GestChannelCount",  "GestSampleRate"),
    ("磁力计",  "MagChannelCount",   "MagSampleRate"),
]

# getParam 探测：NTF 流开关 key（含 1.4.2 新 key 与旧版别名）
NTF_KEYS = [
    "NTF_ACC", "NTF_GYRO", "NTF_MAG", "NTF_EULER_DATA", "NTF_QUATERNION",
    "NTF_GEST", "NTF_EMG", "NTF_ANGLE_DATA", "NTF_EEG", "NTF_ECG",
    "NTF_IMPEDANCE", "NTF_IMU", "NTF_ADS", "NTF_BRTH", "NTF_IMPEDANCE_EXT",
    "NTF_SPO2", "NTF_PPG",
    "NTF_MAG_ANGLE", "NTF_EULER", "NTF_QUAT", "NTF_PPG_RAW",
    "NTF_GFORCE_EULER", "NTF_GFORCE_QUAT", "NTF_GFORCE_ACC", "NTF_GFORCE_GYRO",
]

# getParam 探测：采样率 key
SAMPLE_RATE_KEYS = [
    "EEG_SAMPLE_RATE", "EEG_SAMPLE_RATE_LIST",
    "EMG_SAMPLE_RATE", "EMG_SAMPLE_RATE_LIST",
    "IMU_SAMPLE_RATE", "IMU_SAMPLE_RATE_LIST",
    "PPG_SAMPLE_RATE", "PPG_SAMPLE_RATE_LIST",
    "ECG_SAMPLE_RATE", "ECG_SAMPLE_RATE_LIST",
    "SPO2_SAMPLE_RATE", "SPO2_SAMPLE_RATE_LIST",
    "MAG_SAMPLE_RATE", "MAG_SAMPLE_RATE_LIST",
    "ANGLE_SAMPLE_RATE", "ANGLE_SAMPLE_RATE_LIST",
]

# 采样率档位探测：有 MaxSampleRate 字段的流 -> (标签, ChannelCount, SampleRate, MaxSampleRate, setParam key)
SAMPLE_RATE_GEAR_KEYS = [
    ("EMG", "EmgChannelCount", "EmgSampleRate", "EmgMaxSampleRate", "EMG_SAMPLE_RATE"),
    ("EEG", "EegChannelCount", "EegSampleRate", "EegMaxSampleRate", "EEG_SAMPLE_RATE"),
    ("ECG", "EcgChannelCount", "EcgSampleRate", "EcgMaxSampleRate", "ECG_SAMPLE_RATE"),
]

# getParam 探测：滤波 key（单个 key 通常不可读，仅聚合 FILTER 可读）
FILTER_KEYS = ["FILTER_50HZ", "FILTER_60HZ", "FILTER_HPF", "FILTER_LPF"]


def _field(info, name):
    try:
        return getattr(info, name)
    except AttributeError:
        return "<字段不存在>"
    except Exception as e:
        return f"<{type(e).__name__}: {e}>"


def _param_supported(v):
    """Error 开头或异常包装 '<...>' 视为不支持；异常（非 str）也视为不支持。"""
    if isinstance(v, str):
        return (not (v.startswith("Error") or v.startswith("<"))), v
    return False, repr(v)


def dump_info_fields(info):
    lines = []
    for f in DEVICE_INFO_FIELDS:
        lines.append(f"  {f} = {_field(info, f)!r}")
    return "\n".join(lines)


def capability_summary(info):
    """仅列出 ChannelCount>0 的能力，作为 DeviceInfo 层的支持结论。"""
    lines = []
    for label, ch_field, rate_field in CAPABILITY_PAIRS:
        ch = _field(info, ch_field)
        rate = _field(info, rate_field)
        try:
            supported = int(ch) > 0
        except (TypeError, ValueError):
            supported = False
        if supported:
            lines.append(f"  {label}: 通道数={ch}, 采样率={rate}")
    if not lines:
        lines.append("  (DeviceInfo 中无任何 ChannelCount>0 的能力)")
    return lines


def probe_keys(sensor, keys):
    lines = []
    for key in keys:
        try:
            v = sensor.getParam(key)
        except Exception as e:
            v = f"<{type(e).__name__}: {e}>"
        supported, vv = _param_supported(v)
        lines.append(f"  {key} = {vv!r}  ->  {'支持' if supported else '不支持'}")
    return lines


def _to_int(v):
    try:
        return int(v)
    except (TypeError, ValueError):
        return 0


def probe_sample_rate_gears(sensor, info):
    """对 MaxSampleRate > SampleRate 的流，setParam 探测候选档并回读；测完恢复默认档。

    背景：部分固件 *_SAMPLE_RATE_LIST 只返回当前档位（单值），但 MaxSampleRate 暗示有多档；
    只靠 getParam 会漏档，必须 setParam 实测候选档是否被接受。
    """
    lines = []
    for label, ch_field, rate_field, max_field, key in SAMPLE_RATE_GEAR_KEYS:
        ch = _to_int(_field(info, ch_field))
        rate = _to_int(_field(info, rate_field))
        max_rate = _to_int(_field(info, max_field))
        if ch <= 0:
            lines.append(f"  {label}: 无该能力（ChannelCount=0），跳过")
            continue
        # 候选档：当前档 + MaxSampleRate 暗示的档（去重保序）
        gears = []
        for c in (str(rate), str(max_rate)):
            if c not in gears and _to_int(c) > 0:
                gears.append(c)
        if not gears:
            lines.append(f"  {label}: 无法确定候选档（rate={rate}, max={max_rate}），跳过")
            continue
        results = []
        for g in gears:
            try:
                ret = sensor.setParam(key, g)
                back = sensor.getParam(key)
                results.append(f"{g}→set={ret!r},回读={back!r}")
            except Exception as e:
                results.append(f"{g}→抛异常 {type(e).__name__}: {e}")
        # 恢复默认档（rate），避免残留状态
        restore = ""
        if rate > 0:
            try:
                sensor.setParam(key, str(rate))
                restore = f"（已恢复默认 {rate}）"
            except Exception as e:
                restore = f"（恢复默认 {rate} 抛异常 {type(e).__name__}: {e}）"
        lines.append(f"  {label} [{key}]: 候选档={gears}  {' | '.join(results)} {restore}")
    return lines


class StreamObserver:
    """起流后按 DataType 分桶统计通道数/采样率/样本数/批数。"""

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
    print("probe_device：通用设备能力全探测", flush=True)
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
    iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
    print(f"[init] init() = {iret}  hasInited={sensor.hasInited}", flush=True)

    # ---- 设备信息：先 fetch，失败再轮询 ----
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

    # ---- 第 1 层：DeviceInfo ----
    print("\n[第 1 层] DeviceInfo 全字段 dump", flush=True)
    print(dump_info_fields(info), flush=True)
    print("  --- 能力汇总（ChannelCount>0 表示支持）---", flush=True)
    for line in capability_summary(info):
        print(line, flush=True)

    # ---- 第 2 层：getParam 探测 ----
    print("\n[第 2 层] getParam 探测", flush=True)

    for agg in ["NTF", "FILTER"]:
        try:
            v = sensor.getParam(agg)
        except Exception as e:
            v = f"<{type(e).__name__}: {e}>"
        print(f"  getParam({agg!r}) 聚合 = {v!r}", flush=True)

    print("  --- NTF 流开关 key（Error 开头 = 不支持）---", flush=True)
    for line in probe_keys(sensor, NTF_KEYS):
        print(line, flush=True)

    print("  --- 采样率 key ---", flush=True)
    for line in probe_keys(sensor, SAMPLE_RATE_KEYS):
        print(line, flush=True)

    print("  --- FILTER key（单个 key 通常不可读，仅聚合可读）---", flush=True)
    for line in probe_keys(sensor, FILTER_KEYS):
        print(line, flush=True)

    # ---- 第 3 层：起流观测 ----
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

    # 起流后补测采样率 key（部分 key 未起流时返回 Not supported，起流后可读）
    print("\n[起流后] 采样率 key 补测", flush=True)
    for line in probe_keys(sensor, SAMPLE_RATE_KEYS):
        print(line, flush=True)

    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"  stopDataNotification() 抛异常 {type(e).__name__}: {e}", flush=True)

    # setParam 档位探测（停流后，MaxSampleRate > SampleRate 的流）
    print("\n[档位探测] setParam 候选档（MaxSampleRate > SampleRate 的流）", flush=True)
    for line in probe_sample_rate_gears(sensor, info):
        print(line, flush=True)

    try:
        sensor.disconnect()
    except Exception as e:
        print(f"  disconnect() 抛异常 {type(e).__name__}: {e}", flush=True)

    ctrl.terminate()

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("探测结论", flush=True)
    print("=" * 60, flush=True)
    print("DeviceInfo 声明支持的能力：", flush=True)
    for line in capability_summary(info):
        print(line, flush=True)
    print("实际起流观测到的流：", flush=True)
    for line in observer.snapshot_lines():
        print(line, flush=True)
    print("=" * 60, flush=True)


if __name__ == "__main__":
    main()
