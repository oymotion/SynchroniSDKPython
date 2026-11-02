# -*- coding: utf-8 -*-
"""probe_device_info.py：新设备 spec 摸底工具（交互式四步）。

用途：给一台「还没适配过」的新设备（如 OB3000）摸清能力，并落地成 spec。

四步流程：
  1) 扫描所有 BLE 广播设备，列出让用户确认选哪台（不依赖 config 预配置）。
  2) 连接 -> init -> getDeviceInfo 全字段 dump + getParam 参数探测
     + 起流实测（startDataNotification 收集实际到达的 DataType）。
  3) 自动往 config.py 追加该设备（DEVICES 条目 + MODEL_SPEC 映射）。
  4) 自动生成 device_specs/<型号>.py spec 骨架（能力字段自动填，复杂关系标 TODO）。

用法：
    python probe_device_info.py
    python probe_device_info.py --stream-seconds 8     # 起流实测 8 秒
    python probe_device_info.py --device-index 2       # 跳过选择，直接连扫描结果第 2 台
    python probe_device_info.py --no-write             # 只探测，不写 config/spec
    python probe_device_info.py --yes                  # 写文件前不确认（用默认推断值）

前置条件：
    - 主机蓝牙已开启（或 USB dongle 已就绪）
    - 目标设备上电、在范围内

输出产物（--no-write 时不写）：
    - config.py 追加 DEVICES 条目 + MODEL_SPEC 映射（幂等：identity/name_prefix 已存在则跳过）
    - device_specs/<型号>.py spec 骨架
"""

import argparse
import os
import re
import sys
import time

AUTOMATION_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import _identity_of

READY_TIMEOUT = 15           # 连接后等待 Ready 超时（秒）
STREAM_PROBE_SECONDS = 5.0   # 起流实测默认时长（秒）


# ======================================================================
# DeviceInfo 候选字段（SDK 1.4.2 实测字段，见 sensor/sensor_device.py）
# 注意：1.4.2 已把旧的 MagAngleChannelCount 拆成 MagChannelCount + AngleChannelCount。
# ======================================================================
DEVICE_INFO_FIELDS = [
    "DeviceName", "ModelName", "HardwareVersion", "FirmwareVersion",
    "PpgChannelCount", "PpgSampleRate",
    "Spo2ChannelCount", "Spo2SampleRate",
    "ImpeChannelCount", "ImpeSampleRate",
    "EmgChannelCount", "EmgSampleRate",
    "EegChannelCount", "EegSampleRate",
    "EcgChannelCount", "EcgSampleRate",
    "EmgMaxSampleRate", "EegMaxSampleRate", "EcgMaxSampleRate",
    "ConnectionIntervalMs", "PeripheralLatency", "SupervisionTimeoutMs", "Backend",
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

# 候选 setParam/getParam key（README「Data stream toggles」）
NTF_KEYS = [
    "NTF_GEST", "NTF_EMG", "NTF_EEG", "NTF_ECG", "NTF_IMU",
    "NTF_BRTH", "NTF_IMPEDANCE", "NTF_MAG_ANGLE",
    "NTF_PPG", "NTF_PPG_RAW", "NTF_SPO2",
    "NTF_GFORCE_EULER", "NTF_GFORCE_QUAT", "NTF_GFORCE_ACC", "NTF_GFORCE_GYRO",
]

# 候选滤波参数 key（单个 key 的 getParam 通常返回 Error，只有聚合 getParam("FILTER") 可读）
FILTER_KEYS = ["FILTER_50HZ", "FILTER_60HZ", "FILTER_HPF", "FILTER_LPF"]

# 候选采样率参数 key：(当前值 key, 档位列表 key, 对应 DataType 流名)
SAMPLE_RATE_KEYS = [
    ("EEG_SAMPLE_RATE",   "EEG_SAMPLE_RATE_LIST",   "NTF_EEG"),
    ("EMG_SAMPLE_RATE",   "EMG_SAMPLE_RATE_LIST",   "NTF_EMG"),
    ("IMU_SAMPLE_RATE",   "IMU_SAMPLE_RATE_LIST",   "NTF_IMU"),
    ("PPG_SAMPLE_RATE",   "PPG_SAMPLE_RATE_LIST",   "NTF_PPG"),
    ("ANGLE_SAMPLE_RATE", "ANGLE_SAMPLE_RATE_LIST", "NTF_ANGLE_DATA"),
    ("MAG_SAMPLE_RATE",   "MAG_SAMPLE_RATE_LIST",   "NTF_MAG"),
]

# DataType 名 -> DeviceInfo 通道数字段（用于生成 spec 的 streams）
DT_CHANNEL_FIELD = {
    "NTF_ACC":          "AccChannelCount",
    "NTF_GYRO":         "GyroChannelCount",
    "NTF_MAG":          "MagChannelCount",
    "NTF_EULER_DATA":   "EulerChannelCount",
    "NTF_QUATERNION":   "QuatChannelCount",
    "NTF_GEST":         "GestChannelCount",
    "NTF_EMG":          "EmgChannelCount",
    "NTF_ANGLE_DATA":   "AngleChannelCount",
    "NTF_EEG":          "EegChannelCount",
    "NTF_ECG":          "EcgChannelCount",
    "NTF_IMPEDANCE":    "ImpeChannelCount",
    "NTF_IMU":          "ImuChannelCount",
    "NTF_BRTH":         "BrthChannelCount",
    "NTF_SPO2":         "Spo2ChannelCount",
    "NTF_PPG":          "PpgChannelCount",
}

# 生成 spec 时 streams 的固定顺序（尽量与既有 spec 一致，便于 diff）
STREAM_KEYS = [
    "NTF_ACC", "NTF_GYRO", "NTF_MAG", "NTF_EULER_DATA", "NTF_QUATERNION",
    "NTF_GEST", "NTF_EMG", "NTF_ANGLE_DATA", "NTF_EEG", "NTF_ECG",
    "NTF_IMPEDANCE", "NTF_IMU", "NTF_BRTH", "NTF_SPO2", "NTF_PPG",
]

# NTF_IMU 聚合流可能拆分的子流（DataType 名）
IMU_SUB_STREAMS = ["NTF_ACC", "NTF_GYRO", "NTF_EULER_DATA", "NTF_QUATERNION"]


# ======================================================================
# 工具函数
# ======================================================================
def _is_error(v):
    """getParam 返回值是否为错误（'Error: ...' 开头，或异常包装 '<...>'，表示不支持）。"""
    return isinstance(v, str) and (v.startswith("Error") or v.startswith("<"))


def _getparam(sensor, key):
    try:
        return sensor.getParam(key)
    except Exception as e:
        return f"<{type(e).__name__}: {e}>"


def _parse_rate_list(v):
    """把 '250|500|1000' 解析成 ['250','500','1000']；不可解析返回 []。"""
    if not isinstance(v, str) or v.startswith("Error") or v.startswith("<"):
        return []
    parts = [p.strip() for p in v.replace(" ", "").split("|") if p.strip()]
    return parts


def _dt_name(dt):
    """DataType / int -> 'NTF_XXX'；未知返回 'UNKNOWN(0xXX)'。"""
    if isinstance(dt, DataType):
        return dt.name
    try:
        return DataType(dt).name
    except (ValueError, TypeError):
        try:
            return f"UNKNOWN(0x{int(dt):02X})"
        except (ValueError, TypeError):
            return f"UNKNOWN({dt!r})"


def _fmt_rate(rate):
    """float -> '50' / '250' 之类的整数字符串（非整数保留小数）。"""
    try:
        f = float(rate)
    except (TypeError, ValueError):
        return str(rate)
    if f == int(f):
        return str(int(f))
    return f"{f:g}"


# ======================================================================
# DeviceInfo / 参数 dump
# ======================================================================
def dump_device_info(info):
    """逐个访问候选字段，返回 {字段名: 值}（仅存在的字段）。"""
    lines = []
    info_map = {}
    for f in DEVICE_INFO_FIELDS:
        try:
            v = getattr(info, f)
            info_map[f] = v
            lines.append(f"{f} = {v!r}")
        except AttributeError:
            lines.append(f"{f} = <字段不存在>")
        except Exception as e:
            lines.append(f"{f} = <{type(e).__name__}: {e}>")
    print("\n".join(lines), flush=True)
    return info_map


def probe_params(sensor):
    """dump getParam（聚合 + 逐个 NTF/FILTER/采样率），返回结构化结果。

    返回：
        {
          "ntf":          {key: getParam 返回值},
          "filters":      {FILTER_XX: bool},
          "sample_rates": {BASE: {"current": str, "list": [..], "stream": "NTF_XX"}},
        }
    """
    result = {"ntf": {}, "filters": {}, "sample_rates": {}}

    print("\n--- getParam('NTF') / getParam('FILTER') 聚合 ---", flush=True)
    for agg in ["NTF", "FILTER"]:
        v = _getparam(sensor, agg)
        print(f"getParam({agg!r}) = {v!r}", flush=True)

    print("\n--- getParam('FILTER') 聚合解析 ---", flush=True)
    fv = _getparam(sensor, "FILTER")
    if isinstance(fv, str) and not fv.startswith("Error") and "|" in fv:
        items = fv.split("|")
        for i in range(0, len(items) - 1, 2):
            k = items[i].strip()
            val = items[i + 1].strip()
            result["filters"][k] = (val.upper() == "ON")
            print(f"  {k} = {val}", flush=True)
    else:
        print(f"  getParam('FILTER') = {fv!r}（无聚合内容）", flush=True)

    print("\n--- 逐个 FILTER key 探测 ---", flush=True)
    for key in FILTER_KEYS:
        v = _getparam(sensor, key)
        supported = not _is_error(v)
        if supported and key not in result["filters"]:
            result["filters"][key] = (str(v).upper() == "ON")
        print(f"getParam({key!r}) = {v!r}  ->  {'可读' if supported else '单个key不可读'}", flush=True)

    print("\n--- 逐个 NTF key 探测（Error 开头 = 不支持）---", flush=True)
    for key in NTF_KEYS:
        v = _getparam(sensor, key)
        result["ntf"][key] = v
        print(f"getParam({key!r}) = {v!r}  ->  {'支持' if not _is_error(v) else '不支持'}", flush=True)

    print("\n--- 采样率参数探测（当前值 + 档位列表）---", flush=True)
    for base, list_key, stream in SAMPLE_RATE_KEYS:
        cur = _getparam(sensor, base)
        lst = _getparam(sensor, list_key)
        result["sample_rates"][base] = {
            "current": cur,
            "list": _parse_rate_list(lst),
            "stream": stream,
        }
        print(f"getParam({base!r}) = {cur!r};  {list_key!r} = {lst!r}", flush=True)

    return result


# ======================================================================
# 起流实测
# ======================================================================
class StreamCollector:
    """按 DataType 分桶收集实际到达的数据，统计通道数 / 采样率 / 样本跨度。"""

    def __init__(self):
        self.stats = {}  # dt_name(str) -> dict

    def on_data(self, sensor, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        for d in items:
            try:
                dt = d.getDataType()
            except Exception:
                continue
            name = _dt_name(dt)
            s = self.stats.get(name)
            if s is None:
                s = {
                    "channel_count": 0, "sample_rate": 0.0, "delivered": 0,
                    "min_idx": None, "max_idx": None,
                    "first_ts": None, "last_ts": None, "batches": 0,
                }
                self.stats[name] = s
            try:
                ch = d.getChannelCount()
                smp = d.getSampleCount()
                rate = d.getSampleRate()
            except Exception:
                ch = smp = rate = 0
            if s["channel_count"] == 0 and ch:
                s["channel_count"] = ch
            if s["sample_rate"] == 0 and rate:
                s["sample_rate"] = rate
            if ch and smp:
                s["delivered"] += ch * smp
            s["batches"] += 1
            now = time.time()
            if s["first_ts"] is None:
                s["first_ts"] = now
            s["last_ts"] = now
            # 通道 0 的 sampleIndex 首末跨度（去重样本数估算）
            for si in range(smp):
                try:
                    idx = d.getSampleIndex(0, si)
                except Exception:
                    break
                if idx is None:
                    continue
                if s["min_idx"] is None or idx < s["min_idx"]:
                    s["min_idx"] = idx
                if s["max_idx"] is None or idx > s["max_idx"]:
                    s["max_idx"] = idx

    def summarize(self):
        lines = []
        for name in sorted(self.stats):
            s = self.stats[name]
            ch = s["channel_count"]
            rate = s["sample_rate"]
            delivered = s["delivered"]
            dur = (s["last_ts"] - s["first_ts"]) if s["first_ts"] else 0.0
            unique = 0
            if s["min_idx"] is not None and s["max_idx"] is not None:
                unique = s["max_idx"] - s["min_idx"] + 1
            est_rate = (unique / dur) if dur > 0 else 0.0
            lines.append(
                f"  {name:<16} channels={ch:<3} rate(device)={rate:>7} Hz  "
                f"delivered={delivered:<8} unique_span={unique:<8} est_rate={est_rate:>7.2f} Hz"
            )
        return "\n".join(lines)


# ======================================================================
# 扫描并选择设备
# ======================================================================
def scan_and_select(ctrl, auto_index=None):
    """扫描所有设备并让用户选择；返回选中的 BLEDevice，或 None（用户退出/越界）。"""
    while True:
        try:
            devices = ctrl.scan(config.SCAN_TIMEOUT_MS)
        except Exception as e:
            devices = None
            print(f"[扫描] scan 抛异常: {type(e).__name__}: {e}", flush=True)
        devices = devices or []
        if not devices:
            print("[扫描] 未发现任何 BLE 设备。", flush=True)
            if auto_index is not None:
                return None
            r = input("回车重扫，或输入 q 退出: ").strip().lower()
            if r == "q":
                return None
            continue

        print(f"\n[扫描] 发现 {len(devices)} 台设备：", flush=True)
        for i, d in enumerate(devices):
            n = getattr(d, "Name", "?")
            a = getattr(d, "Address", "?")
            rssi = getattr(d, "RSSI", "?")
            ident = _identity_of(n)
            print(f"  [{i}] {n}  {a}  rssi={rssi}  identity={ident}", flush=True)

        if auto_index is not None:
            if 0 <= auto_index < len(devices):
                return devices[auto_index]
            print(f"[扫描] --device-index {auto_index} 越界（共 {len(devices)} 台）", flush=True)
            return None

        if len(devices) == 1:
            print("[扫描] 仅 1 台设备，自动选择 [0]。", flush=True)
            return devices[0]

        r = input("选择设备序号（回车重扫，q 退出）: ").strip()
        if r.lower() == "q":
            return None
        if r == "":
            continue
        if r.isdigit() and 0 <= int(r) < len(devices):
            return devices[int(r)]
        print("[扫描] 无效输入，请输入列表中的数字序号。", flush=True)


# ======================================================================
# 连接 + 探测
# ======================================================================
def connect_and_probe(ctrl, target, stream_seconds):
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[失败] requireSensor 返回 None", flush=True)
        return None

    print("[连接] SensorProfile.connect() ...", flush=True)
    sensor.connect()
    t0 = time.time()
    while time.time() - t0 < READY_TIMEOUT and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[失败] 未到达 Ready，state={sensor.deviceState}", flush=True)
        return None
    print(f"[连接] 到达 Ready，state={sensor.deviceState}", flush=True)

    print(f"[init] init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)

    info = sensor.getDeviceInfo()
    if info is None:
        print("[提示] getDeviceInfo() 返回 None，尝试 fetchDeviceInfo(3000) ...", flush=True)
        try:
            info = sensor.fetchDeviceInfo(3000)
        except Exception as e:
            info = None
            print(f"[失败] fetchDeviceInfo 抛异常: {type(e).__name__}: {e}", flush=True)
    if info is None:
        print("[失败] 无法获取 DeviceInfo（需 init 成功且 Ready）", flush=True)
        sensor.disconnect()
        return None

    print("\n=== DeviceInfo 全字段 dump ===", flush=True)
    info_map = dump_device_info(info)
    print("=" * 30, flush=True)

    print("\n=== getParam 探测 ===", flush=True)
    param_map = probe_params(sensor)
    print("=" * 30, flush=True)

    print(f"\n=== 起流实测（{stream_seconds}s）===", flush=True)
    collector = StreamCollector()
    sensor.onDataCallback = collector.on_data
    ok = sensor.startDataNotification()
    if not ok:
        print("[警告] startDataNotification() 返回 False，仍尝试等待数据 ...", flush=True)
    time.sleep(stream_seconds)
    sensor.stopDataNotification()
    sensor.onDataCallback = None
    if collector.stats:
        print(collector.summarize(), flush=True)
    else:
        print("  （未收到任何数据）", flush=True)
    print("=" * 30, flush=True)

    return {
        "sensor": sensor,
        "target": target,
        "info": info,
        "info_map": info_map,
        "param_map": param_map,
        "collector": collector,
    }


# ======================================================================
# spec 生成
# ======================================================================
def _build_data_source_notes(name, addr, info_map, param_map, collector):
    notes = []
    model = (info_map.get("ModelName") or "").strip()
    fw = (info_map.get("FirmwareVersion") or "").strip()
    hw = (info_map.get("HardwareVersion") or "").strip()
    notes.append(f"  - 广播名={name!r} MAC={addr!r}")
    notes.append(f"  - DeviceInfo: ModelName={model!r} Firmware={fw!r} Hardware={hw!r}")

    # DeviceInfo 通道数 > 0 的能力
    ch_fields = [
        "EmgChannelCount", "EegChannelCount", "EcgChannelCount",
        "PpgChannelCount", "Spo2ChannelCount", "ImpeChannelCount",
        "AccChannelCount", "GyroChannelCount", "MagChannelCount",
        "AngleChannelCount", "EulerChannelCount", "QuatChannelCount",
        "ImuChannelCount", "BrthChannelCount", "GestChannelCount",
    ]
    caps = [f for f in ch_fields if (info_map.get(f) or 0) > 0]
    if caps:
        notes.append("  - DeviceInfo 声明支持(ChannelCount>0): " +
                     ", ".join(f"{f}={info_map[f]}" for f in caps))
    else:
        notes.append("  - DeviceInfo 声明支持(ChannelCount>0): （无）")

    # 起流实测到的 DataType
    if collector.stats:
        obs = []
        for name in sorted(collector.stats):
            s = collector.stats[name]
            obs.append(f"{name}(ch={s['channel_count']}, rate~{_fmt_rate(s['sample_rate'])}Hz)")
        notes.append("  - 起流实测 DataType: " + ", ".join(obs))
    else:
        notes.append("  - 起流实测 DataType: （无）")

    # 采样率档位
    sr_notes = []
    for base, _lk, _stream in SAMPLE_RATE_KEYS:
        cur = param_map["sample_rates"][base]["current"]
        lst = param_map["sample_rates"][base]["list"]
        if lst:
            sr_notes.append(f"{base}={lst}（当前 {cur!r}）")
        elif not _is_error(cur) and cur.strip() and not cur.startswith("<"):
            sr_notes.append(f"{base}={cur!r}")
    if sr_notes:
        notes.append("  - 采样率参数: " + "; ".join(sr_notes))
    else:
        notes.append("  - 采样率参数: （未探测到）")
    return "\n".join(notes)


def _build_streams_section(info_map, collector):
    observed = collector.stats
    lines = []
    for key in STREAM_KEYS:
        dev_ch = info_map.get(DT_CHANNEL_FIELD.get(key, ""), 0) or 0
        obs = observed.get(key)
        obs_ch = obs["channel_count"] if obs else 0
        channels = obs_ch if obs_ch else dev_ch
        supported = channels > 0
        comment = f"DeviceInfo={dev_ch}"
        if obs_ch:
            comment += f" 实测={obs_ch}"
        if obs_ch and dev_ch and obs_ch != dev_ch:
            comment += " (不一致, TODO 确认)"
        lines.append(
            f'        "{key}": {{"supported": {str(supported)}, "channels": {channels}}},  # {comment}'
        )
    return "\n".join(lines)


def _build_sample_rates_section(info_map, param_map, collector):
    observed = collector.stats
    entries = []
    handled_streams = set()

    def _entry(key, rates, default, stream, settable, co_stream, sub_streams, comment):
        lines = [
            f'        "{key}": {{',
            f'            "rates": {rates}, "default": "{default}",',
            f'            "stream": "{stream}", "settable": {settable}, "co_stream": {co_stream},',
        ]
        if sub_streams:
            lines.append(f'            "sub_streams": {sub_streams},')
        lines.append(f'        }},  # {comment}')
        return "\n".join(lines)

    for base, _lk, stream in SAMPLE_RATE_KEYS:
        sr = param_map["sample_rates"][base]
        cur = sr["current"]
        lst = sr["list"]
        rates = None
        if lst:
            rates = lst
        elif not _is_error(cur) and cur.strip() and not cur.startswith("<"):
            rates = [cur.strip()]
        if not rates:
            continue
        settable = len(rates) > 1
        default = cur.strip() if (not _is_error(cur) and cur.strip() and not cur.startswith("<")) else rates[0]
        sub_streams = None
        if stream == "NTF_IMU":
            subs = [s for s in IMU_SUB_STREAMS if s in observed]
            if subs:
                sub_streams = subs
        comment = "settable" if settable else "固定(观测值)"
        if sub_streams:
            comment += f"；IMU 按 {sub_streams} 拆分投递 (TODO 确认)"
        else:
            comment += "；co_stream/sub_streams TODO"
        entries.append(_entry(base, rates, default, stream, settable, "None", sub_streams, comment))
        handled_streams.add(stream)

    # 阻抗固定 1Hz（ImpeChannelCount>0 或实测到 NTF_IMPEDANCE）
    if (info_map.get("ImpeChannelCount") or 0) > 0 or "NTF_IMPEDANCE" in observed:
        entries.append(_entry("IMPEDANCE", '["1"]', "1", "NTF_IMPEDANCE",
                              "False", "None", None, "固定 1Hz；co_stream TODO"))
        handled_streams.add("NTF_IMPEDANCE")

    # 实测到、但没有对应采样率参数的流（如 GEST/BRTH/SPO2），按观测值固定记录。
    # 跳过 IMU 子流（NTF_ACC/GYRO/EULER/QUAT）：它们由上面的 IMU_SAMPLE_RATE.sub_streams 覆盖。
    for name in sorted(observed):
        if name in handled_streams or name == "NTF_IMU" or name in IMU_SUB_STREAMS:
            continue
        s = observed[name]
        rate = _fmt_rate(s["sample_rate"]) if s["sample_rate"] else "?"
        base = name.replace("NTF_", "") + "_SAMPLE_RATE"
        entries.append(_entry(base, f'["{rate}"]', rate, name,
                              "False", "None", None, "无采样率参数，按实测固定 (TODO 确认)"))
        handled_streams.add(name)

    return "\n".join(entries)


def _build_spec_file(name_prefix, model, name, addr, info_map, param_map, collector):
    data_notes = _build_data_source_notes(name, addr, info_map, param_map, collector)
    streams = _build_streams_section(info_map, collector)
    sample_rates = _build_sample_rates_section(info_map, param_map, collector)

    # imu_sub_channels：acc/gyro/euler/quat 中 >0 的部分
    imu_sub = {}
    for field in ["AccChannelCount", "GyroChannelCount", "EulerChannelCount", "QuatChannelCount"]:
        v = info_map.get(field) or 0
        if v > 0:
            imu_sub[field] = v

    filters = param_map.get("filters") or {}
    filters_lines = []
    for k in sorted(filters):
        filters_lines.append(f'        "{k}": {str(bool(filters[k]))},')
    filters_block = "\n".join(filters_lines) if filters_lines else "        # TODO: 未探测到滤波参数"

    type_summary = "；".join(
        f"{info_map.get(DT_CHANNEL_FIELD.get(k), 0) or 0}ch {k}"
        for k in STREAM_KEYS
        if (info_map.get(DT_CHANNEL_FIELD.get(k), 0) or 0) > 0
    ) or "（待人工补充）"

    return f'''# -*- coding: utf-8 -*-
"""{model} 设备规格（Spec）——由 probe_device_info.py 自动生成的骨架。

本文件是该型号「应该支持」的能力基线，测试用例据此硬断言：
设备行为偏离 spec 即 FAIL（而非 SKIP / 自适应跳过）。

自动生成：能力字段已按 DeviceInfo + getParam + 起流实测填充；
标注 TODO 处需人工确认（co_stream / sub_streams / 互斥关系、单位/量程等）。

数据来源（probe 实测）：
{data_notes}
"""

SPEC = {{
    # ===== 身份 =====
    "model": {model!r},
    "type": {type_summary!r},
    "name_prefix": {name_prefix!r},

    # ===== 数据流能力（DataType 级别，对应 DeviceInfo 的 ChannelCount）=====
    # supported=True 表示该数据流应存在并产出数据；channels 为期望通道数。
    # 通道数口径：优先起流实测值，其次 DeviceInfo 声明值（二者不一致处标 TODO）。
    "streams": {{
{streams}
    }},

    # ===== NTF_IMU 聚合流的子通道组成（无独立 NTF_IMU 数据类型时按多路投递）=====
{("    \"imu_sub_channels\": " + repr(imu_sub) + ",") if imu_sub else "    # TODO: 无 IMU 子通道（或本型号无 IMU 聚合）"}

    # ===== 滤波开关（FILTER_* 支持 ON/OFF）=====
    "filters": {{
{filters_block}
    }},

    # ===== 采样率（供 test_measure_sample_rate.py 遍历）=====
    # stream    : setParam("NTF_XXX","ON") 开关 key，同时是 DataType.NTF_XXX 类型名
    # settable  : True 表示 key 即 setParam 采样率参数名；False 为固定采样率
    # co_stream : 伴随流（如 IMU/阻抗需与主数据流同起才有数据），None 表示无
    "sample_rates": {{
{sample_rates}
    }},
}}
'''


# ======================================================================
# config.py 更新
# ======================================================================
def update_config(name_prefix, mac, identity, spec_filename):
    """往 config.py 追加 DEVICES 条目 + MODEL_SPEC 映射（幂等）。返回变更说明列表。"""
    path = os.path.join(AUTOMATION_DIR, "config.py")
    with open(path, encoding="utf-8") as f:
        text = f.read()

    changes = []

    # 1) DEVICES 条目（identity 已存在则跳过）
    if f'"identity": "{identity}"' in text:
        changes.append(f"DEVICES: identity {identity} 已存在，跳过")
    else:
        entry = (
            f'\n    {{\n'
            f'        "name_prefix": "{name_prefix}",\n'
            f'        "mac": "{mac}",\n'
            f'        "identity": "{identity}",\n'
            f'    }},'
        )
        anchor = text.index("DEVICES = [")
        close = text.index("\n]", anchor)
        text = text[:close] + entry + text[close:]
        changes.append(f"DEVICES: 追加 {name_prefix} ({identity})")

    # 2) MODEL_SPEC 映射（name_prefix 已映射则跳过）
    spec_segment = text.split("MODEL_SPEC = {", 1)[1]
    if f'"{name_prefix}": "' in spec_segment:
        changes.append(f"MODEL_SPEC: {name_prefix} 已存在，跳过")
    else:
        mapping = f'\n    "{name_prefix}": "{spec_filename}",'
        anchor = text.index("MODEL_SPEC = {")
        close = text.index("\n}", anchor)
        text = text[:close] + mapping + text[close:]
        changes.append(f"MODEL_SPEC: 追加 {name_prefix} -> {spec_filename}")

    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return changes


# ======================================================================
# main
# ======================================================================
def main():
    ap = argparse.ArgumentParser(description="新设备 spec 摸底工具")
    ap.add_argument("--stream-seconds", type=float, default=STREAM_PROBE_SECONDS,
                    help=f"起流实测时长（秒），默认 {STREAM_PROBE_SECONDS}")
    ap.add_argument("--device-index", type=int, default=None,
                    help="跳过选择，直接连扫描结果第 N 台")
    ap.add_argument("--no-write", action="store_true", help="只探测，不写 config/spec")
    ap.add_argument("--yes", "-y", action="store_true", help="写文件前不确认")
    args = ap.parse_args()

    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("probe_device_info：新设备 spec 摸底", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    try:
        print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    except Exception as e:
        print(f"ble backend = <{type(e).__name__}: {e}>", flush=True)

    is_enable = ctrl.isEnable
    print(f"[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[失败] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    # 1) 扫描 + 选择
    target = scan_and_select(ctrl, auto_index=args.device_index)
    if target is None:
        print("[失败] 未选择设备", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, "Name", "?")
    addr = (getattr(target, "Address", "?") or "").upper()
    print(f"[已选] {name} {addr}", flush=True)

    # 2) 连接 + 探测
    result = connect_and_probe(ctrl, target, args.stream_seconds)
    if result is None:
        print("[失败] 探测失败", flush=True)
        ctrl.terminate()
        return

    sensor = result["sensor"]
    info_map = result["info_map"]
    param_map = result["param_map"]
    collector = result["collector"]

    # 断开连接（后续只写文件，不再需要设备连接）
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[提示] disconnect 抛异常: {type(e).__name__}: {e}", flush=True)
    ctrl.terminate()

    if args.no_write:
        print("\n[--no-write] 跳过写 config.py / spec。", flush=True)
        return

    # 3) 推断身份信息
    identity = _identity_of(name) or addr.replace(":", "")[-4:]
    broadcast_base = re.sub(r"\([0-9A-Fa-f]{4}\)\s*$", "", name).strip()
    model_name = (info_map.get("ModelName") or "").strip() or broadcast_base or "UNKNOWN"

    def _default_name_prefix():
        # 广播名若是泛型（如 "OB"），优先用更具体的 ModelName
        if broadcast_base and broadcast_base.upper() not in ("OB", "UNKNOWN", ""):
            return broadcast_base
        return model_name

    name_prefix = _default_name_prefix()
    spec_filename = re.sub(r"[^0-9a-zA-Z]+", "_", name_prefix).strip("_").lower() or "unknown"

    if not args.yes:
        print("\n--- 确认设备身份信息（回车采用默认值）---", flush=True)
        v = input(f"name_prefix（spec 加载键，默认 {name_prefix!r}）: ").strip()
        if v:
            name_prefix = v
        v = input(f"spec 文件名（不含 .py，默认 {spec_filename!r}）: ").strip()
        if v:
            spec_filename = v
        v = input(f"identity（默认 {identity!r}）: ").strip()
        if v:
            identity = v.upper()
    else:
        print(f"\n[--yes] 采用默认身份：name_prefix={name_prefix!r} spec={spec_filename!r} identity={identity!r}",
              flush=True)

    # 4) 生成 spec + 写 config
    spec_path = os.path.join(AUTOMATION_DIR, "device_specs", f"{spec_filename}.py")
    spec_content = _build_spec_file(name_prefix, model_name, name, addr, info_map, param_map, collector)

    print("\n--- 即将写入 ---", flush=True)
    print(f"  spec 文件 : {spec_path}", flush=True)
    print(f"  config.py : DEVICES 追加 {name_prefix} ({identity}) + MODEL_SPEC 映射", flush=True)

    if not args.yes:
        v = input("确认写入？(y/N): ").strip().lower()
        if v != "y":
            print("[已取消] 未写入任何文件。", flush=True)
            return

    try:
        os.makedirs(os.path.dirname(spec_path), exist_ok=True)
        with open(spec_path, "w", encoding="utf-8") as f:
            f.write(spec_content)
        print(f"[已写] spec -> {spec_path}", flush=True)
    except OSError as e:
        print(f"[失败] 写 spec 失败: {e}", flush=True)

    try:
        changes = update_config(name_prefix, addr, identity, spec_filename)
        for c in changes:
            print(f"[已写] {c}", flush=True)
    except Exception as e:
        print(f"[失败] 更新 config.py 失败: {type(e).__name__}: {e}", flush=True)

    print("\n完成。请检查生成的 spec 中标注 TODO 的字段（co_stream/sub_streams/互斥/单位）。", flush=True)
    print(f"提醒：如需让通用测试脚本跑这台设备，记得在 config.py 里把 TARGET_IDENTITY 设为 {identity!r}。",
          flush=True)


if __name__ == "__main__":
    main()
