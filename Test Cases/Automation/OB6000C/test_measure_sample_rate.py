# -*- coding: utf-8 -*-
"""OB6000C 采样率实测（spec 驱动，多流并测）。

用途：专项验证 OB6000C 各数据流的真实采样率，不并入正式用例。

做法：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 读取 device_specs/ob6000c.py 的 spec["sample_rates"]
  3) 一次把全部待测流 setParam("NTF_XXX","ON")，跨会话保持 ON
  4) 会话数 = 所有「可设置」流里 rates 数量最多的那个；每会话只设「还没测完」的档
  5) 每会话：setParam 新档 -> 静置 -> 起流 -> 等首批 -> 预热 -> 正式采集 MEASURE_SECONDS 秒
     -> 停流 -> 对本次新设档的流（+第 0 次会话的固定流）按 DataType 分桶统计
  6) 实际采样率 = 唯一样本数 / 该流首末样本到达时长；与标称比对

要点：
  - 采样率来源统一为 spec["sample_rates"]（含固定 1Hz 的 IMPEDANCE），脚本不硬编码。
  - stream 既是 setParam("NTF_XXX","ON") 开关 key，也是 DataType.NTF_XXX 类型名。
  - 多流并测：所有流全程 ON，同一次起流里各 DataType 同时到达，用多路分桶一起统计。
  - 只设没测的档：某流 i >= len(rates) 时不再 setParam 采样率（流仍保持 ON 在流），
    只对本次新设档的流出结果；固定流只在第 0 次会话测一次。
  - 起流后先预热 10s（丢弃起流初期不稳定样本），再正式计数。
  - 去重口径：unique = max_sample_index - min_sample_index + 1（通道0 sampleIndex 首末跨度）。
  - 时长口径：取该流首/末样本到达时刻之差（last_ts - first_ts），与 sampleIndex 跨度对齐，
    避免 SDK 缓冲延迟让样本数虚高约一个批次（~0.2%）。

用法：
  python test_measure_sample_rate.py
  python test_measure_sample_rate.py 6C6B   # 指定 OB6000C identity

前置条件：
  - 主机蓝牙已开启（或 USB dongle 已就绪）
  - 待测 OB6000C 上电、在范围内
"""

import os
import re
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(BASE_DIR)
sys.path.insert(0, AUTOMATION_DIR)

# bin 归档目录（用于报 bug）
RESULT_DIR = os.path.join(BASE_DIR, "Sample Rate Result")

from sensor import *
import config
import common
from common import scan_and_match

MEASURE_SECONDS = 300          # 每次会话正式采集时长（秒）
SETTLE_SECONDS = 2.0           # setParam 后等待设备异步生效的静置时长
WARMUP_SECONDS = 10.0          # 起流后预热时长（丢弃起流初期不稳定样本）
READY_TIMEOUT = 15             # 连接后等待 Ready 超时（秒）
FIRST_DATA_TIMEOUT = 10        # 起流后等待首批数据超时（秒）
SAMPLE_RATE_TOLERANCE = 0.001  # 采样率偏差容差 ±0.1%（千分之一）


def _load_spec(target):
    """由匹配到的设备解析其 spec（优先 identity，回退到 OB6000C）。"""
    name = getattr(target, "Name", "") or ""
    identity = common._identity_of(name)
    if identity:
        try:
            return common.spec_for_identity(identity)
        except Exception:
            pass
    base = re.sub(r"\([0-9A-Fa-f]{4}\)\s*$", "", name).strip()
    if base:
        try:
            return common.load_spec(base)
        except Exception:
            pass
    return common.load_spec("OB6000C")


class RateCollector:
    """多流同时统计：按 DataType 分桶记录样本数、sampleIndex 跨度、首末到达时刻。"""

    def __init__(self):
        self.stats = {}   # dt(int) -> dict

    def reset(self):
        self.stats = {}

    @property
    def has_data(self):
        return bool(self.stats)

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            try:
                dt = d.getDataType()
            except Exception:
                continue
            s = self.stats.get(dt)
            if s is None:
                s = {
                    "delivered": 0, "channel_count": 0,
                    "first_ts": None, "last_ts": None,
                    "min_idx": None, "max_idx": None,
                }
                self.stats[dt] = s
            try:
                n_ch = d.getChannelCount()
                n_smp = d.getSampleCount()
            except Exception:
                n_ch = n_smp = 0
            if n_ch <= 0 or n_smp <= 0:
                continue
            now = time.time()
            if s["first_ts"] is None:
                s["first_ts"] = now
            s["last_ts"] = now
            s["delivered"] += n_ch * n_smp
            if s["channel_count"] == 0:
                s["channel_count"] = n_ch
            # 通道0 sampleIndex 记录全局首末（跨批单调递增，去重）
            for si in range(n_smp):
                try:
                    idx = d.getSampleIndex(0, si)
                except Exception:
                    continue
                if idx is None:
                    continue
                if s["min_idx"] is None or idx < s["min_idx"]:
                    s["min_idx"] = idx
                if s["max_idx"] is None or idx > s["max_idx"]:
                    s["max_idx"] = idx

    def result_for(self, dt):
        """返回 (unique, duration, delivered, channel_count)；无数据返回 (0, 0, 0, 0)。"""
        s = self.stats.get(dt)
        if not s:
            return 0, 0.0, 0, 0
        unique = 0
        if s["min_idx"] is not None and s["max_idx"] is not None:
            unique = s["max_idx"] - s["min_idx"] + 1
        dur = 0.0
        if s["first_ts"] is not None and s["last_ts"] is not None:
            dur = s["last_ts"] - s["first_ts"]
        return unique, dur, s["delivered"], s["channel_count"]


def _mk(key, rate, stream):
    dt_type = getattr(DataType, stream, None) if stream else None
    label_stream = stream or key
    return {
        "key": key,
        "rate": str(rate),
        "expected": float(rate),
        "stream": stream,
        "label": f"{label_stream}@{rate}",
        "dt_type": dt_type,
    }


def _schedule(sample_rates):
    """把 sample_rates 编排成若干会话，返回 (sessions, fixed_items)。

    sessions    : list[list[dict]]，每会话为需 setParam 的档（settable 且 i < len(rates)）。
    fixed_items : list[dict]，固定采样率流（仅第 0 次会话测）。
    """
    settable = [(k, e) for k, e in sample_rates.items() if e.get("settable", True)]
    fixed = [(k, e) for k, e in sample_rates.items() if not e.get("settable", True)]

    n_sessions = max((len(e.get("rates", [])) for _, e in settable), default=0)
    if n_sessions <= 0 and fixed:
        n_sessions = 1

    sessions = []
    for i in range(n_sessions):
        items = []
        for key, entry in settable:
            rates = entry.get("rates", [])
            if i < len(rates):
                items.append(_mk(key, rates[i], entry.get("stream")))
        sessions.append(items)

    fixed_items = []
    for key, entry in fixed:
        rates = entry.get("rates", ["1"])
        it = _mk(key, rates[0], entry.get("stream"))
        it["fixed"] = True
        fixed_items.append(it)

    return sessions, fixed_items


def _list_bins(log_dir):
    if not log_dir or not os.path.isdir(log_dir):
        return {}
    out = {}
    try:
        for fn in os.listdir(log_dir):
            if fn.lower().endswith(".bin"):
                out[fn] = os.path.join(log_dir, fn)
    except OSError:
        pass
    return out


def _get_ble_path(sensor):
    try:
        return sensor.getParam("DEBUG_BLE_DATA_PATH")
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def main():
    target_identity = sys.argv[1].strip().upper() if len(sys.argv) > 1 else None
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("OB6000C 采样率实测（spec 驱动，多流并测）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    # 受控日志/bin 目录：Sample Rate Result
    os.makedirs(RESULT_DIR, exist_ok=True)
    bins_before = _list_bins(RESULT_DIR)
    try:
        ctrl.setLogPath(True, RESULT_DIR)
        log_txt = f"setLogPath(True, {RESULT_DIR}) 无异常"
    except Exception as e:
        log_txt = f"setLogPath 抛异常 {type(e).__name__}: {e}"
    print(f"[bin] {log_txt}", flush=True)
    try:
        ctrl.setDebugEnabled(True)
    except Exception as e:
        print(f"[bin] setDebugEnabled(True) 抛异常 {type(e).__name__}: {e}", flush=True)

    # 扫描
    print(f"\n[扫描] 目标 identity: {target_identity or common.TARGET_IDENTITIES} ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, target_identity=target_identity)
    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        ctrl.terminate()
        return
    print(f"[扫描] 目标设备: {getattr(target, 'Name', '?')} {getattr(target, 'Address', '?')}", flush=True)

    # 加载 spec
    try:
        spec = _load_spec(target)
    except Exception as e:
        print(f"[FAIL] 加载 spec 失败 {type(e).__name__}: {e}", flush=True)
        ctrl.terminate()
        return
    sample_rates = spec.get("sample_rates") or {}
    print(f"[spec] 模型={spec.get('model')} sample_rates 条目数={len(sample_rates)}", flush=True)
    if not sample_rates:
        print("[FAIL] spec 无 sample_rates，无法遍历", flush=True)
        ctrl.terminate()
        return

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] requireSensor 返回 None", flush=True)
        ctrl.terminate()
        return

    # 连接
    try:
        ok = sensor.connect()
    except Exception as e:
        ok = None
        print(f"[连接] connect 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[连接] connect() -> {ok}  state={sensor.deviceState}", flush=True)
    if ok is not True:
        print("[FAIL] connect 失败", flush=True)
        ctrl.terminate()
        return

    t0 = time.time()
    while time.time() - t0 < READY_TIMEOUT and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[FAIL] 未到达 Ready（state={sensor.deviceState}）", flush=True)
        ctrl.terminate()
        return

    # init
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
    except Exception as e:
        iret = None
        print(f"[init] init 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[init] init() -> {iret}  hasInited={sensor.hasInited}", flush=True)
    if iret is not True:
        print("[FAIL] init 失败", flush=True)
        ctrl.terminate()
        return

    # 开启 bin 导出（注意值为字符串 "True"，不是 Python bool）
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
    except Exception as e:
        bret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[bin] setParam DEBUG_BLE_DATA_PATH=True -> {bret!r}", flush=True)

    # 编排会话
    sessions, fixed_items = _schedule(sample_rates)
    print(f"\n[编排] 会话数={len(sessions)}，固定流条目数={len(fixed_items)}", flush=True)

    # 开启所有待测流 + 伴随流（全程保持 ON）
    streams_to_enable = []
    for entry in sample_rates.values():
        s = entry.get("stream")
        if s and s not in streams_to_enable:
            streams_to_enable.append(s)
        co = entry.get("co_stream")
        if co and co not in streams_to_enable:
            streams_to_enable.append(co)
    print(f"[起流] 开启待测流: {streams_to_enable}", flush=True)
    for s in streams_to_enable:
        try:
            sensor.setParam(s, "ON")
        except Exception as e:
            print(f"[setParam] {s} ON 抛异常 {type(e).__name__}: {e}", flush=True)

    collector = RateCollector()
    sensor.onDataCallback = collector.on_data

    summaries = []

    for si, items in enumerate(sessions):
        # 本会话要测的档：可设置流的新档 + 第 0 次会话的固定流
        targets = list(items)
        if si == 0:
            targets = targets + fixed_items

        valid = []
        for it in targets:
            if it["dt_type"] is None:
                summaries.append((it["label"], None, f"DataType.{it['stream']} 不存在"))
            else:
                valid.append(it)
        if not valid:
            continue

        print("\n" + "=" * 40, flush=True)
        new_rates = [f"{it['stream']}@{it['rate']}" for it in valid if not it.get("fixed")]
        fixed_labels = [f"{it['stream']}@{it['rate']}" for it in valid if it.get("fixed")]
        print(f"[会话 {si + 1}/{len(sessions)}] 新设档: {new_rates or '无'}  固定流: {fixed_labels or '无'}", flush=True)

        # 只设没测的档（固定流不设采样率）
        ok_targets = []
        for it in valid:
            if it.get("fixed"):
                print(f"[固定] {it['stream']} 采样率固定 {it['rate']}Hz，跳过 setParam", flush=True)
                ok_targets.append(it)
                continue
            try:
                r = sensor.setParam(it["key"], it["rate"])
            except Exception as e:
                r = f"抛异常 {type(e).__name__}: {e}"
            print(f"[setParam] {it['key']}={it['rate']} -> {r!r}", flush=True)
            if r != "OK":
                summaries.append((it["label"], None, f"setParam {it['key']}={it['rate']} 返回 {r!r}"))
                continue
            try:
                cur = sensor.getParam(it["key"])
                print(f"[getParam] {it['key']} = {cur!r}", flush=True)
            except Exception as e:
                print(f"[getParam] {it['key']} 抛异常 {type(e).__name__}: {e}", flush=True)
            ok_targets.append(it)

        if not ok_targets:
            continue

        # 静置等待设备异步生效
        print(f"[等待] 静置 {SETTLE_SECONDS}s 等待采样率生效 ...", flush=True)
        time.sleep(SETTLE_SECONDS)

        # 起流
        collector.reset()
        try:
            sret = sensor.startDataNotification()
        except Exception as e:
            sret = None
            print(f"[起流] 抛异常 {type(e).__name__}: {e}", flush=True)
        print(f"[起流] startDataNotification() -> {sret}", flush=True)
        if sret is not True:
            for it in ok_targets:
                summaries.append((it["label"], None, "startDataNotification 失败"))
            try:
                sensor.stopDataNotification()
            except Exception:
                pass
            continue

        # 等首批数据到达（任意流）
        print("[采集] 等待首批数据到达 ...", flush=True)
        t_wait0 = time.time()
        while not collector.has_data and time.time() - t_wait0 < FIRST_DATA_TIMEOUT:
            time.sleep(0.05)

        if not collector.has_data:
            print(f"[采集] {FIRST_DATA_TIMEOUT}s 内未收到任何数据", flush=True)
            for it in ok_targets:
                summaries.append((it["label"], None, "起流后无任何数据到达"))
            try:
                sensor.stopDataNotification()
            except Exception:
                pass
            continue

        # 预热：丢弃起流初期 WARMUP_SECONDS 秒数据
        print(f"[采集] 首批已到达，预热 {WARMUP_SECONDS:.0f}s（丢弃起流初期数据）...", flush=True)
        warmup_end = time.time() + WARMUP_SECONDS
        while time.time() < warmup_end:
            time.sleep(0.05)

        # 预热结束，从此刻起正式计数
        collector.reset()
        t_start = time.time()
        t_end = t_start + MEASURE_SECONDS
        print(f"[采集] 预热结束，正式采集 {MEASURE_SECONDS}s ...", flush=True)
        while time.time() < t_end:
            time.sleep(0.05)

        try:
            sensor.stopDataNotification()
        except Exception as e:
            print(f"[停流] 抛异常 {type(e).__name__}: {e}", flush=True)

        # 统计 + 出结果（只对本次新设档 + 固定流）
        for it in ok_targets:
            unique, dur, delivered, ch = collector.result_for(it["dt_type"])
            if dur <= 0 or unique <= 0:
                summaries.append((it["label"], None, f"未收到 {it['stream']} 数据（唯一样本={unique}）"))
                continue
            actual = unique / dur
            dev = abs(actual - it["expected"]) / it["expected"] if it["expected"] > 0 else float("inf")
            ok = dev <= SAMPLE_RATE_TOLERANCE
            print(f"[结果] {it['label']} 期望={it['expected']}Hz, 实际≈{actual:.3f}Hz 偏差={dev:.3%} "
                  f"(唯一样本={unique} 投递样本={delivered} 通道={ch} 时长={dur:.1f}s)", flush=True)
            summaries.append((it["label"], ok,
                              f"期望={it['expected']}Hz, 实际≈{actual:.3f}Hz 偏差={dev:.3%}（唯一样本={unique}）"))

    # 清理
    for s in streams_to_enable:
        try:
            sensor.setParam(s, "OFF")
        except Exception:
            pass

    # 断开前先取 bin 路径（断开后 getParam 会报 "Please connect first"）
    ble_path = _get_ble_path(sensor)
    print(f"[bin] 断开前 getParam('DEBUG_BLE_DATA_PATH') = {ble_path!r}", flush=True)

    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 导出 bin 到 Sample Rate Result ----
    print("\n" + "=" * 60, flush=True)
    print("导出 bin 文件到 Sample Rate Result", flush=True)
    print("=" * 60, flush=True)
    time.sleep(0.5)  # 等 SDK 落盘（close 写 header）
    bins_after = _list_bins(RESULT_DIR)
    new_bins = [fn for fn in bins_after if fn not in bins_before]

    bin_path = None
    if isinstance(ble_path, str) and ble_path.strip():
        cand = ble_path.strip()
        if os.path.isfile(cand):
            bin_path = cand
    if bin_path is None and new_bins:
        bin_path = bins_after[new_bins[0]]

    print(f"[bin] 目录 {RESULT_DIR}", flush=True)
    print(f"[bin] 本次新增 bin: {new_bins or '无'}", flush=True)

    if bin_path and os.path.isfile(bin_path):
        print(f"[bin] 路径: {bin_path}", flush=True)
        try:
            info = ctrl.getBinFileInfo(bin_path)
        except Exception as e:
            info = None
            print(f"[bin] getBinFileInfo 抛异常 {type(e).__name__}: {e}", flush=True)
        if isinstance(info, dict):
            print(f"[bin] replay_duration={info.get('replay_duration')}s  "
                  f"device_mac={info.get('device_mac')}  device_name={info.get('device_name')}", flush=True)
        else:
            print(f"[bin] getBinFileInfo 返回 {info!r}（可能 header 未写入）", flush=True)
    else:
        print(f"[bin] 未找到有效 bin（getParam 返回 {ble_path!r}）", flush=True)

    # 关闭 bin 导出与 debug
    try:
        sensor.setParam("DEBUG_BLE_DATA_PATH", "False")
    except Exception:
        pass
    try:
        ctrl.setDebugEnabled(False)
    except Exception:
        pass

    ctrl.terminate()

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("采样率实测汇总", flush=True)
    print("=" * 60, flush=True)
    all_ok = True
    for label, ok, detail in summaries:
        if ok is None or ok is False:
            print(f"  [FAIL] {label}：{detail}", flush=True)
            all_ok = False
        else:
            print(f"  [PASS] {label}：{detail}", flush=True)
    print("\n结论: " + ("PASS" if all_ok else "FAIL"), flush=True)


if __name__ == "__main__":
    main()
