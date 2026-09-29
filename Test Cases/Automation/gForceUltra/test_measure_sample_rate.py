# -*- coding: utf-8 -*-
"""gForceUltra 采样率实测（spec 驱动，多流并测）。

用途：专项验证 gForceUltra 各数据流的真实采样率，不并入正式用例。

做法：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 读取 device_specs/gforce_ultra.py 的 spec["sample_rates"]
  3) 一次把全部待测流 setParam("NTF_XXX","ON")，跨会话保持 ON
  4) 会话数 = 所有「可设置」流里 rates 数量最多的那个；每会话只设「还没测完」的档
  5) 每会话：setParam 新档 -> 静置 -> 起流 -> 等首批 -> 预热 -> 正式采集 MEASURE_SECONDS 秒
     -> 停流 -> 对本次新设档的流（+第 0 次会话的固定流）按 DataType 分桶统计
  6) 实际采样率 = 唯一样本数 / host 精确窗口（t_end - t_start）；与标称比对

要点：
  - 采样率来源统一为 spec["sample_rates"]（含固定 1Hz 的 IMPEDANCE），脚本不硬编码。
  - stream 既是 setParam("NTF_XXX","ON") 开关 key，也是 DataType.NTF_XXX 类型名。
  - 多流并测：所有流全程 ON，同一次起流里各 DataType 同时到达，用多路分桶一起统计。
  - 只设没测的档：某流 i >= len(rates) 时不再 setParam 采样率（流仍保持 ON 在流），
    只对本次新设档的流出结果；固定流只在第 0 次会话测一次。
  - 起流后先预热 10s（丢弃起流初期不稳定样本），再正式计数。
  - 去重口径：unique = max_sample_index - min_sample_index + 1（通道0 sampleIndex 首末跨度）。
  - 时长口径（主判定）：唯一样本数 / host 精确窗口（t_end - t_start），host 墙钟作参考、与设备无关。
      * 设备端 absTimeStampInSec 已确认是「sampleIndex ÷ 标称采样率」反推（循环论证，永远 ≈ 标称），
        只作「固件自报」对比，不参与主判定。

用法：
  python test_measure_sample_rate.py
  python test_measure_sample_rate.py 80E1            # 指定 gForceUltra identity
  python test_measure_sample_rate.py 80E1 1000       # 只跑 1000Hz 这一档（第二档）

前置条件：
  - 主机蓝牙已开启（或 USB dongle 已就绪）
  - 待测 gForceUltra 上电、在范围内
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

MEASURE_SECONDS = 1800         # 每次会话正式采集时长（秒）= 30 分钟（长时间积分暴露采样时钟漂移）
SETTLE_SECONDS = 2.0           # setParam 后等待设备异步生效的静置时长
WARMUP_SECONDS = 10.0          # 起流后预热时长（丢弃起流初期不稳定样本）
READY_TIMEOUT = 15             # 连接后等待 Ready 超时（秒）
FIRST_DATA_TIMEOUT = 10        # 起流后等待首批数据超时（秒）
SAMPLE_RATE_TOLERANCE = 0.001  # 采样率偏差容差 ±0.1%（千分之一）


def _load_spec(target):
    """由匹配到的设备解析其 spec（优先 identity，回退到 gForceUltra）。"""
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
    return common.load_spec("gForceUltra")


class RateCollector:
    """多流同时统计：按 DataType 分桶记录样本数、sampleIndex 跨度、首末到达时刻。"""

    def __init__(self):
        self.stats = {}   # dt(int) -> dict
        self.arrival_times = []  # 全局数据批到达时刻（time.time()），用于 gap（数据中断）检测

    def reset(self):
        self.stats = {}
        self.arrival_times = []

    @property
    def has_data(self):
        return bool(self.stats)

    def analyze_gaps(self, threshold=3.0):
        """返回 [(gap_start_ts, gap_end_ts, gap_dur), ...]，相邻批间隔超过 threshold 秒视为数据中断。"""
        gaps = []
        at = self.arrival_times
        for i in range(1, len(at)):
            gap = at[i] - at[i - 1]
            if gap > threshold:
                gaps.append((at[i - 1], at[i], gap))
        return gaps

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        self.arrival_times.append(time.time())  # 记录每个回调到达时刻（跨所有流）
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
                    "min_abs_ts": None, "max_abs_ts": None,  # 设备端采样时刻（absTimeStampInSec），与 min/max_idx 对齐
                    "timeline": [],  # 每批 (arrival_ts, batch_min_idx, batch_max_idx)，用于分段漂移趋势
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
            # 通道0 sampleIndex 记录全局首末（跨批单调递增，去重）；同时记录本批首末用于分段趋势
            batch_min = None
            batch_max = None
            for si in range(n_smp):
                try:
                    idx = d.getSampleIndex(0, si)
                except Exception:
                    continue
                if idx is None:
                    continue
                if batch_min is None or idx < batch_min:
                    batch_min = idx
                if batch_max is None or idx > batch_max:
                    batch_max = idx
                if s["min_idx"] is None or idx < s["min_idx"]:
                    s["min_idx"] = idx
                    try:
                        s["min_abs_ts"] = d.getAbsTimeStampInSec(0, si)
                    except Exception:
                        pass
                if s["max_idx"] is None or idx > s["max_idx"]:
                    s["max_idx"] = idx
                    try:
                        s["max_abs_ts"] = d.getAbsTimeStampInSec(0, si)
                    except Exception:
                        pass
            if batch_min is not None:
                s["timeline"].append((now, batch_min, batch_max))

    def result_for(self, dt):
        """返回 (unique, dur_host, dur_abs, delivered, channel_count)；无数据返回 (0, 0.0, 0.0, 0, 0)。

        dur_host : host 侧数据到达首末时刻差（last_ts - first_ts），受 BLE 传输/缓冲影响会偏小。
        dur_abs  : 设备端采样时刻差（max_abs_ts - min_abs_ts），与 sampleIndex 严格对齐；
                   当 absTimeStampInSec 未知（起流锚点未建立）时为 0。
        """
        s = self.stats.get(dt)
        if not s:
            return 0, 0.0, 0.0, 0, 0
        unique = 0
        if s["min_idx"] is not None and s["max_idx"] is not None:
            unique = s["max_idx"] - s["min_idx"] + 1
        dur_host = 0.0
        if s["first_ts"] is not None and s["last_ts"] is not None:
            dur_host = s["last_ts"] - s["first_ts"]
        dur_abs = 0.0
        if (s["min_abs_ts"] is not None and s["max_abs_ts"] is not None
                and s["max_abs_ts"] > s["min_abs_ts"]):
            dur_abs = s["max_abs_ts"] - s["min_abs_ts"]
        return unique, dur_host, dur_abs, s["delivered"], s["channel_count"]

    def minute_rates(self, dt, t_start, t_end, window=60.0):
        """按 window 秒切分 timeline，返回每段采样率数组（unique / 窗口时长）。

        仅在采集结束后调用（timeline 在采集阶段只做轻量 append，不影响采样）。
        分母用 host 窗口（方案 A：host NTP 墙钟作参考，量设备采样时钟相对 host 的漂移）；
        无数据的段记为 0.0，用于暴露断流/静默断流。
        """
        s = self.stats.get(dt)
        if not s or not s["timeline"]:
            return []
        tl = s["timeline"]
        out = []
        i = 0
        n = len(tl)
        k = 0
        while True:
            w_start = t_start + k * window
            if w_start >= t_end:
                break
            w_end = min(w_start + window, t_end)
            seg_min = seg_max = None
            while i < n and tl[i][0] < w_start:
                i += 1
            while i < n and tl[i][0] < w_end:
                ts, mn, mx = tl[i]
                if seg_min is None or mn < seg_min:
                    seg_min = mn
                if seg_max is None or mx > seg_max:
                    seg_max = mx
                i += 1
            if seg_min is None:
                out.append(0.0)
            else:
                unique = seg_max - seg_min + 1
                win_dur = w_end - w_start
                out.append(round(unique / win_dur, 3))
            k += 1
        return out

    def drift_metrics(self, arr):
        """从逐分钟采样率数组提取漂移指标（窗口固定 60s，斜率单位即 Hz/min）。

        arr : minute_rates() 的输出（每段一个速率，断流段为 0.0）。
        返回 (slope, first, last, spread)；有效段不足 2 个时返回 None。
        slope  : 最小二乘线性拟合斜率（Hz/min，负=采样时钟随时间变慢）。
        first/last : 有效段首/末速率；spread = 有效段 max−min（短期抖动幅度）。
        """
        if not arr:
            return None
        pts = [(i, r) for i, r in enumerate(arr) if r > 0]
        if len(pts) < 2:
            return None
        n = len(pts)
        xm = sum(p[0] for p in pts) / n
        ym = sum(p[1] for p in pts) / n
        sxx = sum((p[0] - xm) ** 2 for p in pts)
        sxy = sum((p[0] - xm) * (p[1] - ym) for p in pts)
        slope = sxy / sxx if sxx > 0 else 0.0
        first = pts[0][1]
        last = pts[-1][1]
        spread = max(p[1] for p in pts) - min(p[1] for p in pts)
        return slope, first, last, spread


class SessionMonitor:
    """采集期间监控连接状态/错误/自动重连，记录事件时间线，用于定位数据中断原因。

    通过 SensorProfile 的 onStateChange / onErrorCallback / onAutoReconnect 回调，
    把「断连 / 重连 / 报错」与数据 gap 的时间点对齐，区分「是断连重连」还是「静默断流」。
    """

    def __init__(self):
        self.events = []  # (ts, kind, detail)，kind ∈ {state, error, reconnect}

    def reset(self):
        self.events = []

    def _ts(self):
        return time.strftime("%H:%M:%S", time.localtime())

    def on_state_change(self, sensor, state):
        name = getattr(state, "name", str(state))
        self.events.append((time.time(), "state", name))
        print(f"  [状态] {self._ts()} deviceState -> {name}", flush=True)

    def on_error(self, sensor, msg):
        self.events.append((time.time(), "error", str(msg)))
        print(f"  [错误] {self._ts()} -> {msg}", flush=True)

    def on_auto_reconnect(self, sensor, has_last_session, answer):
        self.events.append((time.time(), "reconnect", f"has_last_session={bool(has_last_session)}"))
        print(f"  [重连] {self._ts()} autoReconnect -> has_last_session={bool(has_last_session)}", flush=True)
        try:
            answer(True)  # 告知 SDK 已处理（继续重连流程）
        except Exception:
            pass


def _mk(key, rate, stream, co_stream=None):
    dt_type = getattr(DataType, stream, None) if stream else None
    label_stream = stream or key
    return {
        "key": key,
        "rate": str(rate),
        "expected": float(rate),
        "stream": stream,
        "co_stream": co_stream,   # 伴随流（随主流量打包投递）非 None；其设备时间戳不可信
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
                items.append(_mk(key, rates[i], entry.get("stream"), entry.get("co_stream")))
        sessions.append(items)

    fixed_items = []
    for key, entry in fixed:
        rates = entry.get("rates", ["1"])
        it = _mk(key, rates[0], entry.get("stream"), entry.get("co_stream"))
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
    rate_filter = sys.argv[2].strip() if len(sys.argv) > 2 else None
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("gForceUltra 采样率实测（spec 驱动，多流并测）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    # dongle 后端检查：稳定性测试仅支持 USB dongle，非 dongle 后端不支持，直接报错退出
    try:
        dongle_ok = ctrl.checkSetupDongle()
    except Exception as e:
        dongle_ok = None
        print(f"[dongle] checkSetupDongle() 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[dongle] checkSetupDongle() -> {dongle_ok!r}", flush=True)
    if not (isinstance(dongle_ok, str) and dongle_ok.startswith("OK")):
        print("[FAIL] 未检测到 USB dongle（backend 非 dongle），稳定性测试不支持非 dongle 后端，已退出。", flush=True)
        ctrl.terminate()
        return
    print("[dongle] 提醒：已确认 backend 为 USB dongle（稳定性测试仅支持 dongle），继续执行。", flush=True)

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

    # 连接状态/错误/重连监控（尽早注册，捕获 connect 之后的断连重连）
    monitor = SessionMonitor()
    sensor.onStateChanged = monitor.on_state_change
    sensor.onErrorCallback = monitor.on_error
    sensor.onAutoReconnect = monitor.on_auto_reconnect

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
    if rate_filter:
        print(f"[编排] 按采样率档过滤：只跑 {rate_filter}Hz 这一档（跳过其它档）", flush=True)
    print(f"[编排] 会话数={len(sessions)}，固定流条目数={len(fixed_items)}", flush=True)

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
        # 指定采样率档时，跳过不含该档的会话
        if rate_filter and all(it["rate"] != rate_filter for it in items):
            continue
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

        # 测 EMG（500/1000Hz）时：关闭手势流 GEST，避免与 EMG 采样冲突。
        # IMU/加速度/陀螺仪/阻抗是 EMG 的伴随流（spec 中 co_stream=NTF_EMG），
        # 需随 EMG 同起才有数据，不能关。
        if any(it["stream"] == "NTF_EMG" for it in ok_targets):
            print("[独占] 关闭手势流 GEST ...", flush=True)
            try:
                sensor.setParam("NTF_GEST", "OFF")
            except Exception as e:
                print(f"[独占] setParam NTF_GEST OFF 抛异常 {type(e).__name__}: {e}", flush=True)

        # 静置等待设备异步生效
        print(f"[等待] 静置 {SETTLE_SECONDS}s 等待采样率生效 ...", flush=True)
        time.sleep(SETTLE_SECONDS)

        # 起流
        collector.reset()
        monitor.reset()  # 本会话采集期间的状态/错误/重连事件从此刻开始记录
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

        # 诊断：数据中断（gap）与连接状态/错误/重连事件，定位采样率偏差原因
        gaps = collector.analyze_gaps(threshold=3.0)
        if gaps:
            total_gap = sum(g[2] for g in gaps)
            print(f"[诊断] 检测到 {len(gaps)} 次数据中断（相邻批间隔>3s），累计约 {total_gap:.1f}s", flush=True)
            for gs, ge, gd in gaps:
                print(f"       中断 {time.strftime('%H:%M:%S', time.localtime(gs))} ~ "
                      f"{time.strftime('%H:%M:%S', time.localtime(ge))}（{gd:.1f}s）", flush=True)
        else:
            print("[诊断] 未检测到数据中断（相邻批间隔均≤3s）", flush=True)
        if monitor.events:
            print(f"[诊断] 采集期间连接事件 {len(monitor.events)} 条:", flush=True)
            for ts, kind, detail in monitor.events:
                print(f"       {time.strftime('%H:%M:%S', time.localtime(ts))} [{kind}] {detail}", flush=True)
        else:
            print("[诊断] 采集期间无状态/错误/重连事件", flush=True)

        # 统计 + 出结果（只对本次新设档 + 固定流）
        # 采样率主口径：唯一样本数 / host 精确窗口（t_end - t_start），host 墙钟作参考、与设备无关。
        #   设备端 absTimeStampInSec 已确认是「sampleIndex ÷ 标称采样率」反推（循环论证，永远 ≈ 标称，
        #   测不出真实偏差），故只作「固件自报」对比，不参与主判定。
        for it in ok_targets:
            unique, dur_host, dur_abs, delivered, ch = collector.result_for(it["dt_type"])
            if unique <= 0:
                summaries.append((it["label"], None, f"未收到 {it['stream']} 数据（唯一样本={unique}）"))
                continue
            win_dur = t_end - t_start
            denom = win_dur
            actual = unique / denom
            dev = abs(actual - it["expected"]) / it["expected"] if it["expected"] > 0 else float("inf")
            ok = dev <= SAMPLE_RATE_TOLERANCE
            print(f"[结果] {it['label']} 期望={it['expected']}Hz, 实际≈{actual:.3f}Hz 偏差={dev:.3%} "
                  f"(唯一样本={unique} 投递样本={delivered} 通道={ch} 分母=host窗口={win_dur:.3f}s)", flush=True)
            # 口径对比：host 到达时刻差（受 BLE 首尾延迟影响，可能偏高）与设备时间戳（固件自报，仅作参考）
            extra = []
            if dur_host > 0:
                extra.append(f"host到达时刻差={dur_host:.3f}s -> {unique / dur_host:.3f}Hz")
            if dur_abs > 0:
                extra.append(f"设备时间戳(固件自报)={dur_abs:.3f}s -> {unique / dur_abs:.3f}Hz")
            if extra:
                print("      [口径] " + " | ".join(extra), flush=True)
            # 分段漂移趋势（方案 A：host NTP 墙钟作参考，逐分钟看速率漂移）
            minute_arr = collector.minute_rates(it["dt_type"], t_start, t_end)
            print(f"      [分段/分钟] {minute_arr}", flush=True)
            # 独立漂移指标：斜率 / 首尾差 / 峰峰抖动
            drift = collector.drift_metrics(minute_arr)
            if drift:
                slope, first, last, spread = drift
                drift_pct = (last - first) / it["expected"] * 100.0 if it["expected"] > 0 else float("nan")
                print(f"      [漂移] 斜率={slope:+.4f}Hz/min | 首尾={first:.3f}->{last:.3f}Hz"
                      f"(Δ{last - first:+.3f}Hz, {drift_pct:+.3f}%) | 峰峰={spread:.3f}Hz", flush=True)
            else:
                print("      [漂移] 有效段不足，无法计算", flush=True)
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
