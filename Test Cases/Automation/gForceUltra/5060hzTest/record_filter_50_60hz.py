# -*- coding: utf-8 -*-
"""50Hz/60Hz 陷波滤波录制脚本（gForceUltra + 信号发生器注入）。

用途：配合信号发生器，分别录制 50Hz / 60Hz 陷波滤波器 ON/OFF 共 4 段 bin，
      用于离线分析滤波效果（陷波器对对应频率工频干扰的抑制能力）。

流程：
  1) 扫描并连接目标设备 gForceUltra（identity 80E1），读 50/60Hz 滤波初始状态
  2) 提示接入 50Hz 100μV 信号 → 确认
  3) 开 FILTER_50HZ → 起流 20s → 停 → 导出 bin（50on）
  4) 关 FILTER_50HZ → 起流 20s → 停 → 导出 bin（50off）
  5) 提示切换 60Hz 100μV 信号 → 确认
  6) 开 FILTER_60HZ → 起流 20s → 停 → 导出 bin（60on）
  7) 关 FILTER_60HZ → 起流 20s → 停 → 导出 bin（60off）
  8) 交叉验证 A：50Hz 滤波 ON + 60Hz 滤波 OFF，注入 60Hz → 60Hz 不应被过滤（cross_60hz）
  9) 交叉验证 B：60Hz 滤波 ON + 50Hz 滤波 OFF，注入 50Hz → 50Hz 不应被过滤（cross_50hz）
 10) 6 个 bin 移动到本目录并重命名；恢复初始滤波状态

说明：
  - 每段采用「独立 connect → init → 起流 → stop → disconnect」，保证每个 bin
    明确对应一段 20s 录制，避免 SDK 持续写临时文件导致 bin 内容边界不清晰。
  - bin 录制无独立 start/stop 接口：连接后 SDK 持续写 BLE 原始包，仅在
    stopDataNotification/disconnect 且 DEBUG_BLE_DATA_PATH=True 时导出 .bin。

前置条件：
  - 主机蓝牙已开启；gForceUltra 上电、在范围内
  - 信号发生器：能输出 50Hz/60Hz、100μV 正弦波，接入 gForceUltra 通道 3
"""

import json
import os
import subprocess
import sys
import time
import math
import shutil

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import scan_and_match

RECORD_SECONDS = 20  # 每段录制时长（秒）
LOG_DIR = os.path.join(BASE_DIR, "logs")


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


def _connect_ready(sensor):
    """connect → 等待 Ready → init，返回是否就绪。"""
    try:
        ok = sensor.connect()
    except Exception as e:
        print(f"[连接] connect 抛异常 {type(e).__name__}: {e}", flush=True)
        return False
    print(f"[连接] connect -> {ok} state={sensor.deviceState}", flush=True)

    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[连接] 未到达 Ready（state={sensor.deviceState}）", flush=True)
        return False

    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
    except Exception as e:
        print(f"[init] 抛异常 {type(e).__name__}: {e}", flush=True)
        return False
    print(f"[init] -> {iret}", flush=True)
    return iret is True


def _read_filter_states(sensor):
    """读聚合 getParam('FILTER')，返回 {key: value}（value 为 'ON'/'OFF'）。

    FILTER 状态由 SDK 以聚合形式提供：getParam('FILTER') 返回
    'FILTER_50HZ|ON|FILTER_60HZ|OFF|...' 这样的字符串，这里解析成字典。
    """
    states = {}
    try:
        raw = sensor.getParam("FILTER")
    except Exception as e:
        print(f"[滤波] getParam('FILTER') 抛异常 {type(e).__name__}: {e}", flush=True)
        return states
    if isinstance(raw, str) and not raw.startswith("Error") and "|" in raw:
        items = raw.split("|")
        for i in range(0, len(items) - 1, 2):
            states[items[i]] = items[i + 1]
    else:
        print(f"[滤波] getParam('FILTER') = {raw!r}（无聚合内容）", flush=True)
    return states


def _record_segment(sensor, filter_settings, seconds, log_dir, before_bins):
    """设置滤波（可多个）→ 开启 bin 导出 → 起流 → sleep → 停流 → 返回导出 bin 路径。"""
    for filter_key, filter_val in filter_settings:
        fret = sensor.setParam(filter_key, filter_val)
        print(f"[滤波] setParam({filter_key}, {filter_val}) -> {fret}", flush=True)

    sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
    try:
        sret = sensor.startDataNotification()
    except Exception as e:
        sret = None
        print(f"[起流] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[起流] startDataNotification -> {sret}", flush=True)
    if sret is not True:
        sensor.setParam("DEBUG_BLE_DATA_PATH", "False")
        return None

    print(f"[录制] 起流 {seconds}s ...", flush=True)
    time.sleep(seconds)

    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)
    time.sleep(0.5)

    # 优先 getParam 读导出路径，回退到日志目录扫描
    bin_path = None
    try:
        p = sensor.getParam("DEBUG_BLE_DATA_PATH")
        if isinstance(p, str) and p.strip():
            bin_path = p.strip()
    except Exception:
        pass
    sensor.setParam("DEBUG_BLE_DATA_PATH", "False")

    if not (bin_path and os.path.isfile(bin_path)):
        after = _list_bins(log_dir)
        new_bins = sorted(set(after.keys()) - set(before_bins.keys()))
        if new_bins:
            bin_path = after[new_bins[-1]]
        print(f"[bin] getParam 未给路径，目录新增 bin: {new_bins}", flush=True)

    if bin_path and os.path.isfile(bin_path):
        print(f"[bin] 导出 {bin_path}", flush=True)
        return bin_path
    print("[bin] 未取得导出 bin", flush=True)
    return None


def _run_one_segment(sensor, log_dir, out_name, filter_settings):
    """独立连接 → 录一段 → 断开 → 移动到本目录并重命名。"""
    desc = " ".join(f"{k}={v}" for k, v in filter_settings)
    print(f"\n---- 录制段 {out_name}（{desc}）----", flush=True)
    if not _connect_ready(sensor):
        print(f"[FAIL] {out_name} 连接/init 失败", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        return None

    before = _list_bins(log_dir)
    bin_path = _record_segment(sensor, filter_settings, RECORD_SECONDS, log_dir, before)
    try:
        sensor.disconnect()
    except Exception:
        pass

    if not bin_path:
        print(f"[FAIL] {out_name} 未取得 bin", flush=True)
        return None

    dest = os.path.join(BASE_DIR, out_name + ".bin")
    try:
        shutil.move(bin_path, dest)
    except Exception as e:
        print(f"[移动] 移动失败 {type(e).__name__}: {e}，保留原路径", flush=True)
        dest = bin_path
    print(f"[OK] {out_name} -> {dest}", flush=True)
    return dest


def _goertzel_amplitude(samples, fs, target_freq):
    """Goertzel 算法计算目标频率的正弦峰幅值（纯 Python，无需 numpy）。"""
    n = len(samples)
    if n == 0:
        return 0.0
    k = int(round(target_freq * n / fs))
    if k <= 0 or k >= n:
        return 0.0
    w = 2.0 * math.pi * k / n
    coeff = 2.0 * math.cos(w)
    s_prev = 0.0
    s_prev2 = 0.0
    for x in samples:
        s = x + coeff * s_prev - s_prev2
        s_prev2 = s_prev
        s_prev = s
    power = s_prev2 * s_prev2 + s_prev * s_prev - coeff * s_prev * s_prev2
    return math.sqrt(max(power, 0.0)) * 2.0 / n


def _analyze_bin(bin_path, target_freqs):
    """在独立子进程回放 bin，收集 EMG 各通道样本，输出每通道时域峰值与各目标频率幅值。

    返回 (channels_amps, channels_peaks, fs)：
      channels_amps  = {通道索引: {频率: Goertzel 峰值}}
      channels_peaks = {通道索引: 时域峰值 (max-min)/2}
    """
    worker = os.path.join(AUTOMATION_DIR, "replay_worker.py")
    cmd = [sys.executable, worker, "--bin", bin_path, "--mode", "samples"]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        lines = [ln for ln in (proc.stdout or "").splitlines() if ln.strip()]
        if not lines:
            print(f"[分析] 子进程无 stdout，returncode={proc.returncode} stderr={(proc.stderr or '')[:500]}", flush=True)
            return {}, {}, None
        result = json.loads(lines[-1])
    except Exception as e:
        print(f"[分析] 子进程回放异常 {type(e).__name__}: {e}", flush=True)
        return {}, {}, None

    if result.get("ok") is not True:
        print(f"[分析] 子进程回放失败：{result.get('error')}", flush=True)
        return {}, {}, None

    fs = result.get("fs") or 500
    raw_channels = result.get("channels", {})
    channels_amps = {}
    channels_peaks = {}
    for ci_str, vals in raw_channels.items():
        if not vals:
            continue
        ci = int(ci_str)
        channels_amps[ci] = {f: _goertzel_amplitude(vals, fs, f) for f in target_freqs}
        channels_peaks[ci] = (max(vals) - min(vals)) / 2.0
    return channels_amps, channels_peaks, fs


def _judge(on_amp, off_amp, label, freq):
    """比较 ON/OFF 幅值，输出抑制比与判定。"""
    if off_amp <= 1e-9:
        return f"{label}: OFF 幅值≈0（{freq} 信号未注入或数据异常），无法判断"
    ratio = on_amp / off_amp
    db = 20.0 * math.log10(ratio) if ratio > 0 else -999.0
    if ratio < 0.3:
        return f"{label}: ON={on_amp:.4f} OFF={off_amp:.4f} 抑制={ratio:.3f}（{db:.1f}dB）-> 生效"
    return f"{label}: ON={on_amp:.4f} OFF={off_amp:.4f} 抑制={ratio:.3f}（{db:.1f}dB）-> 未生效/抑制不足"


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("50Hz / 60Hz 陷波滤波录制（gForceUltra + 信号发生器）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)

    os.makedirs(LOG_DIR, exist_ok=True)
    try:
        ctrl.setLogPath(True, LOG_DIR)
    except Exception as e:
        print(f"[日志] setLogPath 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        ctrl.setDebugEnabled(True)
    except Exception as e:
        print(f"[日志] setDebugEnabled 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[日志] 目录 {LOG_DIR}", flush=True)

    # 环境检查
    if ctrl.isEnable is not True:
        print("[跳过] 电脑蓝牙未开启，请开启后重跑", flush=True)
        ctrl.terminate()
        return

    input(">>> 请确认 gForceUltra 已【开机】且在范围内，按回车开始扫描连接 ...")

    # 扫描
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        ctrl.terminate()
        return
    name = getattr(target, 'Name', '?')
    print(f"[扫描] 目标: {name}", flush=True)

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] requireSensor 返回 None", flush=True)
        ctrl.terminate()
        return

    # 首次连接读初始滤波状态
    if not _connect_ready(sensor):
        print("[FAIL] 首次连接/init 失败", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    filter_states = _read_filter_states(sensor)
    initial_50 = filter_states.get("FILTER_50HZ")
    initial_60 = filter_states.get("FILTER_60HZ")
    print(f"[初始状态] FILTER_50HZ={initial_50}  FILTER_60HZ={initial_60}", flush=True)
    try:
        sensor.disconnect()
    except Exception:
        pass

    outputs = []

    # ---- 50Hz 段 ----
    print("\n" + "=" * 60, flush=True)
    print("请将信号发生器接入 gForceUltra 的【通道 3】，输出 50Hz 100μV 正弦波", flush=True)
    print("=" * 60, flush=True)
    input(">>> 接好后按回车，开始录制 50Hz 滤波 ON / OFF 两段 ...")

    for out_name, settings in [("50on", [("FILTER_50HZ", "ON")]),
                               ("50off", [("FILTER_50HZ", "OFF")])]:
        dest = _run_one_segment(sensor, LOG_DIR, out_name, settings)
        if dest:
            outputs.append(dest)

    # ---- 60Hz 段 ----
    print("\n" + "=" * 60, flush=True)
    print("请将信号发生器切换到 60Hz 100μV 正弦波（通道 3）", flush=True)
    print("=" * 60, flush=True)
    input(">>> 切换好后按回车，开始录制 60Hz 滤波 ON / OFF 两段 ...")

    for out_name, settings in [("60on", [("FILTER_60HZ", "ON")]),
                               ("60off", [("FILTER_60HZ", "OFF")])]:
        dest = _run_one_segment(sensor, LOG_DIR, out_name, settings)
        if dest:
            outputs.append(dest)

    # ---- 交叉验证段 ----
    print("\n" + "=" * 60, flush=True)
    print("交叉验证 A：FILTER_50HZ=ON + FILTER_60HZ=OFF，注入 60Hz（期望 60Hz 不被过滤）", flush=True)
    print("=" * 60, flush=True)
    input(">>> 保持信号发生器 60Hz 100μV（通道 3），按回车录制 ...")
    dest = _run_one_segment(sensor, LOG_DIR, "cross_60hz",
                            [("FILTER_50HZ", "ON"), ("FILTER_60HZ", "OFF")])
    if dest:
        outputs.append(dest)

    print("\n" + "=" * 60, flush=True)
    print("交叉验证 B：FILTER_60HZ=ON + FILTER_50HZ=OFF，注入 50Hz（期望 50Hz 不被过滤）", flush=True)
    print("=" * 60, flush=True)
    input(">>> 将信号发生器切换到 50Hz 100μV（通道 3），按回车录制 ...")
    dest = _run_one_segment(sensor, LOG_DIR, "cross_50hz",
                            [("FILTER_60HZ", "ON"), ("FILTER_50HZ", "OFF")])
    if dest:
        outputs.append(dest)

    # 恢复初始滤波状态
    print("\n[收尾] 恢复初始滤波状态 ...", flush=True)
    if _connect_ready(sensor):
        r50 = initial_50 if initial_50 in ("ON", "OFF") else "OFF"
        r60 = initial_60 if initial_60 in ("ON", "OFF") else "OFF"
        sensor.setParam("FILTER_50HZ", r50)
        sensor.setParam("FILTER_60HZ", r60)
        print(f"[收尾] 恢复 FILTER_50HZ={r50} FILTER_60HZ={r60}", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass

    # ---- 离线分析滤波效果（Goertzel 振幅 + 时域峰值）----
    print("\n" + "=" * 60, flush=True)
    print("滤波效果分析（各通道 Goertzel 幅值 + 时域峰值）", flush=True)
    print("=" * 60, flush=True)

    bin_labels = ["50on", "50off", "60on", "60off", "cross_60hz", "cross_50hz"]
    all_amps = {}
    all_peaks = {}
    for label in bin_labels:
        path = os.path.join(BASE_DIR, label + ".bin")
        if not os.path.isfile(path):
            print(f"[分析] 缺少 {label}.bin，跳过", flush=True)
            continue
        ch_amps, ch_peaks, fs = _analyze_bin(path, [50.0, 60.0])
        all_amps[label] = ch_amps
        all_peaks[label] = ch_peaks
        print(f"[分析] {label}: 采样率={fs}Hz", flush=True)
        for ci in sorted(ch_amps.keys()):
            a = ch_amps[ci]
            pk = ch_peaks[ci]
            print(f"    通道{ci + 1}: 50Hz幅值={a[50.0]:.4f} 60Hz幅值={a[60.0]:.4f} 时域峰值={pk:.3f}", flush=True)

    print("\n[判断] 滤波抑制效果（通道 3）", flush=True)
    ch3 = 2  # 通道 3（0-indexed）
    if "50on" in all_amps and "50off" in all_amps and ch3 in all_amps["50on"] and ch3 in all_amps["50off"]:
        print("  " + _judge(all_amps["50on"][ch3][50.0], all_amps["50off"][ch3][50.0], "50Hz 滤波", "50Hz"), flush=True)
    if "60on" in all_amps and "60off" in all_amps and ch3 in all_amps["60on"] and ch3 in all_amps["60off"]:
        print("  " + _judge(all_amps["60on"][ch3][60.0], all_amps["60off"][ch3][60.0], "60Hz 滤波", "60Hz"), flush=True)

    print("\n[交叉验证] 陷波器频率选择性（bin 内目标/非目标频率比值）", flush=True)
    # 交叉A：注入 60Hz，50Hz 滤波 ON + 60Hz 滤波 OFF
    #   -> 60Hz 应被保留（大）、50Hz 应被 50Hz 陷波杀（小）
    if "cross_60hz" in all_amps and ch3 in all_amps["cross_60hz"]:
        a60 = all_amps["cross_60hz"][ch3][60.0]
        a50 = all_amps["cross_60hz"][ch3][50.0]
        ratio = a60 / a50 if a50 > 1e-9 else float('inf')
        verdict = "选择性正常（60Hz 保留、50Hz 被滤）" if ratio >= 3.0 else "选择性异常"
        print(f"  交叉A(注入60Hz, 50on+60off): 60Hz={a60:.4f} 50Hz={a50:.4f} 比值(60/50)={ratio:.2f} -> {verdict}", flush=True)
    # 交叉B：注入 50Hz，60Hz 滤波 ON + 50Hz 滤波 OFF
    #   -> 50Hz 应被保留（大）、60Hz 应被 60Hz 陷波杀（小）
    if "cross_50hz" in all_amps and ch3 in all_amps["cross_50hz"]:
        a50 = all_amps["cross_50hz"][ch3][50.0]
        a60 = all_amps["cross_50hz"][ch3][60.0]
        ratio = a50 / a60 if a60 > 1e-9 else float('inf')
        verdict = "选择性正常（50Hz 保留、60Hz 被滤）" if ratio >= 3.0 else "选择性异常"
        print(f"  交叉B(注入50Hz, 60on+50off): 50Hz={a50:.4f} 60Hz={a60:.4f} 比值(50/60)={ratio:.2f} -> {verdict}", flush=True)

    ctrl.terminate()

    # 汇总
    print("\n" + "=" * 60, flush=True)
    print("录制结果汇总", flush=True)
    print("=" * 60, flush=True)
    for p in outputs:
        print(f"  {os.path.basename(p)}: {p}", flush=True)
    print(f"\n共 {len(outputs)}/4 个 bin 已落盘到 {BASE_DIR}", flush=True)
    if len(outputs) < 4:
        print("注意：存在未成功录制的段，请检查上方日志", flush=True)


if __name__ == "__main__":
    main()
