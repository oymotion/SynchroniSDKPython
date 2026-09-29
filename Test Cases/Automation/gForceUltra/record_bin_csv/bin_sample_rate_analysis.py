# -*- coding: utf-8 -*-
"""gForceUltra bin 采样率 / 漂移分析（独立正弦参考）。

用途：回放一个 bin，用「独立正弦参考」测出 gForceUltra 的真实采样率，
      并判断是「漂移」（采样时钟随时间变化）还是仅「恒定频偏」。

原理：
  - 一段纯正弦在样本域表现为「每周期固定样本数 spc」。spc = f_adc / f_sig。
  - 若信号频率 f_sig 精确已知（信号发生器 REF），则真实采样率 f_adc = REF × spc。
  - 漂移 = spc 随时间变化；恒定频偏 = spc 恒定但 ≠ 标称值。
  - FFT 主峰最鲁棒（抗噪声/丢包），过零法受丢包污染仅作旁证。

数据口径（关键，避免踩坑）：
  - 时间轴用「样本序号」np.arange(n)，不依赖 sampleIndex 字段
    （回放中 sampleIndex 字段只有包索引语义，不可靠）。
  - FFT 频率轴需要假设一个采样率（fs_nom），得到的是「表观频率」；
    但换算回「每周期样本数 spc = fs_nom / fpk」后，spc 与 fs_nom 无关，
    是设备时钟的绝对量。故 fs_nom 取任意值都不影响 spc / f_adc 结果。
  - 分段漂移直接看「每周期样本数 spc」序列是否平移，与 fs_nom 绝对准确性无关。

用法：
  python bin_sample_rate_analysis <bin路径> [通道] [参考频率Hz] [标称采样率Hz]

  例：
    python bin_sample_rate_analysis gForceUltra_data_xxx.bin            # 自动定位通道、识别参考频率
    python bin_sample_rate_analysis gForceUltra_data_xxx.bin 2          # 只给通道，自动识别 ref/采样率
    python bin_sample_rate_analysis gForceUltra_data_xxx.bin 2 20 1000  # 显式 ch=2, 20Hz, 1000Hz

依赖：numpy + 本仓库 Python SDK。
"""

import sys

import numpy as np
from sensor import *

SEG = 20                           # 分段数（漂移趋势）
LOCATE_BAND_LO = 1.0               # 自动定位通道 / 识别 ref 时的宽频带下界（Hz）
REF_BAND_RATIO = (0.7, 1.3)        # ref 已知后，精确定峰用 [ref*0.7, ref*1.3]
FS_FALLBACK = 1000.0               # getSampleRate 也取不到时的标称采样率兜底
DRIFT_SLOPE_THRESHOLD = 0.05       # 判「无趋势」的斜率阈值（Hz/min，约 100 ppm/min）


def _log(*a):
    print(*a, flush=True)


def _peak_parabolic(mag, i_abs, n, fs):
    """对 FFT 幅度谱在 bin i_abs 附近做抛物线插值，返回精确表观频率(Hz)。"""
    lo = max(i_abs - 1, 0)
    hi = min(i_abs + 1, len(mag) - 1)
    if hi <= lo:
        return i_abs * fs / n
    a, b, c = mag[lo], mag[i_abs], mag[hi]
    denom = a - 2.0 * b + c
    delta = 0.0 if denom == 0 else 0.5 * (a - c) / denom
    return (i_abs + delta) * fs / n


def _find_peak(mag, freqs, lo, hi, n, fs):
    """在 [lo, hi] Hz 内找幅度最大峰，返回抛物线插值后的表观频率(Hz)；无峰返回 None。"""
    idx = np.where((freqs >= lo) & (freqs <= hi))[0]
    if len(idx) == 0:
        return None
    i_abs = idx[int(np.argmax(mag[idx]))]
    return _peak_parabolic(mag, i_abs, n, fs)


def _locate_sine_channel(arr, fs, band_lo, band_hi):
    """返回正弦所在通道：给定频带内峰幅最大的通道（自动定位用宽频带）。"""
    n_ch = arr.shape[0]
    freqs = np.fft.rfftfreq(arr.shape[1], d=1.0 / fs)
    band = (freqs >= band_lo) & (freqs <= band_hi)
    best_ch, best_amp = None, -1.0
    for ci in range(n_ch):
        x = arr[ci]["data"].astype(np.float64)
        x = x - x.mean()
        mag = np.abs(np.fft.rfft(x * np.hanning(len(x))))
        amp = float(mag[band].max())
        if amp > best_amp:
            best_amp, best_ch = amp, ci
    return best_ch, best_amp


def main():
    args = sys.argv[1:]
    if not args:
        _log("用法: python bin_sample_rate_analysis <bin> [通道] [参考频率Hz] [标称采样率Hz]")
        return
    bin_path = args[0]
    ch_arg = int(args[1]) if len(args) > 1 else None
    ref = float(args[2]) if len(args) > 2 else None
    fs_nom = float(args[3]) if len(args) > 3 else None

    ctrl = SensorControllerInstance
    try:
        info = ctrl.getBinFileInfo(bin_path)
        if not info:
            _log(f"[FAIL] getBinFileInfo 返回空: {bin_path}")
            return
        sensor = ctrl.requireSensor(BLEDevice(info.get("device_name") or "",
                                              info.get("device_mac") or "", 0))
        if sensor is None:
            _log("[FAIL] requireSensor 返回 None")
            return

        batches = []
        fs_reported = {"v": None}

        def on_data(profile, data_list):
            items = data_list if isinstance(data_list, list) else [data_list]
            for d in items:
                name = d.getDataType()
                name = name.name if isinstance(name, DataType) else DataType(name).name
                if "EMG" in name.upper():
                    if fs_reported["v"] is None:
                        fs_reported["v"] = d.getSampleRate()
                    batches.append(d.as_numpy())

        sensor.onDataCallback = on_data
        _log(f"[回放] {bin_path}")
        _log(f"       replay_duration={info.get('replay_duration')}  "
             f"device_name={info.get('device_name')}")
        ctrl.replayBinFile(bin_path, sensor, realtime=False)
        _log(f"[回放] 结束，EMG 批数={len(batches)}")

        if not batches:
            _log("[FAIL] 无 EMG 数据")
            return

        arr = np.concatenate(batches, axis=1)
        n_ch, n = arr.shape[0], arr.shape[1]
        fs_reported_v = fs_reported["v"]
        if fs_nom is None:
            fs_nom = float(fs_reported_v) if fs_reported_v else FS_FALLBACK
            _log(f"[数据] 标称采样率：自动读取 getSampleRate = {fs_nom}Hz")
        else:
            _log(f"[数据] 标称采样率：命令行指定 = {fs_nom}Hz")
        _log(f"[数据] 通道={n_ch}  每通道样本={n}  标称采样率={fs_nom}Hz")

        # ---- 1) 定位正弦通道 ----
        if ch_arg is None:
            ch, amp = _locate_sine_channel(arr, fs_nom, LOCATE_BAND_LO, fs_nom * 0.45)
            _log(f"[定位] 正弦通道 ch={ch}（{LOCATE_BAND_LO}~{fs_nom * 0.45:.0f}Hz 宽频带内峰幅最大={amp:.0f}）")
        else:
            ch = ch_arg
            _log(f"[定位] 手动指定通道 ch={ch}")

        if ch is None or ch >= n_ch:
            _log("[FAIL] 通道定位失败")
            return

        data = arr[ch]["data"].astype(np.float64)
        lost = arr[ch]["isLost"].astype(bool)
        _log(f"[数据] ch={ch}  mean={data.mean():.4f}uV  std={data.std():.4f}uV  "
             f"min={data.min():.3f}  max={data.max():.3f}  "
             f"lost={int(lost.sum())} ({lost.mean() * 100:.2f}%)")

        x = data - data.mean()

        # ---- 2) FFT 精确峰值（主判）----
        freqs = np.fft.rfftfreq(n, d=1.0 / fs_nom)
        mag = np.abs(np.fft.rfft(x * np.hanning(n)))

        # 参考频率：未显式传入时，用宽频带主导峰吸附到最近整数 Hz 自动识别
        if ref is None:
            f0 = _find_peak(mag, freqs, LOCATE_BAND_LO, fs_nom * 0.45, n, fs_nom)
            ref = round(f0) if f0 is not None else None
            _log(f"[ref] 自动识别参考频率：主导峰 {f0:.3f}Hz -> 吸附到 {ref}Hz")
        else:
            _log(f"[ref] 手动指定参考频率 = {ref}Hz")
        if ref is None or ref <= 0:
            _log("[FAIL] 无法确定参考频率")
            return

        band_lo, band_hi = ref * REF_BAND_RATIO[0], ref * REF_BAND_RATIO[1]
        fpk = _find_peak(mag, freqs, band_lo, band_hi, n, fs_nom)
        if fpk is None:
            _log(f"[FAIL] {band_lo:.1f}~{band_hi:.1f}Hz 频带内未找到主峰")
            return
        spc_fft = fs_nom / fpk                    # 每周期样本数（与 fs_nom 无关的绝对量）
        f_adc_fft = ref * spc_fft                 # 真实采样率 = 参考频率 × 每周期样本数
        _log(f"\n[FFT 主判] 表观峰频={fpk:.5f}Hz（假设采样率={fs_nom}Hz）")
        _log(f"           每周期样本数 spc = {spc_fft:.5f}（标称应={fs_nom / ref:.2f}）")
        _log(f"           真实采样率 f_adc = {ref}Hz × spc = {f_adc_fft:.5f}Hz"
             f"（相对标称 {fs_nom}Hz 偏差 {(f_adc_fft - fs_nom) / fs_nom * 1e6:+.1f} ppm）")

        # ---- 3) 过零法（旁证）----
        Xf = np.fft.rfft(x)
        f = np.fft.rfftfreq(n, d=1.0 / fs_nom)
        mask = (f >= ref * 0.7) & (f <= ref * 1.3)
        x_bp = np.fft.irfft(Xf * mask, n=n)
        cross = []
        for k in range(1, n):
            p, q = x_bp[k - 1], x_bp[k]
            if p < 0 <= q:
                frac = (0.0 - p) / (q - p)
                cross.append((k - 1) + frac)
        cross = np.array(cross)
        spc_zc = np.diff(cross)
        _log(f"\n[过零 旁证] 上升过零={len(cross)}  每周期样本数 spc: "
             f"mean={spc_zc.mean():.5f}  std={spc_zc.std():.5f}")
        _log(f"            f_adc(过零) = {ref}Hz × {spc_zc.mean():.5f} = {ref * spc_zc.mean():.5f}Hz"
             f"（受丢包/噪声污染，仅供参考）")

        # ---- 4) 分段漂移（看 spc 是否随时间平移）----
        center = (cross[:-1] + cross[1:]) / 2.0
        edges = np.linspace(0, n, SEG + 1)
        rows = []
        for k in range(SEG):
            m = (center >= edges[k]) & (center < edges[k + 1])
            if m.sum() == 0:
                continue
            pc = spc_zc[m]
            t_sec = edges[k] / fs_nom
            rows.append((t_sec, pc.mean(), ref * pc.mean(), pc.std(), int(m.sum())))
        _log(f"\n[分段] 每段约 {n / fs_nom / SEG:.0f}s，看 spc / f_adc 是否平移：")
        _log(f"       {'t(s)':>7} {'spc':>9} {'f_adc(Hz)':>11} {'spc_std':>9} {'cycles':>7}")
        for t_sec, pc, fa, pstd, ncyc in rows:
            _log(f"       {t_sec:7.0f} {pc:9.5f} {fa:11.5f} {pstd:9.5f} {ncyc:7d}")

        slope_spc = slope_hz = first = last = spread = None
        if len(rows) >= 2:
            t = np.array([r[0] for r in rows])
            pc = np.array([r[1] for r in rows])
            slope_spc = np.polyfit(t, pc, 1)[0]          # 样本/周期 每秒
            slope_hz = ref * slope_spc * 60.0            # 折合 Hz/min
            first, last = pc[0], pc[-1]
            spread = pc.max() - pc.min()
            _log(f"\n[漂移] spc 斜率={slope_spc:+.6f} 样本/周期/s"
                 f"  => 折合采样率漂移 {slope_hz:+.5f} Hz/min")
            _log(f"       spc 首尾={first:.5f} -> {last:.5f}"
                 f"（Δ{(last - first):+.5f} = {(last - first) / spc_fft * 1e6:+.1f} ppm）")
            _log(f"       spc 峰峰={spread:.5f}（相对均值 {spread / spc_fft * 1e6:.1f} ppm）")

        # ---- 5) DC 基线漂移 ----
        t_axis = np.arange(n, dtype=np.float64)
        _log(f"\n[基线] 分 {SEG} 段中位数(uV)：")
        dc = []
        for k in range(SEG):
            m = (t_axis >= edges[k]) & (t_axis < edges[k + 1])
            if m.sum() == 0:
                continue
            dc.append((edges[k] / fs_nom, np.median(data[m])))
        for t_sec, med in dc:
            _log(f"       t={t_sec:6.0f}s  median={med:8.3f} uV")
        if len(dc) >= 2:
            meds = np.array([v[1] for v in dc])
            _log(f"       DC 首 {meds[0]:.3f} -> 尾 {meds[-1]:.3f} uV  峰峰 {meds.max() - meds.min():.3f} uV")

        # ---- 结论 ----
        _log("\n" + "=" * 60)
        _log("结论")
        _log("=" * 60)
        _log(f"  恒定频偏: 真实采样率 ≈ {f_adc_fft:.3f}Hz"
             f"（标称 {fs_nom}Hz，偏差 {(f_adc_fft - fs_nom) / fs_nom * 1e6:+.0f} ppm）")
        if slope_hz is not None:
            if abs(slope_hz) < DRIFT_SLOPE_THRESHOLD:
                _log(f"  漂移: 无趋势（斜率 {slope_hz:+.5f} Hz/min，"
                     f"spc 峰峰 {spread / spc_fft * 1e6:.0f} ppm）")
            else:
                _log(f"  漂移: 存在趋势（斜率 {slope_hz:+.5f} Hz/min）")
        _log(f"  （注：过零 spc_std={spc_zc.std():.3f}，若偏大说明受丢包/噪声污染，"
             f"以 FFT 主判 {f_adc_fft:.3f}Hz 为准）")
    finally:
        try:
            ctrl.terminate()
        except Exception as e:
            _log(f"[清理] terminate 异常: {e}")


if __name__ == "__main__":
    main()
