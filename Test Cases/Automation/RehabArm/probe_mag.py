# -*- coding: utf-8 -*-
"""probe_mag.py：探测 RehabArm(1D0E) 磁力计读数与设备摆放（朝向/姿态）的对应关系。

目标：把「磁力计三轴读数」和「设备怎么摆」对应起来，方便写用例断言。
设备外形（用户描述）：类似手臂的两片、可弯曲，开关在外侧。

磁力计 NTF_MAG 输出 3 个通道（10 Hz），本脚本不预设通道物理方向，统一叫
ch0/ch1/ch2，由你通过下面的探测实验确定它们各对应哪个物理方向（东/北/垂直）。

脚本做两件事：
    1) 实时观察：起流后连续打印 ch0/ch1/ch2、总模长 |B|、水平模长、heading，
       你自由转动/翻转设备，看读数怎么变。
    2) 姿态标定：你摆好一个姿态后按回车，脚本采集 2s 求均值并记录，最后汇总
       对比各姿态的读数，据此反推「读数 ↔ 位置」的映射。

如何解读（核心物理规律）：
    - 设备水平放置、绕竖直轴转一圈：水平两轴的读数近似正弦/余弦变化，heading
      从 0→360（或反向）连续变化；竖直轴读数基本不变（≈ 地磁场垂直分量）。
    - 朝「磁北」时：水平两轴里指向北的那个分量达到正的较大值（地磁场水平分量
      约 20~35 uT，视纬度/环境而定）。
    - 上下翻转设备（绕水平轴翻 180°）：竖直轴读数变号，水平两轴也翻转。
    - 读数数量级若在 25~65 uT 附近 => 是 uT；若像几十/几百的整数 => 可能是 raw，
      需再看是否要乘 scale（本脚本用 getData 换算值）。

用法：
    python probe_mag.py          # 用 config.TARGET_IDENTITY（当前 1D0E）
    python probe_mag.py 1D0E     # 指定目标 identity（须在 config.DEVICES 中定义）
"""

import math
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

OBSERVE_SECONDS = 20        # 阶段一：实时观察时长（秒）
SNAPSHOT_SECONDS = 2.0      # 阶段二：每个姿态的采集时长（秒）


def heading_deg(a, b):
    """heading = atan2(b, a)，转度并归一化到 [0, 360)。

    这里以 a 轴为 0° 参考、b 轴为其正交轴；两轴到底谁是东/北，由探测确定。
    """
    deg = math.degrees(math.atan2(b, a))
    if deg < 0:
        deg += 360.0
    return deg


class MagMonitor:
    """只统计 NTF_MAG（3 通道），维护最新值 + 快照累计（均值/标准差）。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = [0.0, 0.0, 0.0]
        self.rate = 0
        self.got_data = False
        self._reset()

    def _reset(self):
        self.snap_sum = [0.0, 0.0, 0.0]
        self.snap_sum2 = [0.0, 0.0, 0.0]
        self.snap_n = 0

    def reset_snapshot(self):
        with self.lock:
            self._reset()

    def __call__(self, profile, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        for it in items:
            try:
                if it.getDataType() != DataType.NTF_MAG:
                    continue
            except Exception:
                continue
            try:
                n_ch = it.getChannelCount()
                n_smp = it.getSampleCount()
                if n_ch <= 0 or n_smp <= 0:
                    continue
                with self.lock:
                    self.got_data = True
                    self.rate = it.getSampleRate()
                    for si in range(n_smp):
                        for ch in range(min(n_ch, 3)):
                            v = float(it.getData(ch, si))
                            self.latest[ch] = v
                            self.snap_sum[ch] += v
                            self.snap_sum2[ch] += v * v
                            self.snap_n += 1
            except Exception:
                continue

    def snapshot(self):
        """返回 (means[3], stds[3])。无样本时返回 None。"""
        with self.lock:
            if self.snap_n == 0:
                return None
            means = [s / self.snap_n for s in self.snap_sum]
            stds = []
            for ch in range(3):
                var = self.snap_sum2[ch] / self.snap_n - means[ch] ** 2
                stds.append(math.sqrt(max(var, 0.0)))
            return means, stds

    def latest_line(self):
        with self.lock:
            bx, by, bz = self.latest
            rate = self.rate
        mag = math.sqrt(bx * bx + by * by + bz * bz)
        hmag = math.sqrt(bx * bx + by * by)
        return (f"  ch0={bx:8.2f} ch1={by:8.2f} ch2={bz:8.2f} "
                f"|B|={mag:7.2f} |Bh|={hmag:7.2f} heading={heading_deg(bx, by):7.1f}° "
                f"(rate={rate}Hz)")


def main():
    ctrl = SensorControllerInstance
    target_identity = resolve_target_identity()

    print("=" * 64, flush=True)
    print("probe_mag：磁力计读数 ↔ 设备摆放 关系探测", flush=True)
    print("=" * 64, flush=True)
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

    monitor = MagMonitor()
    sensor.onDataCallback = monitor

    print("[起流] startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        print(f"  startDataNotification() = {sret}  isDataTransfering={sensor.isDataTransfering}",
              flush=True)
    except Exception as e:
        print(f"  startDataNotification() 抛异常 {type(e).__name__}: {e}", flush=True)
        sret = None

    # 等待 NTF_MAG 数据出现（最多 3s）
    for _ in range(30):
        if monitor.got_data:
            break
        time.sleep(0.1)
    if not monitor.got_data:
        print("[失败] 起流后 3s 内未收到 NTF_MAG 数据，设备可能未推送磁力计流。", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        ctrl.terminate()
        return
    print(f"[起流] 已收到 NTF_MAG 数据，采样率={monitor.rate}Hz", flush=True)

    # ---- 阶段一：实时观察 ----
    print("\n" + "-" * 64, flush=True)
    print(f"[阶段一] 实时观察 {OBSERVE_SECONDS}s。请自由转动/翻转设备，观察读数变化。", flush=True)
    print("  建议：先水平放置绕竖直轴转一圈，再上下翻转，看哪个通道基本不变（那是垂直轴）。",
          flush=True)
    print("-" * 64, flush=True)
    t0 = time.time()
    while time.time() - t0 < OBSERVE_SECONDS:
        time.sleep(0.5)
        print(monitor.latest_line(), flush=True)

    # ---- 阶段二：姿态标定 ----
    print("\n" + "-" * 64, flush=True)
    print("[阶段二] 姿态标定。输入姿态描述后回车，脚本采集 "
          f"{SNAPSHOT_SECONDS:.0f}s 求均值。输入 q 结束。", flush=True)
    print("  示例描述：开关朝上平放 / 开关朝下 / 竖直立起 / 转90°朝东 / 转180° ...", flush=True)
    print("-" * 64, flush=True)

    records = []
    while True:
        try:
            label = input("\n姿态描述（q 结束）：").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if label.lower() in ("q", "quit", "exit", ""):
            break

        monitor.reset_snapshot()
        print(f"  采集 {SNAPSHOT_SECONDS:.0f}s ...", flush=True)
        t0 = time.time()
        while time.time() - t0 < SNAPSHOT_SECONDS:
            time.sleep(0.5)
            print("    " + monitor.latest_line(), flush=True)

        snap = monitor.snapshot()
        if snap is None:
            print("  (未采集到数据)", flush=True)
            continue
        means, stds = snap
        mag = math.sqrt(means[0] ** 2 + means[1] ** 2 + means[2] ** 2)
        hdg = heading_deg(means[0], means[1])
        print(f"  [{label}] 均值 ch0={means[0]:.2f} ch1={means[1]:.2f} ch2={means[2]:.2f} "
              f"|B|={mag:.2f} heading={hdg:.1f}°", flush=True)
        records.append((label, means, stds, mag, hdg))

    # ---- 收尾 ----
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
    print("\n" + "=" * 64, flush=True)
    print("姿态标定汇总", flush=True)
    print("=" * 64, flush=True)
    if not records:
        print("  (未记录任何姿态)", flush=True)
    else:
        print(f"  {'姿态':<24}{'ch0':>10}{'ch1':>10}{'ch2':>10}{'|B|':>10}{'heading':>10}", flush=True)
        for label, means, stds, mag, hdg in records:
            print(f"  {label:<24}{means[0]:>10.2f}{means[1]:>10.2f}{means[2]:>10.2f}"
                  f"{mag:>10.2f}{hdg:>9.1f}°", flush=True)
    print("=" * 64, flush=True)
    print("解读提示：", flush=True)
    print("  - 水平放置转一圈时读数近似正弦/余弦变化的两轴是水平轴（东/北），", flush=True)
    print("    基本不变的那一轴是垂直轴。", flush=True)
    print("  - 朝磁北时水平轴中读数为正且较大的那个分量，即「北向分量」。", flush=True)
    print("  - 上下翻转设备，垂直轴读数变号。", flush=True)


if __name__ == "__main__":
    main()
