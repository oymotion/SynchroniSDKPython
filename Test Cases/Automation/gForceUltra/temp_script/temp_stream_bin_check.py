# -*- coding: utf-8 -*-
"""临时脚本：gForceUltra scan -> connect -> 起流 60s -> 校验 bin 数据完整性。

用途：临时验证「起流 1 分钟后，bin 录制是否完整」，不并入正式用例，仅作排查用。

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) setLogPath 指定日志/bin 目录，开启 DEBUG_BLE_DATA_PATH=True（bin 录制）
  3) startDataNotification 起流，采集 60s
  4) stopDataNotification + disconnect，导出 bin
  5) getBinFileInfo(bin_path) 读 replay_duration，与 60s 比较判完整度

用法：
  python temp_stream_bin_check.py            # 用 config.TARGET_IDENTITY 指定设备
  python temp_stream_bin_check.py 80E5       # 指定 gForceUltra identity

前置条件：
  - 主机蓝牙已开启（或 USB dongle 已就绪）
  - 待测 gForceUltra 上电、在范围内
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import scan_and_match

STREAM_SECONDS = 60            # 起流时长（秒）
BIN_COMPLETENESS_RATIO = 0.99  # bin 录制时长 / 实际起流时长 的完整度下界
PROGRESS_INTERVAL = 10         # 采集期间每多少秒打印一次进度
READY_TIMEOUT = 15             # 连接后等待 Ready 超时（秒）


def _list_bins(log_dir):
    if not log_dir or not os.path.isdir(log_dir):
        return {}
    out = {}
    for fn in os.listdir(log_dir):
        if fn.lower().endswith(".bin"):
            out[fn] = os.path.join(log_dir, fn)
    return out


class Counter:
    """统计收到的批次与样本数（起流是否真有数据的粗校验）。"""

    def __init__(self):
        self.batches = 0
        self.samples = 0

    def __call__(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        self.batches += len(items)
        for it in items:
            try:
                self.samples += it.getChannelCount() * it.getSampleCount()
            except Exception:
                pass


def main():
    target_identity = sys.argv[1].strip().upper() if len(sys.argv) > 1 else None
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("临时脚本：gForceUltra 起流 60s + bin 完整性校验", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    # 日志/bin 目录（固定到脚本旁，便于事后定位）
    log_dir = os.path.join(BASE_DIR, "tmp_logs")
    os.makedirs(log_dir, exist_ok=True)
    print(f"\n[日志目录] {log_dir}", flush=True)
    try:
        ctrl.setLogPath(True, log_dir)
    except Exception as e:
        print(f"[日志] setLogPath 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        ctrl.setDebugEnabled(True)
    except Exception as e:
        print(f"[日志] setDebugEnabled 抛异常 {type(e).__name__}: {e}", flush=True)

    bins_before = _list_bins(log_dir)

    # 扫描
    print(f"\n[扫描] 目标 identity: {target_identity or common.TARGET_IDENTITIES} ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, target_identity=target_identity)
    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        ctrl.terminate()
        return
    print(f"[扫描] 目标设备: {getattr(target, 'Name', '?')} {getattr(target, 'Address', '?')}", flush=True)

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

    # 开启 bin 录制
    try:
        sensor.setParam("DEBUG_LOG_PATH", "True")
    except Exception as e:
        print(f"[bin] setParam('DEBUG_LOG_PATH') 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
        print(f"[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') -> {bret!r}", flush=True)
    except Exception as e:
        print(f"[bin] setParam('DEBUG_BLE_DATA_PATH') 抛异常 {type(e).__name__}: {e}", flush=True)

    counter = Counter()
    sensor.onDataCallback = counter

    # 起流
    try:
        sret = sensor.startDataNotification()
    except Exception as e:
        sret = None
        print(f"[起流] startDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[起流] startDataNotification() -> {sret}  isDataTransfering={sensor.isDataTransfering}", flush=True)
    if sret is not True:
        print("[FAIL] 起流失败", flush=True)
        ctrl.terminate()
        return

    # 采集 60s
    print(f"\n[采集] 起流 {STREAM_SECONDS}s ...", flush=True)
    start_time = time.time()
    while True:
        elapsed = time.time() - start_time
        if elapsed >= STREAM_SECONDS:
            break
        print(f"[采集] {elapsed:.0f}s / {STREAM_SECONDS}s  数据={counter.batches}批/{counter.samples}样本", flush=True)
        time.sleep(PROGRESS_INTERVAL)
    stream_seconds = time.time() - start_time
    print(f"[采集] 结束，实际起流 {stream_seconds:.1f}s  数据={counter.batches}批/{counter.samples}样本", flush=True)

    # 停流 + 断开，导出 bin
    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)

    ble_path = None
    try:
        ble_path = sensor.getParam("DEBUG_BLE_DATA_PATH")
    except Exception:
        pass
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] disconnect 抛异常 {type(e).__name__}: {e}", flush=True)
    if not (isinstance(ble_path, str) and ble_path.strip()):
        try:
            ble_path = sensor.getParam("DEBUG_BLE_DATA_PATH")
        except Exception:
            pass

    time.sleep(0.5)
    bins_after = _list_bins(log_dir)
    new_bins = sorted(set(bins_after.keys()) - set(bins_before.keys()))
    bin_path = ble_path if (isinstance(ble_path, str) and ble_path.strip()) else None
    if not (bin_path and os.path.isfile(bin_path)):
        bin_path = bins_after[new_bins[0]] if new_bins else None
    have_bin = bool(bin_path) and os.path.isfile(bin_path)

    print("\n" + "=" * 60, flush=True)
    print("bin 完整性校验结果", flush=True)
    print("=" * 60, flush=True)
    print(f"[bin] 新增 bin: {new_bins if new_bins else '无'}", flush=True)
    print(f"[bin] 路径: {bin_path!r}  存在={have_bin}", flush=True)

    bin_valid = False
    bin_complete = None
    if have_bin:
        try:
            info = ctrl.getBinFileInfo(bin_path)
        except Exception as e:
            info = None
            print(f"[bin] getBinFileInfo 抛异常 {type(e).__name__}: {e}", flush=True)
        bin_valid = isinstance(info, dict) and bool(info)
        print(f"[bin] 有效性 = {'有效' if bin_valid else '无效'}（getBinFileInfo 返回 {type(info).__name__}）", flush=True)
        if bin_valid:
            device_name = info.get("device_name")
            replay_duration = info.get("replay_duration")
            print(f"[bin] device_name={device_name}  replay_duration={replay_duration}", flush=True)
            if replay_duration is not None:
                try:
                    ratio = float(replay_duration) / float(stream_seconds)
                    bin_complete = ratio >= BIN_COMPLETENESS_RATIO
                    print(f"[bin] 完整性 = {'完整' if bin_complete else '不完整'}"
                          f"（replay_duration={replay_duration}s / 起流 {stream_seconds:.1f}s = {ratio:.1%}）", flush=True)
                except (TypeError, ValueError):
                    print(f"[bin] 完整性 = 无法判定（replay_duration={replay_duration!r}）", flush=True)
            else:
                print("[bin] 完整性 = 无法判定（无 replay_duration 字段）", flush=True)

    # 清理
    try:
        sensor.setParam("DEBUG_BLE_DATA_PATH", "False")
    except Exception:
        pass
    try:
        sensor.setParam("DEBUG_LOG_PATH", "False")
    except Exception:
        pass
    try:
        ctrl.setDebugEnabled(False)
    except Exception:
        pass
    ctrl.terminate()

    if not have_bin:
        verdict = "FAIL（未生成 bin）"
    elif not bin_valid:
        verdict = "FAIL（bin 无效，无法解析）"
    elif bin_complete is False:
        verdict = "FAIL（bin 不完整）"
    elif bin_complete is None:
        verdict = "无法判定完整性（缺 replay_duration）"
    else:
        verdict = "PASS（bin 完整）"
    print("\n结论: " + verdict, flush=True)


if __name__ == "__main__":
    main()
