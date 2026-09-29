# -*- coding: utf-8 -*-
"""gForceUltra 交互式录制 bin 并转 CSV。

流程（按用户指定）：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) startDataNotification 起流（先只起流，不录 bin）
  3) 用户按回车确认，开始录制 bin（setParam DEBUG_BLE_DATA_PATH=True）
  4) 录制 RECORD_SECONDS 秒（默认 30 分钟）
  5) stopDataNotification 停流（结束录制，bin 落盘）
  6) 定位 bin -> getBinFileInfo 校验 -> parseBinToCsv 转 CSV

bin / csv / 日志都写入脚本所在目录（本 record_bin_csv/ 目录）。

用法：
  python record_bin_csv.py            # 用 config.TARGET_IDENTITY（当前 80E1）
  python record_bin_csv.py 80E5       # 覆盖指定 identity

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

RECORD_SECONDS = 1800          # 录制时长（秒）= 30 分钟
PROGRESS_INTERVAL = 50         # 录制期间每多少秒打印一次进度
READY_TIMEOUT = 15             # 连接后等待 Ready 超时（秒）
SETTLE_SECONDS = 2.0           # setParam 后等待设备异步生效的静置时长


def _list_bins(d):
    if not d or not os.path.isdir(d):
        return {}
    out = {}
    for fn in os.listdir(d):
        if fn.lower().endswith(".bin"):
            out[fn] = os.path.join(d, fn)
    return out


def _cleanup(sensor, ctrl):
    """尽力清理：关闭 bin/日志录制、debug 日志，并 terminate。"""
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
    try:
        ctrl.terminate()
    except Exception:
        pass


class Counter:
    """粗统计收到的批次与样本数（确认起流/录制期间确有数据）。"""

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
    print("gForceUltra 交互式录制 bin -> CSV", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)

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

    # 输出目录：脚本所在目录（本 record_bin_csv/ 目录），bin + csv + 日志都放这里
    out_dir = BASE_DIR
    os.makedirs(out_dir, exist_ok=True)
    print(f"[目录] bin/csv 输出目录: {out_dir}", flush=True)
    try:
        ctrl.setLogPath(True, out_dir)
    except Exception as e:
        print(f"[日志] setLogPath 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        ctrl.setDebugEnabled(True)
    except Exception as e:
        print(f"[日志] setDebugEnabled 抛异常 {type(e).__name__}: {e}", flush=True)

    bins_before = _list_bins(out_dir)

    # ---- 步骤 1：扫描 + 连接 ----
    print(f"\n[1/6 连接] 扫描目标 identity: {target_identity or common.TARGET_IDENTITIES} ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, target_identity=target_identity)
    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        ctrl.terminate()
        return
    print(f"[1/6 连接] 目标设备: {getattr(target, 'Name', '?')} {getattr(target, 'Address', '?')}", flush=True)

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] requireSensor 返回 None", flush=True)
        ctrl.terminate()
        return

    try:
        ok = sensor.connect()
    except Exception as e:
        ok = None
        print(f"[1/6 连接] connect 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[1/6 连接] connect() -> {ok}  state={sensor.deviceState}", flush=True)
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

    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
    except Exception as e:
        iret = None
        print(f"[1/6 连接] init 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[1/6 连接] init() -> {iret}  hasInited={sensor.hasInited}", flush=True)
    if iret is not True:
        print("[FAIL] init 失败", flush=True)
        ctrl.terminate()
        return

    # ---- 步骤 2：配置流 + 起流（先不录 bin）----
    # gForceUltra 断流根因：GEST 手势流与 EMG 采样冲突，录制前必须关 GEST；
    # EMG 采样率设 1000Hz；IMU/阻抗是 EMG 的伴随流（co_stream=NTF_EMG），
    # 需随 EMG 同起才有数据，一并开启（顺序与 test_measure_sample_rate.py 一致）。
    try:
        sensor.setParam("DEBUG_LOG_PATH", "True")
    except Exception as e:
        print(f"[2/6 起流] setParam('DEBUG_LOG_PATH') 抛异常 {type(e).__name__}: {e}", flush=True)

    for _key, _val in [
        ("NTF_EMG", "ON"),
        ("NTF_IMU", "ON"),
        ("NTF_IMPEDANCE", "ON"),
        ("EMG_SAMPLE_RATE", "1000"),
        ("NTF_GEST", "OFF"),
    ]:
        try:
            _r = sensor.setParam(_key, _val)
            print(f"[2/6 配置] setParam({_key}, {_val}) -> {_r!r}", flush=True)
        except Exception as e:
            print(f"[2/6 配置] setParam({_key}, {_val}) 抛异常 {type(e).__name__}: {e}", flush=True)
    time.sleep(SETTLE_SECONDS)   # 静置等待采样率/流配置异步生效

    counter = Counter()
    sensor.onDataCallback = counter

    try:
        sret = sensor.startDataNotification()
    except Exception as e:
        sret = None
        print(f"[2/6 起流] startDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[2/6 起流] startDataNotification() -> {sret}  isDataTransfering={sensor.isDataTransfering}", flush=True)
    if sret is not True:
        print("[FAIL] 起流失败", flush=True)
        _cleanup(sensor, ctrl)
        return

    # ---- 步骤 3：用户确认开始录制 ----
    print("\n[3/6 确认] 起流已就绪。", flush=True)
    print(f"          准备好后按回车开始录制 {RECORD_SECONDS}s（= {RECORD_SECONDS / 60:.0f} 分钟）bin，Ctrl+C 取消。", flush=True)
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        print("[取消] 用户未确认，退出。", flush=True)
        _cleanup(sensor, ctrl)
        return

    # ---- 步骤 4：开始录制 + 录制半小时 ----
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
        print(f"[4/6 录制] setParam('DEBUG_BLE_DATA_PATH','True') -> {bret!r}", flush=True)
    except Exception as e:
        print(f"[4/6 录制] setParam('DEBUG_BLE_DATA_PATH') 抛异常 {type(e).__name__}: {e}", flush=True)

    print(f"[4/6 录制] 开始录制 {RECORD_SECONDS}s ...", flush=True)
    start_time = time.time()
    while True:
        elapsed = time.time() - start_time
        if elapsed >= RECORD_SECONDS:
            break
        print(f"[4/6 录制] {elapsed:.0f}s / {RECORD_SECONDS}s  数据={counter.batches}批/{counter.samples}样本", flush=True)
        time.sleep(PROGRESS_INTERVAL)
    record_seconds = time.time() - start_time
    print(f"[4/6 录制] 录制结束，实际录制 {record_seconds:.1f}s", flush=True)

    # ---- 步骤 5：停流（结束录制，bin 落盘）----
    try:
        sensor.stopDataNotification()
        print("[5/6 停流] stopDataNotification() -> OK", flush=True)
    except Exception as e:
        print(f"[5/6 停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        sensor.setParam("DEBUG_BLE_DATA_PATH", "False")
    except Exception as e:
        print(f"[5/6 停流] setParam('DEBUG_BLE_DATA_PATH','False') 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 步骤 6：定位 bin + 转 CSV ----
    ble_path = None
    try:
        ble_path = sensor.getParam("DEBUG_BLE_DATA_PATH")
    except Exception:
        pass
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[6/6 转CSV] disconnect 抛异常 {type(e).__name__}: {e}", flush=True)
    if not (isinstance(ble_path, str) and ble_path.strip()):
        try:
            ble_path = sensor.getParam("DEBUG_BLE_DATA_PATH")
        except Exception:
            pass

    time.sleep(0.5)
    bins_after = _list_bins(out_dir)
    new_bins = sorted(set(bins_after.keys()) - set(bins_before.keys()))
    bin_path = ble_path if (isinstance(ble_path, str) and ble_path.strip()) else None
    if not (bin_path and os.path.isfile(bin_path)):
        bin_path = bins_after[new_bins[0]] if new_bins else None
    have_bin = bool(bin_path) and os.path.isfile(bin_path)

    print("\n" + "=" * 60, flush=True)
    print("录制结果", flush=True)
    print("=" * 60, flush=True)
    print(f"[bin] 新增 bin: {new_bins if new_bins else '无'}", flush=True)
    print(f"[bin] 路径: {bin_path!r}  存在={have_bin}", flush=True)

    csv_path = None
    if have_bin:
        try:
            info = ctrl.getBinFileInfo(bin_path)
        except Exception as e:
            info = None
            print(f"[bin] getBinFileInfo 抛异常 {type(e).__name__}: {e}", flush=True)
        if isinstance(info, dict) and info:
            print(f"[bin] device_name={info.get('device_name')}  replay_duration={info.get('replay_duration')}", flush=True)
        try:
            csv_path = ctrl.parseBinToCsv(bin_path)
        except Exception as e:
            print(f"[CSV] parseBinToCsv 抛异常 {type(e).__name__}: {e}", flush=True)
        csv_ok = isinstance(csv_path, str) and os.path.isfile(csv_path)
        print(f"[CSV] parseBinToCsv -> {csv_path!r}  存在={csv_ok}", flush=True)

    # 清理
    _cleanup(sensor, ctrl)

    if have_bin and csv_path:
        print("\n结论: 完成（bin + csv 均已生成于同一目录）", flush=True)
    else:
        print("\n结论: 见上方错误（bin 或 csv 未生成）", flush=True)


if __name__ == "__main__":
    main()
