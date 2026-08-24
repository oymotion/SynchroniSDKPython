# -*- coding: utf-8 -*-
"""E2E 冒烟：一次走通「连接 → 起流 → 查看 bin」主链路，尽量非交互。

覆盖的主要接口（均为 auto，无需人工动作，前提是设备已上电且在范围内）：
  1) 环境：SensorController.isEnable / getVersion / getBLEBackendName
  2) 扫描：scan_and_match（带重试）
  3) 连接：requireSensor / connect → Ready / init / hasInited
  4) 设备信息：getDeviceInfo（型号、固件、通道数等）
  5) 参数：setParam / getParam（FILTER 聚合查询）
  6) 数据流：startDataNotification / onDataCallback 计数 / isDataTransfering / stopDataNotification
  7) 电量：getBatteryLevel
  8) bin：DEBUG_BLE_DATA_PATH=True 导出 → getParam 取路径 → 文件存在
  9) bin 解析：getBinFileInfo（元数据）→ parseBinToCsv（转 CSV）
 10) bin 回放：replayBinFile(realtime=False) 快速回放并确认有数据回调

说明：
  不设 input() 人工暂停；若设备未上电/未匹配到，直接判 FAIL 并给出原因。
"""

import os
import sys
import time
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(BASE_DIR)
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, _identity_of, scan_and_match

COLLECT_SECONDS = 60  # 起流采集时长（秒），录制足够长便于后续 debug bin

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

class DataCounter:
    def __init__(self):
        self.batches = 0
        self.samples = 0
        self.lock = threading.Lock()

    def __call__(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        with self.lock:
            self.batches += len(items)
            for it in items:
                cs = getattr(it, 'channelSamples', None)
                if not cs:
                    continue
                try:
                    self.samples += sum(len(ch) for ch in cs)
                except TypeError:
                    self.samples += len(cs)

    def snapshot(self):
        with self.lock:
            return self.batches, self.samples

def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("E2E 冒烟：连接 → 起流 → 查看 bin", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    results = []

    # 受控日志目录（固定路径，便于事后定位 bin/log）
    log_dir = os.path.join(BASE_DIR, "e2e_logs")
    os.makedirs(log_dir, exist_ok=True)
    print(f"\n[日志目录] 使用固定目录 {log_dir}", flush=True)
    try:
        ctrl.setLogPath(True, log_dir)
        log_ok = True
        log_txt = f"setLogPath(True, {log_dir}) 无异常"
    except Exception as e:
        log_ok = False
        log_txt = f"setLogPath 抛异常 {type(e).__name__}: {e}"
    print(f"[日志目录] {log_txt}", flush=True)
    record(results, "SensorController.setLogPath 设置日志目录", log_ok,
           "setLogPath(True, dir) 无异常", log_txt)

    try:
        ctrl.setDebugEnabled(True)
        debug_ok = True
    except Exception as e:
        debug_ok = False
        print(f"[日志] SensorController.setDebugEnabled(True) 抛异常 {type(e).__name__}: {e}", flush=True)
    record(results, "SensorController.setDebugEnabled(True) 开启 debug 日志", debug_ok,
           "setDebugEnabled(True) 无异常", "无异常" if debug_ok else "抛异常")

    bins_before = _list_bins(log_dir)

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        record(results, "SensorController.isEnable 为 True", False, "isEnable == True", f"isEnable == {is_enable}")
        ctrl.terminate()
        return
    record(results, "SensorController.isEnable 为 True", True, "isEnable == True", f"isEnable == {is_enable}")

    # 扫描匹配
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含目标设备", f"匹配到 {name} {addr}")

    # requireSensor
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "SensorController.requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    # connect
    print("\n[连接] SensorProfile.connect() ...", flush=True)
    try:
        ok = sensor.connect()
        connect_txt = f"返回 {ok}"
    except Exception as e:
        ok = None
        connect_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[连接] SensorProfile.connect() -> {connect_txt}  state={sensor.deviceState}", flush=True)
    record(results, "SensorProfile.connect 返回 True", ok is True,
           "connect() 返回 True", f"connect() -> {connect_txt}")

    # 到达 Ready
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    ready = (sensor.deviceState == DeviceStateEx.Ready)
    record(results, "connect 后到达 Ready", ready, "deviceState==Ready", f"state={sensor.deviceState}")
    if not ready:
        print("[FAIL] 未到达 Ready，无法继续", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # init
    print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] SensorProfile.init() -> {init_txt}  hasInited={sensor.hasInited}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")
    record(results, "SensorProfile.hasInited 为 True", sensor.hasInited is True,
           "hasInited == True", f"hasInited == {sensor.hasInited}")

    # 设备信息
    try:
        info = sensor.getDeviceInfo()
    except Exception as e:
        info = None
        print(f"[设备信息] getDeviceInfo 抛异常 {type(e).__name__}: {e}", flush=True)
    info_ok = info is not None
    if info_ok:
        model = getattr(info, 'ModelName', '?')
        fw = getattr(info, 'FirmwareVersion', '?')
        emg_ch = getattr(info, 'EmgChannelCount', '?')
        emg_rate = getattr(info, 'EmgSampleRate', '?')
        acc_ch = getattr(info, 'AccChannelCount', '?')
        print(f"[设备信息] Model={model} FW={fw} EmgChannel={emg_ch} EmgRate={emg_rate} AccChannel={acc_ch}", flush=True)
    record(results, "SensorProfile.getDeviceInfo 返回 DeviceInfo", info_ok,
           "init 后 getDeviceInfo() 返回非 None", f"返回 {type(info).__name__}" if info_ok else "返回 None")

    # 参数（FILTER 聚合查询）
    try:
        flt = sensor.getParam("FILTER")
    except Exception as e:
        flt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[参数] SensorProfile.getParam('FILTER') -> {flt!r}", flush=True)
    flt_ok = isinstance(flt, str) and (not flt.startswith("Error"))
    record(results, "SensorProfile.getParam('FILTER') 返回有效值", flt_ok,
           "getParam('FILTER') 不以 'Error' 开头", f"getParam -> {flt!r}")

    # 开启 bin 导出
    print("\n[bin] SensorProfile.setParam('DEBUG_BLE_DATA_PATH', 'True') ...", flush=True)
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
    except Exception as e:
        bret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') -> {bret!r}", flush=True)
    record(results, "setParam('DEBUG_BLE_DATA_PATH', 'True') 返回 OK", bret == "OK",
           "setParam 返回 'OK'", f"setParam -> {bret!r}")

    # 起流
    counter = DataCounter()
    sensor.onDataCallback = counter
    print("\n[起流] SensorProfile.startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        start_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        start_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] SensorProfile.startDataNotification() -> {start_txt}  isDataTransfering={sensor.isDataTransfering}", flush=True)
    record(results, "SensorProfile.startDataNotification 返回 True", sret is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start_txt}")
    record(results, "SensorProfile.isDataTransfering 为 True", sensor.isDataTransfering is True,
           "isDataTransfering == True", f"isDataTransfering == {sensor.isDataTransfering}")

    print(f"\n[采集] 等待 {COLLECT_SECONDS}s ...", flush=True)
    time.sleep(COLLECT_SECONDS)
    batches, samples = counter.snapshot()
    print(f"[采集] 收到 {batches} 批 / {samples} 样本", flush=True)
    record(results, "onDataCallback 收到数据", samples > 0,
           "起流后样本数 > 0", f"批数={batches} 样本数={samples}")

    # 电量
    try:
        battery = sensor.getBatteryLevel()
    except Exception as e:
        battery = None
        print(f"[电量] getBatteryLevel 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[电量] SensorProfile.getBatteryLevel() -> {battery}", flush=True)
    record(results, "SensorProfile.getBatteryLevel 返回有效值", isinstance(battery, int) and battery != -1,
           "getBatteryLevel() 返回 0~100 且不为 -1", f"getBatteryLevel -> {battery}")

    # 停流
    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)

    ble_path = _get_ble_path(sensor)
    print(f"[bin] stop 后 getParam('DEBUG_BLE_DATA_PATH') = {ble_path!r}", flush=True)

    # 断开
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    if not isinstance(ble_path, str) or not ble_path.strip():
        ble_path = _get_ble_path(sensor)
        print(f"[bin] disconnect 后 getParam('DEBUG_BLE_DATA_PATH') = {ble_path!r}", flush=True)

    time.sleep(0.5)

    bins_after = _list_bins(log_dir)
    new_bins = sorted(set(bins_after.keys()) - set(bins_before.keys()))
    bin_path = ble_path if (isinstance(ble_path, str) and ble_path.strip()) else None
    if bin_path is None and new_bins:
        bin_path = bins_after[new_bins[0]]
    have_bin = bool(bin_path) and os.path.isfile(bin_path)
    print(f"[bin] 新增 bin 文件: {new_bins if new_bins else '无'}", flush=True)
    print(f"[bin] bin 路径: {bin_path!r}，文件存在={have_bin}", flush=True)
    record(results, "起流后生成 .bin 文件", have_bin,
           "DEBUG_BLE_DATA_PATH=True 且文件存在", f"{bin_path!r}（存在={have_bin}）")

    # 查看 bin：元数据 + CSV
    if have_bin:
        try:
            info = ctrl.getBinFileInfo(bin_path)
        except Exception as e:
            info = None
            print(f"[bin] getBinFileInfo 抛异常 {type(e).__name__}: {e}", flush=True)
        info_ok = isinstance(info, dict) and info
        if info_ok:
            print(f"[bin] getBinFileInfo -> device_name={info.get('device_name')}, "
                  f"replay_duration={info.get('replay_duration')}", flush=True)
        record(results, "SensorController.getBinFileInfo 返回元数据", info_ok,
               "返回非空 dict", f"返回 {type(info).__name__}" if info else "返回 None/空")

        try:
            csv_path = ctrl.parseBinToCsv(bin_path)
        except Exception as e:
            csv_path = None
            print(f"[bin] parseBinToCsv 抛异常 {type(e).__name__}: {e}", flush=True)
        csv_ok = bool(csv_path) and os.path.isfile(csv_path)
        print(f"[bin] parseBinToCsv -> {csv_path!r}（存在={csv_ok}）", flush=True)
        record(results, "SensorController.parseBinToCsv 生成 CSV", csv_ok,
               "返回 CSV 路径且文件存在", f"{csv_path!r}（存在={csv_ok}）")

        # 回放：快速回放并确认有数据回调
        replay_counter = DataCounter()
        sensor.onDataCallback = replay_counter
        print(f"[回放] SensorController.replayBinFile(realtime=False) ...", flush=True)
        try:
            ctrl.replayBinFile(bin_path, sensor, realtime=False, timeout=60)
            rerr = None
        except Exception as e:
            rerr = f"{type(e).__name__}: {e}"
            print(f"[回放] 抛异常 {rerr}", flush=True)
        rb, rs = replay_counter.snapshot()
        print(f"[回放] 收到 {rb} 批 / {rs} 样本", flush=True)
        record(results, "replayBinFile 回放产生数据", rerr is None and rs > 0,
               "回放后样本数 > 0", f"批数={rb} 样本数={rs}" + (f" err={rerr}" if rerr else ""))
    else:
        record(results, "SensorController.getBinFileInfo 返回元数据", None, "返回非空 dict", "无有效 bin")
        record(results, "SensorController.parseBinToCsv 生成 CSV", None, "返回 CSV 路径且文件存在", "无有效 bin")
        record(results, "replayBinFile 回放产生数据", None, "回放后样本数 > 0", "无有效 bin")

    # 清理
    try:
        sensor.setParam("DEBUG_BLE_DATA_PATH", "False")
    except Exception:
        pass
    try:
        ctrl.setDebugEnabled(False)
    except Exception:
        pass

    ctrl.terminate()

    # 汇总
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for rname, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {rname}（实际: {actual}）", flush=True)
        elif status == "SKIP":
            print(f"  [SKIP] {rname}（{actual}）", flush=True)
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status == "FAIL":
            all_pass = False

    print(f"\n[提示] 本次日志/bin 落盘目录: {log_dir}", flush=True)
    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)

if __name__ == "__main__":
    main()
