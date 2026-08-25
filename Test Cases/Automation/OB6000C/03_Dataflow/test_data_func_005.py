# -*- coding: utf-8 -*-
"""DATA-FUNC-005：各 DataType 流能收到对应 SensorData。

对应用例：03_数据流.md -> DATA-FUNC-005
可自动化：semi-auto（能力判定自动；需人工做手势/晃动/用力激发数据）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) getDeviceInfo() 读取各模态 ChannelCount
  3) 对每个目标 DataType：
     - ChannelCount>0：setParam("NTF_xxx","ON") 起流，人工激发，统计 getDataType()
       断言收到该 DataType 数据且样本数>0
     - ChannelCount==0：记录"不支持，跳过"（SKIP）

目标模态（OB6000C EEG 设备，通道数由 getDeviceInfo() 运行时读取）与映射：
  DataType               setParam key        ChannelCount 字段    采集时长
  NTF_EEG                "NTF_EEG"           EegChannelCount       5s（主模态，静息即有数据）
  NTF_ECG                "NTF_ECG"           EcgChannelCount       5s（EEG/ECG 同写）
  NTF_EMG                "NTF_EMG"           EmgChannelCount       5s（能力门控）
  NTF_GEST               "NTF_GEST"          EmgChannelCount       5s（DeviceInfo 无独立 GEST 标称值）
  NTF_IMPEDANCE          "NTF_IMPEDANCE"     ImpeChannelCount      25s（1Hz，需较长采集）
  NTF_ACC                "NTF_GFORCE_ACC"    AccChannelCount       5s
  NTF_GYRO               "NTF_GFORCE_GYRO"   GyroChannelCount      5s
  NTF_EULER_DATA         "NTF_GFORCE_EULER"  EulerChannelCount     5s
  NTF_QUATERNION         "NTF_GFORCE_QUAT"   QuatChannelCount      5s

说明：
  - 每个模态是否测试由 getDeviceInfo() 的 ChannelCount 运行时判定（>0 才起流）。
  - GEST/EMG 在传统设备互斥，逐个测完 setParam OFF，避免串扰。
  - IMPEDANCE 采样率 1Hz，packageSampleCount=20 约需 20s 凑满一批，故采集 25s。
  - NTF_IMU 聚合流（ImuChannelCount）不在本条覆盖（见 DATA-FUNC-010）。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内，且电极已正确佩戴
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record, scan_and_match


def _dt_name(dt):
    try:
        if isinstance(dt, DataType):
            return dt.name
        return DataType(dt).name
    except Exception:
        return str(dt)


# (显示名, DataType, setParam key, ChannelCount 字段名, 动作提示, 采集秒数)
MODALITIES = [
    ("EEG", DataType.NTF_EEG, "NTF_EEG", "EegChannelCount",
     "请保持电极接触良好，EEG 静息即有信号，无需额外动作", 5),
    ("ECG", DataType.NTF_ECG, "NTF_ECG", "EcgChannelCount",
     "请保持电极接触良好，让 ECG 产生信号", 5),
    ("EMG", DataType.NTF_EMG, "NTF_EMG", "EmgChannelCount",
     "请保持电极接触良好，让 EMG 产生信号", 5),
    ("GEST", DataType.NTF_GEST, "NTF_GEST", "EmgChannelCount",
     "若设备支持，请触发相应手势识别", 5),
    ("IMPEDANCE", DataType.NTF_IMPEDANCE, "NTF_IMPEDANCE", "ImpeChannelCount",
     "请保持电极接触良好，让阻抗测量产生信号", 25),
    ("ACC", DataType.NTF_ACC, "NTF_GFORCE_ACC", "AccChannelCount",
     "请轻微移动设备，让加速度计产生变化", 5),
    ("GYRO", DataType.NTF_GYRO, "NTF_GFORCE_GYRO", "GyroChannelCount",
     "请轻微旋转设备，让陀螺仪产生变化", 5),
    ("EULER", DataType.NTF_EULER_DATA, "NTF_GFORCE_EULER", "EulerChannelCount",
     "请轻微旋转设备，让欧拉角产生变化", 5),
    ("QUAT", DataType.NTF_QUATERNION, "NTF_GFORCE_QUAT", "QuatChannelCount",
     "请轻微旋转设备，让四元数产生变化", 5),
]

# DeviceInfo 候选字段（来自 README L313-323 与 example），用于运行时 dump 验证真实字段名
DEVICE_INFO_FIELDS = [
    "DeviceName", "ModelName", "HardwareVersion", "FirmwareVersion", "MTUSize",
    "PpgChannelCount", "PpgSampleRate",
    "Spo2ChannelCount", "Spo2SampleRate",
    "ImpeChannelCount", "ImpeSampleRate",
    "EmgChannelCount", "EmgSampleRate",
    "EegChannelCount", "EegSampleRate",
    "EcgChannelCount", "EcgSampleRate",
    "AccChannelCount", "AccSampleRate",
    "GyroChannelCount", "GyroSampleRate",
    "BrthChannelCount", "BrthSampleRate",
    "MagAngleChannelCount", "MagAngleSampleRate",
    "EulerChannelCount", "EulerSampleRate",
    "QuatChannelCount", "QuatSampleRate",
    "EmgMaxSampleRate", "EegMaxSampleRate", "EcgMaxSampleRate",
    "ImuChannelCount", "ImuSampleRate",
    "ConnectionIntervalMs", "PeripheralLatency", "SupervisionTimeoutMs",
]


def dump_device_info(info):
    """运行时读取 DeviceInfo 全部候选字段，返回可读字符串（用于验证字段名是否正确）。"""
    parts = []
    for f in DEVICE_INFO_FIELDS:
        try:
            v = getattr(info, f, None)
        except Exception as e:
            v = f"<{type(e).__name__}>"
        parts.append(f"{f}={v}")
    return ", ".join(parts)


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


def _list_logs(log_dir):
    if not log_dir or not os.path.isdir(log_dir):
        return {}
    out = {}
    try:
        for fn in os.listdir(log_dir):
            if fn.lower().endswith(".txt"):
                out[fn] = os.path.join(log_dir, fn)
    except OSError:
        pass
    return out


def _get_ble_path(sensor):
    try:
        return sensor.getParam("DEBUG_BLE_DATA_PATH")
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def _get_profile_log_path(sensor):
    try:
        return sensor.getParam("DEBUG_LOG_PATH")
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


class DataCollector:
    """按 DataType 统计收到的批次与样本数。"""

    def __init__(self):
        self.by_type = {}  # {DataType: {'batches': n, 'samples': m}}

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            dt = d.getDataType()
            entry = self.by_type.setdefault(dt, {'batches': 0, 'samples': 0})
            entry['batches'] += 1
            cs = getattr(d, 'channelSamples', None)
            n = 0
            if cs:
                try:
                    n = sum(len(ch) for ch in cs)
                except TypeError:
                    n = len(cs)
            entry['samples'] += n

    def clear(self):
        self.by_type.clear()


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-005 各 DataType 流能收到对应 SensorData", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内，且电极已正确佩戴", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】、电极已正确佩戴且在范围内，"
          "测试中需要你配合做相应动作，完成后按回车继续 ...")

    results = []

    # 受控日志目录（固定路径，便于把 bin/log 贴到 bug 里）
    log_dir = os.path.join(BASE_DIR, "data_func_005_logs")
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
        debug_txt = "setDebugEnabled(True) 无异常"
    except Exception as e:
        debug_ok = False
        debug_txt = f"setDebugEnabled(True) 抛异常 {type(e).__name__}: {e}"
    print(f"[日志] SensorController.setDebugEnabled(True) -> {debug_txt}", flush=True)
    record(results, "SensorController.setDebugEnabled(True) 开启 debug 日志", debug_ok,
           "setDebugEnabled(True) 无异常", debug_txt)

    bins_before = _list_bins(log_dir)
    logs_before = _list_logs(log_dir)

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    if target is None:
        print("[FAIL] 未匹配到 config 中启用的设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含启用的目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含启用的目标设备", f"匹配到 {name} {addr}")

    # requireSensor
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
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
    print(f"[init] SensorProfile.init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    # getDeviceInfo
    info = sensor.getDeviceInfo()
    record(results, "getDeviceInfo() 返回 DeviceInfo", info is not None,
           "getDeviceInfo() 返回 DeviceInfo（非 None）",
           f"返回 {type(info).__name__ if info is not None else None}")

    if info is not None:
        print(f"\n[DeviceInfo 全字段 dump]", flush=True)
        print(f"  {dump_device_info(info)}", flush=True)

    if info is None:
        print("[FAIL] getDeviceInfo() 返回 None，无法进行能力判定", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # 开启 bin 导出 + profile log 导出（用于把完整数据/log 贴到 bug 里）
    print("\n[导出] 开启 bin 与 profile log 导出 ...", flush=True)
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
    except Exception as e:
        bret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[导出] setParam('DEBUG_BLE_DATA_PATH', 'True') -> {bret!r}", flush=True)
    record(results, "开启 DEBUG_BLE_DATA_PATH 导出", bret == "OK",
           "setParam('DEBUG_BLE_DATA_PATH', 'True') 返回 'OK'", f"返回 {bret!r}")

    try:
        lret = sensor.setParam("DEBUG_LOG_PATH", "True")
    except Exception as e:
        lret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[导出] setParam('DEBUG_LOG_PATH', 'True') -> {lret!r}", flush=True)
    record(results, "开启 DEBUG_LOG_PATH 导出", lret == "OK",
           "setParam('DEBUG_LOG_PATH', 'True') 返回 'OK'", f"返回 {lret!r}")

    # 逐模态能力判定 + 数据验证
    collector = DataCollector()
    sensor.onDataCallback = collector.on_data

    # 先全部 OFF，清理初始状态（尤其 GEST/EMG 互斥）
    print("\n[准备] 关闭所有目标模态，清理初始状态 ...", flush=True)
    for disp, dt, param, field, hint, _sec in MODALITIES:
        try:
            sensor.setParam(param, "OFF")
        except Exception as e:
            print(f"  [清理] {param} OFF 抛异常 {type(e).__name__}: {e}", flush=True)

    for disp, dt, param, field, hint, collect_sec in MODALITIES:
        ch_count = getattr(info, field, 0)
        print(f"\n{'=' * 40}", flush=True)
        print(f"[模态] {disp}  DataType={_dt_name(dt)}  ChannelCount({field})={ch_count}", flush=True)
        print("=" * 40, flush=True)

        if ch_count <= 0:
            record(results, f"{disp} 流能收到数据（ChannelCount>0）", None,
                   f"ChannelCount>0 时收到 {_dt_name(dt)} 数据",
                   f"ChannelCount==0，设备不支持该模态")
            print(f"[SKIP] {disp} ChannelCount==0，设备不支持，跳过", flush=True)
            continue

        # setParam ON
        try:
            p_ret = sensor.setParam(param, "ON")
        except Exception as e:
            p_ret = f"抛异常 {type(e).__name__}: {e}"
        print(f"[setParam] SensorProfile.setParam({param!r}, 'ON') -> {p_ret!r}", flush=True)
        record(results, f"{disp} setParam 返回 OK", p_ret == "OK",
               f"setParam({param!r}, 'ON') 返回 'OK'", f"返回 {p_ret!r}")

        # 起流
        try:
            s_ret = sensor.startDataNotification()
        except Exception as e:
            s_ret = f"抛异常 {type(e).__name__}: {e}"
        print(f"[起流] startDataNotification() -> {s_ret!r}", flush=True)

        # 采集（人工激发）
        print(f"[动作] {hint}", flush=True)
        input("        按回车开始采集（采集期间请持续做动作）...")
        collector.clear()
        time.sleep(collect_sec)

        # 停流 + 关模态
        try:
            sensor.stopDataNotification()
        except Exception:
            pass
        try:
            sensor.setParam(param, "OFF")
        except Exception:
            pass

        # 统计该 DataType 数据
        entry = collector.by_type.get(dt, {'batches': 0, 'samples': 0})
        got = entry['samples'] > 0
        got_types = {_dt_name(k): v['batches'] for k, v in collector.by_type.items()}
        print(f"[采集] {disp} 收到批次={entry['batches']} 样本数={entry['samples']} 全部类型={got_types}", flush=True)
        record(results, f"{disp} 流能收到数据且类型匹配", got,
               f"收到 {_dt_name(dt)} 数据（样本数>0）",
               f"该类型样本数={entry['samples']} 批次={entry['batches']}")

    # 清理
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 导出 bin / log（供贴到 bug）----
    ble_path = _get_ble_path(sensor)
    profile_path = _get_profile_log_path(sensor)
    print(f"\n[导出] disconnect 后 getParam('DEBUG_BLE_DATA_PATH') = {ble_path!r}", flush=True)
    print(f"[导出] disconnect 后 getParam('DEBUG_LOG_PATH') = {profile_path!r}", flush=True)

    # 给文件系统收尾留一点时间
    time.sleep(0.5)

    bins_after = _list_bins(log_dir)
    logs_after = _list_logs(log_dir)
    new_bins = sorted(set(bins_after.keys()) - set(bins_before.keys()))
    new_logs = sorted(set(logs_after.keys()) - set(logs_before.keys()))

    bin_path = ble_path if (isinstance(ble_path, str) and ble_path.strip()) else None
    if bin_path is None and new_bins:
        bin_path = bins_after[new_bins[0]]
    log_path = profile_path if (isinstance(profile_path, str) and profile_path.strip()) else None

    print(f"[导出] 新增 .bin 文件: {new_bins if new_bins else '无'}", flush=True)
    print(f"[导出] 新增 .txt log 文件: {new_logs if new_logs else '无'}", flush=True)
    print(f"[导出] bin 路径: {bin_path!r}（存在={bool(bin_path) and os.path.isfile(bin_path)}）", flush=True)
    if log_path and os.path.isfile(log_path):
        print(f"[导出] profile log 路径: {log_path!r}", flush=True)
    for fn in new_logs:
        print(f"[导出] log 文件: {logs_after[fn]}", flush=True)

    # 关闭导出开关（已导出的 bin/log 文件不会被删除）
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

    # ---- 汇总 ----
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

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()
    print(f"\n[提示] 本次 bin/log 落盘目录: {log_dir}", flush=True)
    print("[提示] 请把该目录下本次新增的 .bin 与 .txt 文件贴到 bug 里", flush=True)


if __name__ == "__main__":
    main()
