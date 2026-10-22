# -*- coding: utf-8 -*-
"""DATA-FUNC-010：NTF_IMU 拆分为四路独立子流（无聚合批）。

对应用例：03_数据流.md -> DATA-FUNC-010
可自动化：auto（能力判定后执行/跳过；设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) getDeviceInfo() 读 ImuChannelCount
  3) ImuChannelCount==0：记录"不支持，跳过"（SKIP，传统设备无聚合流）
  4) ImuChannelCount>0：setParam("NTF_IMU","ON") 起流，校验数据按四路独立子流投递：
     - 无 NTF_IMU(0x13) 聚合数据类型（NTF_IMU 批次==0，属 SDK 实际行为）
     - 收到四路子流 NTF_ACC/NTF_GYRO/NTF_EULER_DATA/NTF_QUATERNION，各样本数>0
     - 各子流通道数 == 固定布局 acc=3/gyro=3/euler=3/quat=4
     - 四路通道数合计 == ImuChannelCount

说明：
  gForceMax 最新固件（与 gForceUltra 同源）的 13 轴 IMU 开关是 "NTF_IMU"，但数据
  并不作为 NTF_IMU(0x13) 聚合批投递，而是拆成 acc/gyro/euler/quat 四路独立子
  DataType（NTF_ACC/NTF_GYRO/NTF_EULER_DATA/NTF_QUATERNION）分别投递。
  DeviceInfo 以 ImuChannelCount=13 描述能力；四路固定布局 acc=3/gyro=3/euler=3/quat=4，
  合计 13。本用例把聚合能力"拆开"校验：验证四路独立子流各自的通道布局与合计一致。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：gForceMax 上电、在范围内
"""

import os
import re
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match


def _dt_name(dt):
    try:
        if isinstance(dt, DataType):
            return dt.name
        return DataType(dt).name
    except Exception:
        return str(dt)


# NTF_IMU 实际拆成四路独立子 DataType 投递（无 NTF_IMU 聚合数据类型）
# (显示名, DataType, DeviceInfo 通道字段, 固定布局通道数)
IMU_SUBSTREAMS = [
    ("ACC",   DataType.NTF_ACC,         "AccChannelCount",  3),
    ("GYRO",  DataType.NTF_GYRO,        "GyroChannelCount", 3),
    ("EULER", DataType.NTF_EULER_DATA,  "EulerChannelCount", 3),
    ("QUAT",  DataType.NTF_QUATERNION,  "QuatChannelCount", 4),
]


class ImuCollector:
    """按 DataType 收集四路独立子流，记录各自批次/样本/通道数。"""

    def __init__(self):
        self.by_type = {}  # {DataType: {'batches': n, 'samples': m, 'channels': c}}

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            dt = d.getDataType()
            entry = self.by_type.setdefault(dt, {'batches': 0, 'samples': 0, 'channels': 0})
            entry['batches'] += 1
            try:
                ch = d.getChannelCount()
                smp = d.getSampleCount()
            except Exception:
                ch = 0
                smp = 0
            entry['channels'] = ch
            entry['samples'] += ch * smp


def check_imu_substreams(collector, info, imu_channels, results):
    """校验 NTF_IMU 拆成的四路独立子流布局。"""

    def add(name, ok, expect, actual):
        record(results, name, ok, expect, actual)

    by_type = collector.by_type

    # 1. 无 NTF_IMU 聚合数据类型（SDK 实际行为：拆成四路独立流，而非 NTF_IMU 聚合批）
    imu_batches = by_type.get(DataType.NTF_IMU, {}).get('batches', 0)
    add("无 NTF_IMU 聚合数据类型（拆成四路独立子流）", imu_batches == 0,
        "NTF_IMU(0x13) 不投递，数据按四路独立子流投递",
        f"NTF_IMU 批次={imu_batches}")

    # 2. 各四路子流收到数据且通道数 == 固定布局；四路通道合计 == ImuChannelCount
    sum_channels = 0
    for disp, dt, field, expected_ch in IMU_SUBSTREAMS:
        entry = by_type.get(dt, {'batches': 0, 'samples': 0, 'channels': 0})
        ch = entry['channels']
        samples = entry['samples']
        field_val = int(getattr(info, field, 0) or 0)
        add(f"{disp} 子流收到数据（{_dt_name(dt)}）", samples > 0,
            f"收到 {_dt_name(dt)} 数据（样本数>0）",
            f"样本数={samples} 批次={entry['batches']}")
        add(f"{disp} 子流通道数==固定布局 {expected_ch}", ch == expected_ch,
            f"getChannelCount()=={expected_ch}",
            f"getChannelCount()={ch}" + (f"（DeviceInfo.{field}={field_val}）" if field_val else ""))
        sum_channels += ch

    add("四路独立子流通道数合计==ImuChannelCount", sum_channels == imu_channels,
        f"四路通道数合计=={imu_channels}", f"四路通道数合计={sum_channels}")


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-010 NTF_IMU 拆分为四路独立子流", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：gForceMax 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 gForceMax 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print(f"[FAIL] 未匹配到目标设备（TARGET_IDENTITY={config.TARGET_IDENTITY}）", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含 gForceMax", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含 gForceMax", f"匹配到 {name} {addr}")

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

    # getDeviceInfo -> ImuChannelCount
    info = sensor.getDeviceInfo()
    record(results, "getDeviceInfo() 返回 DeviceInfo", info is not None,
           "getDeviceInfo() 返回 DeviceInfo（非 None）",
           f"返回 {type(info).__name__ if info is not None else None}")

    if info is None:
        print("[FAIL] getDeviceInfo() 返回 None，无法进行能力判定", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    try:
        imu_channels = int(getattr(info, 'ImuChannelCount', 0) or 0)
    except Exception as e:
        imu_channels = 0
        print(f"[info] 读取 ImuChannelCount 抛异常 {type(e).__name__}: {e}，按 0 处理", flush=True)
    print(f"\n[能力] DeviceInfo.ImuChannelCount = {imu_channels}", flush=True)

    # 能力判定：0 = 传统设备，无 IMU 能力，跳过
    if imu_channels <= 0:
        record(results, "NTF_IMU 能力判定", None,
               "ImuChannelCount>0 时校验四路独立子流布局",
               "ImuChannelCount==0，设备不支持 IMU（传统设备）")
        print("[SKIP] ImuChannelCount==0，设备无 IMU 能力，跳过布局校验", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    # 起 NTF_IMU 流
    print("\n[起流] 先关闭 NTF_IMU 清理状态 ...", flush=True)
    try:
        sensor.setParam("NTF_IMU", "OFF")
    except Exception as e:
        print(f"  [清理] NTF_IMU OFF 抛异常 {type(e).__name__}: {e}", flush=True)

    print("[起流] SensorProfile.setParam('NTF_IMU', 'ON') ...", flush=True)
    try:
        p_ret = sensor.setParam("NTF_IMU", "ON")
        p_txt = f"返回 {p_ret!r}"
    except Exception as e:
        p_ret = None
        p_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] setParam('NTF_IMU', 'ON') -> {p_txt}", flush=True)

    collector = ImuCollector()
    sensor.onDataCallback = collector.on_data

    print("[起流] SensorProfile.startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        start_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        start_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] SensorProfile.startDataNotification() -> {start_txt}", flush=True)
    record(results, "SensorProfile.startDataNotification 返回 True", sret is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start_txt}")

    # 采集窗口
    print(f"\n[采集] 等待 {config.COLLECT_SECONDS}s 观察 NTF_IMU 拆分的四路子流回调 ...", flush=True)
    time.sleep(config.COLLECT_SECONDS)
    got_types = {_dt_name(k): v['batches'] for k, v in collector.by_type.items()}
    print(f"[采集] 收到类型批次={got_types}", flush=True)

    # 停流
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.setParam("NTF_IMU", "OFF")
    except Exception:
        pass

    # 布局校验（四路独立子流）
    got_any = any(e['samples'] > 0 for e in collector.by_type.values())
    if not got_any:
        print("[FAIL] 采集窗口内未收到任何 NTF_IMU 相关子流数据，无法校验布局", flush=True)
        record(results, "收到四路独立子流数据", False,
               "采集窗口内收到四路独立子流数据",
               f"收到类型={ {_dt_name(k): v['batches'] for k, v in collector.by_type.items()} }")
    else:
        check_imu_substreams(collector, info, imu_channels, results)

    # 清理
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

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


if __name__ == "__main__":
    main()
