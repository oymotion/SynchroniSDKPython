# -*- coding: utf-8 -*-
"""DATA-FUNC-010：NTF_IMU 拆分为四路独立子流（无聚合批）。

对应用例：03_数据流.md -> DATA-FUNC-010
可自动化：auto（能力判定后执行/跳过；设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 从 spec["streams"] 判定是否支持 NTF_IMU 聚合流
  3) 不支持 / 无子流定义：记录"不支持，跳过"（SKIP）
  4) 支持：setParam("NTF_IMU","ON") 起流，校验数据按独立子流投递：
     - 无 NTF_IMU(0x13) 聚合数据类型（NTF_IMU 批次==0，属 SDK 实际行为）
     - 收到各子流（取自 spec sample_rates.sub_streams），各样本数>0
     - 各子流通道数 == spec["imu_sub_channels"] 布局
     - 各子流通道数合计 == spec streams.NTF_IMU.channels

说明：
  NTF_IMU 开关对应的数据并不作为 NTF_IMU(0x13) 聚合批投递，而是拆成多路独立子
  DataType（如 NTF_ACC/NTF_GYRO/NTF_EULER_DATA/NTF_QUATERNION）分别投递。
  子流清单与通道布局取自 spec（sample_rates.sub_streams + imu_sub_channels），
  不写死具体型号；本用例把聚合能力"拆开"校验：验证各独立子流的通道布局与合计一致。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：已上电、在范围内
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


# DataType 名 -> DeviceInfo 通道字段名（SDK 固定命名对应，非设备能力假设）
_SUB_FIELD_BY_TYPE = {
    "NTF_ACC": "AccChannelCount",
    "NTF_GYRO": "GyroChannelCount",
    "NTF_EULER_DATA": "EulerChannelCount",
    "NTF_QUATERNION": "QuatChannelCount",
}


def _load_spec(target):
    identity = common.identity_of_device(target)
    if identity is None:
        print("[FAIL] 无法从广播名提取 identity，无法载入设备 spec", flush=True)
        return None
    try:
        return common.spec_for_identity(identity)
    except Exception as e:
        print(f"[FAIL] 载入设备 spec 失败（identity={identity}）: {type(e).__name__}: {e}", flush=True)
        return None


def _imu_substreams(spec):
    """从 spec 解析 NTF_IMU 聚合流拆分出的子流定义。

    返回 [(显示名, DataType, 期望通道数), ...]；spec 未声明 IMU 子流时返回 []。
    期望通道数取自 spec["imu_sub_channels"]（键为 DeviceInfo 字段名）。
    """
    imu_sub = spec.get("imu_sub_channels") or {}
    subs = []
    for e in (spec.get("sample_rates") or {}).values():
        if e.get("stream") == "NTF_IMU" and e.get("sub_streams"):
            for s in e.get("sub_streams"):
                dt = getattr(DataType, s, None)
                if dt is None:
                    continue
                field = _SUB_FIELD_BY_TYPE.get(s)
                expected_ch = int(imu_sub.get(field, 0) or 0) if field else 0
                disp = s[len("NTF_"):] if s.startswith("NTF_") else s
                subs.append((disp, dt, expected_ch))
            break
    return subs


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


def check_imu_substreams(collector, substreams, imu_channels, results):
    """校验 NTF_IMU 拆成的独立子流布局（子流清单来自 spec）。"""

    def add(name, ok, expect, actual):
        record(results, name, ok, expect, actual)

    by_type = collector.by_type

    # 1. 无 NTF_IMU 聚合数据类型（SDK 实际行为：拆成独立子流，而非 NTF_IMU 聚合批）
    imu_batches = by_type.get(DataType.NTF_IMU, {}).get('batches', 0)
    add("无 NTF_IMU 聚合数据类型（拆成独立子流）", imu_batches == 0,
        "NTF_IMU(0x13) 不投递，数据按独立子流投递",
        f"NTF_IMU 批次={imu_batches}")

    # 2. 各子流收到数据且通道数 == spec 布局；子流通道合计 == ImuChannelCount
    sum_channels = 0
    for disp, dt, expected_ch in substreams:
        entry = by_type.get(dt, {'batches': 0, 'samples': 0, 'channels': 0})
        ch = entry['channels']
        samples = entry['samples']
        add(f"{disp} 子流收到数据（{_dt_name(dt)}）", samples > 0,
            f"收到 {_dt_name(dt)} 数据（样本数>0）",
            f"样本数={samples} 批次={entry['batches']}")
        add(f"{disp} 子流通道数==spec 布局 {expected_ch}", ch == expected_ch,
            f"getChannelCount()=={expected_ch}",
            f"getChannelCount()={ch}")
        sum_channels += ch

    add("独立子流通道数合计==ImuChannelCount", sum_channels == imu_channels,
        f"子流通道数合计=={imu_channels}", f"子流通道数合计={sum_channels}")


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-010 NTF_IMU 聚合批通道布局", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：已上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备已【开机】且在范围内，"
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
        print("[FAIL] 未匹配到目标设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含目标设备", f"匹配到 {name} {addr}")

    spec = _load_spec(target)
    if spec is None:
        record(results, "载入设备 spec", False, "能从广播名提取 identity 并载入对应 spec", "spec 载入失败")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "载入设备 spec", True, "能从广播名提取 identity 并载入对应 spec", f"model={spec.get('model')}")

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

    # 能力判定：spec 是否声明 NTF_IMU 聚合流及其子流布局
    imu_cap = (spec.get("streams") or {}).get("NTF_IMU") or {}
    imu_channels = int(imu_cap.get("channels") or 0)
    imu_supported = bool(imu_cap.get("supported") is True and imu_channels > 0)
    substreams = _imu_substreams(spec)
    print(f"\n[能力] spec.streams.NTF_IMU：supported={imu_cap.get('supported')} "
          f"channels={imu_channels} 子流数={len(substreams)}", flush=True)

    # 运行时 DeviceInfo 交叉验证（spec 与设备实际能力是否一致）
    info = sensor.getDeviceInfo()
    record(results, "getDeviceInfo() 返回 DeviceInfo", info is not None,
           "getDeviceInfo() 返回 DeviceInfo（非 None）",
           f"返回 {type(info).__name__ if info is not None else None}")
    if info is not None:
        try:
            dev_imu = int(getattr(info, 'ImuChannelCount', 0) or 0)
        except Exception as e:
            dev_imu = None
            print(f"[info] 读取 ImuChannelCount 抛异常 {type(e).__name__}: {e}", flush=True)
        record(results, "DeviceInfo.ImuChannelCount 与 spec 一致", dev_imu == imu_channels,
               f"DeviceInfo.ImuChannelCount==spec.channels({imu_channels})",
               f"DeviceInfo.ImuChannelCount={dev_imu}")

    # 能力判定：spec 未声明 NTF_IMU 或无子流布局，跳过
    if not imu_supported or not substreams:
        record(results, "NTF_IMU 拆分子流能力判定", None,
               "spec 声明 NTF_IMU（supported 且 channels>0）且 sample_rates 含 sub_streams 时校验布局",
               "spec 未声明 NTF_IMU 聚合流或其子流布局")
        print("[SKIP] spec 未声明 NTF_IMU 聚合流或其子流布局，跳过布局校验", flush=True)
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
        check_imu_substreams(collector, substreams, imu_channels, results)

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
