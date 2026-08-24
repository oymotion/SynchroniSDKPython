# -*- coding: utf-8 -*-
"""DATA-FUNC-010：NTF_IMU 聚合批通道布局（仅新 EMG 设备）。

对应用例：03_数据流.md -> DATA-FUNC-010
可自动化：auto（能力判定后执行/跳过；设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) getDeviceInfo() 读 ImuChannelCount
  3) ImuChannelCount==0：记录"不支持，跳过"（SKIP，传统设备无聚合流）
  4) ImuChannelCount>0：setParam("NTF_IMU","ON") 起聚合流，取第一个非空批次校验：
     - getDataType()==NTF_IMU
     - getChannelCount()==ImuChannelCount 且 len(channelSamples)==ImuChannelCount
     - 固定布局 acc 0-2 / gyro 3-5 / euler 6-8 / quat 9-12 各分段存在且有数据

说明：
  NTF_IMU 聚合流是"新 EMG 设备"才有的能力，把 acc/gyro/euler/quat 四路合成一个
  13 通道批（部分非 QAT6 设备仅 acc+gyro 6 通道）。布局固定：
    通道 0-2=acc、3-5=gyro、6-8=euler、9-12=quat。
  ImuChannelCount 由 DeviceInfo 上报，0 表示无聚合流（传统设备改用四路独立流）。
  OB6000C 实测 ImuChannelCount=13。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
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


# NTF_IMU 聚合批固定布局：acc 0-2 / gyro 3-5 / euler 6-8 / quat 9-12
IMU_LAYOUT = [
    ("ACC",   0, 3),
    ("GYRO",  3, 6),
    ("EULER", 6, 9),
    ("QUAT",  9, 13),
]


class ImuCollector:
    """收集 NTF_IMU 数据，保存第一个非空批次用于布局校验。"""

    def __init__(self):
        self.first_batch = None
        self.batches = 0
        self.total_samples = 0

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            if _dt_name(d.getDataType()) != "NTF_IMU":
                continue
            self.batches += 1
            cs = getattr(d, 'channelSamples', None)
            n = 0
            if cs:
                try:
                    n = sum(len(ch) for ch in cs)
                except TypeError:
                    n = len(cs)
            self.total_samples += n
            if self.first_batch is None and n > 0:
                self.first_batch = d


def check_imu_layout(data, imu_channels, results):
    """对一批 NTF_IMU 数据校验通道布局。"""

    def add(name, ok, expect, actual):
        record(results, name, ok, expect, actual)

    # 1. 数据类型
    dt = data.getDataType()
    dt_txt = _dt_name(dt)
    add("收到 NTF_IMU 聚合数据", dt_txt == "NTF_IMU",
        "getDataType()==NTF_IMU", f"getDataType()={dt_txt}")

    # 2. 元数据通道数 == ImuChannelCount
    meta_ch = data.getChannelCount()
    add("getChannelCount()==ImuChannelCount", meta_ch == imu_channels,
        f"getChannelCount()=={imu_channels}", f"getChannelCount()={meta_ch}")

    # 3. 结构通道数 == ImuChannelCount
    cs = getattr(data, 'channelSamples', None)
    n_ch = len(cs) if cs else 0
    add("channelSamples 通道数==ImuChannelCount", n_ch == imu_channels,
        f"channelSamples 通道数=={imu_channels}", f"channelSamples 通道数={n_ch}")

    # 4. 各布局分段存在且有数据（按 ImuChannelCount 覆盖到的分段）
    for seg_name, start, end in IMU_LAYOUT:
        if imu_channels >= end:
            seg_has = n_ch >= end
            seg_samples = 0
            if seg_has and cs:
                try:
                    seg_samples = sum(len(cs[i]) for i in range(start, end))
                except Exception:
                    seg_samples = 0
            add(f"{seg_name} 分段存在（通道 {start}-{end - 1}）且有数据",
                seg_has and seg_samples > 0,
                f"通道 {start}-{end - 1} 存在且样本数>0",
                f"存在={seg_has} 分段样本数={seg_samples}")
        else:
            add(f"{seg_name} 分段（通道 {start}-{end - 1}）", None,
                f"ImuChannelCount>={end} 时该分段应存在",
                f"ImuChannelCount={imu_channels}，无该分段")


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-010 NTF_IMU 聚合批通道布局", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
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
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    if target is None:
        print("[FAIL] 未匹配到目标设备（OB6000C/80F3）", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含 OB6000C", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含 OB6000C", f"匹配到 {name} {addr}")

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

    # 能力判定：0 = 传统设备，无聚合流，跳过
    if imu_channels <= 0:
        record(results, "NTF_IMU 聚合流能力判定", None,
               "ImuChannelCount>0 时校验聚合布局",
               "ImuChannelCount==0，设备不支持 NTF_IMU 聚合流（传统设备）")
        print("[SKIP] ImuChannelCount==0，设备无 NTF_IMU 聚合流，跳过布局校验", flush=True)
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
    print(f"\n[采集] 等待 {config.COLLECT_SECONDS}s 观察 NTF_IMU 回调 ...", flush=True)
    time.sleep(config.COLLECT_SECONDS)
    print(f"[采集] 收到 NTF_IMU 批次={collector.batches} 总样本数(展开)={collector.total_samples}", flush=True)

    # 停流
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.setParam("NTF_IMU", "OFF")
    except Exception:
        pass

    # 布局校验
    if collector.first_batch is None:
        print("[FAIL] 采集窗口内未收到非空 NTF_IMU 数据，无法校验布局", flush=True)
        record(results, "收到 NTF_IMU 聚合数据", False,
               "采集窗口内收到非空 NTF_IMU 批次",
               f"NTF_IMU 批次={collector.batches} 非空批次=0")
    else:
        check_imu_layout(collector.first_batch, imu_channels, results)

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
