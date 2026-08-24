# -*- coding: utf-8 -*-
"""DATA-FUNC-006：SensorData 元数据合法。

对应用例：03_数据流.md -> DATA-FUNC-006
可自动化：auto（设备上电、在范围内且已佩戴为运行前置，测试中无需人工动作）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) setParam("NTF_EMG", "ON") 起 EMG 流（OYWW1100 主模态，静息也有数据）
  3) startDataNotification 后采集窗口内等回调，取收到的第一批 SensorData
  4) 对第一批数据逐一校验元数据接口：
       getDeviceMac / getDeviceName / getDataType / getSampleRate / getChannelCount /
       getChannelMask / getSampleCount / getLostPackageCount /
       getStartTimeStamp / getStartTimeSec / getDelay / isDataValid
     明确断言：getSampleRate>0、getChannelCount>0、getSampleCount>0、
               getDeviceMac==设备MAC、getDataType==NTF_EMG、isDataValid==True；
     其余接口断言"可调用且返回合法值（>=0）"。

说明：
  元数据是上层解析 SensorData 的基础，需至少拿到一批真实数据方可校验。
  DeviceInfo.ImpeChannelCount=8 但采样率仅 1Hz（约 20s 一批），不适合做元数据源；
  EMG 采样率 500Hz，数秒内即可多批，故选 EMG。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内，且已佩戴（电极接触皮肤）
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
from common import record, scan_and_match


def _dt_name(dt):
    try:
        if isinstance(dt, DataType):
            return dt.name
        return DataType(dt).name
    except Exception:
        return str(dt)


def _norm_mac(s):
    # 去除冒号/横杠等分隔符，统一大写，便于跨格式比较 MAC
    return re.sub(r"[^0-9A-Fa-f]", "", str(s or "")).upper()


class MetaCollector:
    """保存收到的第一批 NTF_EMG SensorData，用于元数据校验。"""

    def __init__(self):
        self.first_batch = None
        self.first_batch_samples = 0  # 该批 channelSamples 展开后的样本数（对照证据）
        self.batches = 0
        self.total_samples = 0
        self.type_counts = {}  # DataType 名称 -> 批数

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            self.batches += 1
            try:
                dt = d.getDataType()
                dt_key = _dt_name(dt)
            except Exception:
                dt_key = "?"
            self.type_counts[dt_key] = self.type_counts.get(dt_key, 0) + 1
            cs = getattr(d, 'channelSamples', None)
            n = 0
            if cs:
                try:
                    n = sum(len(ch) for ch in cs)
                except TypeError:
                    n = len(cs)
            self.total_samples += n
            # 只保存第一个“非空”的 NTF_EMG 批次，避免把 IMU 聚合批当成 EMG
            if self.first_batch is None and n > 0 and dt == DataType.NTF_EMG:
                self.first_batch = d
                self.first_batch_samples = n


def check_metadata(data, expect_mac, results, batch_samples):
    """对一批 SensorData 逐项校验元数据接口，结果写入 results。"""

    def add(name, ok, expect, actual):
        record(results, name, ok, expect, actual)

    # getDataType
    try:
        dt = data.getDataType()
        dt_txt = _dt_name(dt)
    except Exception as e:
        dt, dt_txt = None, f"<{type(e).__name__}>"
    add("SensorData.getDataType 匹配起流模态", dt == DataType.NTF_EMG,
        "getDataType() == DataType.NTF_EMG", f"getDataType() -> {dt_txt}")

    # getSampleRate
    try:
        sr = data.getSampleRate()
    except Exception as e:
        sr = f"<{type(e).__name__}>"
    add("SensorData.getSampleRate > 0", isinstance(sr, (int, float)) and sr > 0,
        "getSampleRate() > 0", f"getSampleRate() -> {sr}")

    # getChannelCount
    try:
        cc = data.getChannelCount()
    except Exception as e:
        cc = f"<{type(e).__name__}>"
    add("SensorData.getChannelCount > 0", isinstance(cc, (int, float)) and cc > 0,
        "getChannelCount() > 0", f"getChannelCount() -> {cc}")

    # getSampleCount
    try:
        sc = data.getSampleCount()
    except Exception as e:
        sc = f"<{type(e).__name__}>"
    add("SensorData.getSampleCount > 0", isinstance(sc, (int, float)) and sc > 0,
        "getSampleCount() > 0",
        f"getSampleCount() -> {sc}；该批 channelSamples 展开样本数={batch_samples}")

    # getChannelMask
    try:
        cm = data.getChannelMask()
    except Exception as e:
        cm = f"<{type(e).__name__}>"
    add("SensorData.getChannelMask 合法（>=0）", isinstance(cm, (int, float)) and cm >= 0,
        "getChannelMask() 返回非负数值", f"getChannelMask() -> {cm}")

    # getLostPackageCount
    try:
        lp = data.getLostPackageCount()
    except Exception as e:
        lp = f"<{type(e).__name__}>"
    add("SensorData.getLostPackageCount 合法（>=0）", isinstance(lp, (int, float)) and lp >= 0,
        "getLostPackageCount() 返回非负数值", f"getLostPackageCount() -> {lp}")

    # getStartTimeStamp
    try:
        st = data.getStartTimeStamp()
    except Exception as e:
        st = f"<{type(e).__name__}>"
    add("SensorData.getStartTimeStamp 合法（>=0）", isinstance(st, (int, float)) and st >= 0,
        "getStartTimeStamp() 返回非负数值", f"getStartTimeStamp() -> {st}")

    # getStartTimeSec（0.9.0 新增：流起始 LSL Unix 秒，未知时为 0.0）
    try:
        sts = data.getStartTimeSec()
    except Exception as e:
        sts = f"<{type(e).__name__}>"
    add("SensorData.getStartTimeSec 合法（>=0）", isinstance(sts, (int, float)) and sts >= 0,
        "getStartTimeSec() 返回非负数值", f"getStartTimeSec() -> {sts}")

    # getDelay
    try:
        dl = data.getDelay()
    except Exception as e:
        dl = f"<{type(e).__name__}>"
    add("SensorData.getDelay 合法（>=0）", isinstance(dl, (int, float)) and dl >= 0,
        "getDelay() 返回非负数值", f"getDelay() -> {dl}")

    # getDeviceMac
    try:
        mac = data.getDeviceMac()
        mac_txt = str(mac)
    except Exception as e:
        mac, mac_txt = None, f"<{type(e).__name__}>"
    got_norm = _norm_mac(mac_txt)
    exp_norm = _norm_mac(expect_mac)
    mac_ok = (mac is not None and got_norm != "" and got_norm == exp_norm)
    add("SensorData.getDeviceMac 匹配设备 MAC", mac_ok,
        f"getDeviceMac() 规范化后 == 设备 MAC({expect_mac})",
        f"getDeviceMac() -> {mac_txt}")

    # getDeviceName（0.9.5 新增：返回设备名）
    try:
        dn = data.getDeviceName()
        dn_txt = str(dn)
    except Exception as e:
        dn, dn_txt = None, f"<{type(e).__name__}>"
    dn_ok = (isinstance(dn, str) and dn.strip() != "")
    add("SensorData.getDeviceName 返回非空设备名", dn_ok,
        "getDeviceName() 返回非空字符串", f"getDeviceName() -> {dn_txt!r}")

    # isDataValid
    try:
        iv = data.isDataValid()
    except Exception as e:
        iv = f"<{type(e).__name__}>"
    add("SensorData.isDataValid == True", iv is True,
        "isDataValid() == True", f"isDataValid() -> {iv}")


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-006 SensorData 元数据合法", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内，且已佩戴（电极接触皮肤）", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】、在范围内且已【佩戴】，"
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

    # 起 EMG 流
    print("\n[起流] SensorProfile.setParam('NTF_EMG', 'ON') ...", flush=True)
    try:
        p_ret = sensor.setParam("NTF_EMG", "ON")
        p_txt = f"返回 {p_ret!r}"
    except Exception as e:
        p_ret = None
        p_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] setParam('NTF_EMG', 'ON') -> {p_txt}", flush=True)

    # 诊断：读取实际 NTF 开关，确认 EMG/IMU 起流状态
    try:
        ntf_state = sensor.getParam("NTF")
        print(f"[诊断] getParam('NTF') -> {ntf_state!r}", flush=True)
    except Exception as e:
        print(f"[诊断] getParam('NTF') 抛异常 {type(e).__name__}: {e}", flush=True)

    collector = MetaCollector()
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

    # 采集窗口，等待第一批数据
    print(f"\n[采集] 等待 {config.COLLECT_SECONDS}s 观察 onDataCallback ...", flush=True)
    time.sleep(config.COLLECT_SECONDS)
    print(f"[采集] 收到批数={collector.batches} 样本数={collector.total_samples}", flush=True)
    print(f"[采集] 各类型批数={collector.type_counts}", flush=True)

    first = collector.first_batch
    if first is None:
        record(results, "采集窗口内收到至少一批 NTF_EMG SensorData", False,
               "onDataCallback 收到 >=1 批 NTF_EMG 数据",
               f"批数={collector.batches} 样本数={collector.total_samples} 各类型={collector.type_counts}")
        print("[FAIL] 未收到 NTF_EMG 批次，无法校验 EMG 元数据（各类型批数见上）", flush=True)
    else:
        record(results, "采集窗口内收到至少一批 NTF_EMG SensorData", True,
               "onDataCallback 收到 >=1 批 NTF_EMG 数据",
               f"批数={collector.batches} 样本数={collector.total_samples} 各类型={collector.type_counts}")
        print(f"\n[校验] 对收到的第一批（非空）NTF_EMG SensorData 逐项校验元数据接口 "
              f"(该批 channelSamples 展开样本数={collector.first_batch_samples}) ...", flush=True)
        check_metadata(first, addr, results, collector.first_batch_samples)

    # 清理
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.setParam("NTF_EMG", "OFF")
    except Exception:
        pass
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
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()
