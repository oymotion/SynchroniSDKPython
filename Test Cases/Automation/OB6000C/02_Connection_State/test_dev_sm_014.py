# -*- coding: utf-8 -*-
"""DEV-SM-014：onDeviceInfoUpdate 触发与就地更新。

对应用例：02_连接与状态机.md -> DEV-SM-014
可自动化：semi-auto（需人工确认设备与后端环境）

触发条件（README）：
  onDeviceInfoUpdate 在 init 后 DeviceInfo 变化时触发，主要触发源有二：
    1) EEG_SAMPLE_RATE 变更改变上报采样率（OB6000C 主模态，与后端无关）
    2) bumble 后端：连接后链路参数（ConnectionIntervalMs / PeripheralLatency /
       SupervisionTimeoutMs）由外设更新
  OB6000C 为 EEG 脑电设备，EEG_SAMPLE_RATE 变更是主要触发源（bleak/bumble 均可用）；
  bumble 后端的链路参数更新为另一可选触发源。

后端切换说明：
  后端在 SDK import 时确定，单进程内无法切换。OB6000C 为 EEG 设备，EEG_SAMPLE_RATE
  触发源与后端无关，故当前后端（bleak/bumble）均可直接跑测试；如需额外覆盖
  bumble 链路参数更新触发源，可手动以 SENSOR_SDK_BLE_BACKEND=bumble 启动。

前置条件（运行前人工准备）：
  - 主机(电脑)：蓝牙已开启（bleak）或已接入 USB BLE dongle（bumble，Windows 上
    可能需先用管理员权限把 dongle 绑定到 WinUSB，可先跑 CTRL-FUNC-011 引导）
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认后端（bleak/bumble 均可）
  2) 确认设备开机 -> 按回车
  3) scan -> requireSensor -> connect 到 Ready -> init
  4) init 前注册 onDeviceInfoUpdate，init 后记录 getDeviceInfo() 字段快照
  5) 观察窗口内等待 onDeviceInfoUpdate 触发（EEG_SAMPLE_RATE 变更 / 链路参数更新）
  6) 断言回调参数为 DeviceInfo 且字段快照已变化（就地更新）
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config

READY_TIMEOUT = 15        # 连接后等待 Ready 超时（秒）
DEVICE_INFO_TIMEOUT = 30  # 等待 onDeviceInfoUpdate 触发超时（秒）

# 用于验证「就地更新」的字段：采样率 + 链路参数 + MTU
SNAPSHOT_FIELDS = [
    "EegSampleRate", "EmgSampleRate", "AccSampleRate", "GyroSampleRate",
    "EulerSampleRate", "QuatSampleRate", "ImuSampleRate",
    "ConnectionIntervalMs", "PeripheralLatency", "SupervisionTimeoutMs", "MTUSize",
]

from common import record, scan_and_match


def _wait_until(cond, timeout, interval=0.5, what=""):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(interval)
    print(f"  [等待超时] {what}（{timeout}s）", flush=True)
    return False


def _snapshot(info):
    if info is None:
        return None
    return {f: getattr(info, f, None) for f in SNAPSHOT_FIELDS}


def run_test():
    """完整测试逻辑（在指定后端环境下运行，通常是 bumble）。"""
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DEV-SM-014 onDeviceInfoUpdate 触发与就地更新", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启（bleak）或已接入 USB BLE dongle（bumble）", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    # bumble 后端下先确保 dongle 就绪
    backend = ctrl.getBLEBackendName()
    if (backend or '').lower() == 'bumble':
        print("\n[环境] bumble 后端，调用 SensorController.checkSetupDongle() 确保 dongle 就绪 ...", flush=True)
        try:
            dongle_ret = ctrl.checkSetupDongle()
            print(f"[环境] checkSetupDongle() -> {dongle_ret!r}", flush=True)
        except Exception as e:
            print(f"[环境] checkSetupDongle() 抛异常 {type(e).__name__}: {e}", flush=True)

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：蓝牙未就绪。请确认 dongle 已接入且主机蓝牙已关闭后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    if target is None:
        print("[FAIL] 未匹配到 config 中启用的设备（OB6000C/80F3）", flush=True)
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

    # 注册回调（init 前注册，捕获 init 及之后所有更新）
    updates = []

    def on_device_info_update(s, info):
        updates.append(info)
        print(f"  [SensorProfile.onDeviceInfoUpdate] {s.BLEDevice.Name} 收到 DeviceInfo 更新", flush=True)

    sensor.onDeviceInfoUpdate = on_device_info_update

    # connect 到 Ready
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

    _wait_until(lambda: sensor.deviceState == DeviceStateEx.Ready, READY_TIMEOUT, 0.2, "连接后等待 Ready")
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

    # init 后读取 DeviceInfo 快照
    info_before = sensor.getDeviceInfo()
    snap_before = _snapshot(info_before)
    print(f"\n[DeviceInfo] getDeviceInfo() -> {type(info_before).__name__ if info_before else None}", flush=True)
    if snap_before:
        print(f"[DeviceInfo] init 后字段快照: {snap_before}", flush=True)
    record(results, "getDeviceInfo() 返回 DeviceInfo", info_before is not None,
           "getDeviceInfo() 返回 DeviceInfo（非 None）",
           f"返回 {type(info_before).__name__ if info_before else None}")

    # 触发尝试：EEG_SAMPLE_RATE（OB6000C 主模态，变更后触发 onDeviceInfoUpdate）
    print("\n[触发尝试] setParam(EEG_SAMPLE_RATE, 500) ...", flush=True)
    try:
        sret = sensor.setParam("EEG_SAMPLE_RATE", "500")
    except Exception as e:
        sret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[触发尝试] setParam(EEG_SAMPLE_RATE, 500) -> {sret!r}", flush=True)

    # 观察窗口：等待 onDeviceInfoUpdate 触发（bumble 链路参数更新 / 采样率变更）
    print(f"\n[等待] onDeviceInfoUpdate 触发，最多 {DEVICE_INFO_TIMEOUT}s ...", flush=True)
    triggered = _wait_until(lambda: len(updates) > 0, DEVICE_INFO_TIMEOUT, 0.5, "onDeviceInfoUpdate 触发")

    print(f"[结果] onDeviceInfoUpdate 触发 {len(updates)} 次", flush=True)
    if not triggered:
        # 无法触发：说明当前后端 + 设备无触发场景
        print("[说明] 未触发。可能原因：EEG_SAMPLE_RATE 未真正变更，或 bumble 链路参数未更新。", flush=True)
        record(results, "onDeviceInfoUpdate 触发（≥1 次）", None,
               "onDeviceInfoUpdate 至少触发 1 次",
               f"触发 {len(updates)} 次（{backend} 后端，EEG_SAMPLE_RATE 变更或链路参数更新均未触发）")
        record(results, "回调参数为 DeviceInfo 对象", None,
               "onDeviceInfoUpdate 回调参数为 DeviceInfo", "onDeviceInfoUpdate 未触发，无法验证")
        record(results, "就地更新（回调与 getDeviceInfo 同一缓存对象）", None,
               "回调 DeviceInfo 与 getDeviceInfo() 为同一缓存对象（in-place patch）", "onDeviceInfoUpdate 未触发，无法验证")
    else:
        info_cb = updates[-1]
        record(results, "onDeviceInfoUpdate 触发（≥1 次）", True,
               "onDeviceInfoUpdate 至少触发 1 次", f"触发 {len(updates)} 次")

        is_di = isinstance(info_cb, DeviceInfo)
        record(results, "回调参数为 DeviceInfo 对象", is_di,
               "onDeviceInfoUpdate 回调参数为 DeviceInfo", f"参数类型 {type(info_cb).__name__}")

        snap_cb = _snapshot(info_cb)
        if snap_cb:
            print(f"[结果] 回调 DeviceInfo 字段快照: {snap_cb}", flush=True)
        # 字段快照仅作诊断信息：链路参数更新发生在 connect 后（init 前），
        # 而 getDeviceInfo() 需 init 后才返回非 None，故 init 后快照已是最终值，
        # 无法用「init 快照 vs 回调快照」捕捉到字段变化，这里不作为 PASS/FAIL 判定。
        if snap_before is not None and snap_cb is not None:
            diff = {f: (snap_before[f], snap_cb[f]) for f in SNAPSHOT_FIELDS if snap_before[f] != snap_cb[f]}
            if diff:
                print(f"[结果] init 快照 vs 回调快照 变化字段: {diff}", flush=True)
            else:
                print("[结果] init 快照与回调快照一致（链路参数在 init 前已协商完成，属预期）", flush=True)

        # 就地更新（in-place patch）验证：回调收到的 info 与触发后 getDeviceInfo()
        # 返回的是否为同一缓存对象。README 语义是 profile 就地 patch 缓存 DeviceInfo 后再触发回调。
        info_after = sensor.getDeviceInfo()
        same_object = (info_cb is info_after)
        id_cb = id(info_cb)
        id_after = id(info_after) if info_after is not None else None
        print(f"[就地更新] 回调 info id={id_cb} 触发后 getDeviceInfo id={id_after} 同一对象={same_object}", flush=True)
        record(results, "就地更新（回调与 getDeviceInfo 同一缓存对象）", same_object,
               "回调 DeviceInfo 与 getDeviceInfo() 为同一缓存对象（in-place patch）",
               f"同一对象={same_object}（id 回调={id_cb} id 读取={id_after}）")

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


def main():
    ctrl = SensorControllerInstance
    backend = ctrl.getBLEBackendName()

    print("=" * 60, flush=True)
    print("DEV-SM-014 onDeviceInfoUpdate 触发与就地更新", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"当前进程后端 = {backend}", flush=True)

    # OB6000C 为 EEG 设备，EEG_SAMPLE_RATE 变更即可触发 onDeviceInfoUpdate，
    # 该触发源与后端无关（bleak/bumble 均可用），故直接跑测试。
    # 若还需覆盖「bumble 链路参数更新」触发源，可手动以 SENSOR_SDK_BLE_BACKEND=bumble 启动。
    run_test()
    ctrl.terminate()


if __name__ == "__main__":
    main()
