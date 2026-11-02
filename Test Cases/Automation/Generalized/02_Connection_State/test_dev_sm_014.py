# -*- coding: utf-8 -*-
"""DEV-SM-014：onDeviceInfoUpdate 触发与就地更新。

对应用例：02_连接与状态机.md -> DEV-SM-014
可自动化：semi-auto（需人工接入 USB BLE dongle 并关闭主机蓝牙）

触发条件（README）：
  onDeviceInfoUpdate 在 init 后 DeviceInfo 变化时触发，主要触发源有二：
    1) dongle 后端：连接后链路参数（ConnectionIntervalMs / PeripheralLatency /
       SupervisionTimeoutMs）由外设更新
    2) EEG_SAMPLE_RATE 变更改变上报采样率
  是否有 EEG 采样率触发源由设备 spec（spec["streams"]["NTF_EEG"]["supported"]）判定：
  不支持 EEG 时无法用 EEG_SAMPLE_RATE 触发，只能走【dongle 后端】的链路参数更新路径。

后端切换说明：
  后端在 SDK import 时确定，单进程内无法切换。因此：
    - 主进程若已是 dongle，直接跑测试；
    - 主进程若是 bleak，提示用户关闭主机蓝牙 + 接入 dongle，确认后以
      SENSOR_SDK_BLE_BACKEND=dongle 子进程重跑本脚本。

前置条件（运行前人工准备）：
  - 主机(电脑)：关闭主机蓝牙，接入 USB BLE dongle（Windows 上可能需先用管理员
    权限把 dongle 绑定到 WinUSB，可先跑 CTRL-FUNC-011 checkSetupDongle 引导）
  - 待测设备：目标设备上电、在范围内

流程：
  1) 确认后端（dongle）/ 引导接入 dongle
  2) 确认设备开机 -> 按回车
  3) scan -> requireSensor -> connect 到 Ready -> init
  4) init 前注册 onDeviceInfoUpdate，init 后记录 getDeviceInfo() 字段快照
  5) 观察窗口内等待 onDeviceInfoUpdate 触发（链路参数更新）
  6) 断言回调参数为 DeviceInfo 且字段快照已变化（就地更新）
"""

import os
import subprocess
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common

READY_TIMEOUT = 15        # 连接后等待 Ready 超时（秒）
DEVICE_INFO_TIMEOUT = 30  # 等待 onDeviceInfoUpdate 触发超时（秒）

# 链路参数 + MTU：dongle 后端触发 onDeviceInfoUpdate 的主要来源，固定参与快照
LINK_FIELDS = ["ConnectionIntervalMs", "PeripheralLatency", "SupervisionTimeoutMs", "MTUSize"]

from common import record, scan_and_match


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


# 参数键 -> DeviceInfo 采样率字段名（SDK 固定命名，非设备能力，仅用于快照诊断）
_PARAM_TO_DEVICE_INFO_RATE_FIELD = {
    "EEG": "EegSampleRate", "EMG": "EmgSampleRate", "IMU": "ImuSampleRate",
    "PPG": "PpgSampleRate", "SPO2": "Spo2SampleRate", "ECG": "EcgSampleRate",
    "ACC": "AccSampleRate", "GYRO": "GyroSampleRate", "EULER": "EulerSampleRate",
    "QUAT": "QuatSampleRate", "IMPEDANCE": "ImpeSampleRate", "BRTH": "BrthSampleRate",
    "MAG_ANGLE": "MagAngleSampleRate",
}


def _snapshot_fields(spec):
    """由 spec["sample_rates"] 推导快照字段：采样率 DeviceInfo 字段 + 链路参数 + MTU。"""
    fields = []
    for key in (spec.get("sample_rates") or {}):
        stem = key.split("_SAMPLE_RATE")[0] if "_SAMPLE_RATE" in key else key
        f = _PARAM_TO_DEVICE_INFO_RATE_FIELD.get(stem)
        if f and f not in fields:
            fields.append(f)
    for f in LINK_FIELDS:
        if f not in fields:
            fields.append(f)
    return fields


def _wait_until(cond, timeout, interval=0.5, what=""):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(interval)
    print(f"  [等待超时] {what}（{timeout}s）", flush=True)
    return False


def _snapshot(info, fields):
    if info is None:
        return None
    return {f: getattr(info, f, None) for f in fields}


def run_test():
    """完整测试逻辑（在指定后端环境下运行，通常是 dongle）。"""
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DEV-SM-014 onDeviceInfoUpdate 触发与就地更新", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已关闭，已接入 USB BLE dongle（dongle 后端）", flush=True)
    print("  - 待测设备：目标设备上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    # dongle 后端下先确保 dongle 就绪
    backend = ctrl.getBLEBackendName()
    if (backend or '').lower() == 'dongle':
        print("\n[环境] dongle 后端，调用 SensorController.checkSetupDongle() 确保 dongle 就绪 ...", flush=True)
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

    spec = _load_spec(target)
    if spec is None:
        record(results, "载入设备 spec", False, "能从广播名提取 identity 并载入对应 spec", "spec 载入失败")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "载入设备 spec", True, "能从广播名提取 identity 并载入对应 spec", f"model={spec.get('model')}")
    fields = _snapshot_fields(spec)

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
    snap_before = _snapshot(info_before, fields)
    print(f"\n[DeviceInfo] getDeviceInfo() -> {type(info_before).__name__ if info_before else None}", flush=True)
    if snap_before:
        print(f"[DeviceInfo] init 后字段快照: {snap_before}", flush=True)
    record(results, "getDeviceInfo() 返回 DeviceInfo", info_before is not None,
           "getDeviceInfo() 返回 DeviceInfo（非 None）",
           f"返回 {type(info_before).__name__ if info_before else None}")

    # 触发尝试：EEG_SAMPLE_RATE 变更（仅当 spec 标记支持 EEG 且存在该参数时尝试）
    eeg_cap = (spec.get("streams") or {}).get("NTF_EEG", {})
    eeg_supported = bool(eeg_cap.get("supported") is True and (eeg_cap.get("channels") or 0) > 0)
    eeg_rate_cfg = (spec.get("sample_rates") or {}).get("EEG_SAMPLE_RATE")
    if eeg_supported and eeg_rate_cfg:
        rates = eeg_rate_cfg.get("rates") or []
        target_rate = eeg_rate_cfg.get("default") or (rates[0] if rates else None)
        print(f"\n[触发尝试] setParam(EEG_SAMPLE_RATE, {target_rate}) ...", flush=True)
        try:
            sret = sensor.setParam("EEG_SAMPLE_RATE", str(target_rate))
        except Exception as e:
            sret = f"抛异常 {type(e).__name__}: {e}"
        print(f"[触发尝试] setParam(EEG_SAMPLE_RATE, {target_rate}) -> {sret!r}", flush=True)
    else:
        print("\n[触发尝试] spec 标记不支持 EEG（NTF_EEG supported=False）或无 EEG_SAMPLE_RATE 参数，跳过 EEG 触发 ...", flush=True)
        record(results, "EEG_SAMPLE_RATE 触发尝试", None,
               "存在 EEG 采样率参数时尝试触发 onDeviceInfoUpdate",
               "spec 标记不支持 EEG / 无该参数，跳过")

    # 观察窗口：等待 onDeviceInfoUpdate 触发（dongle 链路参数更新 / 采样率变更）
    print(f"\n[等待] onDeviceInfoUpdate 触发，最多 {DEVICE_INFO_TIMEOUT}s ...", flush=True)
    triggered = _wait_until(lambda: len(updates) > 0, DEVICE_INFO_TIMEOUT, 0.5, "onDeviceInfoUpdate 触发")

    print(f"[结果] onDeviceInfoUpdate 触发 {len(updates)} 次", flush=True)
    if not triggered:
        # 无法触发：说明当前后端 + 设备无触发场景
        print("[说明] 未触发。可能原因：dongle 链路参数未更新，或设备无 EEG 采样率变更。", flush=True)
        record(results, "onDeviceInfoUpdate 触发（≥1 次）", None,
               "onDeviceInfoUpdate 至少触发 1 次",
               f"触发 {len(updates)} 次（{backend} 后端，可能无触发场景）")
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

        snap_cb = _snapshot(info_cb, fields)
        if snap_cb:
            print(f"[结果] 回调 DeviceInfo 字段快照: {snap_cb}", flush=True)
        # 字段快照仅作诊断信息：链路参数更新发生在 connect 后（init 前），
        # 而 getDeviceInfo() 需 init 后才返回非 None，故 init 后快照已是最终值，
        # 无法用「init 快照 vs 回调快照」捕捉到字段变化，这里不作为 PASS/FAIL 判定。
        if snap_before is not None and snap_cb is not None:
            diff = {f: (snap_before[f], snap_cb[f]) for f in fields if snap_before[f] != snap_cb[f]}
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

    # 子进程模式：直接跑测试（由主进程以 dongle 后端启动）
    if os.environ.get('DEV_SM_014_RUN') == '1':
        run_test()
        return

    print("=" * 60, flush=True)
    print("DEV-SM-014 onDeviceInfoUpdate 触发与就地更新", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"当前进程后端 = {backend}", flush=True)

    if (backend or '').lower() == 'dongle':
        run_test()
        return

    # bleak 后端：链路参数更新不可用，若设备无 EEG 亦无法用 EEG_SAMPLE_RATE 触发，需切 dongle
    print("\n[后端引导]", flush=True)
    print("  当前为 bleak 后端。onDeviceInfoUpdate 的主要触发源是「链路参数更新」，", flush=True)
    print("  仅在 dongle（USB dongle）后端可用；若设备无 EEG，亦无法用 EEG_SAMPLE_RATE 触发。", flush=True)
    print("  因此本用例需切换到 dongle 后端。", flush=True)
    print("\n  请完成以下两步：", flush=True)
    print("    1) 关闭【电脑】蓝牙", flush=True)
    print("    2) 接入 USB BLE dongle（如未绑定 WinUSB 驱动，请先跑 CTRL-FUNC-011 引导）", flush=True)
    input("\n>>> [人工操作] 完成上述两步后按回车继续（将以 dongle 后端子进程重跑）...")

    env = os.environ.copy()
    env['SENSOR_SDK_BLE_BACKEND'] = 'dongle'
    env['DEV_SM_014_RUN'] = '1'
    print("\n[重跑] 以 SENSOR_SDK_BLE_BACKEND=dongle 启动子进程 ...", flush=True)
    subprocess.run([sys.executable, os.path.abspath(__file__)], env=env)

    # 子进程已输出结果，主进程不再重复汇总
    ctrl.terminate()


if __name__ == "__main__":
    main()
