# -*- coding: utf-8 -*-
"""BIN-FUNC-002：getBinFileInfo 返回字段。

对应用例：06_Bin录制回放解析.md -> BIN-FUNC-002
可自动化：auto（设备上电、在范围内为运行前置；测试中无需人工动作）

流程：
  1) setLogPath(True, 受控目录) + setDebugEnabled(True)
  2) scan -> requireSensor -> connect -> 到达 Ready -> init
  3) setParam("DEBUG_BLE_DATA_PATH", "True") 开启 bin 导出（注意：值为字符串 "True"）
  4) startDataNotification 起流，采集数秒后 stopDataNotification、disconnect（close 写 header）
  5) 通过 getParam("DEBUG_BLE_DATA_PATH") 读导出的 bin 路径（回退到目录扫描）
  6) getBinFileInfo(bin_path) 并校验返回 dict 含 device_mac/device_name/chip_type/replay_duration

说明：
  README：getBinFileInfo(file_path) -> Optional[dict]，返回 dict 字段包括 device_mac、
  device_name、chip_type、is_universal_stream、feature_map、device_info、sensor_datas、
  replay_duration（录制秒数，在 close 时写入 header）；文件不存在或无 config record 时
  返回 None。
  本用例只校验"返回 dict 且含四个关键字段（非空）"，不校验字段值语义（值一致性见
  FUNC-009、元数据完整性见后续）。
  注意 DEBUG_BLE_DATA_PATH 的值是字符串 "True"/"False"，不是 Python bool。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
"""

import os
import re
import sys
import time
import tempfile

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, _identity_of, scan_and_match

COLLECT_SECONDS = 3  # 起流后采集时长（秒），确保 bin 有数据且 replay_duration > 0
REQUIRED_KEYS = ["device_mac", "device_name", "chip_type", "replay_duration"]


def _nonempty(v):
    """字段非空：None 视为空；字符串去空白后非空；其他类型（如 chip_type 的 int）非 None 即视为非空。"""
    if v is None:
        return False
    if isinstance(v, str):
        return bool(v.strip())
    return True


def _base_name(name):
    """去掉广播名里的 (XXXX) 尾巴，得到设备基础名（如 OB6000C(80F3) -> OB6000C）。"""
    return re.sub(r"\([0-9A-Fa-f]{4}\)\s*$", "", (name or "")).strip()


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


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-FUNC-002 getBinFileInfo 返回字段", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    # 受控日志目录
    log_dir = tempfile.mkdtemp(prefix="sdklog_bin_")
    print(f"\n[日志目录] 使用受控目录 {log_dir}", flush=True)
    try:
        ctrl.setLogPath(True, log_dir)
        log_ok = True
        log_txt = f"setLogPath(True, {log_dir}) 无异常"
    except Exception as e:
        log_ok = False
        log_txt = f"setLogPath 抛异常 {type(e).__name__}: {e}"
    print(f"[日志目录] {log_txt}", flush=True)
    record(results, "setLogPath 设置受控日志目录", log_ok,
           "setLogPath(True, dir) 无异常", log_txt)

    try:
        ctrl.setDebugEnabled(True)
    except Exception as e:
        print(f"[日志目录] setDebugEnabled(True) 抛异常 {type(e).__name__}: {e}", flush=True)

    bins_before = _list_bins(log_dir)

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    print(f"[扫描] 扫描到 {len(devices) if devices else 0} 台设备:", flush=True)
    if devices:
        for d in devices:
            n = getattr(d, 'Name', '?')
            a = getattr(d, 'Address', '?')
            print(f"  {n} {a} identity={_identity_of(n)}", flush=True)

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

    base_name = _base_name(name)
    identity = _identity_of(name)

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

    # 开启 bin 导出（值必须为字符串 "True"）
    print("\n[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') ...", flush=True)
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
    except Exception as e:
        bret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') -> {bret!r}", flush=True)
    record(results, "setParam('DEBUG_BLE_DATA_PATH', 'True') 返回 OK", bret == "OK",
           "setParam 返回 'OK'", f"setParam -> {bret!r}")

    # 起流
    print("\n[起流] SensorProfile.startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        start_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        start_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] SensorProfile.startDataNotification() -> {start_txt}", flush=True)
    record(results, "SensorProfile.startDataNotification 返回 True", sret is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start_txt}")

    print(f"\n[采集] 等待 {COLLECT_SECONDS}s 让数据流产生并录制 ...", flush=True)
    time.sleep(COLLECT_SECONDS)

    # 停流 + 断开（触发 bin 导出 + close 写 header）
    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)

    ble_path = _get_ble_path(sensor)
    print(f"[bin] stop 后 getParam('DEBUG_BLE_DATA_PATH') = {ble_path!r}", flush=True)

    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    if not isinstance(ble_path, str) or not ble_path.strip():
        ble_path = _get_ble_path(sensor)
        print(f"[bin] disconnect 后 getParam('DEBUG_BLE_DATA_PATH') = {ble_path!r}", flush=True)

    time.sleep(0.5)

    # 取 bin 路径
    bins_after = _list_bins(log_dir)
    new_bins = sorted(set(bins_after.keys()) - set(bins_before.keys()))
    print(f"\n[检查] 日志目录 {log_dir}", flush=True)
    print(f"[检查] 新增 .bin 文件: {new_bins if new_bins else '无'}", flush=True)

    bin_path = ble_path if (isinstance(ble_path, str) and ble_path.strip()) else None
    if bin_path is None and new_bins:
        bin_path = bins_after[new_bins[0]]

    have_bin = bool(bin_path) and os.path.isfile(bin_path)
    print(f"[检查] bin 路径: {bin_path!r}，文件存在={have_bin}", flush=True)
    record(results, "生成有效 bin 供 getBinFileInfo 使用", have_bin,
           "存在可用 bin 文件", f"{bin_path!r}（存在={have_bin}）")

    if not have_bin:
        record(results, "getBinFileInfo 返回含关键字段的 dict", None,
               "返回 dict 且含 device_mac/device_name/chip_type/replay_duration",
               "无有效 bin，无法执行 getBinFileInfo")
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
        print("\n结论: FAIL", flush=True)
        return

    # getBinFileInfo
    print("\n[getBinFileInfo] SensorController.getBinFileInfo(bin_path) ...", flush=True)
    try:
        info = ctrl.getBinFileInfo(bin_path)
        info_txt = f"返回 {type(info).__name__}"
    except Exception as e:
        info = None
        info_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[getBinFileInfo] {info_txt}", flush=True)
    if isinstance(info, dict):
        print(f"[getBinFileInfo] 字段: {sorted(info.keys())}", flush=True)
        for k in REQUIRED_KEYS:
            print(f"    {k} = {info.get(k)!r}", flush=True)

    is_dict = isinstance(info, dict)
    record(results, "getBinFileInfo 返回 dict（非 None）", is_dict,
           "返回 dict", info_txt)

    if is_dict:
        missing = [k for k in REQUIRED_KEYS if k not in info]
        keys_ok = len(missing) == 0
        record(results, "返回含 device_mac/device_name/chip_type/replay_duration 字段", keys_ok,
               f"dict 含 {REQUIRED_KEYS}", f"缺失 {missing}" if missing else f"含全部 {REQUIRED_KEYS}")

        # 关键字段非空/数值合理
        dm = info.get("device_mac")
        dn = info.get("device_name")
        ct = info.get("chip_type")
        rd = info.get("replay_duration")
        nonempty_ok = _nonempty(dm) and _nonempty(dn) and _nonempty(ct)
        rd_ok = isinstance(rd, (int, float)) and rd >= 0
        record(results, "device_mac/device_name/chip_type 非空", nonempty_ok,
               "device_mac/device_name 非空字符串，chip_type 非 None",
               f"device_mac={dm!r} device_name={dn!r} chip_type={ct!r}")
        record(results, "replay_duration 为数值且 >=0", rd_ok,
               "replay_duration 为数值且 >=0", f"replay_duration={rd!r}")
    else:
        record(results, "返回含 device_mac/device_name/chip_type/replay_duration 字段", None,
               f"dict 含 {REQUIRED_KEYS}", "getBinFileInfo 未返回 dict，跳过字段校验")
        record(results, "device_mac/device_name/chip_type 非空", None,
               "device_mac/device_name 非空字符串，chip_type 非 None",
               "getBinFileInfo 未返回 dict，跳过字段校验")
        record(results, "replay_duration 为数值且 >=0", None,
               "replay_duration 为数值且 >=0", "getBinFileInfo 未返回 dict，跳过字段校验")

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

    print(f"\n[提示] 本次 bin 落盘目录: {log_dir}", flush=True)
    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()
