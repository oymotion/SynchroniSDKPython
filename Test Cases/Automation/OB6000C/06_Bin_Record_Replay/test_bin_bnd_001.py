# -*- coding: utf-8 -*-
"""BIN-BND-001：默认不设 DEBUG_BLE_DATA_PATH 时不持久落盘 + 临时文件生命周期观察。

对应用例：06_Bin录制回放解析.md -> BIN-BND-001
可自动化：auto（设备上电、在范围内为运行前置；临时文件观察为 best-effort/informational）

流程：
  1) setLogPath(True, 受控目录) + setDebugEnabled(True)
  2) scan -> requireSensor -> connect -> 到达 Ready -> init
     （关键：本用例【不】调用 setParam("DEBUG_BLE_DATA_PATH", ...)）
  3) startDataNotification 起流，采集数秒
  4) 起流期间观察系统临时目录（%TEMP%）中 mtime 晚于会话起点的疑似临时文件
  5) stopDataNotification、disconnect
  6) 硬断言：受控日志目录中不新增 .bin（默认不持久落盘）
  7) best-effort 观察：起流期间出现的疑似临时文件在断开后是否消失

说明：
  README：会话的原始 BLE 捕获先写进系统临时目录，stop/disconnect 时只有当
  DEBUG_BLE_DATA_PATH 为 True（或路径）才导出为 .bin；为 False/""（默认未设）时
  临时文件被直接删除。因此默认场景下不应在日志目录留下持久 .bin。
  本用例的"临时文件短暂存在后删除"为 best-effort 观察：README 未文档化临时文件的
  确切文件名/位置，且系统临时目录为共享目录，故该观察只作 informational 输出，
  不作为 PASS/FAIL 硬门槛；硬门槛仅"日志目录不新增 .bin"。

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

COLLECT_SECONDS = 3  # 起流后采集时长（秒）


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


def _scan_temp(since, keywords):
    """扫描系统临时目录，返回 mtime>=since 且文件名命中任一 keyword 的文件路径。"""
    tmp = tempfile.gettempdir()
    kws = [k for k in keywords if k]
    out = []
    try:
        for fn in os.listdir(tmp):
            full = os.path.join(tmp, fn)
            try:
                if not os.path.isfile(full):
                    continue
                if os.path.getmtime(full) < since:
                    continue
                if kws and not any(k.lower() in fn.lower() for k in kws):
                    continue
                out.append(full)
            except OSError:
                continue
    except OSError:
        pass
    return out


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-BND-001 默认不设 DEBUG_BLE_DATA_PATH 时不落盘 + 临时文件观察", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    # 受控日志目录：验证默认不在此落盘
    log_dir = tempfile.mkdtemp(prefix="sdklog_bin_neg_")
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
    session_start = time.time()  # 临时文件观察的起点

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
    keywords = [base_name, identity, ".bin"]

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

    # init（注意：本用例【不】调用 setParam("DEBUG_BLE_DATA_PATH", ...)）
    print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] SensorProfile.init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

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

    # 起流期间观察系统临时目录
    print(f"\n[采集] 等待 {COLLECT_SECONDS}s 让数据流产生并写临时文件 ...", flush=True)
    time.sleep(COLLECT_SECONDS)

    during_temp = _scan_temp(session_start, keywords)
    print(f"[观察] 起流期间系统临时目录疑似临时文件（mtime 晚于会话起点）: {during_temp if during_temp else '无'}", flush=True)

    # 停流 + 断开
    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    time.sleep(0.5)

    # 硬断言：受控日志目录不新增 .bin
    bins_after = _list_bins(log_dir)
    new_bins = sorted(set(bins_after.keys()) - set(bins_before.keys()))
    print(f"\n[检查] 日志目录 {log_dir}", flush=True)
    print(f"[检查] 新增 .bin 文件: {new_bins if new_bins else '无'}", flush=True)

    no_persist = len(new_bins) == 0
    record(results, "默认不设 DEBUG_BLE_DATA_PATH 时不持久落盘", no_persist,
           "受控日志目录中不新增 .bin",
           f"新增 {len(new_bins)} 个：{new_bins}")

    # best-effort 观察：疑似临时文件在断开后是否消失
    gone = [p for p in during_temp if not os.path.isfile(p)]
    remain = [p for p in during_temp if os.path.isfile(p)]
    print(f"[观察] 断开后消失: {gone if gone else '无'}", flush=True)
    print(f"[观察] 断开后仍存在: {remain if remain else '无'}", flush=True)
    record(results, "临时文件生命周期观察（best-effort/informational）", None,
           "起流期间出现的疑似临时文件在断开后消失",
           f"起流期间={during_temp} 断开后消失={gone} 仍存在={remain}")

    # 清理
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

    print(f"\n[提示] 本次日志目录: {log_dir}", flush=True)
    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()
