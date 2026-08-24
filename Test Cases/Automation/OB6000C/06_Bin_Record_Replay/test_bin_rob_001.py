# -*- coding: utf-8 -*-
"""BIN-ROB-001：磁盘不足时跳过/停止录制。

对应用例：06_Bin录制回放解析.md -> BIN-ROB-001
可自动化：auto（通过文件系统权限模拟写盘失败，无需实际磁盘不足）

流程：
  1) setLogPath(True, 受控目录) + setDebugEnabled(True)
  2) scan -> requireSensor -> connect -> 到达 Ready -> init
  3) setParam("DEBUG_BLE_DATA_PATH", "True") 开启 bin 导出
  4) 将日志目录设为只读（icacls /deny 当前用户写权限），模拟写盘失败
  5) 注册 onDataCallback 计数，startDataNotification 起流，采集 N 秒
  6) 校验：实时流正常（onDataCallback 收到数据），SDK 不崩溃
  7) stopDataNotification + disconnect
  8) 校验：日志目录无新增 .bin 文件（写盘失败时 SDK 跳过/停止录制）
  9) 恢复目录权限，清理

说明：
  SDK 的 bin 录制在 C++ 层完成，Python 层 mock 无法拦截。
  通过 Windows icacls 将日志目录设为当前用户只读，SDK 写 bin 文件时
  会触发写错误，等效于"写盘异常"场景。
  验证要点：
    - 不崩溃（SDK 进程正常，无异常退出）
    - 实时流继续（onDataCallback 仍收到数据）
    - bin 未生成（写盘失败时跳过录制）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
  - 需要管理员权限（icacls 修改目录权限）
"""

import os
import re
import sys
import time
import tempfile
import subprocess
import getpass

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, _identity_of, scan_and_match

COLLECT_SECONDS = 5  # 起流采集时长（秒）


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


def _deny_write(dir_path):
    """将目录设为当前用户只读。返回 (ok, detail)。"""
    username = getpass.getuser()
    try:
        subprocess.run(
            ["icacls", dir_path, "/deny", f"{username}:(W)"],
            capture_output=True, text=True, check=True, timeout=10,
        )
        return True, f"已对 {username} 拒绝写权限"
    except subprocess.CalledProcessError as e:
        return False, f"icacls /deny 失败: {e.stderr.strip() if e.stderr else str(e)}"
    except FileNotFoundError:
        return False, "icacls 命令不可用"
    except Exception as e:
        return False, f"拒绝写权限异常: {e}"


def _restore_write(dir_path):
    """恢复目录写权限。"""
    username = getpass.getuser()
    try:
        subprocess.run(
            ["icacls", dir_path, "/remove", username],
            capture_output=True, text=True, check=True, timeout=10,
        )
        return True
    except Exception:
        pass
    return False


def _on_error(sensor, reason):
    print(f"[onErrorCallback] {getattr(sensor, 'BLEDevice', None)}: {reason}", flush=True)


class LiveCounter:
    """简单计数回调：验证实时流是否正常产生数据。"""

    def __init__(self):
        self.count = 0

    def __call__(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        self.count += len(items)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-ROB-001 磁盘不足时跳过/停止录制", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)
    print("  - 需要管理员权限（icacls 修改目录权限）", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    log_dir = tempfile.mkdtemp(prefix="sdklog_rob_")
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

    sensor.onErrorCallback = _on_error

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

    # 开启 bin 导出
    print("\n[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') ...", flush=True)
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
    except Exception as e:
        bret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') -> {bret!r}", flush=True)
    record(results, "setParam('DEBUG_BLE_DATA_PATH', 'True') 返回 OK", bret == "OK",
           "setParam 返回 'OK'", f"setParam -> {bret!r}")

    # ---- 模拟写盘失败：将日志目录设为只读 ----
    print(f"\n[模拟] 将日志目录设为只读（模拟写盘失败）...", flush=True)
    deny_ok, deny_detail = _deny_write(log_dir)
    print(f"[模拟] {deny_detail}", flush=True)
    record(results, "日志目录设为只读（模拟写盘失败）", deny_ok,
           "icacls /deny 成功", deny_detail)

    if not deny_ok:
        print("[FAIL] 无法模拟写盘失败，跳过后续测试", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        try:
            ctrl.setDebugEnabled(False)
        except Exception:
            pass
        ctrl.terminate()
        print("\n结论: FAIL", flush=True)
        return

    # ---- 起流采集 ----
    live = LiveCounter()
    sensor.onDataCallback = live

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

    print(f"\n[采集] 等待 {COLLECT_SECONDS}s（写盘应失败，但实时流应继续）...", flush=True)
    time.sleep(COLLECT_SECONDS)

    # 校验 1：实时流正常（收到数据）
    has_data = live.count > 0
    record(results, "写盘失败时实时流继续（onDataCallback 收到数据）", has_data,
           "onDataCallback 收到数据（count > 0）", f"live.count={live.count}")

    # 校验 2：SDK 不崩溃（进程存活，能正常执行 stopDataNotification）
    print("\n[停流] SensorProfile.stopDataNotification() ...", flush=True)
    crash = False
    try:
        sensor.stopDataNotification()
        stop_txt = "无异常"
    except Exception as e:
        stop_txt = f"抛异常 {type(e).__name__}: {e}"
        crash = True
    print(f"[停流] stopDataNotification -> {stop_txt}", flush=True)
    record(results, "写盘失败时 SDK 不崩溃（stopDataNotification 正常）", not crash,
           "stopDataNotification 无异常", stop_txt)

    # disconnect
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    time.sleep(0.5)

    # 校验 3：bin 未生成（写盘失败时跳过录制）
    bins_after = _list_bins(log_dir)
    new_bins = sorted(set(bins_after.keys()) - set(bins_before.keys()))
    no_bin = len(new_bins) == 0
    print(f"\n[检查] 新增 .bin 文件: {new_bins if new_bins else '无'}", flush=True)
    record(results, "写盘失败时跳过录制（无新增 .bin）", no_bin,
           "日志目录无新增 .bin 文件", f"新增 {len(new_bins)} 个 .bin: {new_bins}" if new_bins else "无新增 .bin")

    # ---- 恢复权限并清理 ----
    print(f"\n[清理] 恢复目录写权限 ...", flush=True)
    restored = _restore_write(log_dir)
    print(f"[清理] 权限恢复: {'成功' if restored else '失败（需手动处理）'}", flush=True)

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

    print(f"\n[提示] 日志目录: {log_dir}", flush=True)
    if not restored:
        print(f"[警告] 目录权限未恢复，请手动执行: icacls \"{log_dir}\" /remove {getpass.getuser()}", flush=True)

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()