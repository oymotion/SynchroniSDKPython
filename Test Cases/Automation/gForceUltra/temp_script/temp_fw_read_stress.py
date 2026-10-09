# -*- coding: utf-8 -*-
"""临时脚本：gForceUltra 反复连接/读取固件/断开 压力测试。

用途：临时验证「反复连接 -> Ready -> 读固件 -> 断开 -> 立即重连」的稳定性，
      不并入正式用例，仅作排查用。

流程（循环 MAX_ITERATIONS 次）：
  1) scan -> requireSensor（仅一次，复用同一 sensor 对象反复 connect/disconnect）
  2) connect -> 到达 Ready
  3) fetchDeviceInfo() 读固件信息（每次真正读，不依赖 getDeviceInfo 缓存）
  4) 固件信息读不到 -> 立即报错退出
  5) disconnect -> 到达 Disconnected
  6) 立即进入下一轮（重连）

连接失败：等 CONNECT_RETRY_INTERVAL 秒重试，最多 CONNECT_RETRY_MAX 次，仍失败则报错退出。

用法：
  python temp_fw_read_stress.py            # 用 config.TARGET_IDENTITY 指定设备
  python temp_fw_read_stress.py 80E1       # 指定 gForceUltra identity

前置条件：
  - 主机蓝牙已开启（或 USB dongle 已就绪）
  - 待测 gForceUltra 上电、在范围内
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import scan_and_match

# ---- 可调常数 ----
MAX_ITERATIONS = 500           # 尝试次数（总循环轮数）
CONNECT_RETRY_MAX = 5          # 连接失败后的重试次数
CONNECT_RETRY_INTERVAL = 2     # 连接失败后重试间隔（秒）
RECONNECT_COOLDOWN = 2         # 断开后到重连前的冷却期（秒），给设备端恢复广播
READY_TIMEOUT = 15             # 连接后等待 Ready 的超时（秒）
DISCONNECTED_TIMEOUT = 15      # 断开后等待 Disconnected 的超时（秒）
LOG_SUBDIR = "fw_stress_logs"  # SDK 日志输出子目录（脚本旁）


def _wait_state(sensor, target_state, timeout):
    """轮询等待 deviceState 到达 target_state，返回是否到达。"""
    t0 = time.time()
    while time.time() - t0 < timeout and sensor.deviceState != target_state:
        time.sleep(0.2)
    return sensor.deviceState == target_state


def _connect_to_ready(sensor):
    """连接一次并等待 Ready。返回 True 成功 / False 失败（失败前会尝试断开清理残留状态）。"""
    # 若残留连接状态，先断开清理，确保从 Disconnected 出发
    if sensor.deviceState != DeviceStateEx.Disconnected:
        try:
            sensor.disconnect()
        except Exception:
            pass
        _wait_state(sensor, DeviceStateEx.Disconnected, 5)

    try:
        ok = sensor.connect()
    except Exception as e:
        print(f"      connect 抛异常: {type(e).__name__}: {e}", flush=True)
        return False
    if ok is not True:
        print(f"      connect 返回 {ok}", flush=True)
        return False
    if not _wait_state(sensor, DeviceStateEx.Ready, READY_TIMEOUT):
        print(f"      未到达 Ready（state={sensor.deviceState}）", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        return False
    return True


def _fail(ctrl, msg):
    """报错退出：打印错误、关闭 SDK、以非零码退出。"""
    print(f"\n[FAIL] {msg}", flush=True)
    try:
        ctrl.terminate()
    except Exception:
        pass
    sys.exit(1)


def _scan_after_connect_fail(ctrl, target_addr, target_name):
    """连接失败后做一次广播扫描，判断设备是否仍在广播。

    用于排查 connect 失败到底是「设备 disconnect 后停止广播」还是「设备在广播但建链失败」。
    """
    print(f"\n[排查] 连接失败，执行广播扫描（{config.SCAN_TIMEOUT_MS}ms）...", flush=True)
    try:
        devices = ctrl.scan(config.SCAN_TIMEOUT_MS)
    except Exception as e:
        print(f"[排查] scan 抛异常: {type(e).__name__}: {e}", flush=True)
        devices = None
    if not devices:
        print("[排查] 扫描到 0 台设备 —— 设备可能已停止广播（disconnect 后未恢复广播）", flush=True)
        return
    print(f"[排查] 扫描到 {len(devices)} 台设备:", flush=True)
    found = False
    ta = (target_addr or "").strip().upper()
    tn = (target_name or "").strip()
    for d in devices:
        n = (getattr(d, 'Name', '') or '').strip()
        a = (getattr(d, 'Address', '') or '').strip()
        r = getattr(d, 'RSSI', None)
        rssi = f"RSSI={r}" if r is not None else "RSSI=?"
        mark = ""
        if ta and a.upper() == ta:
            found = True
            mark = "  <== 目标(MAC)"
        elif tn and n == tn:
            found = True
            mark = "  <== 目标(名称)"
        print(f"    {n or '?'}  {a or '?'}  {rssi}{mark}", flush=True)
    if found:
        print("[排查] 结论：目标设备仍在广播（能被扫到）→ 设备在广播但建链失败，问题更可能在 dongle/连接侧", flush=True)
    else:
        print("[排查] 结论：目标设备未出现在扫描结果 → 设备已停止广播，指向 disconnect 后设备端未恢复广播", flush=True)


def main():
    target_identity = sys.argv[1].strip().upper() if len(sys.argv) > 1 else None
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("gForceUltra 反复连接/读取固件/断开 压力测试", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"循环轮数 = {MAX_ITERATIONS}；连接重试 = {CONNECT_RETRY_MAX} 次 / 间隔 {CONNECT_RETRY_INTERVAL}s", flush=True)

    # 开 SDK log
    log_dir = os.path.join(BASE_DIR, LOG_SUBDIR)
    os.makedirs(log_dir, exist_ok=True)
    print(f"\n[日志目录] {log_dir}", flush=True)
    try:
        ctrl.setLogPath(True, log_dir)
    except Exception as e:
        print(f"[日志] setLogPath 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        ctrl.setDebugEnabled(True)
        print("[日志] setDebugEnabled(True) 已开启", flush=True)
    except Exception as e:
        print(f"[日志] setDebugEnabled 抛异常 {type(e).__name__}: {e}", flush=True)

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        _fail(ctrl, "电脑蓝牙未开启，请先开启【电脑】蓝牙后重跑")

    # scan + requireSensor（仅一次）
    print(f"\n[扫描] 目标 identity: {target_identity or common.TARGET_IDENTITIES} ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, target_identity=target_identity)
    if target is None:
        _fail(ctrl, "未匹配到目标设备")
    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        _fail(ctrl, "requireSensor 返回 None")

    # 主循环
    success_count = 0
    start_time = time.time()
    for i in range(1, MAX_ITERATIONS + 1):
        # 连接（带重试）
        connected = False
        for attempt in range(1, CONNECT_RETRY_MAX + 1):
            if _connect_to_ready(sensor):
                connected = True
                break
            print(f"    连接失败（第 {attempt}/{CONNECT_RETRY_MAX} 次），{CONNECT_RETRY_INTERVAL}s 后重试 ...", flush=True)
            time.sleep(CONNECT_RETRY_INTERVAL)
        if not connected:
            _scan_after_connect_fail(ctrl, addr, name)
            _fail(ctrl, f"第 {i} 次迭代连接失败（重试 {CONNECT_RETRY_MAX} 次均失败）")

        # 读固件信息（每次真正读，不依赖 getDeviceInfo 缓存）
        info = sensor.fetchDeviceInfo()
        if info is None or not info.FirmwareVersion:
            detail = "info 为 None" if info is None else "FirmwareVersion 为空"
            _fail(ctrl, f"第 {i} 次迭代固件信息读不到（{detail}），立即退出")

        # 断开
        try:
            sensor.disconnect()
        except Exception as e:
            print(f"    disconnect 抛异常: {type(e).__name__}: {e}", flush=True)
        _wait_state(sensor, DeviceStateEx.Disconnected, DISCONNECTED_TIMEOUT)
        # 断开后先冷却，等设备端重新进入可连接/广播态，再进下一轮重连
        time.sleep(RECONNECT_COOLDOWN)

        success_count = i
        elapsed = time.time() - start_time
        print(f"[OK] 第 {i}/{MAX_ITERATIONS} 轮：固件 {info.FirmwareVersion!r}（累计 {elapsed:.0f}s）", flush=True)

    elapsed = time.time() - start_time
    print("\n" + "=" * 60, flush=True)
    print("压力测试完成", flush=True)
    print("=" * 60, flush=True)
    print(f"  完成轮次: {success_count}/{MAX_ITERATIONS}", flush=True)
    print(f"  总耗时: {elapsed:.0f}s（{elapsed / 60:.1f} 分钟）", flush=True)
    print(f"  日志目录: {log_dir}", flush=True)

    ctrl.terminate()


if __name__ == "__main__":
    main()
