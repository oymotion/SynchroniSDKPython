# -*- coding: utf-8 -*-
"""CTRL-FUNC-013：terminate 重复调用不抛异常（连接态下验证资源释放）。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-013
可自动化：semi-auto（需人工确认设备开机）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 开机且在范围内（需连接）

流程：
  1) 人工确认设备开机 -> 按回车
  2) 扫描并匹配 config 中的 OYWW1100
  3) requireSensor + connect 到 Ready（建立真实 BLE 连接资源）
  4) 【快照】连接态 terminate 前的资源状态
  5) 第1次 SensorController.terminate() -> 断言不抛异常
  6) 【快照】第1次 terminate 后的资源状态（连接应被强制释放）
  7) 第2次 SensorController.terminate() -> 断言不抛异常（幂等）
  8) 【快照】第2次 terminate 后的资源状态
  9) 汇总：terminate 幂等 + 连接数归零 + 资源释放对比
"""

import os
import sys
import threading
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record, _identity_of

try:
    import psutil
    _HAS_PSUTIL = True
except Exception:
    psutil = None
    _HAS_PSUTIL = False


def _safe_attr(ctrl, name, fn=None):
    """安全读取 SDK 状态，返回 (显示文本, 原始值)。异常时返回抛异常文本。"""
    try:
        val = fn() if fn else getattr(ctrl, name)
        return str(val), val
    except Exception as e:
        return f"<抛异常 {type(e).__name__}: {e}>", None


def _snapshot(ctrl, label):
    """打印一份资源状态快照，返回关键指标 dict 供后续对比。"""
    print(f"\n{'─' * 56}", flush=True)
    print(f"[快照] {label}", flush=True)
    print("  --- SensorController 状态 ---", flush=True)

    metrics = {}

    txt, val = _safe_attr(ctrl, 'isEnable')
    print(f"  SensorController.isEnable = {txt}", flush=True)
    metrics['isEnable'] = val

    txt, val = _safe_attr(ctrl, 'isScanning')
    print(f"  SensorController.isScanning = {txt}", flush=True)
    metrics['isScanning'] = val

    txt, val = _safe_attr(ctrl, 'getConnectedSensors', fn=ctrl.getConnectedSensors)
    n = len(val) if isinstance(val, (list, tuple, set)) else -1
    print(f"  SensorController.getConnectedSensors() = {txt}（{n} 台）", flush=True)
    metrics['connectedSensors'] = n

    txt, val = _safe_attr(ctrl, 'getConnectedDevices', fn=ctrl.getConnectedDevices)
    n = len(val) if isinstance(val, (list, tuple, set)) else -1
    print(f"  SensorController.getConnectedDevices() = {txt}（{n} 台）", flush=True)
    metrics['connectedDevices'] = n

    print("  --- 运行时资源 ---", flush=True)
    threads = threading.enumerate()
    metrics['threadCount'] = len(threads)
    print(f"  线程数 = {len(threads)}", flush=True)
    for t in threads:
        print(f"      - {t.name} (daemon={t.daemon})", flush=True)

    if _HAS_PSUTIL:
        proc = psutil.Process(os.getpid())
        is_win = sys.platform.startswith('win')

        try:
            metrics['numHandles'] = proc.num_handles() if hasattr(proc, 'num_handles') else None
        except Exception:
            metrics['numHandles'] = None
        try:
            metrics['numFds'] = proc.num_fds() if hasattr(proc, 'num_fds') else None
        except Exception:
            metrics['numFds'] = None
        try:
            metrics['numThreadsSys'] = proc.num_threads()
        except Exception:
            metrics['numThreadsSys'] = None
        try:
            metrics['rssMB'] = round(proc.memory_info().rss / 1024 / 1024, 1)
        except Exception:
            metrics['rssMB'] = None
        try:
            metrics['children'] = len(proc.children())
        except Exception:
            metrics['children'] = None
        try:
            metrics['numConnections'] = len(proc.net_connections())
        except Exception:
            metrics['numConnections'] = None
        try:
            metrics['numOpenFiles'] = len(proc.open_files())
        except Exception:
            metrics['numOpenFiles'] = None

        if is_win:
            print(f"  进程句柄数 = {metrics['numHandles']}", flush=True)
        else:
            print(f"  进程 fd 数 = {metrics['numFds']}", flush=True)
        print(f"  网络连接数 = {metrics['numConnections']}", flush=True)
        print(f"  打开文件数 = {metrics['numOpenFiles']}", flush=True)
        print(f"  系统线程数 = {metrics['numThreadsSys']}", flush=True)
        print(f"  子进程数 = {metrics['children']}", flush=True)
        print(f"  内存 RSS = {metrics['rssMB']} MB", flush=True)
    else:
        print("  （psutil 未安装，跳过句柄/子进程/内存观测）", flush=True)

    return metrics


def _call_terminate():
    """调用一次 terminate，返回 (是否抛异常, 异常文本)。"""
    try:
        SensorControllerInstance.terminate()
        return False, None
    except Exception as e:
        return True, f"{type(e).__name__}: {e}"


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-013 terminate 重复调用（连接态验证资源释放）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"psutil 可用 = {_HAS_PSUTIL}", flush=True)

    results = []

    # ---- 前置：人工确认设备 ----
    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，完成后按回车继续 ...")

    # ---- 环境检查 ----
    if not ctrl.isEnable:
        print("[FAIL] SensorController.isEnable == False，请先开启电脑蓝牙后重跑", flush=True)
        ctrl.terminate()
        return

    # ---- 扫描并匹配 ----
    print(f"\n扫描 {config.SCAN_TIMEOUT_MS} ms ...", flush=True)
    devices = ctrl.scan(config.SCAN_TIMEOUT_MS)
    print(f"发现 {len(devices)} 台设备：", flush=True)
    for d in devices:
        print(f"  - {getattr(d, 'Name', '?')} {getattr(d, 'Address', '?')}", flush=True)

    target = None
    for cfg in config.DEVICES:
        if not cfg.get("enabled", True):
            continue
        mac = (cfg.get("mac") or "").strip().upper()
        identity = (cfg.get("identity") or "").strip().upper()
        for d in devices:
            addr = (getattr(d, 'Address', '') or '').upper()
            name = getattr(d, 'Name', '') or ''
            if mac and addr == mac:
                target = d
                break
            if identity and _identity_of(name) == identity:
                target = d
                break
        if target is not None:
            break

    if target is None:
        print("[FAIL] 未匹配到设备，请检查 config.py 或设备是否开机", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"\n匹配到设备: {name} {addr}", flush=True)

    # ---- 连接（建立真实 BLE 资源）----
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        ctrl.terminate()
        return

    ok = sensor.connect()
    print(f"SensorProfile.connect() -> {ok}, state={sensor.deviceState}", flush=True)
    record(results, "连接成功（connect 返回 True 且 Ready）",
           ok is True and sensor.deviceState == DeviceStateEx.Ready,
           "connect()==True 且 deviceState==Ready", f"connect={ok}, state={sensor.deviceState}")

    # 连接成功后短暂等待，让 BLE 资源稳定
    time.sleep(1)

    txt, val = _safe_attr(ctrl, 'getConnectedSensors', fn=ctrl.getConnectedSensors)
    n_conn = len(val) if isinstance(val, (list, tuple, set)) else -1
    record(results, "连接态 getConnectedSensors 含 1 台", n_conn >= 1,
           "getConnectedSensors() 至少 1 台", f"返回 {n_conn} 台（{txt}）")

    # 快照 1：连接态 terminate 前
    snap_before = _snapshot(ctrl, "连接态 terminate 前")

    # 检查：第1次 terminate 不抛异常
    raised1, err1 = _call_terminate()
    print(f"\n[检测] 第1次 SensorController.terminate() -> {'抛异常 ' + err1 if raised1 else '无异常'}", flush=True)
    record(results, "第1次 terminate 不抛异常", not raised1,
           "第1次 terminate 不抛异常", f"抛异常 {err1}" if raised1 else "无异常")

    # 快照 2：第1次 terminate 后
    snap_after1 = _snapshot(ctrl, "第1次 terminate 后")

    # 检查：第2次 terminate 不抛异常（幂等）
    raised2, err2 = _call_terminate()
    print(f"\n[检测] 第2次 SensorController.terminate() -> {'抛异常 ' + err2 if raised2 else '无异常'}", flush=True)
    record(results, "第2次 terminate 不抛异常（幂等）", not raised2,
           "第2次 terminate 不抛异常", f"抛异常 {err2}" if raised2 else "无异常")

    # 快照 3：第2次 terminate 后
    snap_after2 = _snapshot(ctrl, "第2次 terminate 后")

    # 检查：terminate 后连接归零（资源释放直接证据）
    record(results, "terminate 后 getConnectedSensors 归零", snap_after1['connectedSensors'] == 0,
           "第1次 terminate 后 getConnectedSensors()==0", f"第1次后 {snap_after1['connectedSensors']} 台")

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for name, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {name}（实际: {actual}）", flush=True)
        else:
            print(f"  [FAIL] {name}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    # ---- 资源释放对比（观测性输出）----
    print("\n--- 资源释放对比（连接态 terminate 前 -> 第1次后 -> 第2次后）---", flush=True)
    print(f"  线程数(threading): {snap_before['threadCount']} -> {snap_after1['threadCount']} -> {snap_after2['threadCount']}", flush=True)
    print(f"  isScanning: {snap_before['isScanning']} -> {snap_after1['isScanning']} -> {snap_after2['isScanning']}", flush=True)
    print(f"  isEnable: {snap_before['isEnable']} -> {snap_after1['isEnable']} -> {snap_after2['isEnable']}", flush=True)
    print(f"  connectedSensors: {snap_before['connectedSensors']} -> {snap_after1['connectedSensors']} -> {snap_after2['connectedSensors']}", flush=True)
    print(f"  connectedDevices: {snap_before['connectedDevices']} -> {snap_after1['connectedDevices']} -> {snap_after2['connectedDevices']}", flush=True)
    if _HAS_PSUTIL:
        if sys.platform.startswith('win'):
            print(f"  句柄数: {snap_before['numHandles']} -> {snap_after1['numHandles']} -> {snap_after2['numHandles']}", flush=True)
        else:
            print(f"  fd 数: {snap_before['numFds']} -> {snap_after1['numFds']} -> {snap_after2['numFds']}", flush=True)
        print(f"  网络连接数: {snap_before['numConnections']} -> {snap_after1['numConnections']} -> {snap_after2['numConnections']}", flush=True)
        print(f"  打开文件数: {snap_before['numOpenFiles']} -> {snap_after1['numOpenFiles']} -> {snap_after2['numOpenFiles']}", flush=True)
        print(f"  系统线程数: {snap_before['numThreadsSys']} -> {snap_after1['numThreadsSys']} -> {snap_after2['numThreadsSys']}", flush=True)
        print(f"  子进程数: {snap_before['children']} -> {snap_after1['children']} -> {snap_after2['children']}", flush=True)
        print(f"  内存RSS(MB): {snap_before['rssMB']} -> {snap_after1['rssMB']} -> {snap_after2['rssMB']}", flush=True)

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()
