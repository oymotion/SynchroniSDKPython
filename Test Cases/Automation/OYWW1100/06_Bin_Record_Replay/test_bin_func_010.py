# -*- coding: utf-8 -*-
"""BIN-FUNC-010：multiReplayBinFile 多文件同步回放。

对应用例：06_Bin录制回放解析.md -> BIN-FUNC-010
可自动化：auto（设备上电、在范围内为运行前置；测试中无需人工动作）

流程：
  1) 扫描匹配 config.TARGET_IDENTITIES 的 ≥2 台目标设备，逐台连接起流采集录制 bin
  2) 对每段 bin 调 getBinFileInfo 校验有效（非 None、有 device_mac），去重 MAC
  3) 对每个有效成员 requireSensor(BLEDevice(name, mac, 0)) 创建虚拟 profile，
     注册 onDataCallback 计数（按 MAC 路由）
  4) multiReplayBinFile(paths, sensors=sensors, realtime=True)
  5) 校验：返回列表长度 == 输入成员数；每个成员返回非 None；每个成员 count > 0

说明：
  SDK 0.9.7 新增 multiReplayBinFile(file_paths, sensors=None, realtime=True, timeout=None)，
  多成员共享对齐时钟（全组最早首条数据为 t=0，保留采集时相对偏移）。
  sensors 参数某项为 None（或整体为 None）时按对应 bin 配置记录自动创建 profile。
  本用例显式创建 profile 以便注册 onDataCallback 校验各成员均产生数据。
  返回列表与 file_paths 对齐，对应成员失败（文件不存在/无配置/MAC 重复/正在传输）为 None。
  本脚本按 config.TARGET_IDENTITIES 连接 ≥2 台目标设备，每台各录制一段 bin（不同 MAC），
  满足 multiReplayBinFile "≥2 个不同 device_mac" 的要求。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：≥2 台 OYWW1100 上电、在范围内（config.TARGET_IDENTITY 配置 ≥2 台）
"""

import os
import sys
import time
import tempfile
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
MULTI_DEVICE_DIR = os.path.join(os.path.dirname(BASE_DIR), "05_MultiDevice")
sys.path.insert(0, AUTOMATION_DIR)
sys.path.insert(0, MULTI_DEVICE_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match_all
from multi_common import check_device_count

MIN_MEMBERS = 2
COLLECT_SECONDS = 5  # 每台设备逐台串行录制时长（秒），足够 count > 0 即可


class MemberCounter:
    """按 MAC 路由的 onDataCallback 计数，记录各成员批数与 DataType 分布。"""

    def __init__(self):
        self.counts = {}     # mac -> int
        self.dts = {}        # mac -> set(dt_name)
        self.lock = threading.Lock()

    def __call__(self, sensor, data):
        mac = sensor.BLEDevice.Address
        items = data if isinstance(data, list) else [data]
        with self.lock:
            self.counts[mac] = self.counts.get(mac, 0) + len(items)
            s = self.dts.setdefault(mac, set())
            for d in items:
                s.add(_dt_name(d))

    def snapshot(self, mac):
        with self.lock:
            return self.counts.get(mac, 0), set(self.dts.get(mac, set()))


def _dt_name(d):
    try:
        dt = d.getDataType()
        return dt.name if isinstance(dt, DataType) else DataType(dt).name
    except Exception:
        return "?"


def _on_error(sensor, reason):
    print(f"[onErrorCallback] {getattr(sensor, 'BLEDevice', None)}: {reason}", flush=True)


def _get_ble_path(sensor):
    try:
        return sensor.getParam("DEBUG_BLE_DATA_PATH")
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def _record_segments(ctrl, matched, collect_seconds, results):
    """逐台连接录制 bin（串行），返回有效 bin 路径列表（每台设备各录一段，时间错开避免同名）。"""
    valid = []
    for identity, device in matched:
        name = getattr(device, 'Name', '?')
        addr = getattr(device, 'Address', '?')
        print(f"\n[录制] {name} ({identity}) {addr} ...", flush=True)

        sensor = ctrl.requireSensor(device)
        if sensor is None:
            record(results, f"requireSensor({identity}) 返回 SensorProfile", False,
                   "返回 SensorProfile", "返回 None")
            continue
        sensor.onErrorCallback = _on_error

        try:
            ok = sensor.connect()
        except Exception:
            ok = None
        if ok is not True:
            record(results, f"connect({identity}) 返回 True", False,
                   "connect() 返回 True", f"connect() -> {ok}")
            continue

        t0 = time.time()
        while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
            time.sleep(0.2)
        if sensor.deviceState != DeviceStateEx.Ready:
            record(results, f"到达 Ready({identity})", False, "deviceState==Ready", f"state={sensor.deviceState}")
            try:
                sensor.disconnect()
            except Exception:
                pass
            continue

        try:
            iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        except Exception:
            iret = None
        if iret is not True:
            record(results, f"init({identity}) 返回 True", False, "init() 返回 True", f"init() -> {iret}")
            try:
                sensor.disconnect()
            except Exception:
                pass
            continue

        try:
            sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
        except Exception:
            pass

        try:
            sret = sensor.startDataNotification()
        except Exception:
            sret = None
        if sret is not True:
            record(results, f"startDataNotification({identity}) 返回 True", False, "返回 True", f"返回 {sret}")
            try:
                sensor.disconnect()
            except Exception:
                pass
            continue

        print(f"[采集] {identity} 等待 {collect_seconds}s ...", flush=True)
        time.sleep(collect_seconds)

        try:
            sensor.stopDataNotification()
        except Exception:
            pass

        p = _get_ble_path(sensor)
        try:
            sensor.disconnect()
        except Exception:
            pass
        if not (isinstance(p, str) and p.strip()):
            p = _get_ble_path(sensor)
        time.sleep(0.5)

        ok = isinstance(p, str) and bool(p.strip()) and os.path.isfile(p)
        print(f"[bin] {identity}: {p!r}（存在={ok}）", flush=True)
        record(results, f"录制 {identity} 生成 bin", ok, "存在可用 bin 文件", f"{p!r}（存在={ok}）")
        if ok:
            valid.append(p)

    return valid


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-FUNC-010 multiReplayBinFile 多文件同步回放", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"[本轮目标设备] {common.TARGET_IDENTITIES}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：≥2 台 OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认 config.TARGET_IDENTITY 已配置 ≥2 台设备、"
          "且这些设备均已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    log_dir = tempfile.mkdtemp(prefix="sdklog_bin_")
    print(f"\n[日志目录] 使用受控目录 {log_dir}", flush=True)
    try:
        ctrl.setLogPath(True, log_dir)
    except Exception as e:
        print(f"[日志目录] setLogPath 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        ctrl.setDebugEnabled(True)
    except Exception as e:
        print(f"[日志目录] setDebugEnabled(True) 抛异常 {type(e).__name__}: {e}", flush=True)

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # ---- 扫描匹配多台目标设备 ----
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    matched, devices = scan_and_match_all(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, required=MIN_MEMBERS)
    print(f"[匹配] 目标设备数: {len(common.TARGET_IDENTITIES)}, 匹配到: {len(matched)}", flush=True)
    if not check_device_count(results, matched, required=MIN_MEMBERS):
        try:
            ctrl.setDebugEnabled(False)
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    # ---- 逐台连接录制 bin（串行，避免同名 bin 冲突） ----
    raw_paths = _record_segments(ctrl, matched, COLLECT_SECONDS, results)

    print(f"\n[输入] 自动录制得到 {len(raw_paths)} 个 bin 路径:", flush=True)
    for p in raw_paths:
        print(f"  {p}", flush=True)

    if len(raw_paths) < MIN_MEMBERS:
        record(results, "自动录制得到 ≥2 个 bin 路径", None,
               f"提供 ≥{MIN_MEMBERS} 个 bin 路径", f"提供 {len(raw_paths)} 个")
        try:
            ctrl.setDebugEnabled(False)
        except Exception:
            pass
        ctrl.terminate()
        print("\n结论: SKIP", flush=True)
        return
    record(results, "自动录制得到 ≥2 个 bin 路径", True,
           f"提供 ≥{MIN_MEMBERS} 个 bin 路径", f"提供 {len(raw_paths)} 个")

    # 校验每个 bin 有效 + 去重 MAC
    members = []       # (path, mac, name, sensor)
    seen_macs = set()
    for path in raw_paths:
        if not os.path.isfile(path):
            print(f"[bin] 文件不存在，跳过: {path!r}", flush=True)
            continue
        try:
            info = ctrl.getBinFileInfo(path)
        except Exception as e:
            info = None
            print(f"[bin] getBinFileInfo 抛异常 {type(e).__name__}: {e} -> {path!r}", flush=True)
        if info is None:
            print(f"[bin] 无配置记录（无效 bin），跳过: {path!r}", flush=True)
            continue
        mac = (info.get("device_mac") or "").strip()
        name = info.get("device_name") or ""
        if not mac:
            print(f"[bin] 配置缺 device_mac，跳过: {path!r}", flush=True)
            continue
        if mac in seen_macs:
            print(f"[bin] 重复 device_mac 跳过: {mac} -> {path!r}", flush=True)
            continue
        seen_macs.add(mac)

        sensor = ctrl.requireSensor(BLEDevice(name, mac, 0))
        if sensor is None:
            print(f"[bin] requireSensor 返回 None，跳过: {mac}", flush=True)
            continue
        sensor.onDataCallback = None  # 由统一 counter 接管
        members.append((path, mac, name, sensor))

    print(f"\n[成员] 有效成员 {len(members)} 个:", flush=True)
    for path, mac, name, _ in members:
        print(f"  {name} {mac} <- {path}", flush=True)

    if len(members) < MIN_MEMBERS:
        record(results, "有效成员数 ≥2", False,
               f"≥{MIN_MEMBERS} 个有效且不同 MAC 的 bin",
               f"有效成员 {len(members)} 个（原始 {len(raw_paths)} 个）")
        try:
            ctrl.setDebugEnabled(False)
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "有效成员数 ≥2", True,
           f"≥{MIN_MEMBERS} 个有效且不同 MAC 的 bin",
           f"有效成员 {len(members)} 个")

    # 注册统一计数回调
    counter = MemberCounter()
    for _, mac, _, sensor in members:
        sensor.onDataCallback = counter
        sensor.onErrorCallback = _on_error

    paths = [p for p, _, _, _ in members]
    sensors = [s for _, _, _, s in members]

    # 多文件回放
    print(f"\n[回放] multiReplayBinFile({len(members)} 个文件, realtime=True) ...", flush=True)
    try:
        results_list = ctrl.multiReplayBinFile(paths, sensors=sensors, realtime=True)
        ret_txt = f"返回列表长度 {len(results_list) if results_list is not None else 'None'}"
    except Exception as e:
        results_list = None
        ret_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[回放] {ret_txt}", flush=True)

    is_list = isinstance(results_list, list)
    record(results, "multiReplayBinFile 返回列表", is_list,
           "返回与 file_paths 对齐的 list", ret_txt)

    if is_list:
        len_ok = len(results_list) == len(members)
        record(results, "返回列表长度 == 成员数", len_ok,
               f"长度 == {len(members)}", f"长度 {len(results_list)}（成员 {len(members)}）")

        all_non_none = all(r is not None for r in results_list)
        record(results, "每个成员返回非 None（成功开始回放）", all_non_none,
               "所有成员非 None",
               f"None 成员 {sum(1 for r in results_list if r is None)}/{len(results_list)}")

    # 各成员产生数据
    print("\n[校验] 各成员回放数据:", flush=True)
    all_have_data = True
    for path, mac, name, _ in members:
        cnt, dts = counter.snapshot(mac)
        print(f"  {name} {mac}: count={cnt} dts={sorted(dts)}", flush=True)
        if cnt <= 0:
            all_have_data = False
    record(results, "每个成员均产生回放数据（count > 0）", all_have_data,
           "所有成员 count > 0", "见上")

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