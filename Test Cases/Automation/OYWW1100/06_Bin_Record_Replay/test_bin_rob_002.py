# -*- coding: utf-8 -*-
"""BIN-ROB-002：multiReplayBinFile 组级 pause/resume/stop 幂等。

对应用例：06_Bin录制回放解析.md -> BIN-ROB-002
可自动化：auto（设备上电、在范围内为运行前置；测试中无需人工动作）

流程：
  1) 扫描匹配 config.TARGET_IDENTITIES 的 ≥2 台目标设备，逐台连接起流采集录制 bin
  2) 对每段 bin 校验有效 + 去重 MAC，requireSensor 创建虚拟 profile 并注册计数回调
  3) 在子线程中 multiReplayBinFile(paths, sensors, realtime=True)
  4) 主线程等待数据流动后，对任一成员 pauseBinReplay -> 观察整组暂停
  5) resumeBinReplay -> 观察整组恢复
  6) stopBinReplay 逐成员停止；停止后重复 stopBinReplay 验证幂等
  7) 每步断言：不崩溃、不抛异常

说明：
  SDK 语义：pause/resume 任一成员暂停/恢复整组；stop 按设备单独生效。
  组回放没有"主"成员，Pause/Stop 对全部成员下发（组时钟语义下幂等）。
  重复控制（重复 stop、重复 pause）不应崩溃。
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
COLLECT_SECONDS = 30      # 每台设备逐台串行录制时长（秒），足够 pause/resume/stop 操作窗口
WAIT_DATA_TIMEOUT = 8     # 等待回放数据流动（秒）
PAUSE_SLEEP = 3           # pause 后观察窗口（秒）
PAUSE_GROWTH_TOLERANCE = 20  # pause 后"刹车滑行"允许的最大批数增长
RESUME_SLEEP = 3          # resume 后观察窗口（秒）
THREAD_JOIN_TIMEOUT = 60  # 等待回放子线程结束（秒）


class MemberCounter:
    """按 MAC 路由的 onDataCallback 计数，线程安全。"""

    def __init__(self):
        self.counts = {}
        self.lock = threading.Lock()

    def __call__(self, sensor, data):
        mac = sensor.BLEDevice.Address
        items = data if isinstance(data, list) else [data]
        with self.lock:
            self.counts[mac] = self.counts.get(mac, 0) + len(items)

    def total(self):
        with self.lock:
            return sum(self.counts.values())

    def per_member(self):
        with self.lock:
            return dict(self.counts)


def _call_ctrl(method, *args):
    try:
        return method(*args), None
    except Exception as e:
        return None, f"抛异常 {type(e).__name__}: {e}"


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
    print("BIN-ROB-002 multiReplayBinFile 组级 pause/resume/stop 幂等", flush=True)
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
            print(f"[bin] 无配置记录，跳过: {path!r}", flush=True)
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
    sensors = []
    for _, mac, _, sensor in members:
        sensor.onDataCallback = counter
        sensor.onErrorCallback = _on_error
        sensors.append(sensor)

    paths = [p for p, _, _, _ in members]
    first_member = sensors[0]

    # 子线程回放
    replay_error = [None]

    def replay_thread():
        try:
            ctrl.multiReplayBinFile(paths, sensors=sensors, realtime=True)
        except Exception as e:
            replay_error[0] = f"{type(e).__name__}: {e}"

    print(f"\n[回放] 子线程启动 multiReplayBinFile({len(members)} 成员, realtime=True) ...", flush=True)
    t = threading.Thread(target=replay_thread, daemon=True)
    t.start()

    # 等待数据流动
    t_wait = time.time()
    before = 0
    while time.time() - t_wait < WAIT_DATA_TIMEOUT:
        before = counter.total()
        if before > 0:
            break
        time.sleep(0.2)
    print(f"[回放] 数据流动批数 = {before}（等待 {time.time() - t_wait:.1f}s）", flush=True)
    record(results, "回放开始产生数据", before > 0,
           "count > 0", f"total={before} err={replay_error[0]}")

    # ---- pause（对任一成员，整组暂停）----
    r, err = _call_ctrl(ctrl.pauseBinReplay, first_member)
    print(f"[pause] pauseBinReplay -> {r!r} err={err}", flush=True)
    record(results, "pauseBinReplay 返回 OK", r == "OK", "返回 'OK'", f"返回 {r!r} err={err}")

    time.sleep(PAUSE_SLEEP)
    during_pause = counter.total()
    paused_growth = during_pause - before
    print(f"[pause] 暂停 {PAUSE_SLEEP}s 后总批数 = {during_pause}（增长 {paused_growth}）", flush=True)
    record(results, "pause 后整组批数停止增长", paused_growth <= PAUSE_GROWTH_TOLERANCE,
           f"增长 <= {PAUSE_GROWTH_TOLERANCE}", f"增长 {paused_growth}")

    # ---- resume（整组恢复）----
    r, err = _call_ctrl(ctrl.resumeBinReplay, first_member)
    print(f"[resume] resumeBinReplay -> {r!r} err={err}", flush=True)
    record(results, "resumeBinReplay 返回 OK", r == "OK", "返回 'OK'", f"返回 {r!r} err={err}")

    time.sleep(RESUME_SLEEP)
    after_resume = counter.total()
    resumed_growth = after_resume - during_pause
    print(f"[resume] resume {RESUME_SLEEP}s 后总批数 = {after_resume}（增长 {resumed_growth}）", flush=True)
    record(results, "resume 后整组批数恢复增长", resumed_growth > 0,
           "增长 > 0", f"增长 {resumed_growth}")

    # ---- stop 逐成员停止 ----
    all_stop_ok = True
    for path, mac, name, sensor in members:
        r, err = _call_ctrl(ctrl.stopBinReplay, sensor)
        print(f"[stop] stopBinReplay({name} {mac}) -> {r!r} err={err}", flush=True)
        if r != "OK":
            all_stop_ok = False
    record(results, "stopBinReplay 逐成员返回 OK", all_stop_ok,
           "全部成员返回 'OK'", "见上")

    # ---- 重复 stop 幂等 ----
    r, err = _call_ctrl(ctrl.stopBinReplay, first_member)
    print(f"[stop] 重复 stopBinReplay -> {r!r} err={err}", flush=True)
    record(results, "重复 stopBinReplay 不崩溃", err is None,
           "不抛异常", f"返回 {r!r} err={err}")

    # 等待子线程结束
    t.join(timeout=THREAD_JOIN_TIMEOUT)
    thread_done = not t.is_alive()
    record(results, "stop 后回放子线程结束", thread_done,
           f"子线程在 {THREAD_JOIN_TIMEOUT}s 内结束",
           f"子线程{'已结束' if thread_done else '仍在运行'}"
           + (f" err={replay_error[0]}" if replay_error[0] else ""))

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
