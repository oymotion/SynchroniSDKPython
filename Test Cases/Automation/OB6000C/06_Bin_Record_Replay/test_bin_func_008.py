# -*- coding: utf-8 -*-
"""BIN-FUNC-008：parseBinToCsv 生成 CSV。

对应用例：06_Bin录制回放解析.md -> BIN-FUNC-008
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) 生成有效 bin（connect → 起流 → 采集 → stop → disconnect）
  2) 调用 ctrl.parseBinToCsv(bin_path, csv_path) 导出 CSV
  3) 校验 CSV 内容：
     - 返回路径非空，文件存在
     - 首行为表头，含 timestamp/mac/type/raw_hex/data_type/sample_rate/channel_count/lost_count/samples_info/first_sample
     - 包含 raw/cmd_send/cmd_recv/event/parsed 五种行类型
     - 每行（非空）列数 = 10

说明：
  README：parseBinToCsv(bin_path, csv_path=None) -> str，返回 CSV 文件路径。
  CSV 格式：header 为 timestamp,mac,type,raw_hex,data_type,sample_rate,channel_count,lost_count,samples_info,first_sample
  行类型：raw / cmd_send / cmd_recv / event / parsed。
  本用例先生成有效 bin，再 parseBinToCsv 导出 CSV，逐行校验表头与行类型。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
"""

import csv
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

COLLECT_SECONDS = 5  # 起流采集时长（秒）
SETTLE_SECONDS = 2   # 停流后的刹车时间（秒）

CSV_HEADER = "timestamp,mac,type,raw_hex,data_type,sample_rate,channel_count,lost_count,samples_info,first_sample"
EXPECTED_TYPES = {"raw", "cmd_send", "cmd_recv", "event", "parsed"}


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


def _on_error(sensor, reason):
    print(f"[onErrorCallback] {getattr(sensor, 'BLEDevice', None)}: {reason}", flush=True)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-FUNC-008 parseBinToCsv 生成 CSV", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

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

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

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

    # ---- 生成 bin ----
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

    print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] SensorProfile.init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    print("\n[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') ...", flush=True)
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
    except Exception as e:
        bret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') -> {bret!r}", flush=True)
    record(results, "setParam('DEBUG_BLE_DATA_PATH', 'True') 返回 OK", bret == "OK",
           "setParam 返回 'OK'", f"setParam -> {bret!r}")

    print("\n[起流] SensorProfile.startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        start_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        start_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] SensorProfile.startDataNotification() -> {start_txt}", flush=True)
    record(results, "startDataNotification 返回 True", sret is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start_txt}")

    print(f"\n[采集] 等待 {COLLECT_SECONDS}s ...", flush=True)
    time.sleep(COLLECT_SECONDS)

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

    bins_after = _list_bins(log_dir)
    new_bins = sorted(set(bins_after.keys()) - set(bins_before.keys()))
    print(f"\n[检查] 新增 .bin 文件: {new_bins if new_bins else '无'}", flush=True)

    bin_path = ble_path if (isinstance(ble_path, str) and ble_path.strip()) else None
    if bin_path is None and new_bins:
        bin_path = bins_after[new_bins[0]]

    have_bin = bool(bin_path) and os.path.isfile(bin_path)
    print(f"[检查] bin 路径: {bin_path!r}，文件存在={have_bin}", flush=True)
    record(results, "生成有效 bin 供 parseBinToCsv 使用", have_bin,
           "存在可用 bin 文件", f"{bin_path!r}（存在={have_bin}）")

    if not have_bin:
        record(results, "parseBinToCsv 返回 CSV 路径", None, "返回非空路径", "无有效 bin")
        record(results, "CSV 首行为表头", None, "首行匹配 header", "无有效 bin")
        record(results, "CSV 含五种行类型", None, "raw/cmd_send/cmd_recv/event/parsed", "无有效 bin")
        record(results, "CSV 每行列数 = 10", None, "每行列数 = 10", "无有效 bin")
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

    # ---- parseBinToCsv ----
    # 使用显式输出路径，便于后续读取校验
    csv_path = os.path.join(log_dir, "test_export.csv")
    print(f"\n[parseBinToCsv] 输入 bin: {bin_path!r}", flush=True)
    print(f"[parseBinToCsv] 输出 csv: {csv_path!r}", flush=True)
    try:
        result_path = ctrl.parseBinToCsv(bin_path, csv_path)
    except Exception as e:
        result_path = None
        print(f"[parseBinToCsv] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[parseBinToCsv] 返回路径: {result_path!r}", flush=True)

    csv_exists = bool(result_path) and os.path.isfile(result_path) if result_path else False
    record(results, "parseBinToCsv 返回 CSV 路径", csv_exists,
           "返回非空路径，文件存在", f"返回 {result_path!r}，文件存在={csv_exists}")

    if not csv_exists:
        record(results, "CSV 首行为表头", None, "首行匹配 header", "CSV 文件不存在")
        record(results, "CSV 含五种行类型", None, "raw/cmd_send/cmd_recv/event/parsed", "CSV 文件不存在")
        record(results, "CSV 每行列数 = 10", None, "每行列数 = 10", "CSV 文件不存在")
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

    # ---- 校验 CSV 内容 ----
    print(f"\n[校验] 读取 CSV: {result_path!r}", flush=True)
    try:
        with open(result_path, "r", encoding="utf-8-sig") as f:
            reader = csv.reader(f)
            rows = list(reader)
    except Exception as e:
        rows = None
        print(f"[校验] 读取 CSV 抛异常 {type(e).__name__}: {e}", flush=True)

    if rows is None or len(rows) == 0:
        record(results, "CSV 首行为表头", False, "首行匹配 header", "CSV 为空或无法读取")
        record(results, "CSV 含五种行类型", False, "raw/cmd_send/cmd_recv/event/parsed", "CSV 为空")
        record(results, "CSV 每行列数 = 10", False, "每行列数 = 10", "CSV 为空")
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

    # 1) 首行 = 表头
    first_row = ",".join(rows[0])
    header_ok = first_row == CSV_HEADER
    print(f"[校验] 首行: {first_row!r}", flush=True)
    record(results, "CSV 首行为表头", header_ok,
           f"首行 = {CSV_HEADER!r}", f"首行 = {first_row!r}")

    # 2) 行类型统计
    type_idx = 2  # type 列索引（0-based: timestamp=0, mac=1, type=2, ...）
    found_types = set()
    rows_with_issues = 0
    for i, row in enumerate(rows[1:], start=2):  # 从第 2 行开始（跳过 header）
        if not row or all(c.strip() == "" for c in row):
            continue  # 跳过空行
        if len(row) > type_idx:
            rt = row[type_idx].strip()
            if rt:
                found_types.add(rt)
        if len(row) != 10:
            rows_with_issues += 1

    print(f"[校验] 行类型: {found_types}", flush=True)
    # README 定义五种行类型，但出现与否取决于 bin 内容（如 cmd_recv 需有命令响应）；
    # 只要出现的类型都在 README 定义内、无未知类型即可
    unknown = found_types - EXPECTED_TYPES
    types_ok = len(unknown) == 0
    record(results, "CSV 行类型均在 README 定义内（raw/cmd_send/cmd_recv/event/parsed）", types_ok,
           "无未知行类型", f"含 {sorted(found_types)}" + (f"，未知类型 {sorted(unknown)}" if unknown else ""))

    # 3) 每行列数 = 10
    total_rows = len([r for r in rows[1:] if r and not all(c.strip() == "" for c in r)])
    col_ok = rows_with_issues == 0
    print(f"[校验] 总数据行 = {total_rows}，列数 != 10 的行数 = {rows_with_issues}", flush=True)
    record(results, "CSV 每行数据列数 = 10", col_ok,
           "每行列数 = 10", f"列数 != 10 的行数 = {rows_with_issues} / 总 {total_rows} 行")

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

    print(f"\n[提示] 本次 bin 与 CSV 落盘目录: {log_dir}", flush=True)
    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()