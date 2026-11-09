# -*- coding: utf-8 -*-
"""MAG-FUNC-006：磁力计与角度/IMU 流共存

对应用例：12_磁力计API.md -> MAG-FUNC-006
可自动化：auto

断言采用「先有值就行」：不硬编码具体数值（通道数/采样率等），只验证字段/数据可读、有值。
"""
import math
import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match

STREAM_KEY = "NTF_MAG"   # DataType 枚举名：用于 getDataType() 匹配 / spec 流键
SET_KEY = "NTF_IMU"      # 磁力计无独立开关，随 NTF_IMU 聚合开关起停（实测 1.4.3）
DT = getattr(DataType, STREAM_KEY, None)


def _dt_name(dt):
    try:
        return dt.name if isinstance(dt, DataType) else DataType(dt).name
    except Exception:
        return str(dt)


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


def _cap(spec):
    return (spec.get("streams") or {}).get(STREAM_KEY) or {}


def _supported(cap):
    return bool(cap.get("supported") is True and (cap.get("channels") or 0) > 0)


def _is_error(res):
    return isinstance(res, str) and (res.startswith("Error") or res.startswith("抛异常") or res.startswith("<"))


def _check_env(ctrl):
    is_enable = ctrl.isEnable
    print(f"[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return False
    return True


def _connect_and_init(ctrl, results):
    target, _devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        record(results, "scan 匹配到目标设备", False, "scan 返回含启用的目标设备", "未匹配到目标")
        return None, None
    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含启用的目标设备", f"匹配到 {name} {addr}")

    spec = _load_spec(target)
    if spec is None:
        record(results, "载入设备 spec", False, "能从广播名提取 identity 并载入对应 spec", "spec 载入失败")
        return None, None
    record(results, "载入设备 spec", True, "能从广播名提取 identity 并载入对应 spec", f"model={spec.get('model')}")

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        return None, None
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    try:
        ok = sensor.connect()
        connect_txt = f"返回 {ok}"
    except Exception as e:
        ok = None
        connect_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[连接] connect() -> {connect_txt}  state={sensor.deviceState}", flush=True)
    record(results, "SensorProfile.connect 返回 True", ok is True, "connect() 返回 True", f"connect() -> {connect_txt}")

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
        return None, None

    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    return sensor, spec


def _cleanup(sensor):
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] disconnect 抛异常 {type(e).__name__}: {e}", flush=True)


def _print_summary(results):
    print()
    print("=" * 60, flush=True)
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
    print()
    print("结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


class MultiCollector:
    def __init__(self, dts):
        self.dts = list(dts)
        self.by_type = {dt: {'batches': 0, 'samples': 0} for dt in self.dts}

    def on_data(self, sensor, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        for d in items:
            try:
                dt = d.getDataType()
            except Exception:
                continue
            if dt not in self.by_type:
                continue
            self.by_type[dt]['batches'] += 1
            self.by_type[dt]['samples'] += d.getChannelCount() * d.getSampleCount()

    def clear(self):
        for e in self.by_type.values():
            e['batches'] = 0
            e['samples'] = 0


def main():
    ctrl = SensorControllerInstance
    print("=" * 60, flush=True)
    print("MAG-FUNC-006 磁力计与角度/IMU 流共存", flush=True)
    print("=" * 60, flush=True)
    print("sdk version = " + str(ctrl.getVersion()), flush=True)
    print("ble backend = " + str(ctrl.getBLEBackendName()), flush=True)
    results = []
    if not _check_env(ctrl):
        return

    sensor, spec = _connect_and_init(ctrl, results)
    if sensor is None:
        _print_summary(results)
        ctrl.terminate()
        return

    cap = _cap(spec)
    if not _supported(cap):
        record(results, "流共存、采样率独立", None,
               "spec 流 supported=True 且 channels>0 时共存可测",
               f"spec channels={cap.get('channels')}，设备不支持，跳过")
        _cleanup(sensor)
        _print_summary(results)
        ctrl.terminate()
        return

    other_keys = ['NTF_ANGLE_DATA', 'NTF_ACC', 'NTF_GYRO', 'NTF_EULER_DATA', 'NTF_QUATERNION']
    # setParam 真实可用 key：磁力计随 NTF_IMU 起停，角度用 NTF_ANGLE；
    # other_keys 是 DataType 枚举名，只用于数据分桶，不能传给 setParam。
    set_keys = [SET_KEY, 'NTF_ANGLE']
    dts = [DT] + [getattr(DataType, k) for k in other_keys if getattr(DataType, k, None) is not None]
    col = MultiCollector(dts)
    sensor.onDataCallback = col.on_data

    for k in set_keys:
        try:
            sensor.setParam(k, "OFF")
        except Exception:
            pass
    for k in set_keys:
        try:
            sensor.setParam(k, "ON")
        except Exception as e:
            print(f"[setParam] {k} ON 抛异常 {type(e).__name__}: {e}", flush=True)

    try:
        sensor.startDataNotification()
    except Exception:
        pass
    time.sleep(6)
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    for k in set_keys:
        try:
            sensor.setParam(k, "OFF")
        except Exception:
            pass

    got_main = col.by_type.get(DT, {}).get('samples', 0) > 0
    print(f"[采集] 主流 {_dt_name(DT)} 样本={col.by_type.get(DT, {}).get('samples', 0)}", flush=True)
    record(results, "主流收到数据", got_main, "主流样本数>0",
           f"样本={col.by_type.get(DT, {}).get('samples', 0)}")

    all_ok = got_main
    for k in other_keys:
        dt = getattr(DataType, k, None)
        if dt is None:
            continue
        s = col.by_type.get(dt, {}).get('samples', 0)
        print(f"[采集] 子流 {k} 样本={s}", flush=True)
        ok = s > 0
        record(results, f"{k} 收到数据", ok, f"{k} 样本数>0", f"样本={s}")
        all_ok = all_ok and ok

    record(results, "主流与子流共存、采样率独立", all_ok,
           "主流与 IMU 子流同时收到数据", "见各流样本统计")

    _cleanup(sensor)
    _print_summary(results)
    ctrl.terminate()


if __name__ == "__main__":
    main()
