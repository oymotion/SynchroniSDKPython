# -*- coding: utf-8 -*-
"""Automation test common utilities — shared across all device scripts.

Import this module in test scripts under gForceUltra/ (or future device directories):
    import common
    from common import record, _identity_of, match_target

The module reads config.TARGET_IDENTITY and config.DEVICES from the shared
config.py in the parent Automation/ directory.
"""

import re
import sys
import os
import time
import asyncio
import importlib

# Ensure the Automation directory is on sys.path so we can import config
AUTOMATION_DIR = os.path.dirname(os.path.abspath(__file__))
if AUTOMATION_DIR not in sys.path:
    sys.path.insert(0, AUTOMATION_DIR)

import config


def _parse_target_identities():
    """Parse config.TARGET_IDENTITY (comma-separated) into a list of uppercase identities.
    
    Validates that every identity exists in config.DEVICES. Raises SystemExit if not.
    """
    raw = (config.TARGET_IDENTITY or "").strip()
    if not raw:
        return []
    ids = [s.strip().upper() for s in raw.split(",") if s.strip()]
    
    # 校验：所有目标 identity 必须在 DEVICES 中有定义
    defined = {c.get("identity", "").strip().upper() for c in config.DEVICES}
    undefined = [i for i in ids if i not in defined]
    if undefined:
        print(f"[common] 错误：TARGET_IDENTITY 中的设备未在 DEVICES 中定义: {undefined}", flush=True)
        print(f"[common] DEVICES 中已定义的 identity: {sorted(defined)}", flush=True)
        raise SystemExit(1)
    
    return ids


# 模块加载时解析并校验
TARGET_IDENTITIES = _parse_target_identities()


def record(results, name, ok, expect, actual):
    """Record a test result.
    
    ok: True=PASS, False=FAIL, None=SKIP/informational
    """
    status = "PASS" if ok is True else ("FAIL" if ok is False else "SKIP")
    results.append((name, status, expect, actual))


class _CompatBinInfo(dict):
    """BinFileInfo 的旧 dict 语义视图。

    既支持旧脚本的 .get("device_mac") 等 dict 用法，也支持 SDK 1.3.9 内部
    replayBinFile 对 .Mac / .DeviceName / .DurationSec 的属性访问（见
    sensor_controller.replayBinFile/_start_replay），保证回放引擎不被破坏。
    """

    _ATTR_TO_KEY = {
        "Mac": "device_mac",
        "DeviceName": "device_name",
        "DurationSec": "replay_duration",
        "ConfigVersion": "version",
        "ChipType": "chip_type",
        "IsUniversalStream": "is_universal_stream",
        "IsNewEmg": "is_new_emg",
        "IsContainQat6": "is_contain_qat6",
        "PpgModel": "ppg_model",
        "FeatureMap": "feature_map",
        "NotifyDataFlag": "notify_data_flag",
        "DeviceInfo": "device_info",
        "SensorDatas": "sensor_datas",
        "Valid": "valid",
        "FirstDataTsMs": "first_data_ts_ms",
        "LastDataTsMs": "last_data_ts_ms",
    }

    def __getattr__(self, name):
        key = self._ATTR_TO_KEY.get(name, name)
        try:
            return self[key]
        except KeyError:
            raise AttributeError(name)

    def __setattr__(self, name, value):
        self[self._ATTR_TO_KEY.get(name, name)] = value


def bin_info_to_dict(info):
    """把 SDK 1.3.9+ 的 BinFileInfo 对象转回旧版 getBinFileInfo 返回的 dict 结构。

    背景：SDK 1.3.9 将 SensorController.getBinFileInfo 返回类型从 dict 改为
    BinFileInfo 对象（属性 PascalCase）。旧脚本用 .get("device_mac") 等 dict 语义，
    升级后会 AttributeError。此函数做兼容转换，使旧脚本无需改动即可继续用 .get()。

    参数：
        info : getBinFileInfo 返回值（可能是 BinFileInfo / dict / None）

    返回：
        - None             : info 为 None
        - _CompatBinInfo   : 兼容旧版键结构（device_mac/device_name/replay_duration/
                             version/chip_type/is_universal_stream/feature_map/
                             device_info/sensor_datas/...），同时支持 .Mac 等属性访问
        - 原样 dict        : info 已是 dict 时原样返回（幂等）
    """
    if info is None:
        return None
    if isinstance(info, dict):
        return info

    def _obj_to_dict(obj):
        if obj is None:
            return None
        if isinstance(obj, dict):
            return obj
        if isinstance(obj, (list, tuple)):
            return [_obj_to_dict(x) for x in obj]
        d = getattr(obj, "__dict__", None)
        if d is not None:
            return {k: _obj_to_dict(v) for k, v in d.items() if not k.startswith("_")}
        return obj

    device_info = _obj_to_dict(getattr(info, "DeviceInfo", None))
    sensor_datas = _obj_to_dict(getattr(info, "SensorDatas", None)) or []

    return _CompatBinInfo({
        "device_mac": getattr(info, "Mac", ""),
        "device_name": getattr(info, "DeviceName", ""),
        "replay_duration": getattr(info, "DurationSec", 0.0),
        "version": getattr(info, "ConfigVersion", 0),
        "chip_type": getattr(info, "ChipType", 0),
        "is_universal_stream": getattr(info, "IsUniversalStream", False),
        "is_new_emg": getattr(info, "IsNewEmg", False),
        "is_contain_qat6": getattr(info, "IsContainQat6", False),
        "ppg_model": getattr(info, "PpgModel", 0),
        "feature_map": getattr(info, "FeatureMap", 0),
        "notify_data_flag": getattr(info, "NotifyDataFlag", 0),
        "device_info": device_info,
        "sensor_datas": sensor_datas,
        "valid": getattr(info, "Valid", False),
        "first_data_ts_ms": getattr(info, "FirstDataTsMs", 0),
        "last_data_ts_ms": getattr(info, "LastDataTsMs", 0),
    })


def setup_sanity_log(base_dir, script_name=None):
    """把脚本 stdout tee 到 SanityLogs/<script>_<timestamp>.log，同时保留终端输出。

    用途：让「长稳 / 压力 / 采样率 / e2e」这类脚本的 print 输出落盘到固定目录，
    跑完能回看「到底跑没跑、结果如何」。仅落盘脚本自身的 print 输出，不改动
    SDK 的 setLogPath 日志目录。

    参数：
        base_dir     : 脚本所在目录（SanityLogs 建在它下面）
        script_name  : 日志文件名前缀；默认取 sys.argv[0] 的 basename（去 .py）

    返回：日志文件的绝对路径；创建失败返回 None（stdout 保持原样）。
    """
    log_dir = os.path.join(base_dir, "SanityLogs")
    try:
        os.makedirs(log_dir, exist_ok=True)
    except OSError:
        return None
    if not script_name:
        script_name = os.path.splitext(os.path.basename(sys.argv[0]))[0] or "script"
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(log_dir, f"{script_name}_{ts}.log")

    try:
        fh = open(log_path, "w", encoding="utf-8")
    except OSError:
        return None

    class _TeeStream:
        def __init__(self, *streams):
            self.streams = streams

        def write(self, data):
            for s in self.streams:
                try:
                    s.write(data)
                except Exception:
                    pass

        def flush(self):
            for s in self.streams:
                try:
                    s.flush()
                except Exception:
                    pass

    sys.stdout = _TeeStream(sys.stdout, fh)
    return log_path


def resolve_target_identity():
    """解析命令行参数指定的目标 identity（逗号分隔，覆盖 config.TARGET_IDENTITY）。

    用法：python <script>.py [identity[,identity...]]
      - 提供参数：作为目标 identity 列表（覆盖 config.TARGET_IDENTITY）
      - 未提供：返回 None，走 config 默认（TARGET_IDENTITIES）

    每个 identity 都必须在 config.DEVICES 中定义，否则报错退出。
    """
    if len(sys.argv) <= 1:
        return None
    raw = sys.argv[1].strip()
    if not raw:
        print("[common] 错误：命令行参数 identity 为空", flush=True)
        raise SystemExit(1)
    ids = [s.strip().upper() for s in raw.split(",") if s.strip()]
    if not ids:
        print("[common] 错误：命令行参数 identity 为空", flush=True)
        raise SystemExit(1)
    defined = {(c.get("identity") or "").strip().upper() for c in config.DEVICES}
    undefined = [i for i in ids if i not in defined]
    if undefined:
        print(f"[common] 错误：命令行指定的 identity {undefined} 未在 config.DEVICES 中定义", flush=True)
        print(f"[common] DEVICES 中已定义的 identity: {sorted(defined)}", flush=True)
        raise SystemExit(1)
    return ids


def _identity_of(name):
    """Extract 4-hex identity from broadcast name like 'gForceUltra(80F3)' -> '80F3'."""
    m = re.search(r"\(([0-9A-Fa-f]{4})\)", name or "")
    return m.group(1).upper() if m else None


def _find_config(identity):
    """Find the config.DEVICES entry for a given identity."""
    for c in config.DEVICES:
        if (c.get("identity") or "").strip().upper() == identity:
            return c
    return None


def load_spec(name_prefix):
    """按 name_prefix 读取设备规格（Spec）。

    config.MODEL_SPEC 映射 name_prefix -> device_specs 目录下的 spec 文件名（不含 .py）。
    返回该 spec 模块的 SPEC 字典（单一事实来源，测试据此硬断言）。

    抛 KeyError：name_prefix 未在 config.MODEL_SPEC 中注册。
    抛 ImportError：对应 spec 模块不存在或 device_specs 包不可导入。
    """
    filename = config.MODEL_SPEC.get(name_prefix)
    if filename is None:
        raise KeyError(
            f"config.MODEL_SPEC 中未定义 name_prefix={name_prefix!r} 的规格映射"
        )
    module = importlib.import_module(f"device_specs.{filename}")
    return module.SPEC


def spec_for_identity(identity):
    """由设备 identity 解析其 name_prefix 并加载对应规格。"""
    cfg = _find_config(identity)
    if cfg is None:
        raise KeyError(f"identity {identity!r} 未在 config.DEVICES 中定义")
    return load_spec(cfg.get("name_prefix"))


def match_target(devices, target_identity=None):
    """Match the first target device from scan results.
    
    Uses config.TARGET_IDENTITIES (parsed from comma-separated config.TARGET_IDENTITY)
    by default. Pass an explicit target_identity to override.
    
    Returns the first matching BLEDevice, or None if no match.
    """
    if target_identity is not None:
        if isinstance(target_identity, str):
            targets = [target_identity.strip().upper()]
        else:
            targets = [t.strip().upper() for t in target_identity if (t or "").strip()]
    else:
        targets = TARGET_IDENTITIES if isinstance(TARGET_IDENTITIES, list) else [TARGET_IDENTITIES]
    
    for tid in targets:
        cfg = _find_config(tid)
        if cfg is None:
            continue
        mac = (cfg.get("mac") or "").strip().upper()
        for d in (devices or []):
            addr = (getattr(d, 'Address', '') or '').upper()
            name = getattr(d, 'Name', '') or ''
            if mac and addr == mac:
                return d
            if _identity_of(name) == tid:
                return d
    return None


def match_all_targets(devices):
    """匹配所有目标设备（按 config.TARGET_IDENTITIES），返回 [(identity, BLEDevice)] 列表（去重）。

    匹配优先级与 match_target 一致：mac 精确 > identity（广播名后四位）。
    """
    matched = []
    seen = set()
    for tid in TARGET_IDENTITIES:
        cfg = _find_config(tid)
        mac = (cfg.get("mac") or "").strip().upper() if cfg else ""
        for d in (devices or []):
            addr = (getattr(d, 'Address', '') or '').upper()
            name = getattr(d, 'Name', '') or ''
            key = None
            if mac and addr == mac:
                key = addr
            elif _identity_of(name) == tid:
                key = addr or name
            if key and key not in seen:
                seen.add(key)
                matched.append((tid, d))
                break
    return matched


def _log_devices(devices):
    """打印扫描到的设备列表，便于定位匹配失败原因。"""
    if not devices:
        print("[common] 扫描到 0 台设备", flush=True)
        return
    print(f"[common] 扫描到 {len(devices)} 台设备:", flush=True)
    for d in devices:
        n = getattr(d, 'Name', '?')
        a = getattr(d, 'Address', '?')
        print(f"    {n} {a} identity={_identity_of(n)}", flush=True)


def scan_and_match(ctrl, scan_ms=None, target_identity=None, retries=3, interval=10):
    """扫描并匹配目标设备；未匹配到时等待 interval 秒后重新扫描，最多尝试 retries 次。

    BLE 设备在短时间内可能不会被重复发现（广播/扫描缓存），连续运行多个用例时，
    下一条用例的首次 scan 往往扫不到目标。此函数在 scan 未匹配到目标时自动重试。

    参数：
        ctrl            : SensorControllerInstance
        scan_ms         : scan 超时毫秒；默认取 config.SCAN_TIMEOUT_MS
        target_identity : 目标 identity（单个），默认 None 表示取 config.TARGET_IDENTITIES
        retries         : 最多尝试次数（含首次），默认 3
        interval        : 两次尝试之间的等待秒数，默认 10

    返回 (target, devices)：
        target  : 匹配到的 BLEDevice，未匹配到为 None
        devices : 最后一次 scan 返回的设备列表（可能为 None）
    """
    if scan_ms is None:
        scan_ms = config.SCAN_TIMEOUT_MS
    devices = None
    for attempt in range(1, retries + 1):
        try:
            devices = ctrl.scan(scan_ms)
        except Exception as e:
            devices = None
            print(f"[common] scan 抛异常（第 {attempt}/{retries} 次）: {type(e).__name__}: {e}", flush=True)
        target = match_target(devices, target_identity=target_identity)
        if target is not None:
            return target, devices
        if attempt < retries:
            print(f"[common] 未匹配到目标设备，{interval}s 后重试（{attempt}/{retries}）...", flush=True)
            time.sleep(interval)
    return None, devices


async def async_scan_and_match(ctrl, scan_ms=None, target_identity=None, retries=3, interval=10):
    """异步版扫描并匹配目标设备（在 asyncio 事件循环内使用）。

    与 scan_and_match 行为一致，但 scan 用 await ctrl.asyncScan(...)，等待用 asyncio.sleep。
    供 09_Async_Interface 等运行在 asyncio.run() 协程内的脚本使用。

    返回 (target, devices)。
    """
    if scan_ms is None:
        scan_ms = config.SCAN_TIMEOUT_MS
    devices = None
    for attempt in range(1, retries + 1):
        try:
            devices = await ctrl.asyncScan(scan_ms)
        except Exception as e:
            devices = None
            print(f"[common] asyncScan 抛异常（第 {attempt}/{retries} 次）: {type(e).__name__}: {e}", flush=True)
        target = match_target(devices, target_identity=target_identity)
        if target is not None:
            return target, devices
        if attempt < retries:
            print(f"[common] 未匹配到目标设备，{interval}s 后重试（{attempt}/{retries}）...", flush=True)
            await asyncio.sleep(interval)
    return None, devices


def scan_and_match_all(ctrl, scan_ms=None, required=None, retries=3, interval=10):
    """扫描并匹配所有目标设备；未匹配齐时等待 interval 秒后重扫，最多尝试 retries 次。

    参数：
        ctrl     : SensorControllerInstance
        scan_ms  : scan 超时毫秒；默认取 config.SCAN_TIMEOUT_MS
        required : 至少需匹配到的设备数；默认 None 表示需匹配齐全部 TARGET_IDENTITIES
        retries  : 最多尝试次数（含首次），默认 3
        interval : 两次尝试之间的等待秒数，默认 10

    返回 (matched, devices)：
        matched : [(identity, BLEDevice)] 列表
        devices : 最后一次 scan 返回的设备列表（可能为 None）
    """
    if scan_ms is None:
        scan_ms = config.SCAN_TIMEOUT_MS
    if required is None:
        required = len(TARGET_IDENTITIES)
    devices = None
    matched = []
    for attempt in range(1, retries + 1):
        try:
            devices = ctrl.scan(scan_ms)
        except Exception as e:
            devices = None
            print(f"[common] scan 抛异常（第 {attempt}/{retries} 次）: {type(e).__name__}: {e}", flush=True)
        _log_devices(devices)
        matched = match_all_targets(devices)
        print(f"[common] 匹配到 {len(matched)} 台目标设备", flush=True)
        if len(matched) >= required:
            return matched, devices
        if attempt < retries:
            print(f"[common] 未匹配齐目标设备（{len(matched)}/{required}），{interval}s 后重试（{attempt}/{retries}）...", flush=True)
            time.sleep(interval)
    return matched, devices


async def async_scan_and_match_all(ctrl, scan_ms=None, required=None, retries=3, interval=10):
    """异步版扫描并匹配所有目标设备（在 asyncio 事件循环内使用）。

    与 scan_and_match_all 行为一致，但 scan 用 await ctrl.asyncScan(...)，等待用 asyncio.sleep。
    返回 (matched, devices)。
    """
    if scan_ms is None:
        scan_ms = config.SCAN_TIMEOUT_MS
    if required is None:
        required = len(TARGET_IDENTITIES)
    devices = None
    matched = []
    for attempt in range(1, retries + 1):
        try:
            devices = await ctrl.asyncScan(scan_ms)
        except Exception as e:
            devices = None
            print(f"[common] asyncScan 抛异常（第 {attempt}/{retries} 次）: {type(e).__name__}: {e}", flush=True)
        _log_devices(devices)
        matched = match_all_targets(devices)
        print(f"[common] 匹配到 {len(matched)} 台目标设备", flush=True)
        if len(matched) >= required:
            return matched, devices
        if attempt < retries:
            print(f"[common] 未匹配齐目标设备（{len(matched)}/{required}），{interval}s 后重试（{attempt}/{retries}）...", flush=True)
            await asyncio.sleep(interval)
    return matched, devices


def _install_bin_info_compat_shim():
    """安装 getBinFileInfo 兼容 shim（SDK 1.3.9 dict -> BinFileInfo 破坏性变更）。

    在 import common 的测试进程里，把 SensorController.getBinFileInfo 包装一层，
    使其返回 bin_info_to_dict 的结果（_CompatBinInfo，dict 子类）：
      - 旧脚本的 isinstance(info, dict) 与 info.get("device_mac") 恢复可用；
      - 同时保留 .Mac/.DeviceName/.DurationSec 属性访问，replayBinFile 内部仍正常。

    幂等：重复 import 不会重复包装（只包裹一次）。
    """
    try:
        import sensor.sensor_controller as _sc

        _orig = _sc.SensorController.getBinFileInfo
        if getattr(_orig, "_sdk139_compat_wrapped", False):
            return

        def _wrapped(self, file_path):
            return bin_info_to_dict(_orig(self, file_path))

        _wrapped._sdk139_compat_wrapped = True
        _sc.SensorController.getBinFileInfo = _wrapped
    except Exception:
        # sensor 未安装或接口变化时静默降级，不影响 common 其他功能
        pass


_install_bin_info_compat_shim()