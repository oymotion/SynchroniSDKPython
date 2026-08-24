# -*- coding: utf-8 -*-
"""Automation test common utilities — shared across all device scripts.

Import this module in test scripts under OYWW1100/ (or future device directories):
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


def _identity_of(name):
    """Extract 4-hex identity from broadcast name like 'OYWW1100(80F3)' -> '80F3'."""
    m = re.search(r"\(([0-9A-Fa-f]{4})\)", name or "")
    return m.group(1).upper() if m else None


def _find_config(identity):
    """Find the config.DEVICES entry for a given identity."""
    for c in config.DEVICES:
        if (c.get("identity") or "").strip().upper() == identity:
            return c
    return None


def match_target(devices, target_identity=None):
    """Match the first target device from scan results.
    
    Uses config.TARGET_IDENTITIES (parsed from comma-separated config.TARGET_IDENTITY)
    by default. Pass an explicit target_identity to override.
    
    Returns the first matching BLEDevice, or None if no match.
    """
    if target_identity is not None:
        targets = [target_identity.strip().upper()]
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