# -*- coding: utf-8 -*-
"""回放子进程 worker：在独立进程里用 fresh controller + fresh profile 做离线回放。

为什么必须在独立子进程里回放：
  - SensorController 是进程内单例（sensor_controller.__new__ 始终返回 _instance）。
  - requireSensor / getSensor 会按 native handle 缓存 SensorProfile，同一 MAC 在进程内
    拿到同一个 Python SensorProfile。
  - 已经 connect / startDataNotification 过的 profile 传给原生回放引擎时，onData
    回调不会被泵入数据（结果 0 批 / 0 样本），随后 disconnect 还会触发 native
    access violation（读取 0x40）。
  - 因此回放必须放到「全新进程」里跑（fresh controller + fresh profile）。

本 worker 不依赖蓝牙硬件，直接读 bin 里的 device_name / device_mac 创建全新 profile。

用法：
  python replay_worker.py --bin <bin 文件> --mode <summary|samples|control|idempotency> [--realtime true|false]

输出：单行 JSON 到 stdout（结果契约见各 run_* 函数）。
"""

import json
import sys
import threading
import time

from sensor import *


def _dt_name(d):
    try:
        dt = d.getDataType()
        return dt.name if isinstance(dt, DataType) else DataType(dt).name
    except Exception:
        return "?"


def _call_ctrl(method, *args):
    """调用 controller 控制方法，返回 (返回值, 异常信息)。"""
    try:
        return method(*args), None
    except Exception as e:
        return None, f"抛异常 {type(e).__name__}: {e}"


def _make_sensor(controller, bin_path):
    """从 bin 元数据创建全新 SensorProfile（本进程从未连接/起流）。"""
    info = controller.getBinFileInfo(bin_path)
    if info is None:
        return None, None, "getBinFileInfo 返回 None（无 config record 或文件无效）"
    sensor = controller.requireSensor(
        BLEDevice(info.get("device_name") or "", info.get("device_mac") or "", 0)
    )
    if sensor is None:
        return None, info, "requireSensor 返回 None（无法创建回放 profile）"
    return sensor, info, None


def run_summary(bin_path, realtime):
    """回放并统计：批数、per-DataType 分布、首/末 startTimeStamp 与首个 delay。"""
    controller = SensorControllerInstance
    sensor, info, err = _make_sensor(controller, bin_path)
    if sensor is None:
        return {"ok": False, "error": err}

    state = {
        "batches": 0,
        "dt_counts": {},
        "first_ts": None,
        "first_delay": None,
        "last_ts": None,
        "last_delay": None,
    }

    def on_data(profile, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        for d in items:
            state["batches"] += 1
            dt = _dt_name(d)
            state["dt_counts"][dt] = state["dt_counts"].get(dt, 0) + 1
            ts = d.getStartTimeStamp()
            if ts:
                if state["first_ts"] is None:
                    state["first_ts"] = ts
                    state["first_delay"] = d.getDelay()
                state["last_ts"] = ts
                state["last_delay"] = d.getDelay()

    sensor.onDataCallback = on_data
    try:
        start = time.time()
        profile = controller.replayBinFile(bin_path, sensor, realtime=realtime)
        elapsed = time.time() - start
        if profile is None:
            return {"ok": False, "error": "replayBinFile 返回 None", "elapsed_sec": elapsed, **state}
        return {"ok": True, "elapsed_sec": elapsed, **state}
    except Exception as e:
        return {"ok": False, "error": f"{type(e).__name__}: {e}", **state}
    finally:
        controller.terminate()


def run_samples(bin_path):
    """回放并抽取 EMG 各通道样本值（物理值 data），供离线频谱分析。"""
    controller = SensorControllerInstance
    sensor, info, err = _make_sensor(controller, bin_path)
    if sensor is None:
        return {"ok": False, "error": err}

    state = {"fs": None, "channels": {}}

    def on_data(profile, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        for d in items:
            name = _dt_name(d)
            if "EMG" not in name.upper():
                continue
            if state["fs"] is None:
                state["fs"] = d.getSampleRate()
            n_ch = d.getChannelCount()
            n_smp = d.getSampleCount()
            for ci in range(n_ch):
                buf = state["channels"].setdefault(ci, [])
                for si in range(n_smp):
                    try:
                        s = d.getChannelSample(ci, si)
                        v = s.data
                        if v is None:
                            v = s.rawData
                        if v is not None:
                            buf.append(float(v))
                    except Exception:
                        continue

    sensor.onDataCallback = on_data
    try:
        profile = controller.replayBinFile(bin_path, sensor, realtime=False, timeout=120)
        if profile is None:
            return {"ok": False, "error": "replayBinFile 返回 None", "fs": state["fs"], "channels": {}}
        return {
            "ok": True,
            "fs": state["fs"] or 500.0,
            "channels": {str(k): v for k, v in state["channels"].items()},
        }
    except Exception as e:
        return {"ok": False, "error": f"{type(e).__name__}: {e}", "fs": state["fs"], "channels": {}}
    finally:
        controller.terminate()


def run_control(bin_path):
    """回放控制（pause/resume/stop）冒烟：realtime=True 回放，主线程做控制并统计批数增减。"""
    controller = SensorControllerInstance
    sensor, info, err = _make_sensor(controller, bin_path)
    if sensor is None:
        return {"ok": False, "error": err}

    counter = {"count": 0}
    lock = threading.Lock()

    def on_data(profile, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        with lock:
            counter["count"] += len(items)

    sensor.onDataCallback = on_data
    replay_error = [None]

    def replay_thread():
        try:
            controller.replayBinFile(bin_path, sensor, realtime=True)
        except Exception as e:
            replay_error[0] = f"{type(e).__name__}: {e}"

    try:
        t = threading.Thread(target=replay_thread, daemon=True)
        t.start()

        t_wait = time.time()
        before_pause = 0
        while time.time() - t_wait < 5:
            with lock:
                before_pause = counter["count"]
            if before_pause > 0:
                break
            time.sleep(0.2)

        pr, perr = _call_ctrl(controller.pauseBinReplay, sensor)
        time.sleep(3)
        with lock:
            during_pause = counter["count"]

        rr, rerr = _call_ctrl(controller.resumeBinReplay, sensor)
        time.sleep(3)
        with lock:
            after_resume = counter["count"]

        sr, serr = _call_ctrl(controller.stopBinReplay, sensor)
        t.join(timeout=60)
        thread_done = not t.is_alive()

        return {
            "ok": True,
            "before_pause": before_pause,
            "pause_return": pr, "pause_error": perr,
            "paused_growth": during_pause - before_pause,
            "resume_return": rr, "resume_error": rerr,
            "resumed_growth": after_resume - during_pause,
            "stop_return": sr, "stop_error": serr,
            "thread_done": thread_done,
            "replay_error": replay_error[0],
        }
    except Exception as e:
        return {"ok": False, "error": f"{type(e).__name__}: {e}"}
    finally:
        controller.terminate()


def run_idempotency(bin_path):
    """回放控制幂等：未回放时/回放中重复 pause/resume/stop，均不崩溃不抛异常。"""
    controller = SensorControllerInstance
    sensor, info, err = _make_sensor(controller, bin_path)
    if sensor is None:
        return {"ok": False, "error": err}

    counter = {"count": 0}
    lock = threading.Lock()

    def on_data(profile, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        with lock:
            counter["count"] += len(items)

    sensor.onDataCallback = on_data
    replay_error = [None]

    def replay_thread():
        try:
            controller.replayBinFile(bin_path, sensor, realtime=True)
        except Exception as e:
            replay_error[0] = f"{type(e).__name__}: {e}"

    try:
        # 1) 未回放时依次调用，不崩溃
        r1, e1 = _call_ctrl(controller.pauseBinReplay, sensor)
        r2, e2 = _call_ctrl(controller.resumeBinReplay, sensor)
        r3, e3 = _call_ctrl(controller.stopBinReplay, sensor)
        no_replay_crash = all(e is None for e in (e1, e2, e3))

        # 2) 回放（realtime=True），等待数据流动
        t = threading.Thread(target=replay_thread, daemon=True)
        t.start()
        t_wait = time.time()
        flowed = False
        while time.time() - t_wait < 5:
            with lock:
                if counter["count"] > 0:
                    flowed = True
                    break
            time.sleep(0.2)

        # 3) 回放中重复 pause 两次
        p1, pe1 = _call_ctrl(controller.pauseBinReplay, sensor)
        p2, pe2 = _call_ctrl(controller.pauseBinReplay, sensor)
        time.sleep(3)

        # 4) 回放中重复 resume 两次
        s1, se1 = _call_ctrl(controller.resumeBinReplay, sensor)
        s2, se2 = _call_ctrl(controller.resumeBinReplay, sensor)
        time.sleep(3)

        # 5) stop 两次
        st1, ste1 = _call_ctrl(controller.stopBinReplay, sensor)
        st2, ste2 = _call_ctrl(controller.stopBinReplay, sensor)

        t.join(timeout=60)
        thread_done = not t.is_alive()

        return {
            "ok": True,
            "no_replay_crash": no_replay_crash,
            "no_replay_errs": [e1, e2, e3],
            "flowed": flowed,
            "pause_crash_free": (pe1 is None and pe2 is None),
            "pause_errs": [pe1, pe2],
            "resume_crash_free": (se1 is None and se2 is None),
            "resume_errs": [se1, se2],
            "stop_crash_free": (ste1 is None and ste2 is None),
            "stop_errs": [ste1, ste2],
            "thread_done": thread_done,
            "replay_error": replay_error[0],
        }
    except Exception as e:
        return {"ok": False, "error": f"{type(e).__name__}: {e}"}
    finally:
        controller.terminate()


def main():
    args = sys.argv[1:]
    if len(args) < 4 or args[0] != "--bin" or args[2] != "--mode":
        print(json.dumps({"ok": False, "error": "usage: replay_worker.py --bin <bin> --mode <mode> [--realtime true|false]"}))
        return 1

    bin_path = args[1]
    mode = args[3]
    realtime = False
    if "--realtime" in args:
        idx = args.index("--realtime")
        if idx + 1 < len(args):
            realtime = (args[idx + 1].lower() == "true")

    if mode == "summary":
        result = run_summary(bin_path, realtime)
    elif mode == "samples":
        result = run_samples(bin_path)
    elif mode == "control":
        result = run_control(bin_path)
    elif mode == "idempotency":
        result = run_idempotency(bin_path)
    else:
        result = {"ok": False, "error": f"unknown mode: {mode}"}

    print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
