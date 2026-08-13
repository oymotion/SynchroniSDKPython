"""Real-machine test for multiStartDataNotification / multiStopDataNotification.

Scans for two OB6000 devices, connects + inits both, starts streaming with
controller.multiStartDataNotification, then reports per-device:
  - SensorData.getStartTimeStamp() (32-bit ms, bin stream-start record ts)
  - SensorData.getDelay() (first raw packet bin record ts - startTimeStamp, ms)
and the cross-device startTimeStamp spread (gate alignment quality).

Each session's bin is exported (DEBUG_BLE_DATA_PATH=True); after multiStop +
disconnect the exported bins are replayed and the restored startTimeStamp/delay
are compared with the live values (they must be identical).
"""

import sys
import time

from sensor import *
from sensor.bin_recorder import (
    BIN_RECORD_CMD_SEND,
    BIN_RECORD_DATA,
    BIN_RECORD_EVENT,
    iter_bin_records_precise,
)
from sensor.gforce import Command

SCAN_DEVICE_PERIOD_IN_MS = 3000
PACKAGE_COUNT = 5
POWER_REFRESH_PERIOD_IN_MS = 5000
TARGET_COUNT = 2
# 设备名前缀可用 --prefix=XXX 覆盖（默认 OB6000，OYM 设备如 OYWW1000 用 --prefix=OYWW）
NAME_PREFIX = next((a.split("=", 1)[1] for a in sys.argv if a.startswith("--prefix=")), "OB6000")

devices_found = {}
first_batch = {}   # mac -> (startTimeStamp, delay)
batch_counts = {}  # mac -> int

replay_first_batch = {}   # mac -> (startTimeStamp, delay)
replay_batch_counts = {}  # mac -> int


def on_found(device_list):
    for d in device_list:
        if d.Name and d.Name.startswith(NAME_PREFIX) and d.Address not in devices_found:
            print(f"found: {d.Name} {d.Address} rssi={d.RSSI}")
            devices_found[d.Address] = d


def on_data(sensor: SensorProfile, data_list: list):
    mac = sensor.BLEDevice.Address
    batch_counts[mac] = batch_counts.get(mac, 0) + len(data_list)
    for data in data_list:
        # startTimeStamp/delay 是 per-stream 常量，且 IPC 按设备 FIFO：
        # 预热段的滞留消息一定先于 multiStart 段到达，逐批覆盖最终收敛到
        # multiStart 会话的值（取首个会抓到 stop 后仍在途的预热批）
        if data.getStartTimeStamp():
            first_batch[mac] = (data.getStartTimeStamp(), data.getDelay())


def on_replay_data(sensor: SensorProfile, data_list: list):
    mac = sensor.BLEDevice.Address
    replay_batch_counts[mac] = replay_batch_counts.get(mac, 0) + len(data_list)
    for data in data_list:
        if mac not in replay_first_batch and data.getStartTimeStamp():
            replay_first_batch[mac] = (data.getStartTimeStamp(), data.getDelay())


def on_error(sensor: SensorProfile, reason: str):
    print(f"error: {sensor.BLEDevice.Name}: {reason}")


def main() -> int:
    controller = SensorControllerInstance
    if not controller.isEnable:
        print("please open bluetooth")
        return 1

    controller.setLogPath(True)
    controller.setDebugEnabled(True)

    controller.onDeviceFoundCallback = on_found
    print("scanning ...")
    controller.startScan(SCAN_DEVICE_PERIOD_IN_MS)
    t0 = time.time()
    while len(devices_found) < TARGET_COUNT and time.time() - t0 < 25:
        time.sleep(0.2)
    controller.stopScan()
    if len(devices_found) < TARGET_COUNT:
        print(f"only {len(devices_found)} {NAME_PREFIX} device(s) found")
        return 1

    sensors = []
    for device in list(devices_found.values())[:TARGET_COUNT]:
        sensor = controller.requireSensor(device)
        if sensor is None:
            print(f"requireSensor failed: {device.Name}")
            return 1
        sensor.onDataCallback = on_data
        sensor.onErrorCallback = on_error
        print(f"connecting: {sensor.BLEDevice.Name} ...")
        if not sensor.connect():
            print(f"connect failed: {sensor.BLEDevice.Name}")
            return 1
        if not sensor.init(PACKAGE_COUNT, POWER_REFRESH_PERIOD_IN_MS):
            print(f"init failed: {sensor.BLEDevice.Name}")
            return 1
        r = sensor.setParam("DEBUG_BLE_DATA_PATH", True)
        print(f"bin export: {sensor.getParam('DEBUG_BLE_DATA_PATH')} ({r})")
        sensors.append(sensor)
        print(f"ready: {sensor.BLEDevice.Name} {sensor.BLEDevice.Address}")

    if "--pre-start" in sys.argv or "--keep-streaming" in sys.argv:
        # multiStart 前先单独 start/stop 一轮，验证预热对同步起流的影响；
        # --keep-streaming：只停第一台，第二台保持在流，验证 multiStart 的
        # restart 语义（在流设备先停后起，与空闲设备同刻起流）
        print("pre-start/stop each device ...")
        for sensor in sensors:
            print(f"pre-start: {sensor.BLEDevice.Name}")
            if not sensor.startDataNotification():
                print(f"pre-start failed: {sensor.BLEDevice.Name}")
                return 1
        time.sleep(3)
        stop_list = sensors[:1] if "--keep-streaming" in sys.argv else sensors
        for sensor in stop_list:
            print(f"pre-stop: {sensor.BLEDevice.Name}")
            sensor.stopDataNotification()
        # 首包统计清零：LIVE 数值反映 multiStart 会话而非预热会话
        first_batch.clear()
        batch_counts.clear()
        time.sleep(1)

    print("multiStart ...")
    # 同型号设备用默认对齐参数，混合型号不做首包时差校验、重试 5 次
    model_names = set()
    for sensor in sensors:
        info = sensor.getDeviceInfo()
        model_names.add(info.ModelName if info else None)
    same_model = len(model_names) == 1 and None not in model_names
    print(f"device models: {sorted(m for m in model_names if m)}, same_model={same_model}")
    if same_model:
        results = controller.multiStartDataNotification(sensors)
    else:
        results = controller.multiStartDataNotification(
            sensors, timeout=60.0, maxDelayDispersionMs=-1, maxAttempts=5)
    print("multiStart results:", results)

    t0 = time.time()
    while time.time() - t0 < 12 and len(first_batch) < len(sensors):
        time.sleep(0.2)
    time.sleep(2)

    ts_list = []
    for sensor in sensors:
        mac = sensor.BLEDevice.Address
        fb = first_batch.get(mac)
        batches = batch_counts.get(mac, 0)
        if fb:
            ts_list.append((mac, fb[0]))
            print(f"LIVE {sensor.BLEDevice.Name} {mac}: startTimeStamp={fb[0]} delay={fb[1]}ms batches={batches}")
        else:
            print(f"LIVE {sensor.BLEDevice.Name} {mac}: NO DATA (batches={batches})")

    if len(ts_list) == TARGET_COUNT:
        spread = (max(t for _, t in ts_list) - min(t for _, t in ts_list)) & 0xFFFFFFFF
        print(f"LIVE startTimeStamp spread across devices: {spread} ms")

    print("multiStop ...")
    results = controller.multiStopDataNotification(sensors)
    print("multiStop results:", results)

    live_only = "--live-only" in sys.argv

    bin_paths = {}
    for sensor in sensors:
        bin_paths[sensor.BLEDevice.Address] = sensor.getParam("DEBUG_BLE_DATA_PATH")
        sensor.disconnect()

    if not live_only:
        # ---- replay the exported bins and compare restored ts/delay with live ----
        for sensor in sensors:
            mac = sensor.BLEDevice.Address
            path = bin_paths.get(mac)
            if not path:
                print(f"REPLAY {mac}: no bin exported, skip")
                continue
            sensor.onDataCallback = on_replay_data
            print(f"replaying: {path}")
            profile = controller.replayBinFile(path, sensor, realtime=False)
            print(f"replay result: {profile}")

        for sensor in sensors:
            mac = sensor.BLEDevice.Address
            live = first_batch.get(mac)
            rep = replay_first_batch.get(mac)
            print(f"CMP {sensor.BLEDevice.Name} {mac}: live={live} replay={rep} "
                  f"replay_batches={replay_batch_counts.get(mac, 0)} "
                  f"{'MATCH' if live and rep and live == rep else 'MISMATCH'}")

    # ---- precise (perf_counter_ns) timing from bin 0x07 records ----
    precise = {}  # mac -> (start_send_ns, first_recv_ns)
    for sensor in sensors:
        mac = sensor.BLEDevice.Address
        path = bin_paths.get(mac)
        if not path:
            continue
        start_send_ns = None
        first_recv_ns = None
        data_records = 0
        precise_records = 0
        for record_type, ts_ms, payload, perf_ns in iter_bin_records_precise(path):
            if perf_ns is not None:
                precise_records += 1
            if record_type == BIN_RECORD_EVENT and payload == b"stream_start":
                # OYM 起流：CCCD 写后的 stream_start 事件
                start_send_ns = perf_ns
                first_recv_ns = None  # 新一轮起流，首包重新计
            elif (record_type == BIN_RECORD_CMD_SEND and len(payload) >= 5
                    and payload[0] == Command.SET_DATA_NOTIF_SWITCH
                    and int.from_bytes(payload[1:5], "little") != 0):
                # RFSTAR 起流：非零订阅的 SET_DATA_NOTIF_SWITCH CMD 写
                start_send_ns = perf_ns
                first_recv_ns = None  # 新一轮起流，首包重新计
            elif record_type == BIN_RECORD_DATA:
                data_records += 1
                if start_send_ns is not None and first_recv_ns is None:
                    first_recv_ns = perf_ns
        precise[mac] = (start_send_ns, first_recv_ns)
        print(f"PRECISE {sensor.BLEDevice.Name} {mac}: data_records={data_records} "
              f"precise_records={precise_records}")

    send_ns_list = [(mac, v[0]) for mac, v in precise.items() if v[0] is not None]
    if len(send_ns_list) == TARGET_COUNT:
        spread_ns = max(t for _, t in send_ns_list) - min(t for _, t in send_ns_list)
        for mac, t in send_ns_list:
            print(f"PRECISE {mac}: start send at {t} ns")
        print(f"PRECISE start send spread across devices: {spread_ns / 1e6:.3f} ms")
    for sensor in sensors:
        mac = sensor.BLEDevice.Address
        send_ns, recv_ns = precise.get(mac, (None, None))
        if send_ns is not None and recv_ns is not None:
            print(f"PRECISE {sensor.BLEDevice.Name} {mac}: first packet delay "
                  f"{(recv_ns - send_ns) / 1e6:.3f} ms")
        else:
            print(f"PRECISE {sensor.BLEDevice.Name} {mac}: no precise timestamps")

    controller.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
