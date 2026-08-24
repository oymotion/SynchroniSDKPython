# -*- coding: utf-8 -*-
import sys, time, traceback, asyncio
from sensor import *


def main():
    ctrl = SensorControllerInstance
    print("backend =", ctrl.getBLEBackendName(), flush=True)
    devices = ctrl.scan(5000)
    for d in devices:
        print("found:", d.Name, d.Address, flush=True)

    target = None
    for d in devices:
        if d.Name and d.Name.startswith("OYWW"):
            target = d
            break
    if target is None:
        print("no OYWW target", flush=True)
        ctrl.terminate()
        return

    sensor = ctrl.requireSensor(target)
    sensor.onStateChanged = lambda s, st: print("[state]", st, flush=True)
    print("connect ->", sensor.connect(), flush=True)
    for _ in range(50):
        if sensor.deviceState == DeviceStateEx.Ready:
            break
        time.sleep(0.2)
    print("state =", sensor.deviceState, "isReady =", sensor.isReady, flush=True)

    print("init ->", sensor.init(20, 1000), flush=True)
    print("start ->", sensor.startDataNotification(), flush=True)
    time.sleep(3)

    print("--- getBatteryLevel ---", flush=True)
    try:
        b = sensor.getBatteryLevel()
        print("BATTERY =", repr(b), type(b), flush=True)
    except Exception as e:
        print("EXCEPTION:", type(e).__name__, str(e), flush=True)
        traceback.print_exc()

    print("--- asyncGetBatteryLevel ---", flush=True)
    try:
        b2 = asyncio.run(sensor.asyncGetBatteryLevel())
        print("ASYNC BATTERY =", repr(b2), type(b2), flush=True)
    except Exception as e:
        print("ASYNC EXCEPTION:", type(e).__name__, str(e), flush=True)

    sensor.stopDataNotification()
    sensor.disconnect()
    ctrl.terminate()
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
