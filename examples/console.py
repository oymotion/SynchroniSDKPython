import signal
import time
from typing import List
from sensor import *

SCAN_DEVICE_PERIOD_IN_MS = 3000
PACKAGE_COUNT = 5
POWER_REFRESH_PERIOD_IN_MS = 5000

def SimpleTest(): 
    if not SensorControllerInstance.isEnable:
        print('please open bluetooth')
        return
    
    if not SensorControllerInstance.hasDeviceFoundCallback:
        SensorControllerInstance.onDeviceFoundCallback = deviceFoundCallback

    if not SensorControllerInstance.isScaning:
        print('start scan')
        if not SensorControllerInstance.startScan(SCAN_DEVICE_PERIOD_IN_MS):
            print('please try scan later')
            
    connectedSensors = SensorControllerInstance.getConnectedSensors()
    for sensor in connectedSensors:
        if sensor.hasInited:
            print(sensor.BLEDevice.Name + " power: " + str(sensor.getBatteryLevel()))
            sensor.disconnect()


def deviceFoundCallback(deviceList: List[BLEDevice]):
    print('stop scan')
    SensorControllerInstance.stopScan()

    filteredDevice = filter(lambda x: x.Name.startswith('OB') or x.Name.startswith('Sync') , deviceList)
    for device in filteredDevice:
        sensor = SensorControllerInstance.requireSensor(device)
        if sensor == None:
            continue

        print('found: ' + sensor.BLEDevice.Name)
        sensor.onDataCallback = onDataCallback
        sensor.onPowerChanged = onPowerChanged
        sensor.onStateChanged = onStateChanged
        sensor.onErrorCallback = onErrorCallback

        #check state & connect
        if sensor.deviceState != DeviceStateEx.Ready:
            print('connecting: ' + sensor.BLEDevice.Address)
            if not sensor.connect():
                print('connect device: ' + sensor.BLEDevice.Name + ' failed')
                continue

        #init & start data transfer
        if not sensor.hasInited:
            result = sensor.setParam('DEBUG_BLE_DATA_PATH','/temp/test.csv')
            if not sensor.init(PACKAGE_COUNT, POWER_REFRESH_PERIOD_IN_MS):
                print('init device: ' + sensor.BLEDevice.Name + ' failed')
                continue
            deviceInfo = sensor.getDeviceInfo()
            print('deviceInfo: Model: ' + str(deviceInfo.ModelName))

        print('start data transfer')
        if not sensor.startDataNotification():
            print('start data transfer with device: ' + sensor.BLEDevice.Name + ' failed')
            continue
        break

def onDataCallback(sensor: SensorProfile, data: SensorData):
    if data.dataType == DataType.NTF_EEG or data.dataType == DataType.NTF_ECG or data.dataType == DataType.NTF_BRTH or data.dataType == DataType.NTF_ACC or data.dataType == DataType.NTF_GYRO:
        print('got data from sensor: ' + sensor.BLEDevice.Name + ' data type: ' + str(data.dataType))
        print(str(data.channelSamples[0][0].sampleIndex))
    
    if data.channelSamples[0][0].sampleIndex == 50:
        sensor.stopDataNotification()

def onPowerChanged(sensor: SensorProfile, power: int):
    print('connected sensor: ' + sensor.BLEDevice.Name + ' power: ' + str(power))
    if not sensor.isDataTransfering:
        sensor.disconnect()
        time.sleep(2)
        SensorControllerInstance.startScan(SCAN_DEVICE_PERIOD_IN_MS)
    
def onStateChanged(sensor: SensorProfile, newstate: DeviceStateEx):
    print('device: ' + sensor.BLEDevice.Name + str(newstate))   

def onErrorCallback(sensor: SensorProfile, reason: str):
    print('device: ' + sensor.BLEDevice.Name + reason)
    pass

def terminate():
    SensorControllerInstance.terminate()
    exit()
    
def main():
    signal.signal(signal.SIGINT, lambda signal, frame: terminate())
    SimpleTest()
    time.sleep(30)
    SensorControllerInstance.terminate()

if __name__ == '__main__':
    main()
