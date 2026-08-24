"""
SynchroniSDKPython_DemoEMG.py 功能性自动化测试脚本

该测试脚本使用 pytest 和 pytest-qt 框架对 EMG Demo 应用进行功能测试
测试范围：
1. UI 组件的初始化
2. 按钮点击交互
3. 下拉框选择交互
4. 蓝牙扫描流程
5. 蓝牙连接流程（使用真实设备 D8:71:4D:8D:81:88）
"""

import sys
import pytest
import time
from unittest.mock import MagicMock, patch
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtTest import QTest
from PyQt5.QtCore import Qt

# 导入被测试的模块
# 添加项目根目录到 sys.path
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, project_root)

from examples.SynchroniSDKPython_DemoEMG import BluetoothDeviceScanner, PERIOD_OPTIONS
from sensor import BLEDevice, SensorController, DeviceStateEx


# 测试设备信息
TEST_DEVICE_ADDRESS = "D8:71:4D:8D:81:88"


@pytest.fixture(scope="session")
def app(qapp):
    """创建 QApplication 实例 - 整个测试会话只创建一次"""
    return qapp


# 使用模块级变量来存储 widget 实例
_scanner_widget_instance = None


@pytest.fixture(scope="session")
def scanner_widget(app, request):
    """
    创建 BluetoothDeviceScanner 实例 - 整个测试会话只创建一次
    窗口只会打开一次，直到所有测试完成才关闭
    """
    global _scanner_widget_instance
    
    if _scanner_widget_instance is None:
        _scanner_widget_instance = BluetoothDeviceScanner()
        print("\n✓ 测试窗口已打开 - 将在所有测试完成后关闭\n")
    
    yield _scanner_widget_instance
    
    # 所有测试结束后清理
    print("\n✓ 所有测试完成 - 正在关闭测试窗口\n")
    try:
        if _scanner_widget_instance.connected_device:
            _scanner_widget_instance.disconnect_device()
        if _scanner_widget_instance.SensorControllerInstance.isScanning:
            _scanner_widget_instance.stop_scan()
        _scanner_widget_instance.close()
    except:
        pass


@pytest.fixture(autouse=True)
def reset_widget_state(scanner_widget, qtbot, request):
    """
    在每个测试后重置 widget 状态
    确保测试之间互不影响
    注意：不使用 qtbot.addWidget() 以避免自动关闭窗口
    """
    yield
    
    # 每个测试后重置状态
    try:
        # 如果正在扫描，停止扫描
        if scanner_widget.SensorControllerInstance.isScanning:
            scanner_widget.stop_scan()
            qtbot.wait(200)
        
        # 重置 UI 状态（如果 UI 组件还存在）
        try:
            scanner_widget.scan_button.setEnabled(True)
            scanner_widget.stop_scan_button.setEnabled(False)
            scanner_widget.device_list.clearSelection()
        except RuntimeError:
            # UI 组件已被删除，忽略
            pass
        
        qtbot.wait(100)
    except:
        pass


class TestUIInitialization:
    """测试 UI 组件的初始化"""
    
    def test_window_title(self, scanner_widget):
        """测试窗口标题"""
        assert scanner_widget.windowTitle() == "SynchroniSDKPython Demo"
    
    def test_window_size(self, scanner_widget):
        """测试窗口初始大小"""
        size = scanner_widget.size()
        assert size.width() == 1000
        assert size.height() == 600
    
    def test_scan_button_exists(self, scanner_widget):
        """测试扫描按钮存在"""
        assert scanner_widget.scan_button is not None
        assert scanner_widget.scan_button.text() == "开始扫描蓝牙设备"
        assert scanner_widget.scan_button.isEnabled() == True
    
    def test_stop_scan_button_exists(self, scanner_widget):
        """测试停止扫描按钮存在且初始禁用"""
        assert scanner_widget.stop_scan_button is not None
        assert scanner_widget.stop_scan_button.text() == "停止扫描蓝牙设备"
        assert scanner_widget.stop_scan_button.isEnabled() == False
    
    def test_disconnect_button_exists(self, scanner_widget):
        """测试断开连接按钮存在且初始禁用"""
        assert scanner_widget.disconnect_button is not None
        assert scanner_widget.disconnect_button.text() == "断开连接"
        assert scanner_widget.disconnect_button.isEnabled() == False
    
    def test_device_list_exists(self, scanner_widget):
        """测试设备列表存在"""
        assert scanner_widget.device_list is not None
        assert scanner_widget.device_list.count() == 0  # 初始为空
    
    def test_period_combobox_exists(self, scanner_widget):
        """测试周期选择下拉框存在"""
        assert scanner_widget.period_combobox is not None
        assert scanner_widget.period_combobox.count() == len(PERIOD_OPTIONS)
        assert scanner_widget.period_combobox.currentText() == "1s"
    
    def test_channel_combobox_exists(self, scanner_widget):
        """测试通道选择下拉框存在"""
        assert scanner_widget.channel_combobox is not None
        assert scanner_widget.channel_combobox.count() == 0  # 初始为空（未连接设备）
    
    def test_filter_checkboxes_exist(self, scanner_widget):
        """测试滤波器复选框存在"""
        assert scanner_widget.hpf_checkbox is not None
        assert scanner_widget.lpf_checkbox is not None
        assert scanner_widget.notch_filter_50_checkbox is not None
        assert scanner_widget.notch_filter_60_checkbox is not None
        
        # 检查初始状态为未选中
        assert scanner_widget.hpf_checkbox.isChecked() == False
        assert scanner_widget.lpf_checkbox.isChecked() == False
        assert scanner_widget.notch_filter_50_checkbox.isChecked() == False
        assert scanner_widget.notch_filter_60_checkbox.isChecked() == False
    
    def test_impedance_label_exists(self, scanner_widget):
        """测试阻抗值标签存在"""
        assert scanner_widget.impedance_label is not None
        assert "阻抗值" in scanner_widget.impedance_label.text()
    
    def test_matplotlib_canvas_exists(self, scanner_widget):
        """测试 Matplotlib 画布存在"""
        assert scanner_widget.canvas is not None
        assert scanner_widget.figure is not None
        assert scanner_widget.ax is not None
    
    def test_initial_data_state(self, scanner_widget):
        """测试初始数据状态"""
        assert scanner_widget.discovered_devices == []
        assert scanner_widget.connected_device is None
        assert scanner_widget.current_sensor is None
        assert scanner_widget.sampling_rate == 250
        assert scanner_widget.period == 1
        assert scanner_widget.current_channel == 0
        assert scanner_widget.EmgChannelCount == 0
        assert scanner_widget.impedance == []


class TestButtonInteractions:
    """测试按钮点击交互"""
    
    def test_scan_button_click(self, scanner_widget, qtbot):
        """测试点击开始扫描按钮"""
        initial_scanning = scanner_widget.SensorControllerInstance.isScanning
        
        # 点击扫描按钮
        qtbot.mouseClick(scanner_widget.scan_button, Qt.LeftButton)
        
        # 等待状态更新
        qtbot.wait(100)
        
        # 验证按钮状态改变
        assert scanner_widget.scan_button.isEnabled() == False
        assert scanner_widget.stop_scan_button.isEnabled() == True
    
    def test_stop_scan_button_click(self, scanner_widget, qtbot):
        """测试点击停止扫描按钮"""
        # 先开始扫描
        qtbot.mouseClick(scanner_widget.scan_button, Qt.LeftButton)
        qtbot.wait(100)
        
        # 点击停止扫描按钮
        qtbot.mouseClick(scanner_widget.stop_scan_button, Qt.LeftButton)
        qtbot.wait(100)
        
        # 验证按钮状态恢复
        assert scanner_widget.scan_button.isEnabled() == True
        assert scanner_widget.stop_scan_button.isEnabled() == False
    
    # def test_filter_checkbox_toggle_without_device(self, scanner_widget, qtbot, capsys):
    #     """测试在未连接设备时切换滤波器复选框"""
    #     # 点击 HPF 复选框
    #     qtbot.mouseClick(scanner_widget.hpf_checkbox, Qt.LeftButton)
    #     qtbot.wait(50)
        
    #     # 捕获输出，应该提示未连接设备
    #     captured = capsys.readouterr()
    #     assert "当前未连接设备" in captured.out


class TestComboboxInteractions:
    """测试下拉框选择交互"""
    
    def test_period_combobox_change(self, scanner_widget, qtbot):
        """测试更改周期选择"""
        # 改变周期到 5s
        index = scanner_widget.period_combobox.findText("5s")
        scanner_widget.period_combobox.setCurrentIndex(index)
        qtbot.wait(100)
        
        # 验证周期已更改
        assert scanner_widget.period == 5
        assert scanner_widget.period_combobox.currentText() == "5s"
    
    def test_all_period_options(self, scanner_widget, qtbot):
        """测试所有周期选项"""
        for period_text, period_value in PERIOD_OPTIONS.items():
            index = scanner_widget.period_combobox.findText(period_text)
            scanner_widget.period_combobox.setCurrentIndex(index)
            qtbot.wait(50)
            
            assert scanner_widget.period == period_value
            assert scanner_widget.period_combobox.currentText() == period_text


class TestBluetoothScan:
    """测试蓝牙扫描流程"""
    
    def test_start_scan_functionality(self, scanner_widget, qtbot):
        """测试扫描功能启动"""
        # 开始扫描
        scanner_widget.start_scan()
        qtbot.wait(100)
        
        # 验证扫描状态
        assert scanner_widget.SensorControllerInstance.isScanning == True
        assert scanner_widget.scan_button.isEnabled() == False
        assert scanner_widget.stop_scan_button.isEnabled() == True
    
    def test_stop_scan_functionality(self, scanner_widget, qtbot):
        """测试停止扫描功能"""
        # 先开始扫描
        scanner_widget.start_scan()
        qtbot.wait(100)
        
        # 停止扫描
        scanner_widget.stop_scan()
        qtbot.wait(100)
        
        # 验证扫描已停止
        assert scanner_widget.SensorControllerInstance.isScanning == False
        assert scanner_widget.scan_button.isEnabled() == True
        assert scanner_widget.stop_scan_button.isEnabled() == False
    
    def test_device_found_callback(self, scanner_widget, qtbot):
        """
        测试设备发现回调
        注意：此测试需要真实的蓝牙设备在范围内才能发现设备
        如果没有设备，测试会跳过验证设备列表
        """
        # 记录初始设备数量
        initial_device_count = scanner_widget.device_list.count()
        initial_discovered_count = len(scanner_widget.discovered_devices)
        
        # 开始扫描
        scanner_widget.start_scan()
        
        # 等待扫描完成（3 秒）
        qtbot.wait(3500)
        
        # 停止扫描
        scanner_widget.stop_scan()
        qtbot.wait(200)
        
        # 验证扫描过程正常执行（不验证是否真的发现了设备）
        # 因为测试环境可能没有蓝牙设备
        final_device_count = scanner_widget.device_list.count()
        final_discovered_count = len(scanner_widget.discovered_devices)
        
        print(f"\n扫描结果: 发现 {final_device_count - initial_device_count} 个新设备")
        print(f"设备列表: {final_discovered_count - initial_discovered_count} 个设备添加到内部列表")
        
        # 如果发现了设备，验证设备信息格式正确
        if final_device_count > initial_device_count:
            # 获取最后添加的设备
            last_item = scanner_widget.device_list.item(final_device_count - 1)
            item_text = last_item.text()
            
            # 验证设备信息格式
            assert "Name: " in item_text, "设备信息应包含 Name"
            assert "Address: " in item_text, "设备信息应包含 Address"
            assert "RSSI: " in item_text, "设备信息应包含 RSSI"
            
            print(f"✓ 发现设备: {item_text}")
        else:
            print("ℹ 未发现蓝牙设备（测试环境可能没有设备在范围内）")


class TestBluetoothConnection:
    """测试蓝牙连接流程（使用真实设备）"""
    
    # @pytest.mark.skip(reason="需要真实蓝牙设备在线")
    def test_device_scan_and_connect(self, scanner_widget, qtbot):
        """
        测试真实设备的扫描和连接流程
        
        此测试需要：
        1. 真实的 EMG 蓝牙设备（地址: D8:71:4D:8D:81:88）
        2. 设备已开机并在蓝牙范围内
        3. 设备未被其他应用程序占用
        
        测试流程遵循 SDK 的正确使用方式：
        1. 真实蓝牙扫描 (这会在 _process_ble_devices 中正确创建 SensorProfile)
        2. 从扫描结果中选择设备
        3. 连接设备
        """
        print(f"\n{'='*60}")
        print(f"开始真实设备连接测试")
        print(f"目标设备地址: {TEST_DEVICE_ADDRESS}")
        print(f"{'='*60}\n")
        # 开始扫描
        scanner_widget.start_scan()
        
        # 等待扫描完成并发现设备（最多等待 10 秒）
        def device_found():
            return scanner_widget.device_list.count() > 0
        
        qtbot.waitUntil(device_found, timeout=10000)
        
        # 查找目标设备
        target_item = None
        for i in range(scanner_widget.device_list.count()):
            item = scanner_widget.device_list.item(i)
            if TEST_DEVICE_ADDRESS in item.text():
                target_item = item
                break
        
        assert target_item is not None, f"未找到设备 {TEST_DEVICE_ADDRESS}，请确保设备已开机并在蓝牙范围内"
        
        # 点击连接设备
        scanner_widget.device_list.setCurrentItem(target_item)
        scanner_widget.connect_device(target_item)
        
        # 等待连接完成（最多等待 15 秒）
        def device_connected():
            return scanner_widget.connected_device is not None
        
        qtbot.waitUntil(device_connected, timeout=15000)
        
        # # 验证连接状态
        # assert scanner_widget.connected_device is not None, "设备连接失败"
        # assert scanner_widget.current_sensor is not None, "传感器对象未创建"
        # assert scanner_widget.disconnect_button.isEnabled() == True, "断开连接按钮未启用"
        
        # # 验证通道下拉框已填充
        # qtbot.waitUntil(lambda: scanner_widget.channel_combobox.count() > 0, timeout=5000)
        # assert scanner_widget.channel_combobox.count() > 0, "通道下拉框未填充"
        
        print(f"\n✓ 成功连接到设备 {TEST_DEVICE_ADDRESS}")
        print(f"✓ 通道数: {scanner_widget.EmgChannelCount}")
        print(f"✓ 采样率: {scanner_widget.sampling_rate}")
        
        time.sleep(60)
    
    # @pytest.mark.skip(reason="需要真实蓝牙设备连接")
    def test_device_disconnect(self, scanner_widget, qtbot):
        """测试断开设备连接"""
        # 检查是否已连接
        if scanner_widget.connected_device is None:
            pytest.skip("设备未连接，请先运行 test_device_scan_and_connect 测试")
        
        device_name = scanner_widget.connected_device.Name
        
        # 点击断开连接
        scanner_widget.disconnect_device()
        
        # 等待断开完成
        qtbot.wait(2000)
        
        # 验证断开状态
        assert scanner_widget.connected_device is None, "设备未断开"
        assert scanner_widget.disconnect_button.isEnabled() == False, "断开连接按钮未禁用"
        
        print(f"\n✓ 成功断开设备 {device_name}")


class TestDataProcessing:
    """测试数据处理功能"""
    
    def test_update_buffer_size(self, scanner_widget):
        """测试缓冲区大小更新"""
        scanner_widget.EmgChannelCount = 4
        scanner_widget.period = 1
        scanner_widget.sampling_rate = 250
        
        scanner_widget.update_buffer_size()
        
        assert scanner_widget.data_buffer is not None
        assert scanner_widget.data_buffer.shape == (4, 250)
    
    def test_change_period_updates_buffer(self, scanner_widget, qtbot):
        """测试更改周期会更新缓冲区"""
        scanner_widget.EmgChannelCount = 2
        scanner_widget.sampling_rate = 250
        
        # 更改周期
        scanner_widget.change_period("5s")
        qtbot.wait(100)
        
        # 验证缓冲区大小正确
        assert scanner_widget.period == 5
        assert scanner_widget.data_buffer.shape == (2, 1250)  # 5s * 250Hz = 1250
    
    def test_change_channel(self, scanner_widget, qtbot):
        """测试更改通道"""
        scanner_widget.EmgChannelCount = 4
        scanner_widget.update_buffer_size()
        
        # 手动添加通道选项
        for i in range(4):
            scanner_widget.channel_combobox.addItem(f"通道 {i + 1}")
        
        # 更改到通道 3
        scanner_widget.channel_combobox.setCurrentIndex(2)
        qtbot.wait(100)
        
        # 验证当前通道
        assert scanner_widget.current_channel == 2


class TestSignalsAndSlots:
    """测试信号和槽连接"""
    
    def test_add_device_signal(self, scanner_widget, qtbot):
        """测试添加设备信号"""
        # 发射添加设备信号
        test_device_text = f"Name: TestDevice, Address: {TEST_DEVICE_ADDRESS}, RSSI: -60"
        scanner_widget.add_device_signal.emit(test_device_text)
        
        qtbot.wait(100)
        
        # 验证设备已添加到列表
        assert scanner_widget.device_list.count() > 0
        assert TEST_DEVICE_ADDRESS in scanner_widget.device_list.item(0).text()
    
    def test_update_plot_signal(self, scanner_widget, qtbot):
        """测试更新绘图信号"""
        scanner_widget.EmgChannelCount = 1
        scanner_widget.update_buffer_size()
        
        # 发射更新绘图信号
        scanner_widget.update_plot_signal.emit()
        
        qtbot.wait(100)
        
        # 信号应该被正确处理（不抛出异常）


class TestEdgeCases:
    """测试边界情况"""
    
    def test_disconnect_without_connection(self, scanner_widget, capsys):
        """测试在未连接时断开连接"""
        scanner_widget.disconnect_device()
        
        captured = capsys.readouterr()
        assert "No device is currently connected" in captured.out
    
    def test_multiple_scan_starts(self, scanner_widget, qtbot):
        """测试多次开始扫描"""
        scanner_widget.start_scan()
        qtbot.wait(100)
        
        # 再次开始扫描应该返回 True（已在扫描中）
        result = scanner_widget.SensorControllerInstance.startScan(3000)
        assert result == True
    
    # def test_period_combobox_boundary_values(self, scanner_widget, qtbot):
    #     """测试周期下拉框的边界值"""
    #     # 测试最小周期 (500ms)
    #     scanner_widget.change_period("500ms")
    #     assert scanner_widget.period == 0.5
        
    #     # 测试最大周期 (60s)
    #     scanner_widget.change_period("60s")
    #     assert scanner_widget.period == 60


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
