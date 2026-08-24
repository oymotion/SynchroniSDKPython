#!/usr/bin/env python
"""
自动化测试运行脚本

提供便捷的方式运行不同类型的测试
"""

import sys
import subprocess
import argparse


def run_command(cmd):
    """运行命令并实时输出结果"""
    print(f"\n执行命令: {' '.join(cmd)}\n")
    print("=" * 80)
    
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        bufsize=1
    )
    
    for line in process.stdout:
        print(line, end='')
    
    process.wait()
    print("=" * 80)
    return process.returncode


def main():
    parser = argparse.ArgumentParser(description='运行 EMG Demo 自动化测试')
    parser.add_argument(
        '--type',
        choices=['all', 'ui', 'button', 'combobox', 'scan', 'connection', 'data', 'signals', 'edge'],
        default='all',
        help='选择要运行的测试类型'
    )
    parser.add_argument(
        '--coverage',
        action='store_true',
        help='生成测试覆盖率报告'
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='详细输出'
    )
    parser.add_argument(
        '--real-device',
        action='store_true',
        help='运行需要真实设备的测试（需要先移除 skip 装饰器）'
    )
    
    args = parser.parse_args()
    
    # 构建基础命令
    base_cmd = [sys.executable, '-m', 'pytest', 'tests/test_emg_demo_functional.py']
    
    # 选择测试类型
    test_class_map = {
        'all': '',
        'ui': '::TestUIInitialization',
        'button': '::TestButtonInteractions',
        'combobox': '::TestComboboxInteractions',
        'scan': '::TestBluetoothScan',
        'connection': '::TestBluetoothConnection',
        'data': '::TestDataProcessing',
        'signals': '::TestSignalsAndSlots',
        'edge': '::TestEdgeCases'
    }
    
    if args.type != 'all':
        base_cmd[2] += test_class_map[args.type]
    
    # 添加详细输出
    if args.verbose:
        base_cmd.append('-v')
    else:
        base_cmd.append('-v')  # 默认也使用详细输出
    
    # 添加覆盖率
    if args.coverage:
        base_cmd.extend(['--cov=examples', '--cov-report=html', '--cov-report=term'])
    
    # 显示测试信息
    print("=" * 80)
    print("SynchroniSDKPython EMG Demo 自动化测试")
    print("=" * 80)
    print(f"测试类型: {args.type}")
    print(f"覆盖率报告: {'是' if args.coverage else '否'}")
    print(f"真实设备测试: {'是' if args.real_device else '否'}")
    
    # 运行测试
    returncode = run_command(base_cmd)
    
    # 显示结果
    print("\n" + "=" * 80)
    if returncode == 0:
        print("✓ 所有测试通过！")
    else:
        print("✗ 某些测试失败，请查看上方输出")
    print("=" * 80)
    
    # 如果生成了覆盖率报告，提示用户
    if args.coverage:
        print("\n覆盖率报告已生成:")
        print("  - HTML 报告: htmlcov/index.html")
        print("  - 在浏览器中打开查看详细信息")
    
    return returncode


if __name__ == '__main__':
    sys.exit(main())
