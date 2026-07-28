# -*- coding: utf-8 -*-

import importlib.util
import os
import runpy
import sys
import types

# matplotlib 的 Qt 绑定选择（在 import matplotlib 之前设定）
os.environ.setdefault("QT_API", "pyqt6")

_EXAMPLES_DIR = os.path.dirname(os.path.abspath(__file__))
if _EXAMPLES_DIR not in sys.path:
    sys.path.insert(0, _EXAMPLES_DIR)


class _Qt:
    """PyQt6 命名空间枚举 → PyQt5 扁平成员（仅原 demo 用到的成员）。"""
    AlignCenter = None        # 下方 _install_qt5_shim 中填充
    Checked = None
    DirectConnection = None
    WindowMinimized = None


class _ForwardingModule(types.ModuleType):
    """属性整体转发到 PyQt6 对应模块的 PyQt5 兼容模块。"""

    def __init__(self, name, target):
        super().__init__(name)
        self._forward_target = target

    def __getattr__(self, name):
        return getattr(self._forward_target, name)


def _install_qt5_shim():
    """把 PyQt5 / backend_qt5agg 的导入重定向到 PyQt6 / backend_qtagg。"""
    import PyQt6.QtCore as qt6core
    import PyQt6.QtWidgets as qtwidgets
    import matplotlib.backends.backend_qtagg as qtagg

    _Qt.AlignCenter = qt6core.Qt.AlignmentFlag.AlignCenter
    _Qt.Checked = qt6core.Qt.CheckState.Checked
    _Qt.DirectConnection = qt6core.Qt.ConnectionType.DirectConnection
    _Qt.WindowMinimized = qt6core.Qt.WindowState.WindowMinimized

    qtcore = _ForwardingModule("PyQt5.QtCore", qt6core)
    qtcore.Qt = _Qt
    qtwidgets = _ForwardingModule("PyQt5.QtWidgets", qtwidgets)

    pyqt5 = types.ModuleType("PyQt5")
    pyqt5.QtCore = qtcore
    pyqt5.QtWidgets = qtwidgets
    pyqt5.__path__ = []

    sys.modules["PyQt5"] = pyqt5
    sys.modules["PyQt5.QtCore"] = qtcore
    sys.modules["PyQt5.QtWidgets"] = qtwidgets
    # backend_qt5agg 强制 PyQt5 绑定；重定向到 QT_API=pyqt6 的 backend_qtagg
    # （FigureCanvasQTAgg / NavigationToolbar2QT 接口一致）
    sys.modules["matplotlib.backends.backend_qt5agg"] = qtagg

    # PyQt6 保留 exec_() 别名；个别版本移除时补回
    if not hasattr(qtwidgets.QApplication, "exec_"):
        qtwidgets.QApplication.exec_ = qtwidgets.QApplication.exec


# 模块级安装：multiprocessing 子进程重导入本模块时同样生效
_install_qt5_shim()

if __name__ == "__main__":
    # 让 spawn 子进程以本模块为 __mp_main__ 导入（获得兼容层），
    # 而不是按文件路径重导入需要 PyQt5 的原 demo
    _mod_name = os.path.splitext(os.path.basename(os.path.abspath(__file__)))[0]
    sys.modules["__main__"].__spec__ = importlib.util.spec_from_file_location(
        _mod_name, os.path.abspath(__file__))

    runpy.run_path(os.path.join(_EXAMPLES_DIR, "SynchroniSDKPython_DemoNewPPG.py"),
                   run_name="__main__")
