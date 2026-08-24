# -*- coding: utf-8 -*-
"""PARAM-FUNC-009：NEUCIR_*（不适用）。

对应用例：04_参数.md -> PARAM-FUNC-009
可自动化：manual（不执行，仅记录）

说明：
  NEUCIR_*（NEUCIR_SET_MODE / NEUCIR_APP_CONTROL）为 NeuCir 远程控制设备专用
  参数，OYWW1100 非 NeuCir 设备，本用例不适用，仅记录"不适用"。

前置条件：无（不连接设备）
"""


def main():
    print("=" * 60, flush=True)
    print("PARAM-FUNC-009 NEUCIR_*（不适用）", flush=True)
    print("=" * 60, flush=True)

    print("\n[结论] 不适用", flush=True)
    print("  - 说明：OYWW1100 非 NeuCir 设备，NEUCIR_SET_MODE / NEUCIR_APP_CONTROL 不适用。", flush=True)
    print("  - 处理：不执行，仅记录。", flush=True)

    print("\n结论: 不适用（SKIP）", flush=True)


if __name__ == "__main__":
    main()
