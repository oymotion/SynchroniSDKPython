# -*- coding: utf-8 -*-
"""设备规格定义（Device Spec）目录。

每个设备型号一份 spec 文件，定义该型号「应该支持」的能力（期望基线）。
测试用例从 spec 读取期望值做硬编码断言，设备行为偏离 spec 即 FAIL（而非 SKIP）。

关联方式：
  config.py 的 MODEL_SPEC 映射 name_prefix -> spec 文件名（不含 .py 后缀），
  例如 {"gForceUltra": "gforce_ultra"}。测试脚本通过 common.load_spec() 读取。

新增一个设备型号的步骤（不再逐次询问）：
  1. 在本目录复制一份已有 spec，改文件名与 SPEC 内容；
  2. 在 config.MODEL_SPEC 加一条 name_prefix -> 文件名 映射；
  3. 测试脚本 import common，用 load_spec(name_prefix) 读取期望值。
"""
