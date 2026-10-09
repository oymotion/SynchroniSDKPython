# -*- coding: utf-8 -*-
"""临时脚本：解析 gForceUltra bin 的 CONFIG，读出 EMG 流分辨率/量程。"""
import struct
import sys

BIN = r"d:\James\Projects\Sensor SDK\Python\SynchroniSDKPython\Test Cases\Automation\gForceUltra\record_bin_csv\gForceUltra_data_20260929_104631.bin"

SLOT_NAMES = ["EEG", "ECG", "ACC", "GYRO", "BRTH", "EMG", "MAG", "QUAT",
              "PPG", "SPO2", "EULER", "(11 unused)", "IMPE", "GEST"]

data = open(BIN, "rb").read()

off = 0
configs = []
while off + 13 <= len(data):
    t = data[off]
    ts = struct.unpack_from("<Q", data, off + 1)[0]
    ln = struct.unpack_from("<I", data, off + 9)[0]
    if ln > 64 * 1024 * 1024 or off + 13 + ln > len(data):
        break
    payload = data[off + 13:off + 13 + ln]
    if t == 0x01 and ln >= 1024:
        configs.append((off, payload))
    off += 13 + ln

print(f"bin size = {len(data)} bytes, CONFIG records = {len(configs)}")

for ci, (coff, cfg) in enumerate(configs):
    # Header flags
    is_univ = cfg[26]
    is_new_emg = cfg[27]
    is_qat6 = cfg[28]
    # DeviceInfo block: firmware at offset 88+96 (4th char[32])
    fw = cfg[88 + 96:88 + 96 + 32].split(b"\x00")[0].decode("latin1", "replace")
    model = cfg[88 + 32:88 + 32 + 32].split(b"\x00")[0].decode("latin1", "replace")
    print(f"\n=== CONFIG #{ci} (file offset {coff}) model={model!r} firmware={fw!r} "
          f"isUniv={is_univ} isNewEmg={is_new_emg} isQat6={is_qat6} ===")
    for slot in range(14):
        base = 288 + slot * 48
        sr = struct.unpack_from("<d", cfg, base + 0)[0]
        k = struct.unpack_from("<d", cfg, base + 8)[0]
        ch = cfg[base + 29]
        idxlen = cfg[base + 30]
        resbits = cfg[base + 31]
        signed = cfg[base + 32]
        pkg = struct.unpack_from("<H", cfg, base + 36)[0]
        if pkg == 0 and ch == 0:
            continue
        full = (1 << resbits) if resbits else 0
        lo = -(1 << (resbits - 1)) if (signed and resbits) else 0
        hi = (1 << (resbits - 1)) - 1 if (signed and resbits) else (full - 1)
        print(f"  slot {slot:2d} {SLOT_NAMES[slot]:>12}: sr={sr:8.1f} k={k:.8g} "
              f"ch={ch} idxLen={idxlen} resBits={resbits} signed={signed} "
              f"pkg={pkg} | raw range=[{lo}, {hi}]")

# ---- 从 DATA 记录里解析 EMG(type=8) 帧，反推每样本字节数（16bit=2 / 24bit=3）----
print("\n=== EMG DATA 帧解析（反推采样位宽） ===")
buf = bytearray()
emg_frame_sizes = []
data_seen = 0
off = 0
while off + 13 <= len(data):
    t = data[off]
    ln = struct.unpack_from("<I", data, off + 9)[0]
    if ln > 64 * 1024 * 1024 or off + 13 + ln > len(data):
        break
    payload = data[off + 13:off + 13 + ln]
    if t == 0x02:  # DATA
        data_seen += 1
        buf += payload
        # 从 buffer 反复剥离 0x55 帧
        while len(buf) >= 3:
            if buf[0] != 0x55:
                del buf[0]
                continue
            n = buf[1]
            need = 1 + 1 + n + 2  # 0x55 + N + payload + crc2
            if len(buf) < need:
                break
            frame_payload = bytes(buf[2:2 + n])
            if frame_payload and (frame_payload[0] & 0x7F) == 8:  # EMG
                emg_frame_sizes.append(n)
            del buf[:need]
    if data_seen >= 500 and len(emg_frame_sizes) >= 5:
        break
    off += 13 + ln

print(f"  DATA 记录扫描 {data_seen} 条，抓到 EMG 帧 {len(emg_frame_sizes)} 个")
if emg_frame_sizes:
    from collections import Counter
    print(f"  EMG 帧 payload 长度 N 分布: {dict(Counter(emg_frame_sizes))}")
    n = emg_frame_sizes[0]
    # 内层包 = type(1) + idxLen(2) + ch*pkg*bps
    for bps in (2, 3):
        calc = 1 + 2 + 8 * 15 * bps
        print(f"    假设 {bps} 字节/样本 -> 帧 payload 应为 {calc} 字节")

