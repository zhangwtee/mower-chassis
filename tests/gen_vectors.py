#!/usr/bin/env python3
"""生成遥测帧的期望字节，给 C 测试比对用。

按 mower-host 里 bridge/mower_bridge/protocol.py 的组帧逻辑算一遍，写进 vectors.h。
这样"固件组出来的帧 == 上位机能解出来的帧"就成了可跑的断言。

用法：
    python tests/gen_vectors.py            # 重新生成 vectors.h
    python tests/gen_vectors.py --check    # 只检查现有 vectors.h 是不是最新

改协议时：先改上位机那份，再跑本脚本，再让 C 测试过。
"""

import os
import sys

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D

CHASSIS_LEN = 24
DISTANCE_LEN = 19
DIST_HEADER = 0xFA
DIST_TAIL = 0xFC


def bcc(data, n):
    """前 n 字节逐字节异或。固件 Check_BCC 同此语义。"""
    v = 0
    for i in range(n):
        v ^= data[i]
    return v


def _u16(v):
    """int16 值 → 大端两字节（负数落成补码）。"""
    v &= 0xFFFF
    return [(v >> 8) & 0xFF, v & 0xFF]


def build_chassis(flag_stop, vel, accel, gyro, voltage_mv):
    """24 字节底盘帧。

    布局：[0]0x7B [1]FlagStop [2:8]Vx,Vy,Vz [8:14]Acc [14:20]Gyro
          [20:22]电压 [22]BCC(22) [23]0x7D
    """
    f = bytearray(CHASSIS_LEN)
    f[0] = FRAME_HEADER
    f[1] = flag_stop & 0xFF
    o = 2
    for v in vel:
        f[o:o + 2] = bytes(_u16(v))
        o += 2
    for v in accel:
        f[o:o + 2] = bytes(_u16(v))
        o += 2
    for v in gyro:
        f[o:o + 2] = bytes(_u16(v))
        o += 2
    f[20:22] = bytes(_u16(voltage_mv))
    f[22] = bcc(f, 22)
    f[23] = FRAME_TAIL
    return bytes(f)


def build_distance(dist_mm):
    """19 字节超声波帧。

    布局：[0]0xFA [1:17]A~H 各 int16 大端 [17]BCC(17) [18]0xFC
    """
    assert len(dist_mm) == 8
    f = bytearray(DISTANCE_LEN)
    f[0] = DIST_HEADER
    o = 1
    for v in dist_mm:
        f[o:o + 2] = bytes(_u16(v))
        o += 2
    f[17] = bcc(f, 17)
    f[18] = DIST_TAIL
    return bytes(f)


# ---- 测试用例（C 端 tests/test_telemetry.c 引用同名符号）----

CHASSIS_CASES = [
    # 名称, flag_stop, vel(Vx,Vy,Vz mm/s), accel, gyro, 电压 mV
    ("chassis_normal", 0, (500, -250, 0), (1, -2, 1671), (-100, 0, 50), 24000),
    ("chassis_stopped", 1, (0, 0, 0), (0, 0, 0), (0, 0, 0), 0),
    ("chassis_extremes", 0, (32767, -32768, 1000), (-32768, 32767, 0), (32767, -32768, 1), 65535),
]

DISTANCE_CASES = [
    ("distance_normal", (123, 456, 789, 1000, 1500, 2000, 0, 0)),
    ("distance_extremes", (32767, -32768, 0, 1, -1, 65535 & 0x7FFF, 0, 0)),
    ("distance_none", (0, 0, 0, 0, 0, 0, 0, 0)),
]


def c_array(name, data):
    body = ",".join("0x%02X" % b for b in data)
    return "static const uint8_t %s[%d] = { %s };" % (name, len(data), body)


def render():
    out = []
    out.append("/* 由 tests/gen_vectors.py 生成，勿手改。")
    out.append(" *")
    out.append(" * 按上位机 mower-host/bridge/mower_bridge/protocol.py 生成")
    out.append(" * 重新生成： python tests/gen_vectors.py")
    out.append(" */")
    out.append("#ifndef TEST_VECTORS_H")
    out.append("#define TEST_VECTORS_H")
    out.append("")
    out.append("#include <stdint.h>")
    out.append("")
    out.append("/* ---- 24 字节底盘帧 ---- */")
    for name, flag, vel, acc, gyr, mv in CHASSIS_CASES:
        out.append("/* flag_stop=%d vel=(%d,%d,%d) accel=(%d,%d,%d) gyro=(%d,%d,%d) mv=%d */"
                   % ((flag,) + vel + acc + gyr + (mv,)))
        out.append(c_array("golden_" + name, build_chassis(flag, vel, acc, gyr, mv)))
        out.append("")
    out.append("/* ---- 19 字节超声波帧 ---- */")
    for name, dist in DISTANCE_CASES:
        out.append("/* dist=(%s) */" % ",".join(str(d) for d in dist))
        out.append(c_array("golden_" + name, build_distance(dist)))
        out.append("")
    out.append("/* ---- 帧长与校验位偏移，供 C 端对齐 ---- */")
    out.append("#define GOLDEN_CHASSIS_LEN   %d" % CHASSIS_LEN)
    out.append("#define GOLDEN_DISTANCE_LEN  %d" % DISTANCE_LEN)
    out.append("#define GOLDEN_DIST_HEADER   0x%02X" % DIST_HEADER)
    out.append("#define GOLDEN_DIST_TAIL     0x%02X" % DIST_TAIL)
    out.append("")
    out.append("#endif /* TEST_VECTORS_H */")
    return "\n".join(out) + "\n"


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    target = os.path.join(here, "vectors.h")
    text = render()

    if "--check" in sys.argv:
        if not os.path.exists(target):
            print("FAIL: vectors.h 不存在")
            return 1
        with open(target, "r", encoding="utf-8", newline="") as f:
            cur = f.read()
        if cur.replace("\r\n", "\n") == text:
            print("OK: vectors.h 与生成器一致")
            return 0
        print("FAIL: vectors.h 已过期，请重新生成")
        return 1

    with open(target, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print("写出 %s（%d 字节）" % (target, len(text)))
    for name, flag, vel, acc, gyr, mv in CHASSIS_CASES:
        print("  %-20s %s" % (name, build_chassis(flag, vel, acc, gyr, mv).hex(" ")))
    for name, dist in DISTANCE_CASES:
        print("  %-20s %s" % (name, build_distance(dist).hex(" ")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
