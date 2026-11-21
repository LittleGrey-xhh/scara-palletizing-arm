# -*- coding: utf-8 -*-
# servo_calib.py —— 舵机标定边界扫描工具
# 用法: python servo_calib.py COM5 [舵机id]
#   例: python servo_calib.py COM5 0
#
# 操作:
#   回车     脉宽 +50（往 us_max 方向）
#   b+回车   反向（往 us_min 方向）
#   数字+回车 直接跳到该脉宽，如 1500
#   r+回车   回中位 1500
#   q+回车   退出并打印本次记录
#
# 判据（耳朵负责）: 正常转动 = 沙沙声; 顶到限位 = 持续嗡嗡 + 外壳变温。
# 听到嗡嗡立刻退 100（按 b 两次），退回后的值就是 us_min / us_max。
#
# 固件 K 命令: K <id> <us>，id 0=大臂 1=小臂 2=腕转 3=夹爪，us 允许 400~2600。

import sys, time
import serial

STEP = 50          # 每按一次回车走的脉宽
CENTER = 1500      # 中位
US_LO, US_HI = 400, 2600   # 和固件 K 命令的限制一致


def clamp_us(us):
    if us < US_LO: return US_LO
    if us > US_HI: return US_HI
    return us


def send_k(ser, sid, us):
    cmd = "K %d %d\n" % (sid, us)
    ser.write(cmd.encode())
    time.sleep(0.05)
    # 固件会回 "OK id us" 或 "ERR n"
    line = ser.readline().decode(errors="replace").strip()
    return line


def main():
    if len(sys.argv) < 2:
        print("用法: python servo_calib.py COM5 [id]")
        sys.exit(1)

    port = sys.argv[1]
    sid = int(sys.argv[2]) if len(sys.argv) >= 3 else 0
    if not (0 <= sid <= 3):
        print("id 只能是 0~3（0=大臂 1=小臂 2=腕转 3=夹爪）")
        sys.exit(1)

    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(0.3)
    # 清一下启动横幅
    ser.reset_input_buffer()

    us = CENTER
    direction = 1                     # 1 = 往上限扫，-1 = 往下限扫
    history = []                      # (脉宽, 固件应答)
    rmin = rmax = None                # 扫出来的边界

    print("=" * 50)
    print("舵机 %d 标定  (%s, 115200)" % (sid, port))
    print("id: 0=大臂 1=小臂 2=腕转 3=夹爪")
    print("回车=+%d  b=反向  数字=直达  r=回中位  q=退出" % STEP)
    print("听到嗡嗡堵转声就退 100（b 按两次），退回值即边界")
    print("=" * 50)

    resp = send_k(ser, sid, us)
    history.append((us, resp))
    print("[%4d us] %s" % (us, resp))

    while True:
        raw = input(">> ").strip().lower()
        if raw == "q":
            break
        elif raw == "b":
            direction = -direction
            print("方向: %s" % ("往上限(+)" if direction == 1 else "往下限(-)"))
            continue
        elif raw == "r":
            us = CENTER
        elif raw == "":
            us = us + direction * STEP
        else:
            try:
                us = int(raw)
            except ValueError:
                print("看不懂，回车步进 / b 反向 / 数字直达 / q 退出")
                continue

        us = clamp_us(us)
        resp = send_k(ser, sid, us)
        history.append((us, resp))
        print("[%4d us] %s" % (us, resp))

        if not resp.startswith("OK"):
            print("!! 固件报错（超范围或串口没通），检查一下")

    # 粗略推断边界：最后一次到达的极值
    if history:
        vals = [h[0] for h in history]
        rmax = max(vals)
        rmin = min(vals)

    print("=" * 50)
    print("本次会话共 %d 条指令" % len(history))
    print("扫到的脉宽范围: %d ~ %d" % (rmin, rmax))
    print("（这只是走过范围，us_min/us_max 以你听到嗡嗡后退回的值为准）")
    print()
    print("填表格式（发给大肥鱼）:")
    print("  id=%d  us_min=____  us_max=____" % sid)
    print("=" * 50)

    ser.close()


if __name__ == "__main__":
    main()
