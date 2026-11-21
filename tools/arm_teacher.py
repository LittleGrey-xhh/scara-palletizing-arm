# arm_teacher.py —— 机械臂示教-复现上位机（V1 演示核心）
# 用法：
#   python arm_teacher.py                 交互模式：手动发命令，可边操作边记录
#   python arm_teacher.py demo_seq.txt    回放模式：按文件里的序列自动执行
#
# 序列文件格式（一行一条）：
#   C 0          命令，原样发到串口
#   !3           等待 3 秒
#   # 注释       回放时跳过
#
# 依赖：pip install pyserial
import serial                                    # pyserial
import serial.tools.list_ports                   # 枚举 COM 口用
import sys                                       # 命令行参数
import time                                      # 等待
import threading                                 # 后台读串口线程

BAUD = 115200                                    # 固件固定 115200 8N1

# ---------- 列出所有 COM 口，让用户挑 ----------
def pick_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:                                # 一个都没有
        print("没找到任何串口，插上 USB 再试")
        sys.exit(1)
    for i, p in enumerate(ports):                # 打印候选列表
        print(f"  [{i}] {p.device}  {p.description}")
    if len(ports) == 1:                          # 只有一个就直接用
        return ports[0].device
    while True:                                  # 多个就问
        s = input("选哪个口（输数字）: ").strip()
        if s.isdigit() and int(s) < len(ports):
            return ports[int(s)].device

# ---------- 后台线程：实时打印固件回显（HOME / ERR / POS 等） ----------
def reader(ser):
    while True:
        try:
            n = ser.in_waiting or 1              # 有多少读多少，没有就等 1 字节
            data = ser.read(n)
            if data:                             # 原样打到屏幕上
                sys.stdout.write(data.decode("ascii", errors="replace"))
                sys.stdout.flush()
        except Exception:                        # 串口关了就退出线程
            break

# ---------- 发一条命令，等它自己的回显落完 ----------
def send(ser, cmd):
    ser.write((cmd + "\n").encode("ascii"))      # 协议铁律：一行一条，\n 结尾
    time.sleep(0.15)                             # 给固件一点打印时间

# ---------- 等待 n 秒，期间串口回显由后台线程自动打印 ----------
def wait_s(sec):
    time.sleep(sec)

# ---------- 回放模式 ----------
def replay(ser, path):
    try:
        lines = open(path, encoding="utf-8").read().splitlines()
    except OSError as e:                         # 文件不存在
        print(f"读不到 {path}: {e}")
        sys.exit(1)

    print(f"=== 回放 {path}，共 {len(lines)} 行，Ctrl+C 中止 ===")
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):     # 空行和注释跳过
            continue
        if line.startswith("!"):                 # 等待指令
            sec = float(line[1:] or "1")
            print(f"... 等 {sec}s")
            wait_s(sec)
            continue
        print(f">> {line}")                      # 正常命令
        send(ser, line)
    print("=== 回放结束 ===")

# ---------- 交互模式：手动发命令，可同时记录 ----------
def interactive(ser):
    seq = []                                     # 记录缓冲
    recording = False
    print("交互模式。特俗命令： rec 开始记录 | save 保存序列 | q 退出")
    print("其他输入都当成串口命令直接发（例：M 180 60 / C 1 / S / !2）")
    while True:
        try:
            s = input("arm> ").strip()           # 读用户输入
        except (EOFError, KeyboardInterrupt):    # Ctrl+C / Ctrl+Z
            print()
            break
        if not s:
            continue
        if s == "q":                             # 退出
            break
        if s == "rec":                           # 开始记录
            recording = True
            seq.clear()
            print("开始记录（每条命令后自动加 !3 等待，可在文件里改）")
            continue
        if s == "save":                          # 保存到文件
            with open("demo_seq.txt", "w", encoding="utf-8") as f:
                f.write("# 示教序列（自动生成，改 !x 的秒数调节奏）\n")
                for l in seq:
                    f.write(l + "\n")
            print(f"已保存 {len(seq)} 行到 demo_seq.txt")
            recording = False
            continue
        if s.startswith("!"):                    # 手动插一条等待
            if recording:
                seq.append(s)
            print(f"... 等 {s[1:]}s")
            wait_s(float(s[1:] or "1"))
            continue
        send(ser, s)                             # 正常命令：发到串口
        if recording and not s.startswith("!"):  # 记录模式下顺手存一条
            seq.append(s)
            seq.append("!3")                     # 默认每步等 3 秒

# ---------- 入口 ----------
def main():
    port = None
    args = [a for a in sys.argv[1:]]             # 命令行参数
    if args and not args[0].startswith("-"):     # 第一个参数是序列文件 = 回放
        seq_file = args[0]
        if len(args) > 1:
            port = args[1]                       # 第二个参数可选指定 COM 口
        ser = serial.Serial(port or pick_port(), BAUD, timeout=0.1)
        threading.Thread(target=reader, args=(ser,), daemon=True).start()
        time.sleep(0.5)                          # 等固件启动打印
        replay(ser, seq_file)
        return
    ser = serial.Serial(pick_port(), BAUD, timeout=0.1)
    threading.Thread(target=reader, args=(ser,), daemon=True).start()
    interactive(ser)

if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as e:          # 口被占用之类的错误
        print(f"串口打开失败：{e}")
