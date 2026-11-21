// app_main.c —— 应用层：串口命令 → 舵机 + 滑台
// 第一版当"标定固件"用：串口直接发脉宽，测出三个舵机的中位和安全范围
#include "app_main.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "servo.h"
#include "step.h"
#include "kinematics.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ---------- 串口接收 ----------
static uint8_t  s_ch;                   // 单字节接收缓冲
static char     s_line[48];             // 命令行缓冲
static uint8_t  s_len = 0;              // 当前攒了多少字符

// ---------- 夹爪两个位置：★ 用脉宽，不用角度 ----------
// 为什么不用角度：夹爪 0~180 映射到 600~2400us，20us 只值 0.9°，取整就没了，
// 标定时摸出来的"20us 回退量"在角度层面根本表达不出来。
//
// 机械爪连杆到两端会顶死，舵机闭环保位静差压在卡点上 → 堵转嗡嗡（2.5A）。
// 11-11 实测有效的做法：先过冲顶到死位（吃干净齿隙），再退 20us 落在有余量的位置。
// ★ 必须走完"过冲 → 回退"两段，直接从中间位置发 settle 值反而会堵转。
#define GRIP_OPEN_OVER_US    1000       // 打开：先冲到 1000
#define GRIP_OPEN_SET_US     1020       // 再退到 1020（安静值）
#define GRIP_CLOSE_OVER_US   1600       // 闭合：先冲到 1600（轻微堵转，正常）
#define GRIP_CLOSE_SET_US    1580       // 再退到 1580（安静值，仍夹得住）
#define GRIP_OVER_MS          300       // 过冲段耗时（ms），夹爪要快
#define GRIP_SET_MS           150       // 回退段耗时（ms）
#define GRIP_SETTLE_TICKS     400       // 过冲到位后等多久再回退（ms）

// ---------- 状态 ----------
static uint32_t s_tick20   = 0;         // 20ms 节拍时间戳
static uint8_t  s_homed    = 0;         // 有没有回过零
static uint8_t  s_grip     = 0;         // 夹爪状态：0=张开 1=闭合
static uint8_t  s_grip_stage = 0;       // 夹爪两段式：0=空闲 1=已过冲，等回退
static uint32_t s_grip_t0    = 0;       // 过冲开始的时间戳
static uint8_t  s_home_last  = 0xFF;    // 上一次播报的回零状态（0xFF = 还没播报过）

// ---------- printf 重定向到 USART1 ----------
// Keil AC5 用 fputc；换成 GCC 工具链要改成 _write
int fputc(int ch, FILE *f)
{
    uint8_t c = (uint8_t)ch;                            // 转成字节
    HAL_UART_Transmit(&huart1, &c, 1, 10);              // 阻塞发一个字节，10ms 超时
    (void)f;                                            // 没用到的参数
    return ch;                                          // 返回原字符表示成功
}

// ---------- 从字符串里取下一个整数，返回剩余部分的指针 ----------
static char *next_int(char *s, int *v)
{
    while (*s == ' ' || *s == ',' || *s == '\t') s++;   // 跳过空白和逗号
    if (*s == 0) return 0;                              // 到结尾了
    *v = atoi(s);                                       // 取整数
    while (*s && *s != ' ' && *s != ',' && *s != '\t') s++;   // 跳过这个数字
    return s;                                           // 返回剩余部分
}

// ---------- 打印帮助 ----------
static void print_help(void)
{
    printf("--- Arm Commands ---\r\n");
    printf("M x y    move, wrist auto-kept perpendicular to X\r\n");
    printf("Z mm     lift to absolute height, 0 = home\r\n");
    printf("ZL mm    set Z soft limit (default 60)\r\n");
    printf("P x y z [rot]  full point, rot = wrist servo deg (auto if omitted)\r\n");
    printf("W deg    wrist servo deg 0-180 (next M/P will override it)\r\n");
    printf("C 0/1    gripper open/close\r\n");
    printf("H        home Z axis\r\n");
    printf("HM       home servos to power-on pose (arm folded)\r\n");
    printf("L        read limit switch: LIM 0/1\r\n");
    printf("S        query position\r\n");
    printf("E 0/1    motor disable/enable\r\n");
    printf("T mm [freq]  test jog, +/-50mm max, freq default 4000 (10mm/s)\r\n");
    printf("K id us [ms]  servo calib slow, id 0-3, ms default 2000\r\n");
    printf("R        clear fault\r\n");
    printf("?        this help\r\n");
}

// ---------- 打印位置 ----------
// 用正解把舵机角反算成桌面坐标，返回的格式和 P 命令的输入格式对齐
static void print_pos(void)
{
    ArmJoint_t j;
    float x = 0.0f, y = 0.0f;

    j.servo1_deg = servo_get_deg(SERVO_ARM1);       // 读大臂当前指令角
    j.servo2_deg = servo_get_deg(SERVO_ARM2);       // 读小臂当前指令角
    servo_to_arm(&j);                               // 舵机指令角 → 关节角
    fk_solve(j.theta1_deg, j.theta2_deg, &x, &y);   // 关节角 → 桌面坐标

    printf("POS %d %d %d %d %d\r\n",
           (int)x, (int)y,                          // 桌面 X Y（mm）
           (int)step_pos_mm(),                      // Z 高度
           (int)servo_get_deg(SERVO_WRIST),         // 腕转角
           (int)s_grip);                            // 夹爪状态
}

// ---------- 一条命令 ----------
static void app_cmd(char *line)
{
    char c = line[0];                                   // 命令首字母
    int  a = 0, b = 0, d = 0;                           // 参数
    char *p;                                            // 解析游标

    switch (c)
    {
        case 'H':                                       // H = 回零 / HM = 舵机回上电姿势
            // ★ 11-20 新增 HM（Home Mechanical）：四路舵机回到上电那个收拢姿势。
            //   以前只能用 K 一条条发脉宽，或者直接断电重启，现在一条命令搞定。
            //   用的是 servo.c 里同一张 g_servo_home_us 表，不会和 init 脱节。
            if (line[1] == 'M' || line[1] == 'm')
            {
                // 先收小臂（半径变小最安全），再大臂，最后腕转；三路同时走 3 秒斜坡
                servo_cal_set_us(SERVO_ARM2,  g_servo_home_us[SERVO_ARM2],  3000);
                servo_cal_set_us(SERVO_ARM1,  g_servo_home_us[SERVO_ARM1],  3000);
                servo_cal_set_us(SERVO_WRIST, g_servo_home_us[SERVO_WRIST], 3000);
                // ★ 夹爪走张开安全位 1020，不留 1500 那个中间值（5.17 堵转温床）
                servo_cal_set_us(SERVO_GRIP,  GRIP_OPEN_SET_US, 1500);
                printf("OK\r\n");
                break;
            }

            step_home_start();                          // H：Z 轴回零
            printf("OK\r\n");
            break;

        case 'M':                                       // M x y 平面移动
            p = next_int(line + 1, &a);
            if (!p) { printf("ERR 1\r\n"); break; }     // 参数不够
            next_int(p, &b);
            {
                ArmJoint_t j;
                int rc = ik_solve((float)a, (float)b, -1, &j);   // 逆解
                if (rc == KIN_ERR_OUTER) { printf("ERR 2\r\n"); break; }   // 太远
                if (rc == KIN_ERR_INNER) { printf("ERR 2\r\n"); break; }   // 太近
                if (rc != KIN_OK)        { printf("ERR 3\r\n"); break; }   // 舵机行程不够

                servo_move_deg(SERVO_ARM1, j.servo1_deg, 1.5f);   // 每拍走 1.5°
                servo_move_deg(SERVO_ARM2, j.servo2_deg, 1.5f);
                // ★ 11-20：腕转跟着解耦，让夹爪绝对朝向恒定（默认垂直 x 轴）
                //   不加这行的话，腕转不动、小臂绝对角在变 → 夹爪朝向随位置变，抓取必夹歪
                servo_move_deg(SERVO_WRIST,
                               wrist_deg_for_abs(j.theta1_deg, j.theta2_deg,
                                                 WRIST_ABS_PERPENDICULAR_X), 3.0f);
                printf("OK\r\n");
            }
            break;

        case 'Z':                                       // Z mm 升降 / ZL mm 设软上限
            if (line[1] == 'L' || line[1] == 'l')       // ★ ZL：物理上限位不做开关，靠这个守
            {
                p = next_int(line + 2, &a);
                if (!p) { printf("ERR 1\r\n"); break; }
                if (a <= 0 || a > (int)STEP_MAX_MM) { printf("ERR 2\r\n"); break; }
                step_set_soft_max((float)a);
                printf("OK %d\r\n", (int)step_soft_max_mm());
                break;
            }
            p = next_int(line + 1, &a);
            if (!p) { printf("ERR 1\r\n"); break; }
            if (!s_homed) { printf("ERR 5\r\n"); break; }      // 没回零，Z 坐标没意义
            {
                int rc = step_move_to_mm((float)a, 4000);      // 10mm/s
                if (rc == STEP_BUSY)       printf("ERR 4\r\n"); // 电机忙
                else if (rc == STEP_RANGE) printf("ERR 2\r\n"); // 超软上限（含负数，下方就是限位）
                else                       printf("OK\r\n");
            }
            break;

        case 'P':                                       // P x y z [rot] 完整点位
            p = next_int(line + 1, &a);
            if (!p) { printf("ERR 1\r\n"); break; }
            p = next_int(p, &b);
            if (!p) { printf("ERR 1\r\n"); break; }
            p = next_int(p, &d);
            if (!p) { printf("ERR 1\r\n"); break; }
            {
                int rot = 0;
                int has_rot = (next_int(p, &rot) != 0);          // 可选的第 4 个参数

                ArmJoint_t j;
                int rc;

                // ★ 先查 Z 再逆解：Z 是唯一会撞机械死挡的轴，越界就直接拒，别让臂白动一趟
                if (!s_homed) { printf("ERR 5\r\n"); break; }
                if (d < 0 || (float)d > step_soft_max_mm()) { printf("ERR 2\r\n"); break; }

                rc = ik_solve((float)a, (float)b, -1, &j);
                if (rc != KIN_OK) { printf("ERR %d\r\n", (rc == KIN_ERR_JOINT) ? 3 : 2); break; }

                if (step_move_to_mm((float)d, 4000) != STEP_OK)    // 先动 Z
                {
                    printf("ERR 4\r\n");                           // 电机还在忙，整条命令不执行
                    break;
                }
                servo_move_deg(SERVO_ARM1, j.servo1_deg, 1.5f);   // 再动臂
                servo_move_deg(SERVO_ARM2, j.servo2_deg, 1.5f);
                // ★ 11-20：给了第 4 个参数就照它当舵机指令角（老行为）；
                //   没给就按"夹爪垂直 x 轴"自动解耦，保证任何位置朝向一致
                if (has_rot)
                    servo_move_deg(SERVO_WRIST, (float)rot, 3.0f);
                else
                    servo_move_deg(SERVO_WRIST,
                                   wrist_deg_for_abs(j.theta1_deg, j.theta2_deg,
                                                     WRIST_ABS_PERPENDICULAR_X), 3.0f);
                printf("OK\r\n");
            }
            break;

        case 'W':                                       // W deg 腕转
            p = next_int(line + 1, &a);
            if (!p) { printf("ERR 1\r\n"); break; }
            if (a < 0 || a > 180) { printf("ERR 2\r\n"); break; }
            servo_move_deg(SERVO_WRIST, (float)a, 3.0f);  // 腕转负载轻，每拍 3° 够稳
            printf("OK\r\n");
            break;

        case 'C':                                       // C 0/1 夹爪（两段式：过冲 + 回退）
            p = next_int(line + 1, &a);
            if (!p) { printf("ERR 1\r\n"); break; }
            s_grip = (a != 0) ? 1 : 0;
            // 第一段：快速冲到死位，把齿隙吃干净
            servo_cal_set_us(SERVO_GRIP,
                             s_grip ? GRIP_CLOSE_OVER_US : GRIP_OPEN_OVER_US,
                             GRIP_OVER_MS);
            s_grip_stage = 1;                           // 标记"已过冲，等回退"
            s_grip_t0    = HAL_GetTick();               // 记下过冲起点
            printf("OK\r\n");
            break;

        case 'T':                                       // T mm [freq]  测试：走相对距离，可指定频率
            p = next_int(line + 1, &a);
            if (!p) { printf("ERR 1\r\n"); break; }
            // ★★ 11-20 事故后收紧：单次限 ±50mm（原来 ±200），速度默认减半
            //   方向约定：正 = 向上，负 = 向下（11-20 修了 DIR 极性之后才成立）
            if (a > 50 || a < -50) { printf("ERR 2\r\n"); break; }
            {
                int   f   = 4000;                       // 默认 4000Hz = 10mm/s
                float tgt;                              // ★ 声明必须在语句前（Keil AC5 是 C90）

                next_int(p, &f);                        // 可选第二参数：脉冲频率
                if (f < 200)   f = 200;                 // 不低于 STEP_F_MIN
                if (f > 12000) f = 12000;               // 不超 30mm/s 硬上限

                // ★ 只有回过零才管绝对位置。没回零时 s_pos 没有基准，
                //   只能靠上面那个 ±50mm 限幅兜着，剩下的靠眼睛
                if (s_homed)
                {
                    tgt = step_pos_mm() + (float)a;     // 走完会到哪
                    if (tgt < 0.0f || tgt > step_soft_max_mm())
                    {
                        printf("ERR 2\r\n");            // 会撞下限位 / 超软上限，拒绝
                        break;
                    }
                }

                if (step_move_mm((float)a, (uint32_t)f) != STEP_OK)
                    printf("ERR 4\r\n");                // 电机还在忙
                else
                    printf("OK %d %d\r\n", a, f);
            }
            break;

        case 'S':                                       // 查询
            print_pos();
            break;

        case 'L':                                       // 限位实时状态（装传感器靠这个验收）
            // 手拿铁片靠近/离开传感器，回显跟着变 = 接线通了
            // 一直是 0 → 上拉正常但传感器没工作（查 24V 供电 / 黑线断）
            // 一直是 1 → 黑线被拉死在 0V（查短路 / 传感器反接）
            printf("LIM %d\r\n", step_limit_active());
            break;

        case 'E':                                       // E 0/1 松轴 / 锁轴
            p = next_int(line + 1, &a);
            if (!p) { printf("ERR 1\r\n"); break; }
            step_enable((a != 0) ? 1 : 0);
            printf("OK\r\n");
            break;

        case 'K':                                       // K id us [ms] 标定：慢速给脉宽
            p = next_int(line + 1, &a);
            if (!p) { printf("ERR 1\r\n"); break; }
            p = next_int(p, &b);
            if (!p) { printf("ERR 1\r\n"); break; }
            if (a < 0 || a >= SERVO_NUM) { printf("ERR 1\r\n"); break; }
            if (b < 400 || b > 2600)     { printf("ERR 2\r\n"); break; }   // 标定允许略超标准值
            {
                int ms = 2000;                          // 默认 2 秒慢慢走，不吵
                next_int(p, &ms);                       // 给了第 3 个参数就用它
                if (ms < 0)      ms = 0;                // 负数当 0 = 直跳（老行为）
                if (ms > 10000)  ms = 10000;            // 上限 10 秒，防手滑
                servo_cal_set_us((ServoId_t)a, (uint16_t)b, (uint16_t)ms);  // ★ 绕过夹紧 + 慢速
                printf("OK %d %d %d\r\n", a, b, ms);
            }
            break;

        case 'R':                                       // 清故障
            step_limit_fault_clear();
            step_home_abort();
            s_homed = 0;
            s_home_last = (uint8_t)HOME_IDLE;           // 播报状态同步回空闲
            printf("OK\r\n");
            break;

        case '?':                                       // 帮助
        case '\0':
            print_help();
            break;

        default:
            printf("ERR 1\r\n");                        // 不认识的命令
            break;
    }
}

// ---------- 串口来一个字节 ----------
static void app_uart_byte(uint8_t b)
{
    if (b == '\n' || b == '\r')                         // 收到换行，一条命令结束
    {
        s_line[s_len] = 0;                              // 补字符串结束符
        if (s_len > 0) app_cmd(s_line);                 // 空行不处理
        s_len = 0;                                      // 清缓冲
        return;
    }

    if (s_len < sizeof(s_line) - 1)                     // 还有地方就存
    {
        s_line[s_len++] = (char)b;
    }
    else                                                // 满了，丢掉并清空，防溢出
    {
        s_len = 0;
    }
}

// ---------- 初始化 ----------
void app_init(void)
{
    servo_init();                                       // 舵机先回中位再启动 PWM
    step_init();                                        // 步进初始化，默认松轴

    printf("\r\n=== Arm Controller v0.1 ===\r\n");
    printf("type ? for help\r\n");
}

// ---------- 主循环 ----------
void app_loop(void)
{
    // 1. 收串口（超时 0，收不到就立刻返回，不阻塞）
    while (HAL_UART_Receive(&huart1, &s_ch, 1, 0) == HAL_OK)
    {
        app_uart_byte(s_ch);
    }

    // 2. 20ms 节拍：舵机慢慢挪、回零状态机推进
    if (HAL_GetTick() - s_tick20 >= 20)
    {
        HomeState_t st;

        s_tick20 = HAL_GetTick();
        servo_update();                                 // 舵机走一小步

        st = step_home_poll();                          // 推进回零状态机

        // ★ 只有 HOME_DONE 才算"坐标可信"。原来写成 poll()==DONE 就置 1 且永不清零，
        //   重新回零失败后 s_homed 还是 1，Z 命令会照着一堆假坐标走
        s_homed = (st == HOME_DONE) ? 1 : 0;

        if ((uint8_t)st != s_home_last)                  // 状态变了就播报一次，别刷屏
        {
            s_home_last = (uint8_t)st;
            if      (st == HOME_BACKOFF) printf("HOME UP\r\n");       // 先向上离开
            else if (st == HOME_SEEK)    printf("HOME SEEK\r\n");     // 向下找原点
            else if (st == HOME_RETREAT) printf("HOME RETREAT\r\n");  // 退让 3mm
            else if (st == HOME_DONE)    printf("HOME DONE Z0 %d\r\n",
                                                (int)step_home_offset_pulse());  // 末位=触发点相对起点的脉冲
            else if (st == HOME_ERROR)   printf("ERR 9\r\n");         // 回零失败
        }
    }

    // 3. 夹爪两段式第二段：过冲过去之后退 20μs，落在安静的位置
    if (s_grip_stage && (HAL_GetTick() - s_grip_t0 >= GRIP_SETTLE_TICKS))
    {
        servo_cal_set_us(SERVO_GRIP,
                         s_grip ? GRIP_CLOSE_SET_US : GRIP_OPEN_SET_US,
                         GRIP_SET_MS);
        s_grip_stage = 0;                               // 两段走完，回到空闲
    }

    // 4. 限位故障：不是回零时撞限位 = 出事了，停机
    if (step_limit_fault())
    {
        step_stop();
        printf("ERR 6\r\n");                            // 报一次就够，别刷屏
        step_limit_fault_clear();

        // ★ 撞了限位说明脉冲计数已经和实际对不上了，回零结果作废，必须重新 H。
        //   只清 s_homed 没用——下一拍 s_homed 会被 poll 的 DONE 状态重新置 1
        step_home_abort();
        s_home_last = (uint8_t)HOME_IDLE;
    }
}

// ---------- TIM 中断回调 ----------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)                         // 只有步进脉冲用定时器中断
    {
        step_on_tim_update();                           // 计一个脉冲
    }
}

// ---------- 外部中断回调 ----------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LIMIT_Z_Pin)                        // 下限位触发
    {
        step_on_limit_triggered();                      // 立刻停 + 记零点
    }
}
