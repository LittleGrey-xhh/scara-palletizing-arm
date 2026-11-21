// servo.c —— 三路舵机驱动
// 核心就一句话：角度 0~180 线性映射成脉宽 us_min~us_max，写进 CCR
// 因为 TIM2/TIM4 都是 PSC=71 → 1MHz 计数、ARR=19999 → 50Hz，
// 所以 CCR 的数值就是脉宽的微秒数，不用任何换算
#include "servo.h"
#include "tim.h"        // CubeMX 生成的，里面有 htim2 / htim4
#include <math.h>       // sqrtf / fabsf

// ---------- 速度参数 ----------
#define SERVO_STEP_MAX   1.5f       // 默认最大速度：度/拍。20ms 一拍 → 75°/s
#define SERVO_STEP_MIN   0.15f      // 最低速度（3°/s），再慢舵机就抖着走了
#define SERVO_ACCEL      0.15f      // 每拍允许的速度增量（度/拍²）。0→1.5 要 10 拍 = 0.2s

// ---------- 上电回正 ----------
#define SERVO_HOME_MS   3000        // 上电回正斜坡时长（ms）。想快想慢改这一个值
                                    // ★ 0 = 回到老行为：PWM 一启动直接写目标，舵机猛冲

// ---------- 标定表：P0-1 测完把实测值填进来 ----------
// 顺序必须和 ServoId_t 一致：0=大臂 1=小臂 2=腕转 3=夹爪
// 理论值：MG996R 是 180° 舵机，0.5ms=一端，2.5ms=另一端，1.5ms=中位
// ★ 500/2500 是理论极限，实际顶到这个位置舵机已经顶死摩擦了，
//   所以标定完要把 us_min / us_max 收紧到"机械真正能到的位置"
const ServoCal_t g_servo_cal[SERVO_NUM] =
{
    // us_min, us_max, dir   ★ dir 这个字段换算里没用（方向靠 kinematics.h 的 SERVOx_DIR），别指望它
    // 实测 2026-11-11，手机指南针，−90°/+90° 两端
    {   520,   2450,   1.0f },      // 0 大臂  −90°=520  +90°=2450  跨度1930μs 增益0.0933
    {   550,   2460,   1.0f },      // 1 小臂  −90°=550  +90°=2460  跨度1910μs 增益0.0942
    {   500,   2520,   1.0f },      // 2 腕转  −90°=500  +90°=2520  跨度2020μs 增益0.0891
    {   600,   2400,  -1.0f },      // 3 夹爪  ★未标定：先给保守窄范围，装机械爪后按实际张开/闭合改
};

static ServoState_t s_st[SERVO_NUM];    // 四路的运行状态

// ---------- 上电回正位（脉宽，μs），顺序同 ServoId_t ----------
// ★ 小臂回 2460 = 折到底（θ2=−180，舵机角 180°）：
//   小臂和夹爪的质心收回到大臂轴附近，静力矩最小 = 真正的待机收拢
//   （550 伸直是静力矩最大的姿势，交接 5.9 弯臂超载，不能当待机位）
// ★ 腕转回 85°（1454μs）：夹爪平行于大臂打开方向，11-17 实测
// ★ 11-20 复验：这个姿势下小臂叠在大臂舵机下方、夹爪收在最里侧原点，
//   不顶夹板 → 所以 SERVO2_MAX_DEG 可以放开到 180.5，M 35 0 也能走回原点
// ★ 11-20 起 servo_init() 和 HM 命令共用这一份，别在别处另写一套
const uint16_t g_servo_home_us[SERVO_NUM] = { 1485, 2460, 1454, 1500 };

// ---------- 标定慢速斜坡（K 命令专用） ----------
// 原来的 K 命令是直接写 CCR，舵机以自身最大速度冲过去，动静大。
// 这里维护每路一条"脉宽斜坡"，由 servo_update() 每 20ms 推一小步，
// 走到目标就停，默认 2 秒走完，唱得慢、听着安静。
static uint16_t s_cal_cur[SERVO_NUM];   // 当前脉宽（跟踪值，斜坡起点）
static uint16_t s_cal_tgt[SERVO_NUM];   // 目标脉宽
static int16_t  s_cal_inc[SERVO_NUM];   // 每拍走多少微秒（带符号）
static uint8_t  s_cal_run[SERVO_NUM];   // 1 = 这路正在斜坡中

// 把浮点数夹到区间里
static float clampf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;              // 低于下限
    if (v > hi) return hi;              // 高于上限
    return v;                           // 区间内原样
}

// 角度（0~180）换算成脉宽微秒数
uint16_t servo_deg_to_us(ServoId_t id, float deg)
{
    const ServoCal_t *c = &g_servo_cal[id];         // 取这路的标定数据
    float us;

    deg = clampf_local(deg, 0.0f, 180.0f);          // 先夹住角度，绝不给舵机超程指令
    us  = (float)c->us_min                          // 从下限开始
        + ((float)(c->us_max - c->us_min) * deg / 180.0f);  // 按比例加上行程

    return (uint16_t)(us + 0.5f);                   // 四舍五入成整数微秒
}

// 直接写脉宽（标定固件用这个，绕过角度换算）
void servo_set_us(ServoId_t id, uint16_t us)
{
    const ServoCal_t *c;                            // 标定数据指针

    if (id >= SERVO_NUM) return;                    // ★ 编号越界直接退，否则后面会越界写 s_cal_cur

    c = &g_servo_cal[id];                           // 取标定数据
    if (us < c->us_min) us = c->us_min;             // ★ 硬夹住，绝不让机械顶死
    if (us > c->us_max) us = c->us_max;

    switch (id)                                     // 按编号写对应的 CCR
    {
        case SERVO_ARM1:                            // 大臂走 TIM2 通道 1
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, us);
            break;

        case SERVO_ARM2:                            // 小臂走 TIM2 通道 2
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, us);
            break;

        case SERVO_WRIST:                           // 腕转走 TIM4 通道 3
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, us);
            break;

        case SERVO_GRIP:                            // 夹爪走 TIM4 通道 1
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, us);
            break;

        default:
            break;                                  // 编号非法，什么都不做
    }

    // ★★ 11-20 修：同步标定斜坡用的"当前位置" s_cal_cur
    //   原来只有 servo_cal_set_us() 和 servo_update() 的斜坡分支维护它，
    //   M / W / C / P 这些走角度逻辑的命令完全不更新。
    //   后果：先 W 30 再发 HM，斜坡起点还停在旧值（比如 1500），
    //   第一拍就把 CCR 写成旧值附近 —— 舵机从真实位置猛窜过去，之后才慢慢爬完剩下的。
    //   现象就是"HM 明显比设定的 3 秒快，而且一上来抽一下"。
    s_cal_cur[id] = us;
}

// ★ 标定专用：不夹紧，直接把脉宽写进 CCR
// 只有 K 命令会调它，用来试探舵机的真实行程边界
// 用完记得把实测值填进 g_servo_cal，之后就回到 servo_set_us() 的夹紧保护
void servo_set_us_raw(ServoId_t id, uint16_t us)
{
    const ServoCal_t *c;                            // 标定数据指针

    if (id >= SERVO_NUM) return;                    // ★ 编号越界直接退

    c = &g_servo_cal[id];                           // 取标定数据

    switch (id)                                     // 按编号写对应的 CCR
    {
        case SERVO_ARM1:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, us);
            break;

        case SERVO_ARM2:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, us);
            break;

        case SERVO_WRIST:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, us);
            break;

        case SERVO_GRIP:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, us);
            break;

        default:
            break;
    }

    s_cal_cur[id] = us;                             // ★ 11-20：这条路径也要同步斜坡起点

    // ★ 同步软件记录的角度
    // 不同步的话，标定完停在某个角度，再发 M/P 命令时软件还以为舵机在旧位置，
    // 会一次跳出很大角度，看着像"舵机抽风"
    if (c->us_max > c->us_min)
    {
        float deg = (float)(us - c->us_min) * 180.0f
                  / (float)(c->us_max - c->us_min);   // 脉宽反算角度
        s_st[id].cur_deg  = deg;                      // 当前位置
        s_st[id].tgt_deg  = deg;                      // 目标也是它，不会自己动
        s_st[id].step_now = 0.0f;                     // 速度归零
    }
}

// ★ 标定慢速移动：在 ms 毫秒内把脉宽从当前位置平滑推到 target_us
// 只有 K 命令会调它。ms = 0 表示直跳（老行为），ms > 0 走 20ms 节拍斜坡
// 和 servo_move_deg 的区别：这条走的是"脉宽"，绕过 us_min/us_max 夹紧，
// 因为标定的目的就是试探边界，夹住了就探不出来了
void servo_cal_set_us(ServoId_t id, uint16_t target_us, uint16_t ms)
{
    int32_t  d;                                 // 还差多少微秒（带符号）
    uint16_t ticks;                             // 要走多少拍

    if (id >= SERVO_NUM) return;                // 编号越界，直接不理

    d = (int32_t)target_us - (int32_t)s_cal_cur[id];   // 算差值

    // 1. 不需要移动 或 要求直跳：走原来的立刻生效逻辑
    if (d == 0 || ms == 0)
    {
        s_cal_cur[id] = target_us;              // 跟踪值直接对齐
        s_cal_run[id] = 0;                      // 退出斜坡
        servo_set_us_raw(id, target_us);        // 立刻写 CCR
        return;
    }

    // 2. 算每拍步长：总距离 ÷ 总拍数（20ms 一拍）
    ticks = ms / 20;
    if (ticks < 1) ticks = 1;                   // 至少一拍，防除零

    s_cal_tgt[id] = target_us;                  // 记下目标
    s_cal_inc[id] = (int16_t)(d / (int32_t)ticks);   // 每拍走多少微秒

    // 3. 距离比拍数还小的时候整除会算出 0，兜底成至少 1μs/拍
    if (s_cal_inc[id] == 0)
        s_cal_inc[id] = (d > 0) ? 1 : -1;

    s_cal_run[id] = 1;                          // 启动斜坡
}


void servo_set_deg(ServoId_t id, float deg)
{
    deg = clampf_local(deg, 0.0f, 180.0f);          // 夹住角度
    servo_set_us(id, servo_deg_to_us(id, deg));     // 换算成脉宽写下去
    s_st[id].cur_deg = deg;                         // 记下当前位置
    s_st[id].tgt_deg = deg;                         // 目标也是它，等于已到位
}

// 设一个目标角度 + 最大速度，之后由 servo_update() 一点一点转过去
// 为什么要慢慢转：
//   1. 舵机从 30° 直接跳到 150°，惯性会让整个臂甩一下，小臂末端能甩出十几毫米
//   2. 四个舵机同时猛启动，浪涌电流会叠起来把电源拉垮
void servo_move_deg(ServoId_t id, float deg, float deg_per_tick)
{
    deg = clampf_local(deg, 0.0f, 180.0f);          // 夹住角度
    s_st[id].tgt_deg  = deg;                        // 记目标
    s_st[id].step_max = (deg_per_tick > SERVO_STEP_MIN)
                        ? deg_per_tick : SERVO_STEP_MIN;   // 最大速度，给个下限防卡死
}

// ★ 按"总耗时"设定运动：想在 ms 毫秒内转过去，自己算速度
// 返回 0 = 成功，-1 = 参数不对
// 注意：加了梯形加减速，实际耗时比设定值长约 10~20%
int servo_move_time(ServoId_t id, float deg, uint16_t ms)
{
    float dist, ticks, v;

    deg  = clampf_local(deg, 0.0f, 180.0f);         // 夹住角度
    dist = fabsf(deg - s_st[id].cur_deg);           // 要走多远
    if (dist < 0.05f) return 0;                     // 已经很近了，不用动
    if (ms < 40)      ms = 40;                      // 至少两拍，再快就成瞬间跳了

    ticks = (float)ms / 20.0f;                      // 20ms 一拍，多少拍
    v     = dist / ticks;                           // 需要的平均速度

    if (v > 7.0f) v = 7.0f;                         // ★ 别超过舵机真实能力（MG996R 353°/s = 7°/拍）

    servo_move_deg(id, deg, v);                     // 交给普通移动
    return 0;
}

// 主循环里定时调用（建议 20ms 一次），每次让每路往目标挪一小步
// ★ 带梯形加减速：速度不是恒定的，远离目标时快、接近时慢，起停都不顿
void servo_update(void)
{
    uint8_t i;

    for (i = 0; i < SERVO_NUM; i++)                 // 四路都过一遍
    {
        // ★ 声明必须先于语句（Keil AC5 是 C90，声明放在语句后编译不过）
        float d    = s_st[i].tgt_deg - s_st[i].cur_deg;    // 还差多少度（带符号）
        float dist = fabsf(d);                             // 还差多少（绝对值）
        float dir  = (d > 0.0f) ? 1.0f : -1.0f;            // 往哪个方向
        float v_allow, step;

        // ★ 标定斜坡优先：这路在慢速走脉宽，就只推斜坡，本拍不碰角度逻辑
        if (s_cal_run[i])
        {
            int32_t n   = (int32_t)s_cal_cur[i] + (int32_t)s_cal_inc[i];   // 下一拍位置
            int16_t inc = s_cal_inc[i];                                     // 本拍增量

            // 1. 到了或越过目标，就钉在目标上，斜坡结束
            if ((inc > 0 && n >= (int32_t)s_cal_tgt[i]) ||
                (inc < 0 && n <= (int32_t)s_cal_tgt[i]))
            {
                n = (int32_t)s_cal_tgt[i];      // 钉准，不留零点几 μs 的残差
                s_cal_run[i] = 0;               // 这路斜坡走完
            }

            // 2. 夹在硬件允许的脉宽范围里，再野也不能超出 K 命令的界限
            if (n < 400)  n = 400;
            if (n > 2600) n = 2600;

            s_cal_cur[i] = (uint16_t)n;                             // 更新跟踪值
            servo_set_us_raw((ServoId_t)i, (uint16_t)n);            // 写 CCR + 同步角度
            continue;                                               // 本拍到此为止
        }


        if (dist < 0.05f)                           // 差得比 0.05° 还小就算到位
        {
            s_st[i].cur_deg  = s_st[i].tgt_deg;     // 直接对齐，避免永远差一点点
            s_st[i].step_now = 0.0f;                // 速度归零
            continue;                               // 这路不用动
        }

        // 用 v² = 2·a·s 反算：剩这么点距离，最多允许跑多快
        v_allow = sqrtf(2.0f * SERVO_ACCEL * dist);
        if (v_allow > s_st[i].step_max) v_allow = s_st[i].step_max;   // 别超过设定上限

        // 让速度的变化受加速度限制 —— 这就是加速段和减速段的来源
        step = s_st[i].step_now;
        if (v_allow > step + SERVO_ACCEL)          // 允许速度比现在高不少 → 加速
            step += SERVO_ACCEL;
        else if (v_allow < step - SERVO_ACCEL)     // 允许速度比现在低不少 → 减速
            step -= SERVO_ACCEL;
        else                                        // 差距不大 → 直接跟上
            step = v_allow;

        if (step < SERVO_STEP_MIN) step = SERVO_STEP_MIN;   // 最低速，别慢到停住
        if (step > dist)           step = dist;             // 最后一步别过冲

        s_st[i].step_now = step;                    // 记下这一拍的速度
        s_st[i].cur_deg += dir * step;              // 走一小步

        servo_set_us((ServoId_t)i, servo_deg_to_us((ServoId_t)i, s_st[i].cur_deg));   // 写进 CCR
    }
}

// 三路是否都到位了
uint8_t servo_is_busy(void)
{
    uint8_t i;
    for (i = 0; i < SERVO_NUM; i++)
    {
        float d = s_st[i].tgt_deg - s_st[i].cur_deg;    // 差值
        if (d > 0.05f || d < -0.05f) return 1;          // 有一路还在动
    }
    return 0;                                           // 全到位
}

// 读当前角度
float servo_get_deg(ServoId_t id)
{
    return s_st[id].cur_deg;
}

// 上电初始化：先按 1500μs 启动 PWM，再用 3 秒斜坡缓走到各自回正点
//
// ★★ 为什么分两段：上电时四颗舵机的物理位置都是未知的，
//   第一个 PWM 脉冲发出去，舵机会以自身最大速度冲向那个脉宽——
//   这个"第一下"指令端控制不了（舵机内部闭环决定速度），绕不开。
//   能做的是：第一下统一冲到 1500（标称中位），之后用斜坡慢慢走，
//   只要断电前舵机停在 1500 附近，全程就是平滑的。
//
// ★★ 启动顺序不能反：先预置 CCR 再 Start，否则舵机先收到低电平脉冲会猛抽
// ★★ 四路必须错开 100ms 启动：同时回正的浪涌会叠成 4~6A，把降压模块拉垮
void servo_init(void)
{
    uint8_t  i;

    for (i = 0; i < SERVO_NUM; i++)
    {
        s_st[i].cur_deg  = 90.0f;                   // 角度状态先按中位占位
        s_st[i].tgt_deg  = 90.0f;                   // （斜坡推进时 raw 会同步真实角度）
        s_st[i].step_now = 0.0f;
        s_st[i].step_max = SERVO_STEP_MAX;

        // 斜坡起点统一 1500：第一拍脉冲发 1500，之后缓走
        s_cal_cur[i] = 1500;
        s_cal_tgt[i] = 1500;
        s_cal_inc[i] = 0;
        s_cal_run[i] = 0;

        servo_set_us_raw((ServoId_t)i, 1500);       // 预置 CCR = 1500（PWM 还没启动，只写寄存器）
    }

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);       // 大臂  PA0
    HAL_Delay(100);                                 // 等它转稳，别和下一个抢电流
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);       // 小臂  PA1
    HAL_Delay(100);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);       // 腕转  PB8
    HAL_Delay(100);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);       // 夹爪  PB6
    HAL_Delay(100);                                 // 最后一个也等稳，再进主循环

    // 全部启动后，统一 3 秒斜坡缓走到各自回正点
    // servo_update() 在主循环 20ms 节拍里自动推斜坡，这里只管"下达命令"
    for (i = 0; i < SERVO_NUM; i++)
    {
        servo_cal_set_us((ServoId_t)i, g_servo_home_us[i], SERVO_HOME_MS);
    }
}
