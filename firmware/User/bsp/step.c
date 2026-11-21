// step.c —— 滑台步进驱动 + 回零
// 脉冲方案：TIM1 硬件 PWM（1MHz 计数，ARR 决定频率，CCR 固定占空比 50%）
//           每次 TIM1 更新中断 = 一个脉冲 = 0.0025mm
#include "step.h"
#include "tim.h"        // htim1
#include "main.h"       // STEP_DIR_Pin / STEP_ENA_Pin / LIMIT_Z_Pin

#define STEP_INF    (0x7FFFFFFFL)   // 剩余脉冲数等于这个值表示"无限走"

// ★★★ 11-20 实测纠正：DIR 电平与实际转向相反
//   原代码假设 "s_dir > 0 → DIR 输出高 → 向上"，实测 T 发正数滑块却往下走。
//   结果回零整套反了：BACKOFF 往下压、SEEK 往上无限走，一路顶到顶（11-20 险出事故）。
//   修法：只翻转写进 DIR 引脚的电平，不动 s_dir 的语义——
//   s_dir(+1) 仍然代表"向上"，s_pos 的符号也仍然是"向上为正"，坐标语义不变。
#define STEP_DIR_STATE(dir)   (((dir) > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET)

// ---------- 运行时状态（中断和主循环都会碰，必须 volatile） ----------
static volatile int32_t  s_pos      = 0;            // 绝对位置，单位脉冲，0 = 零点
static volatile int32_t  s_left     = 0;            // 本次还剩多少脉冲
static volatile uint32_t s_done     = 0;            // 本次已经走了多少脉冲
static volatile uint32_t s_total    = 0;            // 本次总共要多少脉冲
static volatile uint32_t s_ramp     = 0;            // 加减速段长度（脉冲）
static volatile uint32_t s_f_lo     = STEP_F_MIN;   // 起停频率
static volatile uint32_t s_f_hi     = 4000;         // 巡航频率（4000 = 10mm/s）
static volatile int8_t   s_dir      = 1;            // +1 向上，-1 向下
static volatile uint8_t  s_busy     = 0;            // 电机在动

// 回零用
static volatile HomeState_t s_home  = HOME_IDLE;    // 回零状态机
static volatile uint32_t    s_home_t0 = 0;          // 回零阶段起始时间戳
static volatile uint8_t     s_home_step_done = 0;   // 本阶段的动作是否已经发起
static volatile uint8_t     s_limit_fault = 0;      // 非回零状态下撞限位的标志
static volatile int32_t     s_home_base  = 0;       // 本次回零开始时的坐标
static volatile int32_t     s_home_offset = 0;      // 触发点相对回零起点的脉冲偏移

// 软上限：回零之后向上最多能走多少（物理上限位不做开关，靠这个守）
static volatile float       s_soft_max = STEP_SOFT_MAX_MM;

// ---------- 内部：改频率（同时改 ARR 和 CCR，必须一起改） ----------
static void step_set_freq(uint32_t f)
{
    uint32_t arr;

    if (f < STEP_F_MIN)       f = STEP_F_MIN;       // 夹住下限
    if (f > STEP_F_MAX_LIMIT) f = STEP_F_MAX_LIMIT; // 夹住上限，不许超 30mm/s

    arr = 1000000UL / f;                            // 1MHz 计数，一个周期 arr 个计数
    if (arr < 2) arr = 2;                           // 防 0，ARR 至少 2
    arr -= 1;                                       // ARR 是"从 0 数到 arr"，所以减 1

    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);          // 改周期，下个 update 生效
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr / 2);   // 占空比固定 50%
}

// ---------- 内部：按当前进度算下一个频率，写进去 ----------
static void step_advance_speed(void)
{
    uint32_t f;

    if (s_total == STEP_INF)                        // 连续走，不加不减速，恒速
    {
        f = s_f_hi;
    }
    else if (s_ramp == 0)                           // 没设加减速段
    {
        f = s_f_hi;
    }
    else if (s_done < s_ramp)                       // 加速段：线性升频
    {
        f = s_f_lo + (s_f_hi - s_f_lo) * s_done / s_ramp;
    }
    else if (s_done + s_ramp > s_total)             // 减速段：线性降频
    {
        f = s_f_lo + (s_f_hi - s_f_lo) * (s_total - s_done) / s_ramp;
    }
    else                                            // 匀速段
    {
        f = s_f_hi;
    }

    if (f > s_f_hi) f = s_f_hi;                     // ★ 别让加速段算过头，超过巡航速度

    step_set_freq(f);                               // 写进定时器
}

// ---------- 初始化 ----------
void step_init(void)
{
    // ★ 顺序不能反：step_set_freq() 内部会把 CCR 设成 arr/2（占空比 50%），
    //   必须在它之后再清 CCR，否则 PWM 一启动就是连续脉冲，电机上电就跑
    step_set_freq(STEP_F_MIN);                              // 1. 先设周期
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);        // 2. 再清 CCR → 占空比 0%，光耦不导通
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);               // 3. 启动 PWM，此时不发脉冲
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);            // 4. 更新中断先关着，走的时候再开

    step_enable(0);                                         // 5. 上电先松轴，方便手动摆臂
}

// ---------- 使能 / 松轴 ----------
// ★★ 极性以实测为准（2026-11-16 实测 + TB6600 规格书确认）：
//   共阳接法下 GPIO 输出【低】→ 光耦导通 → ENA 信号【有效】
//   而 TB6600 规格书原文："ENA 有效时电机转子处于自由状态（脱机状态）"
//   → GPIO 低 = 脱机松轴，GPIO 高 = 锁轴
//   所以 on=1（锁轴）要输出高，on=0（松轴）要输出低。
//   ★ 之前写成"低=锁轴"是反的，实测 E1 时轴能自由转动才发现。
void step_enable(uint8_t on)
{
    HAL_GPIO_WritePin(STEP_ENA_GPIO_Port, STEP_ENA_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ---------- 停 ----------
// ★ 用 CCR=0 停，不用 HAL_TIM_PWM_Stop()
//    Stop 会把 PA8 切回高阻，电平不确定，可能残留高电平让光耦一直通
void step_stop(void)
{
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);   // 先关中断，止住计数
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);   // 占空比 0，输出恒低
    s_busy = 0;                                     // 标记空闲
    s_left = 0;
}

// ---------- 相对移动 ----------
// mm：带符号，正 = 向上，负 = 向下
// f_max：巡航脉冲频率（8000 = 20mm/s）
int step_move_mm(float mm, uint32_t f_max)
{
    int32_t steps;
    int32_t abs_steps;

    if (s_busy) return STEP_BUSY;                   // 上一次还没走完，拒绝

    steps = (int32_t)(mm / STEP_MM_PER_PULSE);      // 毫米换算成脉冲
    if (steps == 0) return STEP_OK;                 // 不足一个脉冲，当没动

    s_dir = (steps > 0) ? 1 : -1;                   // 记方向

    abs_steps = (steps > 0) ? steps : -steps;       // 取绝对值当总脉冲数

    // 1. 先设方向（DIR 到 PUL 之间要留建立时间，下面启动 PWM 到第一个脉冲至少一个周期，够了）
    HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, STEP_DIR_STATE(s_dir));

    // 2. 填运动参数
    s_total = (uint32_t)abs_steps;
    s_left  = abs_steps;
    s_done  = 0;
    s_f_hi  = f_max;

    // 3. 加减速段：短行程就少加几步，长行程给 200 步（0.5mm）
    s_ramp = (s_total / 4 < 200) ? (s_total / 4) : 200;

    // 4. 起速不能高于巡航速度
    s_f_lo = (f_max / 4 < STEP_F_MIN) ? STEP_F_MIN : (f_max / 4);

    // 5. 上电锁轴、清中断标志、开中断、启动
    step_enable(1);                                 // 锁轴
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);  // 清掉可能挂起的旧标志
    s_busy = 1;                                     // 先置忙，防止中断里提前判定
    step_set_freq(s_f_lo);                          // 从低速起步
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);     // 开更新中断

    return STEP_OK;
}

// ---------- 走到绝对位置 ----------
// ★ 这里守软上限：Z / P 命令都走这条路，越界直接拒绝，不发脉冲
int step_move_to_mm(float mm_abs, uint32_t f_max)
{
    float delta;

    if (mm_abs < 0.0f)            return STEP_RANGE;   // 下面就是限位，别钻
    if (mm_abs > s_soft_max)      return STEP_RANGE;   // 上面超过软上限，撞死挡会丢步

    delta = mm_abs - step_pos_mm();                    // 目标减当前 = 要走多远
    return step_move_mm(delta, f_max);                 // 转成相对移动
}

// ---------- 连续走（回零用） ----------
int step_jog(int8_t dir, uint32_t f)
{
    if (s_busy) return STEP_BUSY;                   // 忙着就别插队

    s_dir = (dir > 0) ? 1 : -1;                     // 记方向

    HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, STEP_DIR_STATE(s_dir));

    s_total = (uint32_t)STEP_INF;                   // ★ 特殊值：不倒数，一直走
    s_left  = STEP_INF;
    s_done  = 0;
    s_ramp  = 0;                                    // 连续走不加减速
    s_f_hi  = f;
    s_f_lo  = f;

    step_enable(1);                                 // 锁轴
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    s_busy = 1;
    step_set_freq(f);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

    return STEP_OK;
}

// ---------- 位置查询 ----------
int32_t step_pos_pulse(void)
{
    return s_pos;                                   // 32 位读是原子的，不用关中断
}

float step_pos_mm(void)
{
    return (float)s_pos * STEP_MM_PER_PULSE;        // 脉冲换算成毫米
}

void step_set_zero(void)
{
    s_pos = 0;                                      // 当前位置就是零点
}

// ---------- 软上限 ----------
void step_set_soft_max(float mm)
{
    if (mm < 5.0f)          mm = 5.0f;              // 太小没意义，至少留 5mm
    if (mm > STEP_MAX_MM)   mm = STEP_MAX_MM;       // 不许超过丝杆物理行程

    s_soft_max = mm;
}

float step_soft_max_mm(void)
{
    return s_soft_max;
}

// ---------- 限位实时状态 ----------
// NPN 常开 + 内部上拉：释放 = 悬空 = 读到高；触发 = 拉到 0V = 读到低
uint8_t step_limit_active(void)
{
    return (HAL_GPIO_ReadPin(LIMIT_Z_GPIO_Port, LIMIT_Z_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}

uint8_t step_is_busy(void)
{
    return s_busy;
}

// ---------- TIM1 更新中断：一个脉冲到了 ----------
void step_on_tim_update(void)
{
    s_pos += (s_dir > 0) ? 1 : -1;                  // 累计位置（带符号）
    s_done++;                                       // 本次进度 +1

    if (s_left != STEP_INF)                         // 定量模式才倒数
    {
        s_left--;
        if (s_left <= 0)                            // 走完了
        {
            step_stop();                            // 停
            return;                                 // 直接返回，别再算速度
        }
    }

    step_advance_speed();                           // 还没走完，算下一个频率
}

// ---------- 限位触发（EXTI 里调） ----------
// ★ 记的是"触发那一瞬间"的位置，不是"停住之后"的位置
// ★ 只有 HOME_SEEK 阶段触发才是"找到原点"；回零的其他阶段还在向上走，
//   这时再触发只能是传感器抖动或接线噪声，忽略掉，不要打断回零
void step_on_limit_triggered(void)
{
    step_stop();                                    // 立刻停，不等主循环

    if (s_home == HOME_SEEK)                        // 正在回零找原点，这才是正常触发
    {
        // ★ 先记下"触发瞬间相对回零起点"的偏移，再清位置。
        //   顺序反了就永远是 0，回零重复性根本没法测
        s_home_offset = s_pos - s_home_base;
        s_pos = 0;                                  // ★ 触发点就是零点
        s_home = HOME_RETREAT;                      // 进下一阶段：退开
        s_home_step_done = 0;                       // 重置阶段标志
        s_home_t0 = HAL_GetTick();
    }
    else if (s_home == HOME_BACKOFF || s_home == HOME_RETREAT)
    {
        // 回零中途的抖动，不判故障也不改零点
        // ★ 但电机已经被 step_stop() 停了，状态机下一拍会看到 !s_busy 而往下推进，正好
    }
    else                                            // 非回零时撞限位 = 出事了
    {
        s_limit_fault = 1;                          // 置故障标志，主循环负责报警
        // 不清零位置！否则误触发一次坐标全废
    }
}

// 查询限位故障标志（主循环用）
uint8_t step_limit_fault(void)
{
    return s_limit_fault;
}

void step_limit_fault_clear(void)
{
    s_limit_fault = 0;
}

// ★★ 回零速度：11-20 统一提到 4000Hz = 10mm/s
//   原来 2000Hz(5mm/s) 太慢，找一段 30mm 要 6 秒。实测 10mm/s 触发后能稳稳刹住
//   （中断里 step_stop() 立即执行，1ms 延迟 = 0.01mm，可忽略）
#define HOME_FREQ        4000UL
// ★★ 向下找原点的行程上限：超过这个距离还没触发，只能是传感器没通
//   原来靠 60 秒超时兜底，滑块会一路顶到底部机械硬挡丢步整整一分钟（11-20 教训）
//   实测可用行程约 130mm，留到 150mm 就够
#define HOME_SEEK_MAX_MM 150.0f

// ---------- 回零 ----------
void step_home_start(void)
{
    if (s_home == HOME_BACKOFF || s_home == HOME_SEEK || s_home == HOME_RETREAT)
        return;                                     // 已经在回零了，别重复启动

    s_home_base = s_pos;                            // ★ 记下起点，用来算触发点偏移
    s_home_offset = 0;
    s_home = HOME_BACKOFF;                          // ① 先向上离开
    s_home_step_done = 0;
    s_home_t0 = HAL_GetTick();
}

int32_t step_home_offset_pulse(void)
{
    return s_home_offset;
}

// 主循环每 10~20ms 调一次
HomeState_t step_home_poll(void)
{
    switch (s_home)
    {
        case HOME_BACKOFF:                          // ① 向上走 5mm 离开可能的触发区
            if (!s_home_step_done)                  // 动作还没发起
            {
                if (step_move_mm(5.0f, HOME_FREQ) == STEP_OK)   // 慢速向上 5mm
                    s_home_step_done = 1;           // 发起成功
            }
            else if (!s_busy)                       // 走完了
            {
                s_home_step_done = 0;               // 清标志给下一阶段用
                s_home = HOME_SEEK;                 // 进下一阶段
                s_home_t0 = HAL_GetTick();
            }
            break;

        case HOME_SEEK:                             // ② 向下慢速找原点
            if (!s_home_step_done)                  // 动作还没发起
            {
                // ★ 用返回值判断是否真的启动了。step_jog 在电机忙时静默返回 BUSY，
                //   原来不看返回值就置 done，下一拍会误判成"停了没触发"→ 直接 ERROR
                if (step_jog(-1, HOME_FREQ) == STEP_OK)  // 向下 10mm/s，一直走
                    s_home_step_done = 1;           // 发起成功
            }
            // ★★ 向下走超过 150mm 还没触发 → 传感器没通 / 线断了 / 探头装歪了
            //   不等 60 秒，立刻中止。否则滑块顶到底部硬挡，电机丢步丢一分钟
            else if ((int32_t)(s_home_base - s_pos) >
                     (int32_t)(HOME_SEEK_MAX_MM / STEP_MM_PER_PULSE))
            {
                step_stop();
                s_home_step_done = 0;
                s_home = HOME_ERROR;                // 报 ERR 9
            }
            else if (!s_busy)                       // 停了但没触发 → 异常
            {
                s_home_step_done = 0;
                s_home = HOME_ERROR;                // 超程 / 传感器坏了
            }
            else if ((HAL_GetTick() - s_home_t0) > 60000UL)   // 60s 超时保护
            {
                step_stop();
                s_home_step_done = 0;
                s_home = HOME_ERROR;
            }
            break;

        case HOME_RETREAT:                          // ④ 向上退 3mm 让传感器释放
            if (!s_home_step_done)
            {
                if (step_move_mm(3.0f, HOME_FREQ) == STEP_OK)
                    s_home_step_done = 1;
            }
            else if (!s_busy)
            {
                // ★★ 退让完成后把这里定为 Z = 0。
                //   不这么做的话，回零结束时 s_pos 还停在上方 3mm（1200 脉冲），
                //   发 Z 0 会往下走 3mm 重新压到限位上 → 立刻报 ERR 6 故障，
                //   等于回零白做。定零之后 Z 0 = 触发点上方 3mm，安全高度。
                step_set_zero();
                s_home_step_done = 0;
                s_home = HOME_DONE;                 // 回零完成
            }
            break;

        case HOME_DONE:                             // 完成，停在这里
        case HOME_ERROR:                            // 出错，等人工处理
        case HOME_IDLE:                             // 没在回零
        default:
            break;
    }

    return s_home;
}

void step_home_abort(void)
{
    step_stop();                                    // 停电机
    s_home = HOME_IDLE;                             // 状态复位
    s_home_step_done = 0;
}
