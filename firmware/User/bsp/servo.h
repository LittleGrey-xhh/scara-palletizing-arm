// servo.h —— 四路舵机驱动（大臂 / 小臂 / 腕转 / 夹爪）
// 依赖 CubeMX 生成的 tim.h（htim2 / htim4）
#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

#define SERVO_NUM  4        // 一共四路

// 舵机编号，顺序和标定表、代码里的数组下标一一对应
// ★ 和旧版比：编号 2 从"夹爪"改成了"腕转"，夹爪挪到 3，标定命令的 id 要跟着改
typedef enum
{
    SERVO_ARM1  = 0,        // 大臂   PA0  TIM2_CH1
    SERVO_ARM2  = 1,        // 小臂   PA1  TIM2_CH2
    SERVO_WRIST = 2,        // 腕转   PB8  TIM4_CH3   ← 绕竖直轴转夹爪朝向
    SERVO_GRIP  = 3,        // 夹爪   PB6  TIM4_CH1   ← 开合
} ServoId_t;

// 单个舵机的标定数据，P0-1 实测后填
typedef struct
{
    uint16_t us_min;        // 指令角 0°   对应的脉宽（实测安全下限）
    uint16_t us_max;        // 指令角 180° 对应的脉宽（实测安全上限）
    float    dir;           // 机械转向：装反了填 -1.0f
} ServoCal_t;

// 单路运行状态（非阻塞慢慢转用）
typedef struct
{
    float cur_deg;          // 当前角度
    float tgt_deg;          // 目标角度
    float step_now;         // 当前速度（度/拍），会变
    float step_max;         // 最大速度（度/拍）
} ServoState_t;

extern const ServoCal_t g_servo_cal[SERVO_NUM];

// ★ 上电回正位（脉宽 μs），顺序同 ServoId_t：大臂 / 小臂 / 腕转 / 夹爪
// 提到头文件里是为了让 HM 命令和 servo_init() 用同一份数据，避免两处写死不同步
extern const uint16_t g_servo_home_us[SERVO_NUM];

void     servo_init(void);                              // 上电初始化，四路先回中位
void     servo_set_us(ServoId_t id, uint16_t us);       // 给脉宽，带安全夹紧
void     servo_set_us_raw(ServoId_t id, uint16_t us);   // ★ 不夹紧，标定专用，慎用
void     servo_cal_set_us(ServoId_t id, uint16_t target_us, uint16_t ms);  // ★ 标定慢速移动：ms 毫秒内平滑推到目标脉宽，ms=0 直跳
void     servo_set_deg(ServoId_t id, float deg);        // 直接给角度，立刻生效（会甩）
void     servo_move_deg(ServoId_t id, float deg, float deg_per_tick);  // 设最大速度
int      servo_move_time(ServoId_t id, float deg, uint16_t ms);        // ★ 设总耗时（毫秒）
void     servo_update(void);                            // 主循环每 20ms 调一次
uint8_t  servo_is_busy(void);                           // 还有舵机在路上没到位
float    servo_get_deg(ServoId_t id);                   // 读当前角度
uint16_t servo_deg_to_us(ServoId_t id, float deg);      // 角度换算脉宽（调试用）

#endif
