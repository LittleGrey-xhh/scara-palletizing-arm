// step.h —— 滑台步进驱动 + 回零
// 依赖 CubeMX 生成的 tim.h（htim1）和 main.h（STEP_DIR / STEP_ENA / LIMIT_Z 的宏）
#ifndef __STEP_H
#define __STEP_H

#include <stdint.h>

// ---------- 机械常量 ----------
#define STEP_MM_PER_PULSE   0.0025f     // 脉冲当量：导程 4mm ÷ 1600 脉冲/转
#define STEP_MAX_MM         200.0f      // 物理行程上限（丝杆总长）
// ★★ 11-20 实测改：这台装好夹板后 Z 实际只能上行 ~70mm（回零起点在触发点上方
//   29297 脉冲 = 73.2mm，减去 Z=0 相对触发点的 3mm）。原来默认 80 比真实上限还高，
//   Z 75 会被批准然后一头撞顶丢步。默认值收到 60，留 5mm 给 HOME_BACKOFF、5mm 机械余量。
//   演示只用 0~60mm，够。
#define STEP_SOFT_MAX_MM    60.0f       // ★ 软上限默认值：回零后向上最多能走多少
                                        //   换机械 / 重新装夹板后用 ZL 重新标
#define STEP_F_MIN          200UL       // 最低脉冲频率，再慢就没意义
#define STEP_F_MAX_LIMIT    12000UL     // ★ 硬上限 30mm/s，再快必失步

// ---------- 返回码 ----------
#define STEP_OK     0
#define STEP_BUSY   1
#define STEP_RANGE  2                   // 目标超出软上限，拒绝执行

// ---------- 回零状态 ----------
typedef enum
{
    HOME_IDLE = 0,      // 没在回零
    HOME_BACKOFF,       // ① 向上离开，确保传感器是释放状态
    HOME_SEEK,          // ② 向下慢速找原点
    HOME_RETREAT,       // ④ 向上退 3mm 让传感器释放
    HOME_DONE,          // 回零完成
    HOME_ERROR,         // 超程没找到原点
} HomeState_t;

// ---------- 运动 ----------
void    step_init(void);                                    // 上电初始化
int     step_move_mm(float mm, uint32_t f_max);             // 相对走 mm（带符号），非阻塞
int     step_move_to_mm(float mm_abs, uint32_t f_max);      // 走到绝对位置，非阻塞
int     step_jog(int8_t dir, uint32_t f);                   // 连续走，直到 step_stop()
void    step_stop(void);                                    // 立刻停（清 CCR）
uint8_t step_is_busy(void);                                 // 还在走吗

// ---------- 位置 ----------
int32_t step_pos_pulse(void);                               // 当前绝对位置（脉冲）
float   step_pos_mm(void);                                  // 当前绝对位置（毫米）
void    step_set_zero(void);                                // 把当前位置设成零点

// ---------- 软上限 ----------
// ★ 物理上限位不做开关，靠脉冲计数守。软上限 ≤ STEP_MAX_MM
void  step_set_soft_max(float mm);                          // 改软上限（运行时，掉电不存）
float step_soft_max_mm(void);                               // 查当前软上限

// ---------- 限位实时状态（装传感器 / 排查接线专用） ----------
// 1 = 触发（黑线被拉到 0V），0 = 释放。跟随传感器，不依赖回零状态
uint8_t step_limit_active(void);

// ---------- 回零 ----------
void        step_home_start(void);                          // 启动回零，非阻塞
HomeState_t step_home_poll(void);                           // 主循环里查状态
void        step_home_abort(void);                          // 中止回零

// 上一次回零：触发点相对"回零起点"走了多少脉冲（负 = 向下）
// ★ 验收重复性就看这个值：连续回零 10 次，这个数字的极差 ≤ ±5 脉冲 = 完美
//   偏差 = 这个值 − (本次回零前在起点的理论值)，同一起点下直接比大小即可
int32_t step_home_offset_pulse(void);

// ---------- 使能 ----------
void step_enable(uint8_t on);                               // 1=通电锁轴 0=松轴（手动摆臂用）

// ---------- 限位中断回调（EXTI 里调这个） ----------
void    step_on_limit_triggered(void);

// ---------- 限位故障（非回零状态下撞限位） ----------
uint8_t step_limit_fault(void);
void    step_limit_fault_clear(void);

// ---------- TIM1 更新中断回调（HAL_TIM_PeriodElapsedCallback 里调） ----------
void    step_on_tim_update(void);

#endif
