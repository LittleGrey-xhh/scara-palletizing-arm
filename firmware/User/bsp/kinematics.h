// kinematics.h —— SCARA 平面二连杆正逆解
// 只做数学，不碰任何 HAL，PC 和 STM32 都能编译
#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>

// ---------- 返回码 ----------
#define KIN_OK              0     // 解算成功
#define KIN_ERR_OUTER       1     // 超出最大臂展（太远）
#define KIN_ERR_INNER       2     // 落入内死区（太近）
#define KIN_ERR_JOINT       3     // 解出的角度超出舵机行程

// ---------- 机械参数：装配后必须实测修正 ----------
// ★ 2026-11-14 实测（大臂减到 18cm 后）：L1=17cm，L2=13.5cm
#define ARM_L1_MM          170.0f   // 大臂：底座舵机轴心 → 小臂舵机轴心
#define ARM_L2_MM          135.0f   // 小臂：小臂舵机轴心 → 腕转/夹爪中心

// ---------- 舵机行程窗（单位：度，对应舵机 0~180 的指令角） ----------
// 留出机械安全余量，别让舵机顶到 0 或 180 死点，顶死会烧
// ★ servo2 越小 = 小臂越伸直 = 臂展越长 = 大臂舵机越吃力，所以下限留大一点
#define SERVO1_MIN_DEG      8.0f    // 大臂舵机最小允许角
#define SERVO1_MAX_DEG    172.0f    // 大臂舵机最大允许角
#define SERVO2_MIN_DEG     20.0f    // 小臂舵机最小允许角（别伸到最长，大臂扛不住）
// ★★ 11-20 放开 172 → 180.5：回正位（小臂 2460μs = 舵机角 180°）实测不顶夹板，
//   175 这个上限却把 M 35 0（原点收拢位）挡在门外、回 ERR 3。
//   多给 0.5° 是因为 r=35 恰好落在工作区内边界上，acosf(-1) 出来的浮点可能是
//   180.00002，写死 180 会偶发判定越界；servo_deg_to_us() 内部本来就会夹到 0~180
#define SERVO2_MAX_DEG    180.5f    // 小臂舵机最大允许角

// ---------- 标定常量：2026-11-14 装配后实测 ----------
// 含义：舵机指令角 90°（即 1500us）时，机械臂的关节角是多少
#define THETA1_HOME_DEG     0.0f    // 1500us 时大臂指向正前方（实测≈0°）
#define THETA2_HOME_DEG   -90.0f    // ★ 1500us 时小臂相对大臂折 −90°（垂直，指向右侧）
#define SERVO1_DIR          (1.0f)  // 大臂转向符号：发 M 验证，反了改 −1.0f
#define SERVO2_DIR         (-1.0f)  // ★ 实测：脉宽增大 → 小臂继续顺时针折，故取反

// ---------- 腕转姿态解耦（★ 11-20 新增） ----------
// ★★ 为什么必须有：腕转舵机装在**小臂末端**，夹爪的绝对朝向是
//       夹爪绝对角 = (θ1 + θ2) + 腕转指令角 + 偏置
//   而 M / P 只动大臂小臂，腕转指令角不变 → 小臂绝对角一变，夹爪朝向跟着变。
//   实测：HM 收拢位（θ1+θ2 = −180°）腕转 85° 时夹爪平行 x 轴；
//         走到 M 100 0（−91.6°）同样 85°，夹爪就变成垂直 x 轴了；
//         再走到 M 200 0（−57.1°）就歪成 123°，抓方块必夹到角。
//   所以要把"夹爪朝向"当目标值，反解腕转角——这样同一个朝向在任何位置都成立。
#define WRIST_ABS_OFFSET_DEG      265.0f    // 偏置：夹爪绝对角 A = (θ1+θ2) − 腕转角 + 265
                                            // ★ 注意是【减】腕转角：腕转脉宽变大，夹爪绝对角变小
                                            // ★ 11-20 用四个实测点拟合（M 100/150/200/250 各自调到垂直）：
                                            //     r=100 → 1472μs、150 → 1650、200 → 1800、250 → 2000
                                            //   反推 C 分别为 268.1 / 265.5 / 263.0 / 263.5，取 265
                                            //   ★ 只用 HM 一个点定不出方向（θ1+θ2=−180 时两种符号都能凑 175°）
#define WRIST_ABS_PARALLEL_X        0.0f    // 夹爪平行 x 轴（收拢待机用）
#define WRIST_ABS_PERPENDICULAR_X  90.0f    // 夹爪垂直 x 轴（★ 作业默认，正对抓取方向）

// 给定关节角 + 想要的夹爪绝对朝向，反解出腕转指令角（0~180）
float wrist_deg_for_abs(float theta1_deg, float theta2_deg, float target_abs_deg);

// ---------- 一组关节解 ----------
typedef struct
{
    float theta1_deg;    // 大臂绝对角（相对 +X 轴，逆时针为正）
    float theta2_deg;    // 小臂相对大臂的夹角（0=伸直，正=肘下一侧）
    float servo1_deg;    // 换算后的大臂舵机指令角 0~180
    float servo2_deg;    // 换算后的小臂舵机指令角 0~180
} ArmJoint_t;

// 逆解：桌面坐标 (x,y) → 关节角
// elbow = +1 取一组解，-1 取另一组（肘上/肘下），任选其一都合法
// 返回 KIN_OK 表示 out 有效
int   ik_solve(float x_mm, float y_mm, int elbow, ArmJoint_t *out);

// 纯数学逆解：只判断工作空间，不判断舵机行程
// PC 端验证数学正确性用这个；真机下发指令请用上面的 ik_solve
int   ik_solve_raw(float x_mm, float y_mm, int elbow, ArmJoint_t *out);

// 正解：关节角 → 桌面坐标（用来验证逆解，也用来算当前末端在哪）
void  fk_solve(float theta1_deg, float theta2_deg, float *x_mm, float *y_mm);

// 工作空间边界（毫米）
float arm_reach_max(void);   // L1 + L2，最远能到
float arm_reach_min(void);   // |L1 - L2|，最近能到（本例两臂等长，数学上是 0）

// 关节角 → 舵机指令角（含中位与方向换算）
void  arm_to_servo(ArmJoint_t *j);
// 舵机指令角 → 关节角（回读当前姿态用）
void  servo_to_arm(ArmJoint_t *j);

#endif
