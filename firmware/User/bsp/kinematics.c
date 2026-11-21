// kinematics.c —— SCARA 平面二连杆正逆解
// 纯数学，不引用任何 HAL 头文件，PC 上也能直接编译测试
#include "kinematics.h"
#include <math.h>

#define DEG2RAD  0.017453293f    // π/180，角度转弧度
#define RAD2DEG  57.295779513f   // 180/π，弧度转角度

// 把浮点数限制在 [lo, hi] 区间
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;                          // 低于下界就取下界
    if (v > hi) return hi;                          // 高于上界就取上界
    return v;                                       // 区间内原样返回
}

// 最大臂展：两臂伸直时末端到回转中心的距离
float arm_reach_max(void)
{
    return ARM_L1_MM + ARM_L2_MM;                   // 137 + 137 = 274mm
}

// 最小臂展：两臂完全重合时的距离
// 两臂等长时为 0，但那只是数学值，机械上小臂会撞大臂
float arm_reach_min(void)
{
    return fabsf(ARM_L1_MM - ARM_L2_MM);            // 137 - 137 = 0
}

// 纯数学逆解：只算角度，不判断舵机行程
// PC 端回验数学正确性用这个，真机下发指令请用 ik_solve
int ik_solve_raw(float x_mm, float y_mm, int elbow, ArmJoint_t *out)
{
    float r2, r, c2, s2, t2, t1;                    // r2=r²，c2=cosθ2，s2=sinθ2，t2=θ2，t1=θ1

    if (out == 0) return KIN_ERR_OUTER;             // 空指针直接拒绝，别让后面崩

    // 1. 先算末端到回转中心的距离
    r2 = x_mm * x_mm + y_mm * y_mm;                 // 勾股定理 r² = x² + y²
    r  = sqrtf(r2);                                 // 开根号得到实际半径

    // 2. 工作空间粗判，越界直接返回，不进 acos 免得算出 NaN
    if (r > arm_reach_max()) return KIN_ERR_OUTER;  // 比 L1+L2 还远，物理上够不到
    if (r < arm_reach_min()) return KIN_ERR_INNER;  // 比 |L1-L2| 还近，落在内死区

    // 3. 余弦定理求 θ2
    c2 = (r2 - ARM_L1_MM * ARM_L1_MM - ARM_L2_MM * ARM_L2_MM)
         / (2.0f * ARM_L1_MM * ARM_L2_MM);          // 这就是那个公式，一行搞定
    c2 = clampf(c2, -1.0f, 1.0f);                   // 边界上浮点误差会让 cos 略微超 1，必须夹住

    t2 = acosf(c2);                                 // 得到 0~π 的角度
    if (elbow < 0) t2 = -t2;                        // 取负号就是另一组镜像解（肘上/肘下）

    // 4. 由 θ2 反推 θ1
    s2 = sinf(t2);                                  // θ2 的正弦
    t1 = atan2f(y_mm, x_mm)                       // 末端方向角
       - atan2f(ARM_L2_MM * s2,                     // 减去小臂对方向的修正量
                ARM_L1_MM + ARM_L2_MM * cosf(t2));  // 分母是大臂加小臂在径向的投影

    // 5. 存进结构体
    out->theta1_deg = t1 * RAD2DEG;                 // 弧度转角度
    out->theta2_deg = t2 * RAD2DEG;                 // 弧度转角度

    // 6. 关节角换算成舵机能听懂的 0~180 指令角
    arm_to_servo(out);

    return KIN_OK;                                  // 数学部分通过
}

// 逆解（真机用）：纯数学解 + 舵机行程检查
// 核心只有余弦定理：cos(θ2) = (r² - L1² - L2²) / (2·L1·L2)
int ik_solve(float x_mm, float y_mm, int elbow, ArmJoint_t *out)
{
    int rc = ik_solve_raw(x_mm, y_mm, elbow, out);         // 先拿数学解
    if (rc != KIN_OK) return rc;                           // 工作空间就不通，直接返回

    if (out->servo1_deg < SERVO1_MIN_DEG || out->servo1_deg > SERVO1_MAX_DEG) return KIN_ERR_JOINT;
    if (out->servo2_deg < SERVO2_MIN_DEG || out->servo2_deg > SERVO2_MAX_DEG) return KIN_ERR_JOINT;

    return KIN_OK;                                  // 解算全通过
}

// 正解：关节角 → 桌面坐标，逆解写完必须用它回验
void fk_solve(float theta1_deg, float theta2_deg, float *x_mm, float *y_mm)
{
    float t1 = theta1_deg * DEG2RAD;                // 大臂角转弧度
    float t2 = theta2_deg * DEG2RAD;                // 小臂夹角转弧度

    if (x_mm) *x_mm = ARM_L1_MM * cosf(t1)                     // 大臂在 X 上的投影
                    + ARM_L2_MM * cosf(t1 + t2);               // 加小臂在 X 上的投影
    if (y_mm) *y_mm = ARM_L1_MM * sinf(t1)                     // 大臂在 Y 上的投影
                    + ARM_L2_MM * sinf(t1 + t2);               // 加小臂在 Y 上的投影
}

// 关节角 → 舵机指令角
// 舵机指令角 90 对应 1500us，±1 度对应 ±1 度机械转角（MG996R 是 1 度/us 的线性映射）
void arm_to_servo(ArmJoint_t *j)
{
    if (j == 0) return;                                        // 空指针保护

    j->servo1_deg = 90.0f                                      // 舵机自身的中位指令角
                  + (j->theta1_deg - THETA1_HOME_DEG)          // 与标定中位的角度差
                  * SERVO1_DIR;                                // 装上反了就取负
    j->servo2_deg = 90.0f                                      // 舵机自身的中位指令角
                  + (j->theta2_deg - THETA2_HOME_DEG)          // 与标定中位的角度差
                  * SERVO2_DIR;                                // 装上反了就取负
}

// 舵机指令角 → 关节角，逆运算，用来算"现在臂在哪"
void servo_to_arm(ArmJoint_t *j)
{
    if (j == 0) return;                                        // 空指针保护

    j->theta1_deg = THETA1_HOME_DEG                            // 从标定中位出发
                  + (j->servo1_deg - 90.0f) * SERVO1_DIR;      // 加上实际转角，符号同向
    j->theta2_deg = THETA2_HOME_DEG                            // 小臂同理
                  + (j->servo2_deg - 90.0f) * SERVO2_DIR;
}

// 给定关节角 + 想要的夹爪绝对朝向，反解出腕转指令角
// 推导：夹爪绝对角 A = (θ1+θ2) − 腕转角 + WRIST_ABS_OFFSET_DEG
//       → 腕转角 = (θ1+θ2) + 偏置 − A
// ★★ 是【减】腕转角：11-20 四个实测点拟合出来的，腕转脉宽越大夹爪绝对角越小。
//   上一版写成加号，是因为只用 HM 一个点反推 —— θ1+θ2=−180 时正负两种符号
//   都能凑出 175°，单点定不了方向。至少两个不同位置才能把符号钉死。
// ★ 平行爪有 180° 对称：算出来是负数或超 180 都不要紧，折回 [0,180) 仍是同一个朝向
float wrist_deg_for_abs(float theta1_deg, float theta2_deg, float target_abs_deg)
{
    float w = (theta1_deg + theta2_deg) + WRIST_ABS_OFFSET_DEG - target_abs_deg;

    while (w <    0.0f) w += 180.0f;                           // 折到正区间
    while (w >= 180.0f) w -= 180.0f;                           // 折回行程内

    return w;
}
