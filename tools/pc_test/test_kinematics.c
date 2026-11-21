// test_kinematics.c —— 逆运动学 PC 端验证
// 编译：gcc -I. kinematics.c test_kinematics.c -o test_kin -lm
// 干什么：把逆解算出来的角度再喂回正解，看能不能回到原来的点。
//         这是唯一能在装机械之前确认数学没错的办法。
#include <stdio.h>
#include <math.h>
#include "kinematics.h"

#define PI_F 3.14159265f

// 角度归一化到 [-180, 180)
static float norm180(float a)
{
    while (a >= 180.0f)  a -= 360.0f;               // 大了就减一圈
    while (a < -180.0f)  a += 360.0f;               // 小了就加一圈
    return a;                                       // 归一化结果
}

int main(void)
{
    float t1, t2, x, y, x2, y2, err, max_err = 0.0f;
    int   n_test = 0, n_fail = 0, n_outer = 0;

    printf("==================================================\n");
    printf("逆运动学闭环验证  L1=%.1f  L2=%.1f  Rmax=%.1f\n",
           ARM_L1_MM, ARM_L2_MM, arm_reach_max());
    printf("==================================================\n");

    // ---------- 1. 闭环验证：正解 → 逆解 → 正解 ----------
    for (t1 = -180.0f; t1 < 180.0f; t1 += 3.0f)     // 大臂扫一圈
    {
        for (t2 = -180.0f; t2 < 180.0f; t2 += 3.0f) // 小臂扫一圈
        {
            ArmJoint_t j;                           // 存逆解结果
            int elbow = (t2 >= 0.0f) ? 1 : -1;      // θ2 为正走 +1 分支，为负走 -1 分支

            fk_solve(t1, t2, &x, &y);               // 正解拿到真实末端坐标

            if (ik_solve_raw(x, y, elbow, &j) != KIN_OK) continue;  // 理论上不该失败

            n_test++;                               // 计数
            fk_solve(j.theta1_deg, j.theta2_deg, &x2, &y2);         // 逆解结果再正解回去

            err = hypotf(x - x2, y - y2);           // 往返后与原点的距离误差
            if (err > max_err) max_err = err;       // 记录最差值

            if (err > 0.01f)                        // 超过 0.01mm 就算失败
            {
                n_fail++;
                if (n_fail <= 5)                    // 只打印前 5 个，免得刷屏
                {
                    printf("  [FAIL] t1=%7.1f t2=%7.1f  err=%.4f mm\n", t1, t2, err);
                }
            }
        }
    }

    printf("\n1. 闭环验证：测了 %d 个姿态，失败 %d 个\n", n_test, n_fail);
    printf("   最大往返误差 = %.6f mm  ->  %s\n",
           max_err, (max_err < 0.01f) ? "通过" : "数学有问题");

    // ---------- 2. 工作空间边界 ----------
    printf("\n2. 工作空间（两臂等长时的可达环带）\n");
    printf("   数学最小半径 = %.1f mm（两臂完全重合）\n", arm_reach_min());
    printf("   最大半径     = %.1f mm（两臂伸直）\n", arm_reach_max());

    // ---------- 3. 每个半径下，末端方位角能覆盖多大 ----------
    // 等长臂时末端方位角 φ = θ1 + θ2/2（推导见下）
    // 所以 φ 的跨度 = θ1 的跨度 = 164°，但 φ 的中心被 θ2/2 推着走
    printf("\n3. 每个半径下的末端方位角覆盖（φ = θ1 + θ2/2）\n");
    printf("   r(mm) |  θ2(°) | φ中心(°) | φ 范围(°)\n");
    printf("   ------+--------+----------+------------------\n");
    {
        float r;
        const float half = (SERVO1_MAX_DEG - SERVO1_MIN_DEG) / 2.0f;   // θ1 半行程 = 82

        for (r = 20.0f; r <= 270.0f; r += 10.0f)
        {
            float c2 = (r * r - ARM_L1_MM * ARM_L1_MM - ARM_L2_MM * ARM_L2_MM)
                     / (2.0f * ARM_L1_MM * ARM_L2_MM);          // 余弦定理
            float t2, ctr, lo, hi;

            if (c2 < -1.0f) c2 = -1.0f;                          // 夹住，防 NaN
            if (c2 >  1.0f) c2 =  1.0f;

            t2  = acosf(c2) * 180.0f / PI_F;                     // θ2，单位度
            ctr = t2 / 2.0f;                                     // φ 的中心
            lo  = ctr - half;                                    // φ 下限
            hi  = ctr + half;                                    // φ 上限

            while (lo < 0.0f) lo += 360.0f;                      // 归一化到 [0,360)
            while (hi < 0.0f) hi += 360.0f;

            printf("   %5.0f | %6.1f | %8.1f | %5.1f ~ %5.1f%s\n",
                   r, t2, ctr, lo, hi, (lo > hi) ? "  ← 跨 0°" : "");
        }
    }

    // ---------- 3b. 所有半径都能覆盖的公共方位角 ----------
    {
        float r, lo_max = -999.0f, hi_min = 999.0f;
        const float half = (SERVO1_MAX_DEG - SERVO1_MIN_DEG) / 2.0f;

        // 只看不跨 0° 的那一段（跨 0° 的按 +360 展开算，取主区间）
        for (r = 120.0f; r <= 260.0f; r += 5.0f)               // 实际建议工作区
        {
            float c2 = (r * r - ARM_L1_MM * ARM_L1_MM - ARM_L2_MM * ARM_L2_MM)
                     / (2.0f * ARM_L1_MM * ARM_L2_MM);
            float t2, ctr, lo, hi;

            if (c2 < -1.0f) c2 = -1.0f;
            if (c2 >  1.0f) c2 =  1.0f;

            t2  = acosf(c2) * 180.0f / PI_F;
            ctr = t2 / 2.0f;
            lo  = ctr - half;
            hi  = ctr + half;

            if (lo > lo_max) lo_max = lo;                      // 取所有半径里最靠上的下界
            if (hi < hi_min) hi_min = hi;                      // 取最靠下的上界
        }
        printf("\n   3b. r ∈ [120,260] 时，所有半径都够得到的公共方位角：\n");
        printf("       φ ∈ [%.1f°, %.1f°]，共 %.1f°\n", lo_max, hi_min, hi_min - lo_max);
    }

    // ---------- 4. 拿几个具体点验证 ----------
    printf("\n4. 具体点解算（以 +X 为 0°，逆时针为正）\n");
    printf("   x(mm)  y(mm) | θ1(°)  θ2(°) | 舵机1 舵机2\n");
    printf("   ------------+-------------+--------------\n");
    {
        const float pts[][2] = {
            { 250.0f,   0.0f },                 // 正前方最远
            { 150.0f, 150.0f },                 // 左前 45°
            {   0.0f, 250.0f },                 // 正左
            {-150.0f, 150.0f },                 // 左后
            { 200.0f,  80.0f },                 // 斜前
        };
        int i;
        for (i = 0; i < (int)(sizeof(pts) / sizeof(pts[0])); i++)
        {
            ArmJoint_t j;
            int rc = ik_solve(pts[i][0], pts[i][1], 1, &j);
            if (rc == KIN_OK)
                printf("   %6.0f %6.0f | %6.1f %6.1f | %5.1f %5.1f\n",
                       pts[i][0], pts[i][1], j.theta1_deg, j.theta2_deg,
                       j.servo1_deg, j.servo2_deg);
            else if (rc == KIN_ERR_OUTER)
                printf("   %6.0f %6.0f | 太远，够不到\n", pts[i][0], pts[i][1]);
            else if (rc == KIN_ERR_INNER)
                printf("   %6.0f %6.0f | 太近，内死区\n", pts[i][0], pts[i][1]);
            else
                printf("   %6.0f %6.0f | 舵机行程不够（θ1=%.1f θ2=%.1f）\n",
                       pts[i][0], pts[i][1], j.theta1_deg, j.theta2_deg);
        }
    }

    printf("\n");
    return 0;
}
