// test_servo_speed.c —— 在 PC 上看梯形加减速的速度曲线
// 编译（在 pc_test 目录下）：
//   gcc -I../User/bsp -Imock ../User/bsp/servo.c test_servo_speed.c -o test_speed.exe -lm
// 跑：./test_speed.exe
#include <stdio.h>
#include <stdint.h>
#include "mock_hal.h"
#include "servo.h"

// ---------- 假的硬件句柄 ----------
TIM_HandleTypeDef  htim1, htim2, htim4;
UART_HandleTypeDef huart1;
TIM_TypeDef        mock_tim1;
GPIO_TypeDef       gpio_a, gpio_b, gpio_c;

// ---------- 假的 HAL ----------
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch) { (void)h; (void)ch; return HAL_OK; }
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h, uint32_t ch)  { (void)h; (void)ch; return HAL_OK; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s) { (void)p; (void)pin; (void)s; }
uint32_t HAL_GetTick(void) { return 0; }
void HAL_Delay(uint32_t ms) { (void)ms; }

int main(void)
{
    int   i;
    float prev, now, v;

    servo_init();                                       // 四路回中位 90°

    printf("=== 测试：大臂 90° -> 150°（60°），设定 2 秒 ===\n");
    printf("每拍 = 20ms，2 秒 = 100 拍\n\n");
    printf("拍数   角度(°)   本拍速度(°/拍)   等效速度(°/s)\n");
    printf("-----  --------  -------------   -------------\n");

    servo_move_time(SERVO_ARM1, 150.0f, 2000);          // 2 秒转 60°
    prev = servo_get_deg(SERVO_ARM1);

    for (i = 1; i <= 140; i++)                          // 最多跑 2.8 秒
    {
        servo_update();                                 // 走一拍
        now = servo_get_deg(SERVO_ARM1);                // 读当前角度
        v   = now - prev;                               // 本拍走了多少度

        if (i <= 12 || (i % 10 == 0) || now >= 149.99f)
            printf("%5d  %8.2f  %13.3f   %13.1f\n", i, now, v, v * 50.0f);

        prev = now;
        if (now >= 149.99f && i > 1 && v == 0.0f)       // 到了且不动了
        {
            printf("\n★ 实际用时：%d 拍 = %.1f 秒（设定 2.0 秒）\n", i, i * 0.02f);
            break;
        }
    }

    // ---------- 对比：设一个很快的速度 ----------
    printf("\n=== 对比：同样 60°，最大速度 1.5°/拍（75°/s） ===\n");
    servo_move_deg(SERVO_ARM1, 90.0f, 1.5f);            // 先回 90
    for (i = 0; i < 200 && servo_is_busy(); i++) servo_update();
    printf("回中位用了 %d 拍\n", i);

    servo_move_deg(SERVO_ARM1, 150.0f, 1.5f);           // 再走 60°
    prev = servo_get_deg(SERVO_ARM1);
    for (i = 1; i <= 200; i++)
    {
        servo_update();
        now = servo_get_deg(SERVO_ARM1);
        v   = now - prev;
        if (i <= 8 || (i % 10 == 0) || now >= 149.99f)
            printf("%5d  %8.2f  %13.3f   %13.1f\n", i, now, v, v * 50.0f);
        prev = now;
        if (now >= 149.99f && v == 0.0f)
        {
            printf("\n★ 实际用时：%d 拍 = %.1f 秒\n", i, i * 0.02f);
            break;
        }
    }

    return 0;
}
