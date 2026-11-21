// test_protocol.c —— 在 PC 上跑一遍串口命令解析，不需要任何硬件
// 编译（在 pc_test 目录下）：
//   gcc -I../User/bsp -I../User/app -Imock
//       ../User/bsp/servo.c ../User/bsp/step.c ../User/bsp/kinematics.c
//       ../User/app/app_main.c test_protocol.c -o test_proto.exe -lm
// 跑：./test_proto.exe
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "mock_hal.h"

// ---------- 假的硬件句柄 ----------
TIM_HandleTypeDef  htim1, htim2, htim4;
UART_HandleTypeDef huart1;
TIM_TypeDef        mock_tim1;
GPIO_TypeDef       gpio_a, gpio_b, gpio_c;

// ---------- 假的输入流 ----------
static const char *g_in  = "";
static int         g_idx = 0;

// ---------- 假的 HAL 实现 ----------
int HAL_UART_Receive(UART_HandleTypeDef *h, uint8_t *d, uint16_t n, uint32_t t)
{
    (void)h; (void)n; (void)t;
    if (g_in[g_idx] == 0) return 1;          // 没数据了
    *d = (uint8_t)g_in[g_idx++];             // 喂一个字节
    return HAL_OK;
}

int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *d, uint16_t n, uint32_t t)
{
    (void)h; (void)t;
    while (n--) putchar((char)*d++);         // 直接打到屏幕
    return HAL_OK;
}

int HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch) { (void)h; (void)ch; return HAL_OK; }
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h, uint32_t ch)  { (void)h; (void)ch; return HAL_OK; }

void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s) { (void)p; (void)pin; (void)s; }
uint32_t HAL_GetTick(void) { return 0; }     // 时间不走，20ms 节拍不触发
void HAL_Delay(uint32_t ms) { (void)ms; }    // PC 上不等

// ---------- 应用层入口 ----------
extern void app_init(void);
extern void app_loop(void);

// 喂一整行命令
static void feed(const char *cmd)
{
    static char buf[64];
    snprintf(buf, sizeof(buf), "%s\n", cmd);   // 补上换行，协议要求以 \n 结尾

    g_in  = buf;
    g_idx = 0;
    printf("> %s\n", cmd);
    app_loop();                              // 一次 loop 把整行吃完
    printf("\n");
}

int main(void)
{
    static const char *cmds[] = {
        "?",                                 // 帮助
        "K 0 1500",                          // 标定：大臂走中位
        "K 2 1500",                          // 标定：腕转走中位
        "K 3 1500",                          // 标定：夹爪走中位
        "K 4 1500",                          // 编号越界 → ERR 1
        "K 1 3000",                          // 脉宽超限 → ERR 2
        "M 180 60",                          // 可达点
        "M 250 0",                           // 正前方最远，可达
        "M 500 0",                           // 超出行程 → ERR 2
        "M 10 10",                           // 内死区 → 可能 ERR 2/3
        "Z 50",                              // 没回零 → ERR 5
        "W 90",                              // 腕转 90°
        "W 200",                             // 超出 0~180 → ERR 2
        "C 1",                               // 夹爪闭合
        "P 150 150 80",                      // 完整点位，没回零 → ERR 5
        "S",                                 // 查询
        "X",                                 // 未知命令 → ERR 1
        0
    };
    int i;

    app_init();                              // 上电初始化

    for (i = 0; cmds[i]; i++)                // 逐条喂命令
        feed(cmds[i]);

    printf("=== 测试结束 ===\n");
    return 0;
}
