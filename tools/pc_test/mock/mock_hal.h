// mock_hal.h —— 假的 HAL 头文件，只为在 PC 上做语法检查
// 不参与任何实际运行，纯粹用来在 CubeMX 工程生成之前抓拼写/类型错误
#ifndef __MOCK_HAL_H
#define __MOCK_HAL_H

#include <stdint.h>

// ---- 基本类型 ----
typedef struct { volatile uint32_t ARR; volatile uint32_t CCR1; volatile uint32_t CCR2; volatile uint32_t CCR3; } TIM_TypeDef;
typedef struct { uint32_t dummy; } GPIO_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

// ---- 全局句柄（CubeMX 生成的真东西） ----
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

extern GPIO_TypeDef gpio_a, gpio_b, gpio_c;

// ---- 常量 ----
#define TIM_CHANNEL_1   0x00000000U
#define TIM_CHANNEL_2   0x00000004U
#define TIM_CHANNEL_3   0x00000008U
#define TIM_IT_UPDATE   0x0001U
#define TIM_FLAG_UPDATE 0x0001U

// ---- 函数 ----
void          HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
uint32_t      HAL_GetTick(void);
void HAL_Delay(uint32_t ms);

int           HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);
int           HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel);

// ---- 宏（和真 HAL 同名，行为换成空操作） ----
#define __HAL_TIM_SET_COMPARE(h, ch, val)   do { (void)(h); (void)(ch); (void)(val); } while (0)
#define __HAL_TIM_SET_AUTORELOAD(h, val)    do { (void)(h); (void)(val); } while (0)
#define __HAL_TIM_ENABLE_IT(h, it)          do { (void)(h); (void)(it); } while (0)
#define __HAL_TIM_DISABLE_IT(h, it)         do { (void)(h); (void)(it); } while (0)
#define __HAL_TIM_CLEAR_FLAG(h, f)          do { (void)(h); (void)(f); } while (0)


// ---- 串口 ----
typedef struct { void *Instance; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1;
int HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size, uint32_t timeout);
int HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size, uint32_t timeout);
#define HAL_OK 0

// ---- 定时器外设基址（真实 HAL 里是 (TIM_TypeDef*)0x40012C00 这类地址） ----
extern TIM_TypeDef mock_tim1;
#define TIM1 (&mock_tim1)

#endif
