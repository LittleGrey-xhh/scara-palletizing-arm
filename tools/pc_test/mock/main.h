#ifndef __MAIN_H
#define __MAIN_H
#include "mock_hal.h"
#define STEP_DIR_Pin      ((uint16_t)0x0200)
#define STEP_DIR_GPIO_Port (&gpio_b)
#define STEP_ENA_Pin      ((uint16_t)0x4000)
#define STEP_ENA_GPIO_Port (&gpio_b)
#define LIMIT_Z_Pin       ((uint16_t)0x1000)
#define LIMIT_Z_GPIO_Port (&gpio_b)
#endif
