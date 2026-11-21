// app_main.h —— 应用层入口
#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#include <stdint.h>

void app_init(void);        // 上电初始化，放在 main() 的 while(1) 之前
void app_loop(void);        // 主循环体，放在 while(1) 里面

#endif
