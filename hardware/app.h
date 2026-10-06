#ifndef COREXY_APP_H
#define COREXY_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* 外设初始化完成后调用；KEY1自检，KEY2沿+Y点动。 */
void App_Init(void);
void App_Poll(void);

#ifdef __cplusplus
}
#endif

#endif
