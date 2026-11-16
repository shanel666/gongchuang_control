#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

void Key_Init(void);
uint8_t Key_GetNum(void);
/* 键码：1=PB1  2=PB11  3=PB10  4=PA12(舵机角度+1)  5=PA11(舵机角度-1) */

/* PA8 是否按下（带 20 ms 消抖）。返回 1 = 按下。
 * ★ 注意极性：PA8 是"按下接 GND"，和其它三个键（按下接 3.3V）相反，
 *   调用前要把 PA8 配成 GPIO_Mode_IPU。 */
uint8_t Key_PA8Hit(void);

/* ---------------------------------------------------------------------------
 * PB1 / PA7 / PA5 这三个键的读键工具（夹取动作和视觉闭环共用）
 * ---------------------------------------------------------------------------
 * 极性见 Key.c 里的 GRAB_KEY_ACTIVE_HIGH：本板是"一脚接 3.3V、一脚接引脚"，
 * 所以配下拉输入(IPD)、读到高电平 = 按下。
 *   PB1：单步 / 切换     PA7：连跑 / 点动自检     PA5：中止 / 急停
 * ------------------------------------------------------------------------- */

/* 初始化 PB1 / PA7 / PA5（下拉输入） */
void grab_key_init(void);

/* 读一个按键：1 = 按下。port/pin 用 GPIOB/GPIO_Pin_1 这样的写法 */
uint8_t grab_key_down(GPIO_TypeDef *port, uint16_t pin);

/* 等松手（最多 2 s 超时）。返回 1 = 已松开，0 = 超时还在按 */
uint8_t grab_wait_release(GPIO_TypeDef *port, uint16_t pin);

/* PA7：连跑键（消抖 + 等松手）。返回 1 = 这一次按下了 */
uint8_t grab_start_hit(void);

/* PB1：单步键（消抖 + 等松手）。返回 1 = 这一次按下了 */
uint8_t grab_step_hit(void);

/* PA5：中止键（消抖，不等松手）。返回 1 = 正按着 */
uint8_t grab_abort_hit(void);

/* 等 ms 毫秒，期间每 10 ms 检查 PA5：按了就提前返回。
 * 返回 1 = 正常等完，0 = 被 PA5 中止。 */
uint8_t grab_wait(uint16_t ms);

#endif
