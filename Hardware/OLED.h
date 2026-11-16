#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

/* 硬件 I2C1：PB6=SCL，PB7=SDA，默认 400kHz */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/* 硬件 I2C 效率测试：清屏 + 整屏刷新计时，返回耗时 us */
uint32_t OLED_MeasureFullFrame(void);

/* 测量全屏刷新并在第 3、4 行显示 us / KB/s / 总线占用率，返回耗时 us */
uint32_t OLED_ShowI2CStats(void);

/* 面板是否在线：OLED_Init() 会探测一次；不在线时所有绘图函数直接返回，
 * 不会再去等 I2C 超时（没接 OLED 时上电不会卡十几秒） */
uint8_t OLED_IsReady(void);

/* I2C 写失败累计次数（从不断清零）。
 * 屏幕出现"缺笔划 / 只显示上半截"时看这个数：
 *   一直在涨 -> I2C 时序/接线问题（先降 OLED_I2C_SPEED）
 *   不涨     -> 不是总线问题，是画的内容/坐标问题 */
uint32_t OLED_GetI2cErrors(void);

#endif
