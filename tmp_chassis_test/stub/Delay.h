#ifndef STUB_DELAY_H
#define STUB_DELAY_H

#include <stdint.h>

/* PC 端测试桩：只累计延时，不真的睡 */
void Delay_ms(uint32_t ms);

#endif
