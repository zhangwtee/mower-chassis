// 故障屏。异常上下文里直接把 CFSR 写到 OLED,然后停住
#include "stm32f4xx.h"
#include "fault.h"
#include "oled.h"
#include "led.h"
#include "buzzer.h"

void fault_halt(const char *tag)
{
    uint32_t cfsr = SCB->CFSR;
    uint32_t hfsr = SCB->HFSR;
    uint32_t far = 0;

    //有 precise 错误才拿得到出错地址
    if (cfsr & 0x8000u) {
        far = SCB->MMFAR;          //MMARVALID
    } else if (cfsr & 0x800000u) {
        far = SCB->BFAR;           //BFARVALID
    }

    led_set(LED_RED);
    buzzer_set(1);

    oled_clear();
    oled_str(0, 0, "FAULT");
    oled_str(48, 0, tag);

    oled_str(0, 1, "C");
    oled_hex(12, 1, cfsr);

    oled_str(0, 2, "H");
    oled_hex(12, 2, hfsr);

    oled_str(0, 3, "A");
    oled_hex(12, 3, far);

    oled_flush();

    for (;;) {
    }
}
