// 参数区。用 sector 3(0x0800C000,16KB)。
//
// 选 16KB 的扇区而不是最后一个 128KB 的,是因为擦除时间和扇区大小成正比:
// 16KB 大约 250ms,128KB 要 1~2 秒。擦写期间 Flash 被占用、CPU 停在那儿,
// 越短越好。
//
// 代码现在 22KB,离 0x0800C000 还有 26KB 余量。代码涨过 48KB 就要把
// 参数区挪到别的扇区,记得同步改这里和 docs/架构.md。
#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

#define FLASH_PARAM_ADDR    0x0800C000u
#define FLASH_PARAM_SIZE    0x4000u

//写入,自动先擦。len 会向上取整到 4 的倍数。返回 0 成功
uint8_t flash_param_write(const void *buf, uint32_t len);

#endif
