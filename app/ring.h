// 通用环形缓冲。一个写者一个读者,不需要加锁。
#ifndef RING_H
#define RING_H

#include <stdint.h>

typedef struct {
    uint8_t *buf;
    uint16_t size;      //必须是 2 的幂
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_t;

void ring_init(ring_t *r, uint8_t *buf, uint16_t size);

//返回 1 写入成功,0 表示满
uint8_t ring_put(ring_t *r, uint8_t c);

//返回 1 取到,0 表示空
uint8_t ring_get(ring_t *r, uint8_t *c);

uint16_t ring_used(const ring_t *r);

//还能放多少个字节。容量是 size-1
uint16_t ring_free(const ring_t *r);

#endif
