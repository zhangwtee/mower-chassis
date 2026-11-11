// 串口。收发都走中断:发送进环形缓冲,中断里出;接收直接回调。
#ifndef UART_H
#define UART_H

#include <stdint.h>

typedef void (*uart_rx_fn)(uint8_t c);

void uart_init(void);

//两路串口累计丢掉的整帧数(缓冲满时整帧丢弃)
uint32_t uart_tx_drop(void);

void uart3_write(const uint8_t *d, uint32_t n);

//返回 0 表示发送缓冲满,这次没写进去

void uart4_set_rx(uart_rx_fn fn);

void uart1_write(const uint8_t *d, uint32_t n);

void uart1_set_rx(uart_rx_fn fn);
void uart3_set_rx(uart_rx_fn fn);

#endif
