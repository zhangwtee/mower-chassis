// 串口。UART3 接上位机,UART4 接蓝牙
#include "stm32f4xx.h"
#include "uart.h"
#include "ring.h"

#define UART_TX_SIZE    512

typedef struct {
    USART_TypeDef *us;
    ring_t tx;
    uint8_t txbuf[UART_TX_SIZE];
    uart_rx_fn rx_fn;
    uint32_t tx_drop;
} uart_t;

static uart_t k_u1;
static uart_t k_u3;
static uart_t k_u4;

static void uart_hw_init(uart_t *u, USART_TypeDef *us, uint32_t bound)
{
    GPIO_InitTypeDef g;
    USART_InitTypeDef s;
    NVIC_InitTypeDef n;

    u->us = us;
    ring_init(&u->tx, u->txbuf, UART_TX_SIZE);
    u->rx_fn = 0;
    u->tx_drop = 0;

    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;

    if (us == USART1) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
        g.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
        GPIO_Init(GPIOA, &g);
        n.NVIC_IRQChannel = USART1_IRQn;
    } else if (us == USART3) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
        g.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
        GPIO_Init(GPIOB, &g);
        n.NVIC_IRQChannel = USART3_IRQn;
    } else {
        //UART4 = PC10/PC11,接蓝牙 APP
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
        g.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
        GPIO_Init(GPIOC, &g);
        n.NVIC_IRQChannel = UART4_IRQn;
    }

    s.USART_BaudRate = bound;
    s.USART_WordLength = USART_WordLength_8b;
    s.USART_StopBits = USART_StopBits_1;
    s.USART_Parity = USART_Parity_No;
    s.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    s.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(us, &s);

    USART_ITConfig(us, USART_IT_RXNE, ENABLE);
    USART_Cmd(us, ENABLE);

    //不调 FreeRTOS API,优先级可以高于 configMAX_SYSCALL
    n.NVIC_IRQChannelPreemptionPriority = 5;  //= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
}

uint32_t uart_tx_drop(void)
{
    return k_u1.tx_drop + k_u3.tx_drop;
}

void uart_init(void)
{
    uart_hw_init(&k_u1, USART1, 115200);
    uart_hw_init(&k_u3, USART3, 115200);

    //蓝牙 APP 走 9600,和老固件一致
    uart_hw_init(&k_u4, UART4, 9600);
}

static void uart_write(uart_t *u, const uint8_t *d, uint32_t n)
{
    uint32_t i;

    //放不下就整帧丢掉,不发半帧。对面虽然有帧头重同步能丢掉残帧,
    //但那是别人的容错,不该当成自己的机制用
    if (ring_free(&u->tx) < n) {
        u->tx_drop++;
        return;
    }

    for (i = 0; i < n; i++) {
        (void)ring_put(&u->tx, d[i]);
    }

    USART_ITConfig(u->us, USART_IT_TXE, ENABLE);
}

void uart1_write(const uint8_t *d, uint32_t n)
{
    uart_write(&k_u1, d, n);
}

void uart3_write(const uint8_t *d, uint32_t n)
{
    uart_write(&k_u3, d, n);
}




void uart4_set_rx(uart_rx_fn fn)
{
    k_u4.rx_fn = fn;
}

void uart1_set_rx(uart_rx_fn fn)
{
    k_u1.rx_fn = fn;
}

void uart3_set_rx(uart_rx_fn fn)
{
    k_u3.rx_fn = fn;
}

static void uart_irq(uart_t *u)
{
    if (USART_GetITStatus(u->us, USART_IT_RXNE) != RESET) {
        uint8_t c = (uint8_t)(u->us->DR & 0xFF);

        if (u->rx_fn) u->rx_fn(c);
    }

    if (USART_GetITStatus(u->us, USART_IT_TXE) != RESET) {
        uint8_t c;

        if (ring_get(&u->tx, &c)) {
            u->us->DR = c;
        } else {
            USART_ITConfig(u->us, USART_IT_TXE, DISABLE);
        }
    }
}

void USART1_IRQHandler(void)
{
    uart_irq(&k_u1);
}

void USART3_IRQHandler(void)
{
    uart_irq(&k_u3);
}

void UART4_IRQHandler(void)
{
    uart_irq(&k_u4);
}
