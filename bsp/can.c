// CAN1。超声波板挂这上面
#include "stm32f4xx.h"
#include "can.h"

static can_rx_fn s_rx;

static void can_gpio_init(void)
{
    GPIO_InitTypeDef g;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    //PD0 = CAN1_RX,PD1 = CAN1_TX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    g.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOD, &g);
}

void can_init(void)
{
    CAN_InitTypeDef c;
    CAN_FilterInitTypeDef f;
    NVIC_InitTypeDef n;

    can_gpio_init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    //APB1 = 42MHz,(1+3+3) 个时间片,分频 6 → 1Mbps
    c.CAN_TTCM = DISABLE;
    //自动离线恢复。没有 ACK 时每帧都算错,TEC 涨到 255 就 bus-off,
    //而 ABOM 关着的话会一直停在那儿直到复位。超声波板晚上电、总线
    //插拔都会触发,调试初期很容易踩到
    c.CAN_ABOM = ENABLE;
    c.CAN_AWUM = DISABLE;
    //不自动重传:总线上少一个节点时不要卡在发送里
    c.CAN_NART = ENABLE;
    c.CAN_RFLM = DISABLE;
    c.CAN_TXFP = DISABLE;
    c.CAN_Mode = CAN_Mode_Normal;
    c.CAN_SJW = CAN_SJW_1tq;
    c.CAN_BS1 = CAN_BS1_3tq;
    c.CAN_BS2 = CAN_BS2_3tq;
    c.CAN_Prescaler = 6;
    CAN_Init(CAN1, &c);

    //全通滤波,收什么都要,ID 由回调自己分
    f.CAN_FilterNumber = 0;
    f.CAN_FilterMode = CAN_FilterMode_IdMask;
    f.CAN_FilterScale = CAN_FilterScale_32bit;
    f.CAN_FilterIdHigh = 0x0000;
    f.CAN_FilterIdLow = 0x0000;
    f.CAN_FilterMaskIdHigh = 0x0000;
    f.CAN_FilterMaskIdLow = 0x0000;
    f.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    f.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&f);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    n.NVIC_IRQChannel = CAN1_RX0_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 5;  //= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
}

void can_set_rx(can_rx_fn fn)
{
    s_rx = fn;
}

uint8_t can_send(uint16_t id, const uint8_t *data)
{
    CanTxMsg tx;
    uint32_t guard = 0xFFFFu;
    uint8_t mbox;
    uint8_t i;

    tx.StdId = id;
    tx.ExtId = 0;
    tx.IDE = CAN_Id_Standard;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    for (i = 0; i < 8; i++) {
        tx.Data[i] = data[i];
    }

    mbox = CAN_Transmit(CAN1, &tx);
    if (mbox == CAN_TxStatus_NoMailBox) {
        return 1;
    }

    //等这一帧真的出去,不等邮箱空
    while (CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Pending && guard) {
        guard--;
    }

    return (guard == 0u) ? 2u : 0u;
}

void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) == RESET) {
        return;
    }

    CAN_Receive(CAN1, CAN_FIFO0, &rx);

    if (rx.IDE == CAN_Id_Standard && s_rx) {
        s_rx(rx.StdId, rx.Data);
    }
}
