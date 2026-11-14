// CAN1。1Mbps,PD0=RX,PD1=TX。
//
// 这条总线上不止我们一个节点,所以 ID 分配集中写在这里,不要在别处写裸数字。
#ifndef CAN_H
#define CAN_H

#include <stdint.h>

/* 超声波板:六路,只发不收 */
#define CAN_ID_ULTRA_ABCD       0x101u
#define CAN_ID_ULTRA_EF         0x102u

/* 底盘遥测,分三帧发。注意不能用 0x101/0x102,那两个是超声波板的 */
#define CAN_ID_CHASSIS_1        0x201u
#define CAN_ID_CHASSIS_2        0x202u
#define CAN_ID_CHASSIS_3        0x203u

/* 上游控制指令(本机接收) */
#define CAN_ID_CHASSIS_CMD      0x181u

/* 自动回充板(本工程不实现,ID 先占着) */
#define CAN_ID_CHARGER_SET      0x105u
#define CAN_ID_CHARGER_STATE    0x182u

typedef void (*can_rx_fn)(uint16_t id, const uint8_t *data);

void can_init(void);

void can_set_rx(can_rx_fn fn);

//发送 8 字节标准帧。返回 0 成功,1 无空闲邮箱,2 超时
uint8_t can_send(uint16_t id, const uint8_t *data);

#endif
