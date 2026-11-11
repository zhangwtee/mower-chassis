// 11 字节下行帧解析。帧头 0x7B,帧尾 0x7D,BCC 取前 9 字节。
// 中断只把字节塞进环形缓冲,解析在任务里做。协议见 docs/协议.md
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PROTOCOL_FRAME_LEN   11
#define PROTOCOL_BCC_LEN     9
#define PROTOCOL_HEADER      0x7B
#define PROTOCOL_TAIL        0x7D
#define PROTOCOL_CMD_COUNT   9

#define PROTOCOL_SERVO_MIN   500
#define PROTOCOL_SERVO_MAX   2500

//100Hz 轮询下 115200 每周期最多约 115 字节,留一倍余量
#define PROTOCOL_RX_RING     256

//帧来源,沿用老固件的控制方式位
#define PROTOCOL_SRC_ROS     0x01
#define PROTOCOL_SRC_USART   0x20

//解析结果回调。必须全部填上,不给空指针。
typedef struct protocol_sink {
    void (*on_velocity)(float vx, float vy, float vz, uint8_t src);
    void (*on_recharge)(float vx, float vy, float vz, uint8_t nav, uint8_t src);
    void (*on_dock)(float vx, float vy, float vz);
    void (*on_servo)(uint16_t pulse_us);
    void (*on_selftest)(void);
    void (*on_blade)(uint8_t on);
    void (*on_estop)(void);
    void (*on_estop_release)(void);
} protocol_sink_t;

typedef struct {
    const protocol_sink_t *sink;
    uint8_t src;

    //中断写 head,任务写 tail
    uint8_t buf[PROTOCOL_RX_RING];
    volatile uint16_t head;
    volatile uint16_t tail;

    uint32_t overflow;
    uint32_t bad_frame;
    uint32_t unknown_cmd;

    uint8_t frame[PROTOCOL_FRAME_LEN];
    uint8_t idx;
} protocol_t;

void protocol_init(protocol_t *p, const protocol_sink_t *sink, uint8_t src);

//串口接收中断里调,只入队
void protocol_feed_isr(protocol_t *p, uint8_t byte);

//任务里调,把队列里的字节全部解析掉
void protocol_poll(protocol_t *p);

int16_t protocol_decode_i16(uint8_t hi, uint8_t lo);

uint8_t protocol_bcc(const uint8_t *data, uint32_t n);

#endif
