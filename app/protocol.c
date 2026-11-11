// 下行帧。11 字节一帧,解完交给 link
#include "protocol.h"

static uint16_t ring_next(uint16_t i)
{
    return (i + 1) % PROTOCOL_RX_RING;
}

int16_t protocol_decode_i16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

uint8_t protocol_bcc(const uint8_t *data, uint32_t n)
{
    uint8_t v = 0;

    while (n--) {
        v ^= *data++;
    }
    return v;
}

//mm/s 转 m/s
static float speed_at(const uint8_t *f, uint8_t off)
{
    return (float)protocol_decode_i16(f[off], f[off + 1]) / 1000.0f;
}

static void handle_velocity(protocol_t *p, const uint8_t *f)
{
    p->sink->on_velocity(speed_at(f, 3), speed_at(f, 5), speed_at(f, 7), p->src);
}

static void handle_recharge(protocol_t *p, const uint8_t *f)
{
    p->sink->on_recharge(speed_at(f, 3), speed_at(f, 5), speed_at(f, 7),
                         f[1] == 1 ? 1 : 0, p->src);
}

static void handle_dock(protocol_t *p, const uint8_t *f)
{
    p->sink->on_dock(speed_at(f, 3), speed_at(f, 5), speed_at(f, 7));
}

//舵机脉宽再夹一次,顶到死点会堵转
static void handle_servo(protocol_t *p, const uint8_t *f)
{
    int16_t v = protocol_decode_i16(f[3], f[4]);

    if (v < PROTOCOL_SERVO_MIN) v = PROTOCOL_SERVO_MIN;
    if (v > PROTOCOL_SERVO_MAX) v = PROTOCOL_SERVO_MAX;

    p->sink->on_servo((uint16_t)v);
}

static void handle_selftest(protocol_t *p, const uint8_t *f)
{
    (void)f;
    p->sink->on_selftest();
}

//刀片只看 [3] 一个字节
static void handle_blade(protocol_t *p, const uint8_t *f)
{
    p->sink->on_blade(f[3] ? 1 : 0);
}

static void handle_estop(protocol_t *p, const uint8_t *f)
{
    (void)f;
    p->sink->on_estop();
}

static void handle_estop_release(protocol_t *p, const uint8_t *f)
{
    (void)f;
    p->sink->on_estop_release();
}

typedef void (*protocol_handler_t)(protocol_t *p, const uint8_t *f);

static const protocol_handler_t k_handlers[PROTOCOL_CMD_COUNT] = {
    handle_velocity,
    handle_recharge,   //1
    handle_recharge,   //2
    handle_dock,       //3
    handle_servo,      //4
    handle_selftest,   //5
    handle_blade,      //6
    handle_estop,      //7
    handle_estop_release  //8
};

//校验不过时在已收的 11 字节里找下一个帧头,把后面的内容前移接着组。
//不清零,免得一次误码把下一帧也带走。
static void resync(protocol_t *p)
{
    uint8_t k, n, i;

    for (k = 1; k < PROTOCOL_FRAME_LEN; k++) {
        if (p->frame[k] == PROTOCOL_HEADER) break;
    }

    if (k >= PROTOCOL_FRAME_LEN) {
        p->idx = 0;
        return;
    }

    n = PROTOCOL_FRAME_LEN - k;
    for (i = 0; i < n; i++) {
        p->frame[i] = p->frame[k + i];
    }
    p->idx = n;
}

static void dispatch(protocol_t *p, const uint8_t *f)
{
    uint8_t cmd = f[1];

    if (cmd < PROTOCOL_CMD_COUNT) {
        k_handlers[cmd](p, f);
    } else {
        //命令号超出范围:不动,不计。安全上靠这里挡住就够
    }
}

static void parse_byte(protocol_t *p, uint8_t c)
{
    if (p->idx == 0) {
        if (c == PROTOCOL_HEADER) {
            p->frame[0] = c;
            p->idx = 1;
        }
        return;
    }

    p->frame[p->idx++] = c;
    if (p->idx < PROTOCOL_FRAME_LEN) return;

    p->idx = 0;

    if (p->frame[10] == PROTOCOL_TAIL && protocol_bcc(p->frame, PROTOCOL_BCC_LEN) == p->frame[9]) {
        dispatch(p, p->frame);
    } else {
        p->bad_frame++;
        resync(p);
    }
}

void protocol_init(protocol_t *p, const protocol_sink_t *sink, uint8_t src)
{
    uint8_t i;

    p->sink = sink;
    p->src = src;
    p->head = 0;
    p->tail = 0;
    p->overflow = 0;
    p->bad_frame = 0;
    p->unknown_cmd = 0;
    p->idx = 0;

    for (i = 0; i < PROTOCOL_FRAME_LEN; i++) {
        p->frame[i] = 0;
    }
}

void protocol_feed_isr(protocol_t *p, uint8_t c)
{
    uint16_t nh = ring_next(p->head);

    if (nh == p->tail) {
        p->overflow++;
        return;
    }

    p->buf[p->head] = c;
    p->head = nh;
}

void protocol_poll(protocol_t *p)
{
    while (p->tail != p->head) {
        uint8_t c = p->buf[p->tail];
        p->tail = ring_next(p->tail);
        parse_byte(p, c);
    }
}
