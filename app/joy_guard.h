// 摇杆命令守门。
//
// 上位机那条路靠 cmd_age 500ms 超时保护;蓝牙摇杆这条路原来没有保护 ——
// 它每拍都算"新指令",链路断了车会按最后方向一直跑。
//
// 这里给它单独算年龄:APP 每发一个方向字节,序号 +1。序号变了说明人还按着,
// 年龄清零;序号不动说明没有新输入,年龄照涨。涨过 JOY_TIMEOUT_MS 就作废。
//
// **这是个取舍**,写在文档里:WHEELTEC 的 APP 只在按下/松开时各发一个字节,
// 按住期间不发。所以超时值必须比"一次正常按住"长,否则按着按着车就停了。
// 如果实测发现 APP 其实会周期性重发,就把这个值调小;如果发现按住会自己停,
// 说明它不重发,那就把值调大,或者当成"每 N 毫秒要松一下"来用。
#ifndef JOY_GUARD_H
#define JOY_GUARD_H

#include <stdint.h>

#ifndef JOY_TIMEOUT_MS
#define JOY_TIMEOUT_MS      5000
#endif

typedef struct {
    uint32_t last_seq;
    uint16_t age;
    uint8_t active;
} joy_guard_t;

void joy_guard_init(joy_guard_t *j);

//dir: 当前方向(0 = 松开)。seq: APP 每收到一个方向字节就 +1。
//返回 1 表示摇杆命令仍然有效
uint8_t joy_guard_step(joy_guard_t *j, uint32_t seq, uint8_t dir, uint16_t dt_ms);

//当前摇杆命令年龄,给诊断页看
uint16_t joy_guard_age(const joy_guard_t *j);

#endif
