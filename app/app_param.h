// APP 调参帧与运行时可改的参数。
//
// 参数全部用整数存,因为写的是 UART4 中断、读的是 100Hz 任务,
// 浮点的读写在 M4 上不是原子的,32 位整数是。
#ifndef APP_PARAM_H
#define APP_PARAM_H

#include <stdint.h>

typedef struct {
    volatile uint16_t rc_speed_mm;   //遥控速度基准,mm/s
    volatile uint16_t kp;
    volatile uint16_t ki;
    volatile uint16_t step_mm;       //速度平滑步进,mm/s 每拍
    volatile uint16_t line_diff;     //纠偏系数 0~100
} tune_t;

extern tune_t g_tune;

//APP 摇杆方向,0 表示松开,1~8 对应 A~H
extern volatile uint8_t g_app_dir;

//每收到一个方向字节就 +1,给摇杆超时判定用
extern volatile uint32_t g_app_dir_seq;

//收到 {W:...} 时置 1,由任务在停车时真正写 Flash
extern volatile uint8_t g_param_save_req;

void app_param_init(void);

//喂一个字节;收到右括号时解析完返回 1
uint8_t app_param_feed(uint8_t c);

//当前是否正在一个帧里
uint8_t app_param_in_frame(void);

//摇杆方向 + 速度基准 → 三轴速度。纯函数,可测
void app_joystick(uint8_t dir, uint16_t rc_mm, float *vx, float *vz);

#endif
