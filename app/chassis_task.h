// 控制主环。100Hz,唯一修改电机输出的地方。
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

//建任务前先初始化状态机和 PI
void chassis_ctx_init(void);

void chassis_task(void *arg);

#endif
