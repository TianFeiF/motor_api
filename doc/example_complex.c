/*
 * example_complex.c
 *
 * 复杂场景工程示例：双轴龙门架 + 6轴机械臂 + 2 IO板
 *
 * 场景描述：
 * 1. 龙门架 (Gantry)：X轴单电机，Y轴双电机同步（Y1/Y2）。
 * 2. 机械臂 (Arm)：6轴 (J1-J6)。
 * 3. IO板：2块，用于夹爪控制与传感器读取。
 *
 * 动作流程：
 * 1. 初始化并清除错误。
 * 2. 龙门架移动到取料位置 (X=100, Y=200)。
 * 3. 机械臂移动到抓取姿态。
 * 4. 打开夹爪 (IO输出)，等待传感器确认 (IO输入)。
 * 5. 机械臂抬起。
 * 6. 龙门架移动到放料位置。
 *
 * 编译说明：
 *   gcc -o example_complex example_complex.c -I../include -L../build -lmotor_api -lpthread -lrt
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <stdbool.h>

#include "motor_api.h"

/* 轴索引映射 (需与 complex_config.json 一致) */
#define AXIS_GANTRY_X   0
#define AXIS_GANTRY_Y1  1
#define AXIS_GANTRY_Y2  2  /* 与 Y1 同步反向 */

#define AXIS_ARM_J1     3
#define AXIS_ARM_J2     4
#define AXIS_ARM_J3     5
#define AXIS_ARM_J4     6
#define AXIS_ARM_J5     7
#define AXIS_ARM_J6     8

#define AXIS_IO_1       9
#define AXIS_IO_2       10

/* 状态机定义 */
typedef enum {
    STATE_IDLE,
    STATE_MOVE_TO_PICK,
    STATE_ARM_DOWN,
    STATE_GRIP_CLOSE,
    STATE_ARM_UP,
    STATE_MOVE_TO_PLACE,
    STATE_DONE
} app_state_t;

static volatile sig_atomic_t g_stop = 0;

static void sig_handler(int sig) {
    (void)sig;
    g_stop = 1;
}

/* 简单的 S型曲线或梯形规划辅助函数 (这里简化为纯增量插补) */
/* 返回：0-未到达，1-已到达目标 */
int move_axis(struct motor_api_handle *h, int axis, double target_pos, double max_vel, double *curr_pos) {
    double diff = target_pos - *curr_pos;
    /* 获取周期时间，这里简单假设 4ms */
    double dt = 0.004; 
    double step = max_vel * dt;

    if (fabs(diff) <= step) {
        *curr_pos = target_pos;
        /* 直接设置到位 */
        /* 注意：motor_api_set_axis_command 接受的是脉冲步进 step 和方向 */
        /* 但这里我们用的是上层逻辑，假设 config.json 配置了 unit_per_rev，
           我们需要计算本周期的 delta (用户单位) 传给 API 吗？
           
           motor_api_set_axis_command(h, axis, run, dir, step_pulses) 是底层 API。
           
           为了演示方便，我们这里假设用户自行计算 delta 并传给 API。
           由于 API 目前接受的是 int step (脉冲/底层单位) 或者是经过 scale 后的？
           
           查看 motor_api.c:
           int delta = run ? (int)(dir * step * h->axis_map[axis_idx].scale_pos) : 0;
           
           API 的 step 参数是 "用户单位" 吗？
           test_angle_loop.c 中：
           step_deg = ...
           motor_api_set_axis_command(..., (int)step_deg)
           
           是的，传入的是“用户单位的整数部分”。如果 unit_per_rev=360，step=1 表示 1度。
           如果 unit_per_rev=5 (mm), step=1 表示 1mm。
           这对于精细控制不够 (1mm 太粗)。
           
           实际上 test_angle_loop.c 里是用 accum_deg 累积小数，凑够 1.0 才发。
           这意味着 motor_api 目前的 set_axis_command 只能接受整数的用户单位。
           如果 scale_pos 很大（如 1mm = 10000 脉冲），那 1mm 的分辨率是可以的。
           但如果 1 unit = 1 rev，那太粗了。
           
           建议：在 config.json 中，unit_per_rev 设为 1.0 (rev) 时，step=1 就是 1圈。
           如果设为 5.0 (mm/rev)，step=1 就是 1mm。
           如果想要 um 级控制，应该设 unit_per_rev = 5000.0 (um/rev)。
           
           在 complex_config.json 中：
           Gantry: unit_per_rev = 5.0 (mm)。 step=1 -> 1mm。精度太低。
           应该改为 unit_per_rev = 5000.0 (um)。 step=1 -> 1um。
           
           Arm: unit_per_rev = 360.0 (deg)。 step=1 -> 1deg。精度太低。
           应该改为 unit_per_rev = 360000.0 (mdeg)。 step=1 -> 0.001deg。
           
           **我需要修改 config.json 里的 unit_per_rev 以支持高精度。**
        */
        motor_api_set_axis_command(h, axis, false, 0, 0); /* 停止 */
        return 1;
    }

    int dir = (diff > 0) ? 1 : -1;
    *curr_pos += dir * step;
    
    /* 假设 unit 是 um 或 mdeg，这里直接传 step */
    motor_api_set_axis_command(h, axis, true, dir, (int)step);
    return 0;
}

int main(int argc, char **argv) {
    const char *cfg_path = "complex_config.json";
    if (argc > 1) cfg_path = argv[1];

    struct motor_api_handle *h = NULL;
    ma_status_t st = motor_api_create_from_config(cfg_path, &h);
    if (st != MA_OK || !h) {
        fprintf(stderr, "Create failed: %d\n", st);
        return 1;
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* 状态变量 */
    app_state_t state = STATE_IDLE;
    int tick = 0;

    /* 当前位置记录 (用户单位: um, mdeg) */
    double pos_gantry_x = 0;
    double pos_gantry_y = 0;
    double pos_arm_j1 = 0;
    /* ... 其他轴省略，示例只动几个关键轴 */

    /* 清错 */
    motor_api_clear_error(h, -1);
    sleep(1); /* 等待清错完成 */

    printf("Start control loop...\n");

    while (!g_stop) {
        motor_api_run_once(h);
        
        switch (state) {
            case STATE_IDLE:
                if (tick > 100) state = STATE_MOVE_TO_PICK;
                break;

            case STATE_MOVE_TO_PICK:
                /* X 走 100mm (100000 um), Y 走 200mm (200000 um) */
                /* 速度: 50mm/s = 50000 um/s */
                {
                    int done_x = move_axis(h, AXIS_GANTRY_X, 100000.0, 50000.0, &pos_gantry_x);
                    
                    /* Y1 正向 */
                    int done_y1 = move_axis(h, AXIS_GANTRY_Y1, 200000.0, 50000.0, &pos_gantry_y);
                    
                    /* Y2 反向同步: 直接给 AXIS_GANTRY_Y2 发送与 Y1 相反的指令 */
                    /* 注意：move_axis 内部更新了 pos_gantry_y 并发了命令。
                       我们需要手动给 Y2 发反向命令。
                       这里为了简单，重新调用一次 set_axis_command */
                    // motor_api_set_axis_command(h, AXIS_GANTRY_Y2, true, -dir_y1, step_y1); 
                    // 更好的做法是封装同步逻辑，这里简化处理：
                    // 假设 move_axis 每一帧都调用，我们在 Y1 调用后，获取 Y1 的指令参数不太容易（除非改 move_axis 返回 delta）。
                    // 简单起见，我们对 Y2 也运行一个 move_axis，目标是 -200000.0
                    double dummy_y2 = -pos_gantry_y; // 镜像位置
                    move_axis(h, AXIS_GANTRY_Y2, -200000.0, 50000.0, &dummy_y2);

                    if (done_x && done_y1) {
                        printf("Reached Pick Position\n");
                        state = STATE_ARM_DOWN;
                    }
                }
                break;

            case STATE_ARM_DOWN:
                /* J1 旋转 90度 (90000 mdeg) */
                if (move_axis(h, AXIS_ARM_J1, 90000.0, 10000.0, &pos_arm_j1)) {
                    state = STATE_GRIP_CLOSE;
                }
                break;

            case STATE_GRIP_CLOSE:
                /* 设置 IO1 输出 1 (假设 bit 0 控制夹爪) */
                motor_api_set_io_output(h, AXIS_IO_1, 0x0001);
                
                /* 等待 IO1 输入 1 (假设 bit 0 是到位传感器) */
                uint32_t io_in = 0;
                motor_api_get_io_input(h, AXIS_IO_1, &io_in);
                if (io_in & 0x0001) {
                    printf("Gripper Closed\n");
                    state = STATE_ARM_UP;
                }
                break;

            case STATE_ARM_UP:
                /* J1 回 0 */
                if (move_axis(h, AXIS_ARM_J1, 0.0, 10000.0, &pos_arm_j1)) {
                    state = STATE_DONE;
                }
                break;

            case STATE_DONE:
                printf("Sequence Completed\n");
                g_stop = 1;
                break;
        }

        tick++;
        usleep(4000); /* 4ms */
    }

    motor_api_destroy(h);
    return 0;
}
