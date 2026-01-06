/*
 * test_angle_loop.c
 *
 * 用途：
 * - 依据 ../config/config.json 创建 motor_api；
 * - 以“角度（度）”作为上层控制单位驱动电机；
 * - 从当前位置开始，按平均 90 度/秒的速度累积增量；
 * - 每累计到一个 segment_deg（默认 90 度）就反向，形成正反转往返。
 *
 * 为什么这里可以用“度”来驱动：
 * - config.json 为每个 axis 配置了：
 *     encoder_res / gear_ratio / unit_per_rev
 * - motor_api 内部会计算 scale = encoder_res * gear_ratio / unit_per_rev
 * - 每周期写目标位置时（CSP）使用：
 *     delta_pulse = dir * step * scale
 * - 因此当 unit_per_rev=360.0 时，把 step 当作“度”，就能换算到脉冲增量。
 *
 * 运行（在 build 目录）：
 *   ./test_angle_loop ../config/config.json
 *
 * 常改参数：
 * - axis_idx：控制哪个逻辑轴（0-based，需与 config.json 的 axis_id 对应）
 * - deg_per_sec：角速度（度/秒）
 * - segment_deg：每次往返的角度段长度（走满就反向）
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <stdbool.h>

#include "motor_api.h"
#include "motor_api_internal.h"

/* Ctrl+C 后置 1，用于干净退出主循环 */
static volatile sig_atomic_t g_stop = 0;

static void sig_handler(int sig) {
    (void)sig;
    g_stop = 1;
}

int main(int argc, char **argv) {
    /* 默认配置文件路径（从 build 目录运行时有效）；可用 argv[1] 覆盖 */
    const char *cfg_path = "../config/config.json";
    if (argc > 1) cfg_path = argv[1];

    /* 根据 JSON 配置创建句柄：会读取 ENI、建立轴映射、并应用单位换算参数 */
    struct motor_api_handle *handle = NULL;
    ma_status_t st = motor_api_create_from_config(cfg_path, &handle);
    if (st != MA_OK || !handle) {
        fprintf(stderr, "motor_api_create_from_config failed: %d\n", st);
        return 1;
    }

    /* 注册退出信号 */
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* 取内部句柄，主要为了拿到 cycle_us（控制周期） */
    motor_api_handle_t *h = (motor_api_handle_t *)handle;

    /* 需要控制的轴数量：这里示例为 3 轴（axis_id=0/1/2） */
    uint32_t axes = 3;
    if (h->axis_count < axes) {
        axes = h->axis_count;
    }
    if (axes == 0) {
        fprintf(stderr, "no axes available (axis_count=0)\n");
        motor_api_destroy(handle);
        return 1;
    }

    /* 角速度（度/秒）：三轴默认一致；如需不同轴不同速度，可分别改数组 */
    double deg_per_sec[3] = {90.0, 180.0, 720.0};

    /* 单段角度（度）：每轴累计到该角度就反向 */
    double segment_deg[3] = {90.0, 180.0, 720.0};

    /* 控制周期（微秒），用于把“度/秒”换算成“每周期应增加多少度” */
    double cycle_us = (double)h->cycle_us;

    /* 每轴的浮点累积器：把每周期的小数度数累积起来，凑够整数再下发 step */
    double accum_deg[3] = {0.0, 0.0, 0.0};

    /* 每轴当前段已经累计走过的角度，用于判断是否到达 segment_deg 需要反向 */
    double seg_deg[3] = {0.0, 0.0, 0.0};

    /* 每轴方向：1 正向，-1 反向 */
    int dir[3] = {1, 1, 1};

    //motor_api_clear_error(handle, 1);

    while (!g_stop) {
        for (uint32_t axis = 0; axis < axes; ++axis) {
            /* 每周期需要走的角度（可能为小数） */
            double step_deg = deg_per_sec[axis] * (cycle_us / 1000000.0);

            /* 把小数累积起来 */
            accum_deg[axis] += step_deg;

            /* 只下发整数“度”步进，避免把浮点直接传入 step */
            int step_int = (int)accum_deg[axis];
            if (step_int > 0) {
                /* 扣掉已消费的整数部分，小数继续留在 accum_deg */
                accum_deg[axis] -= (double)step_int;

                /* 这一段剩余角度不足时，裁剪 step，保证刚好停在段终点 */
                if (seg_deg[axis] + step_int > segment_deg[axis]) {
                    step_int = (int)(segment_deg[axis] - seg_deg[axis]);
                }
            }

            /* 当 step_int==0 时，本周期不推进目标（run=false）；否则按 dir 方向推进 */
            bool run = step_int > 0;

            /* set_axis_command 内部会把 step 限制到 >=1；这里也给个兜底值 */
            int cmd_step = step_int > 0 ? step_int : 1;
            if (run) {
                seg_deg[axis] += step_int;
            }

            /* 下发单轴命令：
             * - run=true 才会在周期控制中应用 dir/step；
             * - step 的单位是“度”（由 config.json 的 unit_per_rev=360.0 确立）。
             */
            ma_status_t rc = motor_api_set_axis_command(handle, (int)axis, run, dir[axis], cmd_step);
            if (rc != MA_OK) {
                fprintf(stderr, "motor_api_set_axis_command(axis=%u) failed: %d\n", axis, rc);
                g_stop = 1;
                break;
            }

            /* 段走完：该轴反向，并重置段计数 */
            if (seg_deg[axis] >= segment_deg[axis]) {
                dir[axis] = -dir[axis];
                seg_deg[axis] = 0.0;
            }
        }
        if (g_stop) break;

        /* 执行一个控制周期（收包/处理/写入/发包 + 状态机推进 + 目标更新） */
        motor_api_run_once(handle);

        /* 简单的周期睡眠；需要更精确可改用定时器/实时线程 */
        usleep(h->cycle_us);
    }

    /* 释放资源 */
    motor_api_destroy(handle);
    return 0;
}
