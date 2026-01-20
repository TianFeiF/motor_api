/*
 * test_manual_control.c
 *
 * 功能：
 * - 加载 test_manual_config.json 配置（针对从站 4/5/6 的双轴控制）。
 * - 提供简单的键盘交互界面，选择轴并进行点动（Jog）控制。
 * - 实时显示各轴状态（位置、状态字）。
 *
 * 操作说明：
 * - 数字键 1-6：选择当前控制的轴（Axis 0-5）。
 * - 方向键 'w' / 's'：增加 / 减小 目标位置（Jog）。
 * - 空格键 ' '：停止当前轴运动。
 * - 'c'：清除所有轴错误。
 * - 'q'：退出程序。
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/time.h>
#include <stdbool.h>

#include "motor_api.h"

static volatile sig_atomic_t g_stop = 0;

static void sig_handler(int sig) {
    (void)sig;
    g_stop = 1;
}

/* 非阻塞键盘读取辅助函数 */
static int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

static int getch(void) {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char **argv) {
    const char *cfg_path = "../test/test_manual_config.json";
    //const char *cfg_path = "../config/config.json";
    if (argc > 1) cfg_path = argv[1];

    printf("Loading config: %s\n", cfg_path);

    struct motor_api_handle *handle = NULL;
    ma_status_t st = motor_api_create_from_config(cfg_path, &handle);
    if (st != MA_OK || !handle) {
        fprintf(stderr, "Failed to create motor_api handle: %d\n", st);
        return 1;
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* 获取内部句柄以访问 axis_count 等 */
    /* 注意：在正式应用中应通过 API 获取，这里为了测试方便强转 */
    /* 假设 motor_api_handle_t 定义在 motor_api_internal.h，但该头文件不公开 */
    /* 我们可以包含 motor_api_internal.h 进行编译，或者只用公开 API */
    /* 为了演示标准用法，我们尽量用公开 API，但 axis_count 目前没有 getter */
    /* 暂时假定我们知道有 6 个轴 */
    int total_axes = 6; 
    
    int current_axis = 0;
    int jog_speed = 10; /* 度/次 loop? 还是脉冲? */
    /* 配置文件 unit_per_rev=360.0，所以 step=1 是 1度 */
    /* 我们设 jog_speed = 1 (1度/周期)，周期 4ms -> 250度/秒，有点快 */
    /* 设 jog_speed = 0 (停止) */
    
    /* 记录每个轴的运动状态 */
    int axis_dir[6] = {0}; /* 0: stop, 1: positive, -1: negative */

    /* 预热/清错 */
    printf("Initializing... (Clear Error)\n");
    motor_api_clear_error(handle, -1);
    sleep(1);

    printf("Start Manual Control Loop. Press 'q' to quit.\n");
    printf("Select Axis: 1-3\n");
    printf("Control: 'w' (Forward), 's' (Backward), ' ' (Stop)\n");

    /* 诊断 JSON 缓冲区 */
    char diag_buf[4096];

    while (!g_stop) {
        /* 处理输入 */
        if (kbhit()) {
            int c = getch();
            if (c == 'q') g_stop = 1;
            else if (c >= '1' && c <= '6') {
                current_axis = c - '1';
                printf("\nSelected Axis: %d\n", current_axis);
            }
            else if (c == 'w') {
                axis_dir[current_axis] = 1;
                printf("Axis %d: Moving Forward\n", current_axis);
            }
            else if (c == 's') {
                axis_dir[current_axis] = -1;
                printf("Axis %d: Moving Backward\n", current_axis);
            }
            else if (c == ' ') {
                axis_dir[current_axis] = 0;
                printf("Axis %d: Stopped\n", current_axis);
            }
            else if (c == 'c') {
                printf("Clearing Errors...\n");
                motor_api_clear_error(handle, -1);
            }
        }

        /* 执行控制 */
        for (int i = 0; i < total_axes; ++i) {
            bool run = (axis_dir[i] != 0);
            int dir = axis_dir[i];
            
            /* 目标速度：10 度/秒
             * 控制周期：4000us (4ms) -> 250 Hz
             * 新配置 unit_per_rev = 360000.0 (1 unit = 0.001 deg)
             * 
             * 10 deg/s = 10000 mdeg/s
             * 每周期步长 = 10000 / 250 = 40 mdeg (units)
             * 
             * 现在每周期直接下发 40 单位即可，无需分频，运动更平滑。
             */
            
            if (run) {
                motor_api_set_axis_command(handle, i, true, dir, 40);
            } else {
                motor_api_set_axis_command(handle, i, false, 0, 0);
            }
        }

        motor_api_run_once(handle);

        /* 打印状态 (每 100ms 一次) */
        static int print_tick = 0;
        if (++print_tick >= 25) {
            print_tick = 0;
            motor_api_format_diag_json(handle, diag_buf, sizeof(diag_buf));
            /* 简单解析 JSON 或只打印部分信息有点麻烦，直接打印 JSON 字符串 */
            /* 为了避免刷屏太快，覆盖打印 */
            //printf("\rStatus: %s", diag_buf);
            fflush(stdout);
        }

        usleep(4000);
    }

    printf("\nExiting...\n");
    motor_api_destroy(handle);
    return 0;
}
