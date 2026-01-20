/*
 * test_arm_control.c
 *
 * 功能：
 * - 加载 config/config_dual.json 配置（针对 3个双轴从站 + 1个IO）。
 * - 提供键盘控制界面，可选择轴并控制其运动。
 *
 * 操作说明：
 * - 数字键 1-6：选择当前控制的轴（Axis 0-5）。
 * - 方向键 'w' / 's'：正向 / 反向 运动（Jog）。
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
    const char *cfg_path = "../config/config_dual.json";
    if (argc > 1) cfg_path = argv[1];

    printf("Loading config: %s\n", cfg_path);

    struct motor_api_handle *handle = NULL;
    ma_status_t st = motor_api_create_from_config(cfg_path, &handle);
    if (st != MA_OK || !handle) {
        // Try fallback to relative path if running from build
        const char *fallback_path = "../config/config_dual.json";
        printf("Failed to load %s, trying %s...\n", cfg_path, fallback_path);
        st = motor_api_create_from_config(fallback_path, &handle);
        if (st != MA_OK || !handle) {
            fprintf(stderr, "Failed to create motor_api handle: %d\n", st);
            return 1;
        }
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    int total_axes = 6; 
    
    int current_axis = 0;
    
    /* 记录每个轴的运动状态 */
    int axis_dir[6] = {0}; /* 0: stop, 1: positive, -1: negative */

    /* 预热/清错 */
    printf("Initializing... (Clear Error)\n");
    motor_api_clear_error(handle, -1);
    sleep(1);

    printf("Start Arm Control Loop. Press 'q' to quit.\n");
    printf("Select Axis: 1-6\n");
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
            
            /* 目标速度：10 度/秒 */
            double velocity = 10.0; 
            
            if (run) {
                /* 注意：motor_api_set_axis_command 最后一个参数是 velocity (units/s) */
                motor_api_set_axis_command(handle, i, true, dir, velocity);
            } else {
                motor_api_set_axis_command(handle, i, false, 0, 0);
            }
        }

        motor_api_run_once(handle);

        /* 打印状态 (每 200ms 一次) */
        static int print_tick = 0;
        if (++print_tick >= 50) {
            print_tick = 0;
            motor_api_format_diag_json(handle, diag_buf, sizeof(diag_buf));
            // printf("\r%s", diag_buf); // Optional: print status
            // fflush(stdout);
        }

        usleep(4000);
    }

    printf("\nExiting...\n");
    motor_api_destroy(handle);
    return 0;
}
