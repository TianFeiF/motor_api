/*
 * test_io.c
 * 
 * 简易 IO 板测试程序
 * 功能：
 * 1. 初始化 EtherCAT 主站 (使用 test_io.json 配置)
 * 2. 切换到 OP 状态
 * 3. 循环控制 IO 输出 (闪烁)
 * 
 * 编译：
 * gcc -o test_io test_io.c -I../include -L../build -lmotor_api -lpthread -lrt
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>

#include "motor_api.h"

static volatile sig_atomic_t g_stop = 0;

static void sig_handler(int sig) {
    (void)sig;
    g_stop = 1;
}

int main(int argc, char **argv) {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    printf("Initializing Motor API for IO Test...\n");

    struct motor_api_handle *h = NULL;
    
    // 加载配置
    if (motor_api_create_from_config("../config/test_io.json", &h) != MA_OK) {
        fprintf(stderr, "Failed to create motor api handle\n");
        return 1;
    }

    printf("EtherCAT Master Initialized. Running...\n");
    printf("Usage: ./test_io [hex_value]\n");
    printf("  If hex_value is provided (e.g. 0xFF), sets outputs to that value and holds.\n");
    printf("  If no argument, blinks all outputs.\n");

    // 循环控制
    // IO 板被映射为 axis_id 0 (见 test_io.json)
    
    int counter = 0;
    uint32_t output_val = 0;
    int axis_id = 0;
    
    bool manual_mode = false;
    if (argc > 1) {
        manual_mode = true;
        output_val = (uint32_t)strtoul(argv[1], NULL, 0); // 自动识别 0x 或十进制
        printf("Manual Mode: Holding output at 0x%08X\n", output_val);
    }

    while (!g_stop) {
        if (!manual_mode) {
            // 简单的闪烁逻辑: 每 250 个周期 (1s @ 4ms) 切换一次
            if (counter % 2500 == 0) {
                // 翻转逻辑：强制在 0x00 和 0xFF 之间切换
                if (output_val != 0) {
                    output_val = 0xff00;
                } else {
                    output_val = 0xFF;
                }
                
                printf("Blinking: Setting Output to: 0x%02X\n", output_val);
                
                // 同时控制所有映射的 IO 轴 (axis_id 0-8)
                for (int i = 0; i < 9; i++) {
                     // 增加调试信息，确保 API 被调用
                     // printf("  -> Axis %d val=0x%08X\n", i, output_val);
                     motor_api_set_io_output(h, i, output_val);
                }
            }
            // 关键修正：必须在每一帧都调用 set_io_output 保持状态！
            // 之前的逻辑只在 counter%250==0 的那一帧调用了一次 set。
            // 虽然理论上 PDO 映射的数据指针内容应该保持不变，但如果 motor_api 内部
            // 在每一帧开始时清空了 buffer 或者有其他逻辑覆盖，那么只写一次是不够的。
            // 对于实时控制，最佳实践是在每个周期都刷新输出值。
            for (int i = 0; i < 9; i++) {
                 motor_api_set_io_output(h, i, output_val);
            }
        } else {
             // 手动模式：持续发送设定值
             for (int i = 0; i < 9; i++) {
                 motor_api_set_io_output(h, i, output_val);
             }
        }

         // 必须调用 run_once 以发送/接收 EtherCAT 数据
          ma_status_t rc = motor_api_run_once(h);
          
          counter++;
          
          // 打印 PDO 信息用于验证
          if (counter % 250 == 0) { // 每秒打印一次
              if (rc != MA_OK) {
                  printf("[WARN] run_once returned %d\n", rc);
              }
              
              uint32_t input_val = 0;
             // 读取输入状态
             if (motor_api_get_io_input(h, axis_id, &input_val) == MA_OK) {
                 printf("[PDO Monitor] DO (Sent): 0x%08X | DI (Read): 0x%08X\n", output_val, input_val);
             } else {
                 printf("[PDO Monitor] DO (Sent): 0x%08X | DI (Read): Failed\n", output_val);
             }
         }
 
         usleep(4000); // 模拟 4ms 周期
    }

    printf("Stopping...\n");
    motor_api_destroy(h);
    return 0;
}
