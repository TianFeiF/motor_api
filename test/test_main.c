/*
 * 文件名称: test_main.c
 * 文件说明: 硬件功能测试主程序入口
 */

#include "test_config.h"

/* 外部测试函数声明 */
extern int test_eni_read(void);
extern int test_init_hardware(struct motor_api_handle **handle);
extern int test_http_interface(struct motor_api_handle *handle);
extern int test_motion_control(struct motor_api_handle *handle);
extern int test_error_handling(void);
extern void cleanup_test(struct motor_api_handle *handle);

/* 全局统计变量 */
int g_test_fail_count = 0;
int g_test_pass_count = 0;

void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("Options:\n");
    printf("  --all       Run all tests (default)\n");
    printf("  --offline   Run only offline tests (ENI read, Error handling)\n");
    printf("  --help      Show this help message\n");
}

int main(int argc, char **argv) {
    int run_hardware = 1; // 默认运行硬件测试
    
    // 参数解析
    if (argc > 1) {
        if (strcmp(argv[1], "--offline") == 0) {
            run_hardware = 0;
            LOG_WARN("Running in OFFLINE mode (Hardware tests skipped)");
        } else if (strcmp(argv[1], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    LOG_INFO("========================================");
    LOG_INFO("    Motor API Hardware Test Suite       ");
    LOG_INFO("========================================");

    // 1. 离线测试部分
    if (test_eni_read() == 0) g_test_pass_count++; else g_test_fail_count++;
    if (test_error_handling() == 0) g_test_pass_count++; else g_test_fail_count++;

    // 2. 在线硬件测试部分
    if (run_hardware) {
        struct motor_api_handle *handle = NULL;
        
        // 必须先初始化硬件才能进行后续测试
        if (test_init_hardware(&handle) == 0) {
            g_test_pass_count++;
            
            // 只有初始化成功才运行后续硬件相关测试
            if (test_http_interface(handle) == 0) g_test_pass_count++; else g_test_fail_count++;
            if (test_motion_control(handle) == 0) g_test_pass_count++; else g_test_fail_count++;
            
            // 清理
            cleanup_test(handle);
        } else {
            g_test_fail_count++;
            LOG_FAIL("Skipping remaining hardware tests due to init failure");
        }
    }

    // 3. 测试报告
    LOG_INFO("========================================");
    LOG_INFO("Test Summary:");
    printf("  Passed: %d\n", g_test_pass_count);
    printf("  Failed: %d\n", g_test_fail_count);
    LOG_INFO("========================================");

    return (g_test_fail_count == 0) ? 0 : 1;
}
