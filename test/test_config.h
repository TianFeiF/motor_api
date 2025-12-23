/*
 * 文件名称: test_config.h
 * 文件说明: 硬件功能测试套件配置文件
 * 版本信息: v1.0.0
 */

#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include "motor_api.h"

/* 硬件参数配置 (基于 HCFAX3E.xml) */
#define TEST_ENI_PATH       "doc/HCFAX3E.xml"
#define TEST_CYCLE_US       4000
#define TEST_SLAVE_COUNT_EXPECTED 1 // 假设至少有一个从站
#define TEST_VENDOR_ID      0x00000025 // Slave 0 Vendor ID based on test_read output
#define TEST_PRODUCT_CODE   0x00000530 // Slave 0 Product Code based on test_read output

/* 测试控制参数 */
#define TEST_HTTP_PORT      8088
#define TEST_MOTION_STEPS   500
#define TEST_MOTION_TARGET  100000
#define TEST_DURATION_SEC   5

/* 颜色输出宏 */
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define LOG_INFO(fmt, ...)  printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#define LOG_PASS(fmt, ...)  printf(ANSI_COLOR_GREEN "[PASS] " fmt ANSI_COLOR_RESET "\n", ##__VA_ARGS__)
#define LOG_FAIL(fmt, ...)  printf(ANSI_COLOR_RED "[FAIL] " fmt ANSI_COLOR_RESET "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  printf(ANSI_COLOR_YELLOW "[WARN] " fmt ANSI_COLOR_RESET "\n", ##__VA_ARGS__)

/* 断言宏 */
#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            LOG_FAIL("Assertion failed: %s (Line %d)", msg, __LINE__); \
            return -1; \
        } \
    } while(0)

/* 全局变量声明 */
extern int g_test_fail_count;
extern int g_test_pass_count;

#endif /* TEST_CONFIG_H */
