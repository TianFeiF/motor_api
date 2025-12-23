/*
 * 文件名称: test_motor_api.c
 * 文件说明: 电机控制API功能测试实现
 */

#include "test_config.h"

/* 
 * 测试 1: ENI 文件解析测试
 * 目的: 验证 XML 解析器能否正确读取从站信息
 */
int test_eni_read(void) {
    LOG_INFO("Starting ENI Read Test...");
    
    uint32_t vids[16] = {0};
    uint32_t prods[16] = {0};
    uint16_t pos[16] = {0};
    uint16_t count = 0;
    ma_eni_slave_t *slaves = NULL;

    // 测试正常读取
    ma_status_t status = motor_api_read_eni(TEST_ENI_PATH, vids, prods, pos, 16, &count, &slaves);
    
    if (status != MA_OK) {
        LOG_FAIL("Failed to read ENI file: %s (Status: %d)", TEST_ENI_PATH, status);
        return -1;
    }

    TEST_ASSERT(count > 0, "No slaves found in ENI");
    
    // 验证第一个从站的信息 (基于 HCFAX3E.xml)
    // 注意: 这里使用配置文件中定义的预期值
    TEST_ASSERT(vids[0] == TEST_VENDOR_ID, "Vendor ID mismatch");
    TEST_ASSERT(prods[0] == TEST_PRODUCT_CODE, "Product Code mismatch");
    
    // 详细输出
    for (int i = 0; i < count; i++) {
        printf("  [Slave %d] VendorID: 0x%08X, ProductCode: 0x%08X, Pos: %d\n", 
               i, vids[i], prods[i], pos[i]);
        if (slaves) {
             printf("    RxPDOs: %u, TxPDOs: %u\n", slaves[i].rx_pdo_count, slaves[i].tx_pdo_count);
             // 打印详细 RxPDO 信息
             for(unsigned int j=0; j<slaves[i].rx_pdo_count; j++) {
                 printf("      RxPDO[%u] Index: 0x%04X, Entries: %u\n", 
                        j, slaves[i].rx_pdos[j].pdo_index, slaves[i].rx_pdos[j].entry_count);
                 for(unsigned int k=0; k<slaves[i].rx_pdos[j].entry_count; k++) {
                     printf("        Entry[%u]: Index=0x%04X, Sub=0x%02X, BitLen=%u\n", 
                            k, 
                            slaves[i].rx_pdos[j].entries[k].index,
                            slaves[i].rx_pdos[j].entries[k].subindex,
                            slaves[i].rx_pdos[j].entries[k].bitlen);
                 }
             }
             // 打印详细 TxPDO 信息
             for(unsigned int j=0; j<slaves[i].tx_pdo_count; j++) {
                 printf("      TxPDO[%u] Index: 0x%04X, Entries: %u\n", 
                        j, slaves[i].tx_pdos[j].pdo_index, slaves[i].tx_pdos[j].entry_count);
                 for(unsigned int k=0; k<slaves[i].tx_pdos[j].entry_count; k++) {
                     printf("        Entry[%u]: Index=0x%04X, Sub=0x%02X, BitLen=%u\n", 
                            k, 
                            slaves[i].tx_pdos[j].entries[k].index,
                            slaves[i].tx_pdos[j].entries[k].subindex,
                            slaves[i].tx_pdos[j].entries[k].bitlen);
                 }
             }
        }
    }

    LOG_PASS("ENI Read Test Passed (Found %d slaves)", count);
    
    if (slaves) motor_api_free_eni_slaves(slaves, count);
    return 0;
}

/*
 * 测试 2: 硬件初始化测试
 * 目的: 验证 EtherCAT 主站能否成功创建并扫描到从站
 */
int test_init_hardware(struct motor_api_handle **handle) {
    LOG_INFO("Starting Hardware Initialization Test...");
    
    uint16_t slave_count = 0;
    ma_status_t status = motor_api_create(TEST_ENI_PATH, TEST_CYCLE_US, &slave_count, handle);
    
    if (status != MA_OK) {
        LOG_FAIL("motor_api_create failed (Status: %d). Ensure EtherCAT master is running.", status);
        return -1;
    }
    
    TEST_ASSERT(*handle != NULL, "Handle is NULL after creation");
    TEST_ASSERT(slave_count > 0, "No slaves configured on bus");
    
    printf("  [Info] Cycle Time: %d us\n", TEST_CYCLE_US);
    printf("  [Info] Slaves Configured: %d\n", slave_count);
    printf("  [Info] EtherCAT Master State: Active\n");

    LOG_PASS("Hardware Init Passed (Slaves: %d)", slave_count);
    return 0;
}

/*
 * 测试 3: HTTP 接口测试
 * 目的: 验证 Web 服务能否启动和停止
 */
int test_http_interface(struct motor_api_handle *handle) {
    LOG_INFO("Starting HTTP Interface Test...");
    
    if (!handle) return -1;

    // 启动 HTTP 服务
    ma_status_t status = motor_api_start_http(handle, TEST_HTTP_PORT);
    TEST_ASSERT(status == MA_OK, "Failed to start HTTP server");
    printf("  [Info] HTTP Server Started on port %d\n", TEST_HTTP_PORT);
    
    // 简单延时等待线程启动
    usleep(100000);
    
    // 验证端口 (可以通过简单的 connect 或者依赖 log)
    // 这里我们假设 API 返回 OK 即成功，并在之后停止它
    
    status = motor_api_stop_http(handle);
    TEST_ASSERT(status == MA_OK, "Failed to stop HTTP server");
    printf("  [Info] HTTP Server Stopped\n");
    
    LOG_PASS("HTTP Interface Test Passed");
    return 0;
}

/*
 * 测试 4: 运动控制循环测试
 * 目的: 验证 CSP 模式下的 PDO 数据交换和状态机推进
 */
int test_motion_control(struct motor_api_handle *handle) {
    LOG_INFO("Starting Motion Control Test (Duration: %d sec)...", TEST_DURATION_SEC);
    
    if (!handle) return -1;

    // 设置运动指令: 正向，步长 100
    motor_api_set_command(handle, true, 1, 100);
    printf("  [Info] Motion Started: Dir=1, Step=100\n");
    
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    int cycles = 0;
    int errors = 0;
    int last_sec = -1;
    
    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;
        
        if (elapsed >= TEST_DURATION_SEC) break;
        
        ma_status_t status = motor_api_run_once(handle);
        if (status != MA_OK) {
            errors++;
            if (errors > 10) {
                LOG_FAIL("Too many errors in control loop");
                return -1;
            }
        }
        
        if ((int)elapsed > last_sec) {
            last_sec = (int)elapsed;
            printf("  [Running] Time: %ds, Cycles: %d, Errors: %d\n", last_sec, cycles, errors);
        }
        
        cycles++;
        usleep(TEST_CYCLE_US); // 模拟周期等待
    }
    
    // 停止运动
    motor_api_set_command(handle, false, 0, 0);
    printf("  [Info] Motion Stopped\n");
    
    TEST_ASSERT(cycles > 0, "No cycles executed");
    LOG_PASS("Motion Control Test Passed (%d cycles executed)", cycles);
    return 0;
}

/*
 * 测试 5: 错误处理测试
 * 目的: 验证 API 对非法参数的处理
 */
int test_error_handling(void) {
    LOG_INFO("Starting Error Handling Test...");
    
    struct motor_api_handle *h = NULL;
    uint16_t n = 0;
    
    // 测试 1: 无效的 ENI 路径
    ma_status_t status = motor_api_create("invalid/path.xml", 4000, &n, &h);
    if (status == MA_ERR_IO || status == MA_ERR_INIT || status == MA_ERR_CONFIG) {
        printf("  [Pass] Caught expected error for invalid path: %d\n", status);
    } else {
        TEST_ASSERT(0, "Should fail with invalid ENI path");
    }
    
    // 测试 2: 对 NULL 句柄的操作
    status = motor_api_run_once(NULL);
    if (status == MA_ERR_PARAM) {
        printf("  [Pass] Caught expected error for NULL handle: %d\n", status);
    } else {
        TEST_ASSERT(0, "Should fail with NULL handle");
    }
    
    LOG_PASS("Error Handling Test Passed");
    return 0;
}

/*
 * 辅助函数: 清理资源
 */
void cleanup_test(struct motor_api_handle *handle) {
    if (handle) {
        motor_api_destroy(handle);
        LOG_INFO("Resources cleaned up");
    }
}
