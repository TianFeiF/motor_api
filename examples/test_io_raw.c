/*
 * test_io_raw.c
 * 
 * 使用 ecrt.h 原生接口直接控制 IO 板 (INEXBOT-IO-R4)
 * 不依赖 motor_api 库，用于排查底层控制问题。
 *
 * 逻辑：
 * 1. 初始化 EtherCAT 主站
 * 2. 配置 Slave 0 (INEXBOT-IO-R4)
 * 3. 注册 PDO Entry (Output: 0x7000:01-09, Input: 0x6000:00)
 * 4. 激活主站并进入循环
 * 5. 每秒翻转一次输出 (0x00 <-> 0xFF)
 *
 * 编译:
 * gcc -o test_io_raw test_io_raw.c -I/usr/local/include -lethercat -lpthread -lrt
 */

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>

#include "ecrt.h"

// --- 配置参数 ---
#define CYCLE_US 4000  // 4ms 周期
#define BusAlias 0
#define BusPos   0
#define VendorID 0x00000025
#define ProductCode 0x00000530

// --- PDO 偏移量变量 ---
static unsigned int off_output_1; // 0x7000:01
static unsigned int off_output_2; // 0x7000:02
static unsigned int off_output_3; // 0x7000:03
static unsigned int off_output_4; // 0x7000:04
static unsigned int off_output_5; // 0x7000:05
static unsigned int off_output_6; // 0x7000:06
static unsigned int off_output_7; // 0x7000:07
static unsigned int off_output_8; // 0x7000:08
static unsigned int off_output_9; // 0x7000:09

static unsigned int off_input_0;  // 0x6000:00 (DI)
static unsigned int off_input_1;  // 0x6001:00 (DI)
static unsigned int off_input_2;  // 0x6002:00 (DI)
static unsigned int off_input_3;  // 0x6003:00 (DI)
static unsigned int off_input_4;  // 0x6004:00 (DI)
static unsigned int off_input_5;  // 0x6005:00 (DI)
static unsigned int off_input_6;  // 0x6006:00 (DI)
static unsigned int off_input_7;  // 0x6007:00 (DI)
static unsigned int off_input_8;  // 0x6008:00 (DI)
static unsigned int off_input_9;  // 0x6009:00 (DI)
static unsigned int off_input_10; // 0x600a:00 (DI)
static unsigned int off_input_11; // 0x600b:00 (DI)

// --- Domain 注册数组 ---
const static ec_pdo_entry_reg_t domain1_regs[] = {
    {BusAlias, BusPos, VendorID, ProductCode, 0x7000, 1, &off_output_1},
    {BusAlias, BusPos, VendorID, ProductCode, 0x7000, 2, &off_output_2},
    {BusAlias, BusPos, VendorID, ProductCode, 0x7000, 3, &off_output_3},
    {BusAlias, BusPos, VendorID, ProductCode, 0x7000, 4, &off_output_4},
    {BusAlias, BusPos, VendorID, ProductCode, 0x7000, 5, &off_output_5},
    {BusAlias, BusPos, VendorID, ProductCode, 0x7000, 6, &off_output_6},
    {BusAlias, BusPos, VendorID, ProductCode, 0x7000, 7, &off_output_7},
    {BusAlias, BusPos, VendorID, ProductCode, 0x7000, 8, &off_output_8},
    {BusAlias, BusPos, VendorID, ProductCode, 0x7000, 9, &off_output_9},
    {BusAlias, BusPos, VendorID, ProductCode, 0x6000, 0, &off_input_0},
    {BusAlias, BusPos, VendorID, ProductCode, 0x6001, 0, &off_input_1},
    {BusAlias, BusPos, VendorID, ProductCode, 0x6002, 0, &off_input_2},
    {BusAlias, BusPos, VendorID, ProductCode, 0x6003, 0, &off_input_3},
    {BusAlias, BusPos, VendorID, ProductCode, 0x6004, 0, &off_input_4},
    {BusAlias, BusPos, VendorID, ProductCode, 0x6005, 0, &off_input_5},
    {BusAlias, BusPos, VendorID, ProductCode, 0x6006, 0, &off_input_6},
    {BusAlias, BusPos, VendorID, ProductCode, 0x6007, 0, &off_input_7},
    {BusAlias, BusPos, VendorID, ProductCode, 0x6008, 0, &off_input_8},
    {BusAlias, BusPos, VendorID, ProductCode, 0x6009, 0, &off_input_9},
    {BusAlias, BusPos, VendorID, ProductCode, 0x600a, 0, &off_input_10},
    {BusAlias, BusPos, VendorID, ProductCode, 0x600b, 0, &off_input_11},
    {},
};

// --- PDO 配置 (基于 io_board.xml) ---
// RxPDO 0x1600 (Output)
static ec_pdo_entry_info_t slave_0_pdo_entries_1600[] = {
    {0x7000, 0x01, 32},
    {0x7000, 0x02, 32},
    {0x7000, 0x03, 16},
    {0x7000, 0x04, 16},
    {0x7000, 0x05, 32},
    {0x7000, 0x06, 16},
    {0x7000, 0x07, 16},
    {0x7000, 0x08, 16},
    {0x7000, 0x09, 32},
};

// TxPDO 0x1A00 (Input)
static ec_pdo_entry_info_t slave_0_pdo_entries_1a00[] = {
    {0x6000, 0x00, 32},
    {0x6001, 0x00, 32},
    {0x6002, 0x00, 16},
    {0x6003, 0x00, 16},
    {0x6004, 0x00, 32},
    {0x6005, 0x00, 16},
    {0x6006, 0x00, 16},
    {0x6007, 0x00, 16},
    {0x6008, 0x00, 32},
    {0x6009, 0x00, 32},
    {0x600a, 0x00, 32},
    {0x600b, 0x00, 32},
};

static ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 9, slave_0_pdo_entries_1600},
    {0x1a00, 12, slave_0_pdo_entries_1a00},
};

static ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

static volatile int run = 1;

void signal_handler(int sig) {
    run = 0;
}

// 简单的周期等待函数 (CLOCK_MONOTONIC)
void sleep_until(struct timespec *ts, long delay_us) {
    ts->tv_nsec += delay_us * 1000;
    while (ts->tv_nsec >= 1000000000) {
        ts->tv_nsec -= 1000000000;
        ts->tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts, NULL);
}

int main(int argc, char **argv) {
    ec_master_t *master = NULL;
    ec_domain_t *domain1 = NULL;
    ec_slave_config_t *sc = NULL;
    uint8_t *domain1_pd = NULL;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("Requesting EtherCAT master...\n");
    master = ecrt_request_master(0);
    if (!master) {
        fprintf(stderr, "Failed to request master.\n");
        return -1;
    }

    printf("Creating domain...\n");
    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        fprintf(stderr, "Failed to create domain.\n");
        return -1;
    }

    printf("Configuring slave 0...\n");
    sc = ecrt_master_slave_config(master, BusAlias, BusPos, VendorID, ProductCode);
    if (!sc) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    printf("Registering PDO entries...\n");
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed.\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "Failed to activate master.\n");
        return -1;
    }

    domain1_pd = ecrt_domain_data(domain1);
    if (!domain1_pd) {
        fprintf(stderr, "Failed to retrieve domain data pointer.\n");
        return -1;
    }

    printf("Started.\n");

    struct timespec wakeup_time;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    printf("wakeup_time: %ld.%09ld\n", wakeup_time.tv_sec, wakeup_time.tv_nsec);

    int counter = 0;
    uint32_t output_val = 0;

    while (run) {
        sleep_until(&wakeup_time, CYCLE_US);

        // 接收数据
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        // 闪烁逻辑 (每 250 个周期 / 1秒 翻转一次)
        if (counter++ % 500 == 0) {
            output_val = (output_val == 0) ? 0xFFFF : 0x0000;
            printf("Blinking: 0x%04X\n", output_val);
        }

        // 写入 Output (全写 0x7000:01-09)
        // 注意：根据 XML, 部分是 U32, 部分是 U16
        EC_WRITE_U32(domain1_pd + off_output_1, 0X32002EE0);
        
        EC_WRITE_U32(domain1_pd + off_output_2, 0X32002EE0);
        EC_WRITE_U16(domain1_pd + off_output_3, 0X0000);
        EC_WRITE_U16(domain1_pd + off_output_4, 0X0000);
        EC_WRITE_U32(domain1_pd + off_output_5, 0X0000);


        EC_WRITE_U16(domain1_pd + off_output_6, output_val);//OUTPUT_1~16
        EC_WRITE_U16(domain1_pd + off_output_7, 0x07ff);//AD_OUTPUT_1
        EC_WRITE_U16(domain1_pd + off_output_8, 0x0fff);//AD_OUTPUT_2
        EC_WRITE_U32(domain1_pd + off_output_9, 0X00000000);
        
        uint32_t input_val_0 = EC_READ_U32(domain1_pd + off_input_0);
        uint32_t input_val_1 = EC_READ_U32(domain1_pd + off_input_1);
        uint32_t input_val_2 = EC_READ_U16(domain1_pd + off_input_2);
        uint32_t input_val_3 = EC_READ_U16(domain1_pd + off_input_3);
        uint32_t input_val_4 = EC_READ_U32(domain1_pd + off_input_4);
        uint32_t input_val_5 = EC_READ_U16(domain1_pd + off_input_5);
        uint32_t input_val_6 = EC_READ_U16(domain1_pd + off_input_6);
        uint32_t input_val_7 = EC_READ_U16(domain1_pd + off_input_7);
        uint32_t input_val_8 = EC_READ_U32(domain1_pd + off_input_8);
        uint32_t input_val_9 = EC_READ_U32(domain1_pd + off_input_9);
        uint32_t input_val_10 = EC_READ_U32(domain1_pd + off_input_10);
        uint32_t input_val_11 = EC_READ_U32(domain1_pd + off_input_11);


        printf("input_val_0: 0x%08X\n", input_val_0);
        printf("input_val_1: 0x%08X\n", input_val_1);
        printf("input_val_2: 0x%04X\n", input_val_2);
        printf("input_val_3: 0x%04X\n", input_val_3);
        printf("input_val_4: 0x%08X\n", input_val_4);
        printf("input_val_5: 0x%04X\n", input_val_5);//INPUT_1~16
        printf("input_val_6: 0x%04X (%.2fV)\n", input_val_6, (float)input_val_6/0x0FFF*10.0);//AD_INPUT_1
        printf("input_val_7: 0x%04X (%.2fV)\n", input_val_7, (float)input_val_7/0x0FFF*10.0);//AD_INPUT_2
        printf("input_val_8: 0x%08X\n", input_val_8);
        printf("input_val_9: 0x%08X\n", input_val_9);
        printf("input_val_10: 0x%08X\n", input_val_10);
        printf("input_val_11: 0x%08X\n\n", input_val_11);
        

        // 发送数据
        //printf("wakeup_time: %ld.%09ld\n", wakeup_time.tv_sec, wakeup_time.tv_nsec);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }

    printf("Releasing master...\n");
    ecrt_release_master(master);
    return 0;
}
