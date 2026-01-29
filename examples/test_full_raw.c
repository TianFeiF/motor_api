/*
 * test_full_raw.c
 * 
 * 基于 ethercat cstruct 输出生成的全网络测试代码。
 * 包含 8 个从站：
 * - Slave 0: INEXBOT-IO-R4 (IO)
 * - Slave 1-3: HCFA X3E (Servo)
 * - Slave 4-6: Hans Robot (Dual Axis Servo)
 * - Slave 7: F2838x (Custom IO/Modbus)
 *
 * 功能：
 * 1. 配置所有从站的 PDO
 * 2. 激活主站
 * 3. 循环中：
 *    - Slave 0: IO 跑马灯 (Output 0x7000)
 *    - Slave 1-6: 打印 StatusWord (0x6041) 和 ActualPosition (0x6064)
 *    - Slave 7: 打印 AnalogInput (0x6002)
 *
 * 编译:
 * gcc -o test_full_raw test_full_raw.c -I/usr/local/include -lethercat -lpthread -lrt
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

#define CYCLE_US 4000

// --- Slave 0: INEXBOT-IO-R4 ---
#define BusPos0 0
#define VendorID0 0x00000025
#define ProductCode0 0x00000530

static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x7000, 0x01, 32}, {0x7000, 0x02, 32}, {0x7000, 0x03, 16}, {0x7000, 0x04, 16},
    {0x7000, 0x05, 32}, {0x7000, 0x06, 16}, {0x7000, 0x07, 16}, {0x7000, 0x08, 16},
    {0x7000, 0x09, 32}, 
    {0x6000, 0x00, 32}, {0x6001, 0x00, 32}, {0x6002, 0x00, 16}, {0x6003, 0x00, 16},
    {0x6004, 0x00, 32}, {0x6005, 0x00, 16}, {0x6006, 0x00, 16}, {0x6007, 0x00, 16},
    {0x6008, 0x00, 32}, {0x6009, 0x00, 32}, {0x600a, 0x00, 32}, {0x600b, 0x00, 32},
};
static ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 9, slave_0_pdo_entries + 0},
    {0x1a00, 12, slave_0_pdo_entries + 9},
};
static ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// --- Slave 1-3: HCFA X3E ---
#define VendorID_HCFA 0x000116c7
#define ProductCode_HCFA 0x003e0402

static ec_pdo_entry_info_t slave_hcfa_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x6060, 0x00, 8}, {0x607a, 0x00, 32}, {0x60b8, 0x00, 16},
    {0x603f, 0x00, 16}, {0x6041, 0x00, 16}, {0x6064, 0x00, 32}, {0x6061, 0x00, 8},
    {0x60b9, 0x00, 16}, {0x60ba, 0x00, 32}, {0x60f4, 0x00, 32}, {0x60fd, 0x00, 32},
    {0x213f, 0x00, 16},
};
static ec_pdo_info_t slave_hcfa_pdos[] = {
    {0x1600, 4, slave_hcfa_pdo_entries + 0},
    {0x1a00, 9, slave_hcfa_pdo_entries + 4},
};
static ec_sync_info_t slave_hcfa_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_hcfa_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_hcfa_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// --- Slave 4-6: Hans Robot (Dual Axis) ---
#define VendorID_Hans 0x0000001a
#define ProductCode_Hans 0x50440200

static ec_pdo_entry_info_t slave_hans_pdo_entries[] = {
    // RxPDO (Output)
    {0x6040, 0x00, 16}, {0x6060, 0x00, 8}, {0x0000, 0x00, 8}, {0x607a, 0x00, 32},
    {0x6071, 0x00, 16}, {0x3097, 0x00, 16}, {0x0000, 0x00, 32}, {0x0000, 0x00, 32},
    {0x0000, 0x00, 32}, {0x0000, 0x00, 32}, {0x0000, 0x00, 32},
    {0x6840, 0x00, 16}, {0x6860, 0x00, 8}, {0x0000, 0x00, 8}, {0x687a, 0x00, 32},
    {0x6871, 0x00, 16}, {0x3897, 0x00, 16}, {0x0000, 0x00, 32}, {0x0000, 0x00, 32},
    {0x0000, 0x00, 32}, {0x0000, 0x00, 32}, {0x0000, 0x00, 32},
    // TxPDO (Input)
    {0x6041, 0x00, 16}, {0x6077, 0x00, 16}, {0x6064, 0x00, 32}, {0x606c, 0x00, 32},
    {0x603f, 0x00, 16}, {0x6061, 0x00, 8}, {0x0000, 0x00, 8}, {0x3154, 0x00, 16},
    {0x2001, 0x00, 16}, {0x6164, 0x00, 32}, {0x0000, 0x00, 32}, {0x0000, 0x00, 32},
    {0x6841, 0x00, 16}, {0x6877, 0x00, 16}, {0x6864, 0x00, 32}, {0x686c, 0x00, 32},
    {0x683f, 0x00, 16}, {0x6861, 0x00, 8}, {0x0000, 0x00, 8}, {0x3954, 0x00, 16},
    {0x2801, 0x00, 16}, {0x6964, 0x00, 32}, {0x0000, 0x00, 32}, {0x0000, 0x00, 32},
};
static ec_pdo_info_t slave_hans_pdos[] = {
    {0x1600, 11, slave_hans_pdo_entries + 0},
    {0x1610, 11, slave_hans_pdo_entries + 11},
    {0x1a00, 12, slave_hans_pdo_entries + 22},
    {0x1a10, 12, slave_hans_pdo_entries + 34},
};
static ec_sync_info_t slave_hans_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_hans_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 2, slave_hans_pdos + 2, EC_WD_DISABLE},
    {0xff}
};

// --- Slave 7: F2838x ---
#define VendorID_F28 0x00201911
#define ProductCode_F28 0x10003201

static ec_pdo_entry_info_t slave_f28_pdo_entries[] = {
    // Output (17 entries)
    {0x7001, 0x01, 16}, {0x7002, 0x01, 16}, {0x7003, 0x01, 16}, {0x7004, 0x01, 16},
    {0x7005, 0x01, 16}, {0x7006, 0x01, 16}, {0x7007, 0x01, 16}, {0x7008, 0x01, 16},
    {0x7009, 0x01, 16}, {0x700a, 0x01, 16}, {0x700b, 0x01, 16}, {0x700c, 0x01, 16},
    {0x700d, 0x01, 16}, {0x700e, 0x01, 16}, {0x700f, 0x01, 16}, {0x7010, 0x01, 16},
    {0x7011, 0x01, 16},
    // Input (20 entries)
    {0x6001, 0x01, 16}, {0x6002, 0x01, 16}, {0x6003, 0x01, 16}, {0x6004, 0x01, 16},
    {0x6005, 0x01, 16}, {0x6006, 0x01, 16}, {0x6007, 0x01, 16}, {0x6008, 0x01, 16},
    {0x6009, 0x01, 16}, {0x600a, 0x01, 16}, {0x600b, 0x01, 16}, {0x600c, 0x01, 16},
    {0x600d, 0x01, 16}, {0x600e, 0x01, 16}, {0x600f, 0x01, 16}, {0x6010, 0x01, 16},
    {0x6011, 0x01, 16}, {0x6012, 0x01, 16}, {0x6013, 0x01, 16}, {0x6014, 0x01, 16},
};
static ec_pdo_info_t slave_f28_pdos[] = {
    {0x1600, 17, slave_f28_pdo_entries + 0},
    {0x1a00, 20, slave_f28_pdo_entries + 17},
};
static ec_sync_info_t slave_f28_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_f28_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_f28_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// --- Offsets ---
// Slave 0
static unsigned int off_s0_out[9];

// Slave 1-3 (Servo)
static unsigned int off_s1_stat, off_s1_pos, off_s1_ctrl;
static unsigned int off_s2_stat, off_s2_pos, off_s2_ctrl;
static unsigned int off_s3_stat, off_s3_pos, off_s3_ctrl;

// Slave 4-6 (Hans) - Axis 1 & 2
static unsigned int off_s4_stat1, off_s4_pos1, off_s4_stat2, off_s4_pos2;
static unsigned int off_s5_stat1, off_s5_pos1, off_s5_stat2, off_s5_pos2;
static unsigned int off_s6_stat1, off_s6_pos1, off_s6_stat2, off_s6_pos2;

// Slave 7 (F28)
static unsigned int off_s7_in_ana1; // 0x6002:01

const static ec_pdo_entry_reg_t domain_regs[] = {
    // Slave 0
    {0, 0, VendorID0, ProductCode0, 0x7000, 1, &off_s0_out[0]},
    {0, 0, VendorID0, ProductCode0, 0x7000, 2, &off_s0_out[1]},
    {0, 0, VendorID0, ProductCode0, 0x7000, 3, &off_s0_out[2]},
    {0, 0, VendorID0, ProductCode0, 0x7000, 4, &off_s0_out[3]},
    {0, 0, VendorID0, ProductCode0, 0x7000, 5, &off_s0_out[4]},
    {0, 0, VendorID0, ProductCode0, 0x7000, 6, &off_s0_out[5]},
    {0, 0, VendorID0, ProductCode0, 0x7000, 7, &off_s0_out[6]},
    {0, 0, VendorID0, ProductCode0, 0x7000, 8, &off_s0_out[7]},
    {0, 0, VendorID0, ProductCode0, 0x7000, 9, &off_s0_out[8]},

    // Slave 1
    {0, 1, VendorID_HCFA, ProductCode_HCFA, 0x6041, 0, &off_s1_stat},
    {0, 1, VendorID_HCFA, ProductCode_HCFA, 0x6064, 0, &off_s1_pos},
    {0, 1, VendorID_HCFA, ProductCode_HCFA, 0x6040, 0, &off_s1_ctrl},

    // Slave 2
    {0, 2, VendorID_HCFA, ProductCode_HCFA, 0x6041, 0, &off_s2_stat},
    {0, 2, VendorID_HCFA, ProductCode_HCFA, 0x6064, 0, &off_s2_pos},
    {0, 2, VendorID_HCFA, ProductCode_HCFA, 0x6040, 0, &off_s2_ctrl},

    // Slave 3
    {0, 3, VendorID_HCFA, ProductCode_HCFA, 0x6041, 0, &off_s3_stat},
    {0, 3, VendorID_HCFA, ProductCode_HCFA, 0x6064, 0, &off_s3_pos},
    {0, 3, VendorID_HCFA, ProductCode_HCFA, 0x6040, 0, &off_s3_ctrl},

    // Slave 4 (Hans)
    {0, 4, VendorID_Hans, ProductCode_Hans, 0x6041, 0, &off_s4_stat1},
    {0, 4, VendorID_Hans, ProductCode_Hans, 0x6064, 0, &off_s4_pos1},
    {0, 4, VendorID_Hans, ProductCode_Hans, 0x6841, 0, &off_s4_stat2},
    {0, 4, VendorID_Hans, ProductCode_Hans, 0x6864, 0, &off_s4_pos2},

    // Slave 5 (Hans)
    {0, 5, VendorID_Hans, ProductCode_Hans, 0x6041, 0, &off_s5_stat1},
    {0, 5, VendorID_Hans, ProductCode_Hans, 0x6064, 0, &off_s5_pos1},
    {0, 5, VendorID_Hans, ProductCode_Hans, 0x6841, 0, &off_s5_stat2},
    {0, 5, VendorID_Hans, ProductCode_Hans, 0x6864, 0, &off_s5_pos2},

    // Slave 6 (Hans)
    {0, 6, VendorID_Hans, ProductCode_Hans, 0x6041, 0, &off_s6_stat1},
    {0, 6, VendorID_Hans, ProductCode_Hans, 0x6064, 0, &off_s6_pos1},
    {0, 6, VendorID_Hans, ProductCode_Hans, 0x6841, 0, &off_s6_stat2},
    {0, 6, VendorID_Hans, ProductCode_Hans, 0x6864, 0, &off_s6_pos2},

    // Slave 7 (F28)
    {0, 7, VendorID_F28, ProductCode_F28, 0x6002, 1, &off_s7_in_ana1},

    {}
};

static volatile int run = 1;
void signal_handler(int sig) { run = 0; }

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
    ec_domain_t *domain = NULL;
    ec_slave_config_t *sc[8] = {0};
    uint8_t *pd = NULL;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    master = ecrt_request_master(0);
    if (!master) return -1;
    domain = ecrt_master_create_domain(master);
    if (!domain) return -1;

    // --- Configure Slaves ---
    // Slave 0
    sc[0] = ecrt_master_slave_config(master, 0, 0, VendorID0, ProductCode0);
    ecrt_slave_config_pdos(sc[0], EC_END, slave_0_syncs);

    // Slave 1-3
    for (int i = 1; i <= 3; ++i) {
        sc[i] = ecrt_master_slave_config(master, 0, i, VendorID_HCFA, ProductCode_HCFA);
        ecrt_slave_config_pdos(sc[i], EC_END, slave_hcfa_syncs);
    }

    // Slave 4-6
    for (int i = 4; i <= 6; ++i) {
        sc[i] = ecrt_master_slave_config(master, 0, i, VendorID_Hans, ProductCode_Hans);
        ecrt_slave_config_pdos(sc[i], EC_END, slave_hans_syncs);
    }

    // Slave 7
    sc[7] = ecrt_master_slave_config(master, 0, 7, VendorID_F28, ProductCode_F28);
    ecrt_slave_config_pdos(sc[7], EC_END, slave_f28_syncs);

    // --- Register Domains ---
    if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) {
        fprintf(stderr, "PDO registration failed.\n");
        return -1;
    }

    if (ecrt_master_activate(master)) return -1;
    pd = ecrt_domain_data(domain);
    if (!pd) return -1;

    printf("Master activated. Slaves 0-7 configured.\n");

    struct timespec wakeup_time;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    int counter = 0;
    uint32_t blink_val = 0;

    while (run) {
        sleep_until(&wakeup_time, CYCLE_US);
        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        // Blink Slave 0 (1 Hz)
        if (counter++ % 250 == 0) {
            blink_val = (blink_val == 0) ? 0xFF : 0x00;
        }
        
        // Slave 0 Output
        EC_WRITE_U32(pd + off_s0_out[0], blink_val);
        EC_WRITE_U32(pd + off_s0_out[1], blink_val);
        EC_WRITE_U16(pd + off_s0_out[2], (uint16_t)blink_val);
        EC_WRITE_U16(pd + off_s0_out[3], (uint16_t)blink_val);
        EC_WRITE_U32(pd + off_s0_out[4], blink_val);
        EC_WRITE_U16(pd + off_s0_out[5], (uint16_t)blink_val);
        EC_WRITE_U16(pd + off_s0_out[6], (uint16_t)blink_val);
        EC_WRITE_U16(pd + off_s0_out[7], (uint16_t)blink_val);
        EC_WRITE_U32(pd + off_s0_out[8], blink_val);

        // Print Status (every 1s)
        if (counter % 250 == 0) {
            printf("--- Cycle %d ---\n", counter);
            // Slave 1-3
            for (int i=1; i<=3; ++i) {
                unsigned int off_s = (i==1)?off_s1_stat:(i==2)?off_s2_stat:off_s3_stat;
                unsigned int off_p = (i==1)?off_s1_pos:(i==2)?off_s2_pos:off_s3_pos;
                printf("S%d: Stat=0x%04X Pos=%d\n", i, EC_READ_U16(pd+off_s), EC_READ_S32(pd+off_p));
            }
            // Slave 4-6
            for (int i=4; i<=6; ++i) {
                unsigned int os1 = (i==4)?off_s4_stat1:(i==5)?off_s5_stat1:off_s6_stat1;
                unsigned int op1 = (i==4)?off_s4_pos1:(i==5)?off_s5_pos1:off_s6_pos1;
                unsigned int os2 = (i==4)?off_s4_stat2:(i==5)?off_s5_stat2:off_s6_stat2;
                unsigned int op2 = (i==4)?off_s4_pos2:(i==5)?off_s5_pos2:off_s6_pos2;
                printf("S%d: Ax1(0x%04X, %d) Ax2(0x%04X, %d)\n", i, 
                       EC_READ_U16(pd+os1), EC_READ_S32(pd+op1),
                       EC_READ_U16(pd+os2), EC_READ_S32(pd+op2));
            }
            // Slave 7
            printf("S7: Ana1=%d\n", EC_READ_U16(pd + off_s7_in_ana1));
        }

        ecrt_domain_queue(domain);
        ecrt_master_send(master);
    }

    ecrt_release_master(master);
    return 0;
}
