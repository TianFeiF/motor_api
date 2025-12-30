#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "motor_api_internal.h"

/*
 * 示例：读取 ENI -> 初始化设备 -> 周期读取每个从站的 actualPosition。
 *
 * 关键点：
 * - ENI 解析：使用 motor_api_read_eni()，打印从站的 VID/PID/Position。
 * - 设备初始化：使用 motor_api_create()，完成主站/域/PDO/DC 等配置并返回句柄。
 * - actualPosition 读取：来自 PDO 0x6064（Actual Position）。
 *
 * 注意：
 * - 该示例为了直接按轴读取 actualPosition，包含 motor_api_internal.h，并使用：
 *   - motor_api_handle_t（内部句柄结构）
 *   - MA_RD_S32()（基于域数据 + offset 的读取宏）
 *   这部分属于“库内部实现细节”，适合测试/示例，不建议作为对外稳定接口依赖。
 */

static volatile sig_atomic_t g_stop = 0;
static void sig_handler(int s) { (void)s; g_stop = 1; }

static bool is_drive_axis(const motor_api_handle_t *h, uint16_t axis_idx) {
    return h &&
           h->out[axis_idx].controlWord != UINT_MAX &&
           h->in[axis_idx].statusword != UINT_MAX &&
           h->in[axis_idx].actualPosition != UINT_MAX;
}

static bool is_axis_enabled(const motor_api_handle_t *h, uint16_t axis_idx) {
    if (!is_drive_axis(h, axis_idx)) return true;
    uint16_t sw = MA_RD_U16((motor_api_handle_t *)h, h->in[axis_idx].statusword);
    return ((sw & 0x6F) == 0x27);
}

static void clear_error_and_enable_all(struct motor_api_handle *pub_handle, uint32_t cycle_us, uint64_t timeout_ns) {
    motor_api_handle_t *h = (motor_api_handle_t *)pub_handle;
    if (!h) return;

    (void)motor_api_clear_error(pub_handle, -1);

    if (cycle_us == 0) cycle_us = 1000;

    uint64_t start = motor_api_monotonic_ns();
    while (!g_stop && (motor_api_monotonic_ns() - start) < timeout_ns) {
        motor_api_run_once(pub_handle);
        bool all_ok = true;
        for (uint16_t i = 0; i < h->slave_count; ++i) {
            if (!is_drive_axis(h, i)) continue;
            if (!is_axis_enabled(h, i)) { all_ok = false; break; }
        }
        if (all_ok) {
            printf("[INFO] all drive axes reached enabled state\n");
            return;
        }
        usleep(cycle_us);
    }
    printf("[WARN] enable wait timeout (%.3f s), continue anyway\n", (double)timeout_ns / 1000000000.0);
}

/*
 * pick_readable_eni_path
 * 功能：优先使用 argv[1]，否则从常见相对路径中挑选一个可读的 ENI 文件路径。
 */
static const char *pick_readable_eni_path(int argc, char **argv, char *buf, size_t buf_size) {
    if (argc > 1 && argv[1] && access(argv[1], R_OK) == 0) return argv[1];
    const char *candidates[] = {
        "motor_api/doc/HCFAX3E.xml",
        "../doc/HCFAX3E.xml",
        "./doc/HCFAX3E.xml",
        NULL
    };
    for (int i = 0; candidates[i]; ++i) {
        if (access(candidates[i], R_OK) == 0) return candidates[i];
    }
    if (buf && buf_size) {
        snprintf(buf, buf_size, "%s", (argc > 1 && argv[1]) ? argv[1] : candidates[0]);
        return buf;
    }
    return (argc > 1 && argv[1]) ? argv[1] : candidates[0];
}

int main(int argc, char **argv) {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /*
     * 第一步：定位一个可读的 ENI 文件路径。
     */
    char eni_buf[256];
    const char *eni_path = pick_readable_eni_path(argc, argv, eni_buf, sizeof(eni_buf));
    if (access(eni_path, R_OK) != 0) {
        fprintf(stderr, "ENI not readable: '%s' (errno=%d)\n", eni_path, errno);
        return 1;
    }

    /*
     * 第二步：读取并解析 ENI，拿到从站数量与基础识别信息（VID/PID/Position）。
     */
    uint32_t vendor_ids[MA_MAX_SLAVES];
    uint32_t product_codes[MA_MAX_SLAVES];
    uint16_t positions[MA_MAX_SLAVES];
    memset(vendor_ids, 0, sizeof(vendor_ids));
    memset(product_codes, 0, sizeof(product_codes));
    memset(positions, 0, sizeof(positions));

    ma_eni_slave_t *eni_slaves = NULL;
    uint16_t eni_slave_count = 0;

    printf("Reading ENI: %s\n", eni_path);
    ma_status_t st = motor_api_read_eni(eni_path,
                                        vendor_ids,
                                        product_codes,
                                        positions,
                                        MA_MAX_SLAVES,
                                        &eni_slave_count,
                                        &eni_slaves);
    if (st != MA_OK) {
        fprintf(stderr, "motor_api_read_eni failed: status=%d\n", st);
        return 1;
    }

    printf("ENI slaves=%u\n", eni_slave_count);
    for (uint16_t i = 0; i < eni_slave_count; ++i) {
        printf("  slave[%u] vid=0x%08X pid=0x%08X pos=%u\n",
               i, vendor_ids[i], product_codes[i], positions[i]);
    }
    motor_api_free_eni_slaves(eni_slaves, eni_slave_count);

    /*
     * 第三步：初始化 EtherCAT 主站/域/PDO 映射，获得库句柄。
     *
     * 说明：
     * - cycle_us 是控制周期（微秒），需与驱动插补周期等配置匹配；
     * - 这里用 4ms 作为示例默认值。
     */
    uint32_t cycle_us = 4000;
    struct motor_api_handle *pub_handle = NULL;
    uint16_t configured_slave_count = 0;
    st = motor_api_create(eni_path, cycle_us, &configured_slave_count, &pub_handle);
    if (st != MA_OK || !pub_handle) {
        fprintf(stderr, "motor_api_create failed: status=%d\n", st);
        return 1;
    }

    motor_api_handle_t *h = (motor_api_handle_t *)pub_handle;
    printf("motor_api created. configured_slaves=%u cycle_us=%u\n",
           configured_slave_count, (unsigned)cycle_us);

    clear_error_and_enable_all(pub_handle, cycle_us, 5000000000ULL);
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        if (!is_drive_axis(h, i)) continue;
        uint16_t sw = MA_RD_U16(h, h->in[i].statusword);
        printf("enabled[%u]=%s statusword=0x%04X\n", i, is_axis_enabled(h, i) ? "true" : "false", sw);
    }

    /*
     * 第五步：进入周期循环：
     * - motor_api_run_once()：完成收包/处理/写入/发包；
     * - 之后读取每个从站的 actualPosition（PDO 0x6064）并打印。
     */
    uint64_t print_interval = (cycle_us > 0) ? (1000000ULL / (uint64_t)cycle_us) : 0;
    if (print_interval == 0) print_interval = 1;
    uint64_t tick = 0;
    while (!g_stop) {
        motor_api_run_once(pub_handle);

        if ((tick % print_interval) == 0) {
            printf("actualPosition:");
            for (uint16_t i = 0; i < h->slave_count; ++i) {
                if (h->in[i].actualPosition == UINT_MAX) {
                    printf(" [%u]=NA", i);
                } else {
                    int32_t ap = MA_RD_S32(h, h->in[i].actualPosition);
                    printf(" [%u]=%d", i, ap);
                }
            }
            printf("\n");
        }

        ++tick;
        usleep(cycle_us);
    }

    motor_api_destroy(pub_handle);
    return 0;
}
