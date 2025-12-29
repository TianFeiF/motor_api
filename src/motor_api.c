/*
 * 版权所有 (C) 2025 phi
 * 文件名称: motor_api.c
 * 版本信息: v1.0.0
 * 文件说明: 通用电机控制库实现文件，封装 EtherCAT 主站生命周期、ENI 解析、
 *           DC 同步、CiA-402 状态机、CSP/CSV 运行、HTTP 控制与诊断等逻辑。
 * 模块关系: 与头文件 motor_api.h 配套；示例程序 example_csp.c 调用本模块 API。
 * 修改历史:
 *   - 2025-11-28: 初始实现，支持 ENI 读取、PDO 注册、DC 配置、HTTP 服务。
 *   - 2025-11-28: 增加“全轴使能(0x27)后延时 1s 同步起动”的栅栏机制与调试输出。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>

#include "motor_api.h"
#include "motor_api_internal.h"

/*
 * MA_MAX_DELTA_PER_CYCLE
 * 功能：限制单周期内目标位置增量的最大绝对值，用于保护驱动与避免指令突变。
 * 说明：delta = dir * step（来自网络/用户命令），这里做限幅防止过大。
 */
#define MA_MAX_DELTA_PER_CYCLE 400000

/*
 * eth_initDLL
 * 功能：为某些上层调用约定提供的“一键初始化”入口。
 *
 * 与 motor_api_create 的主要差异：
 * - 自动从固定目录 doc/ 选择最新的 ENI(xml)；
 * - 可选输出每个从站的产品名称（从 ENI 尽量提取 Name/ProductName）；
 * - 在初始化后等待从站响应（超时由 timeout_ms 控制），并返回配置是否匹配的标志。
 *
 * 说明：
 * - 本函数包含 EtherCAT 初始化过程（会调用 ecrt_*），因此属于 EtherCAT 核心模块；
 * - ENI 相关的扫描/解析工作由 motor_api_common.c 提供的函数完成。
 */
EXTERNFUNC ma_status_t eth_initDLL(uint32_t timeout_ms,
                                   uint16_t *out_slave_count,
                                   char (*out_product_names)[64],
                                   uint16_t max_slaves,
                                   bool *out_config_valid,
                                   struct motor_api_handle **out_handle) {
    const char *docdir = "/home/phi/ecmotor_api/motor_api/doc";
    uint32_t tmo = timeout_ms ? timeout_ms : 5000;
    char eni[512];
    /* 在 docdir 中选择“修改时间最新”的 xml 作为 ENI */
    if (motor_api_find_latest_eni_xml(docdir, eni, sizeof(eni)) != MA_OK) { if (out_config_valid) { *out_config_valid = false; } return MA_ERR_IO; }
    uint32_t vids[MA_MAX_SLAVES] = {0};
    uint32_t prods[MA_MAX_SLAVES] = {0};
    uint16_t poss[MA_MAX_SLAVES] = {0};
    uint16_t cnt = 0;
    ma_eni_slave_t *eni_slaves = NULL;
    /* 解析 ENI：得到 VID/PID/Position 以及 PDO 描述（用于后续从站配置与 PDO 映射） */
    ma_status_t rc = motor_api_read_eni(eni, vids, prods, poss, (uint16_t)max_slaves, &cnt, &eni_slaves);
    if (rc != MA_OK || cnt == 0) { if (out_config_valid) { *out_config_valid = false; } return MA_ERR_IO; }
    /* 若需要展示名称，尽量从 ENI 中提取 Name/ProductName，失败则用 PID_0x... 兜底 */
    if (out_product_names) (void)motor_api_fill_product_names_from_eni(eni, prods, cnt, out_product_names, max_slaves);
    motor_api_handle_t *h = (motor_api_handle_t *)calloc(1, sizeof(*h));
    if (!h) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_RUNTIME;
    }
    pthread_mutex_init(&h->cmd_mutex, NULL);
    h->master = ecrt_request_master(0);
    if (!h->master) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        free(h);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_INIT;
    }
    h->domain = ecrt_master_create_domain(h->master);
    if (!h->domain) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        ecrt_release_master(h->master);
        free(h);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_INIT;
    }
    h->slave_count = cnt;
    for (uint16_t i = 0; i < cnt; ++i) {
        h->vendor_id[i] = vids[i];
        h->product_code[i] = prods[i];
        h->position[i] = poss[i];
    }
    for (uint16_t i = 0; i < cnt; ++i) {
        h->sc[i] = ecrt_master_slave_config(h->master, 0, poss[i], vids[i], prods[i]);
        if (!h->sc[i]) {
            if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
            ecrt_release_master(h->master);
            free(h);
            if (out_config_valid) { *out_config_valid = false; }
            return MA_ERR_INIT;
        }
        (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 2, (uint8_t)-3);
        (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 1, (uint8_t)(h->cycle_us ? (h->cycle_us/1000U) : 4));
        (void)ecrt_slave_config_sdo32(h->sc[i], 0x6081, 0, 100000);
        (void)ecrt_slave_config_sdo32(h->sc[i], 0x6083, 0, 50000);
        (void)ecrt_slave_config_sdo32(h->sc[i], 0x6084, 0, 50000);
    }
    if (eni_slaves) {
        for (uint16_t i = 0; i < cnt; ++i) {
            unsigned int rx_n = eni_slaves[i].rx_pdo_count;
            unsigned int tx_n = eni_slaves[i].tx_pdo_count;
            ec_pdo_info_t *rx_infos = rx_n ? (ec_pdo_info_t *)calloc(rx_n, sizeof(*rx_infos)) : NULL;
            ec_pdo_info_t *tx_infos = tx_n ? (ec_pdo_info_t *)calloc(tx_n, sizeof(*tx_infos)) : NULL;
            ec_pdo_entry_info_t **rx_entries = rx_n ? (ec_pdo_entry_info_t **)calloc(rx_n, sizeof(*rx_entries)) : NULL;
            ec_pdo_entry_info_t **tx_entries = tx_n ? (ec_pdo_entry_info_t **)calloc(tx_n, sizeof(*tx_entries)) : NULL;
            for (unsigned int p = 0; p < rx_n; ++p) {
                unsigned int ecnt = eni_slaves[i].rx_pdos[p].entry_count;
                rx_entries[p] = ecnt ? (ec_pdo_entry_info_t *)calloc(ecnt, sizeof(*rx_entries[p])) : NULL;
                for (unsigned int e = 0; e < ecnt; ++e) {
                    rx_entries[p][e].index = eni_slaves[i].rx_pdos[p].entries[e].index;
                    rx_entries[p][e].subindex = eni_slaves[i].rx_pdos[p].entries[e].subindex;
                    rx_entries[p][e].bit_length = eni_slaves[i].rx_pdos[p].entries[e].bitlen;
                }
                rx_infos[p].index = eni_slaves[i].rx_pdos[p].pdo_index;
                rx_infos[p].entries = rx_entries[p];
                rx_infos[p].n_entries = ecnt;
            }
            for (unsigned int p = 0; p < tx_n; ++p) {
                unsigned int ecnt = eni_slaves[i].tx_pdos[p].entry_count;
                tx_entries[p] = ecnt ? (ec_pdo_entry_info_t *)calloc(ecnt, sizeof(*tx_entries[p])) : NULL;
                for (unsigned int e = 0; e < ecnt; ++e) {
                    tx_entries[p][e].index = eni_slaves[i].tx_pdos[p].entries[e].index;
                    tx_entries[p][e].subindex = eni_slaves[i].tx_pdos[p].entries[e].subindex;
                    tx_entries[p][e].bit_length = eni_slaves[i].tx_pdos[p].entries[e].bitlen;
                }
                tx_infos[p].index = eni_slaves[i].tx_pdos[p].pdo_index;
                tx_infos[p].entries = tx_entries[p];
                tx_infos[p].n_entries = ecnt;
            }
            ec_sync_info_t syncs[] = {
                {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                {2, EC_DIR_OUTPUT, (uint8_t)rx_n, rx_infos, EC_WD_ENABLE},
                {3, EC_DIR_INPUT, (uint8_t)tx_n, tx_infos, EC_WD_DISABLE},
                {0xFF, (ec_direction_t)0, 0, NULL, EC_WD_DISABLE}
            };
            if (ecrt_slave_config_pdos(h->sc[i], EC_END, syncs)) {
                for (unsigned int p = 0; p < rx_n; ++p) free(rx_entries[p]);
                for (unsigned int p = 0; p < tx_n; ++p) free(tx_entries[p]);
                free(rx_entries);
                free(tx_entries);
                free(rx_infos);
                free(tx_infos);
                motor_api_free_eni_slaves(eni_slaves, cnt);
                ecrt_release_master(h->master);
                free(h);
                if (out_config_valid) { *out_config_valid = false; }
                return MA_ERR_CONFIG;
            }
            for (unsigned int p = 0; p < rx_n; ++p) free(rx_entries[p]);
            for (unsigned int p = 0; p < tx_n; ++p) free(tx_entries[p]);
            free(rx_entries);
            free(tx_entries);
            free(rx_infos);
            free(tx_infos);
        }
    }
    ec_pdo_entry_reg_t regs[MA_MAX_SLAVES * 24 + 1];
    memset(regs, 0, sizeof(regs));
    size_t r = 0;
    for (uint16_t i = 0; i < cnt; ++i) {
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6040, .subindex = 0x00, .offset = &h->out[i].controlWord };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6060, .subindex = 0x00, .offset = &h->out[i].workModeOut };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x607A, .subindex = 0x00, .offset = &h->out[i].targetPosition };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60B8, .subindex = 0x00, .offset = &h->out[i].touchProbeFunc };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6041, .subindex = 0x00, .offset = &h->in[i].statusword };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6064, .subindex = 0x00, .offset = &h->in[i].actualPosition };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6061, .subindex = 0x00, .offset = &h->in[i].workModeIn };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x603F, .subindex = 0x00, .offset = &h->in[i].errorCode };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60F4, .subindex = 0x00, .offset = &h->in[i].followingError };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60FD, .subindex = 0x00, .offset = &h->in[i].digitalInputs };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60B9, .subindex = 0x00, .offset = &h->in[i].touchProbeStatus };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60BA, .subindex = 0x00, .offset = &h->in[i].touchProbePos };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x213F, .subindex = 0x00, .offset = &h->in[i].servoErrorCode };
    }
    regs[r] = (ec_pdo_entry_reg_t){0};
    if (ecrt_domain_reg_pdo_entry_list(h->domain, regs)) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        ecrt_release_master(h->master);
        free(h);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_CONFIG;
    }
    h->cycle_us = 4000;
    h->dc_sync0_period_ns = (uint64_t)h->cycle_us * 1000ULL;
    ecrt_master_select_reference_clock(h->master, h->sc[0]);
    for (uint16_t i = 0; i < cnt; ++i) {
        (void)ecrt_slave_config_dc(h->sc[i], 0x0300, h->dc_sync0_period_ns, 0, 0, 0);
    }
    if (ecrt_master_activate(h->master)) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        ecrt_release_master(h->master);
        free(h);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_INIT;
    }
    h->domain_pd = ecrt_domain_data(h->domain);
    if (!h->domain_pd) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        ecrt_release_master(h->master);
        free(h);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_INIT;
    }
    uint64_t start = motor_api_monotonic_ns();
    uint64_t end_ns = start + (uint64_t)tmo * 1000000ULL;
    uint16_t resp = 0;
    while (motor_api_monotonic_ns() < end_ns) {
        ec_master_state_t ms;
        ecrt_master_state(h->master, &ms);
        resp = (uint16_t)ms.slaves_responding;
        if (resp >= cnt) break;
        usleep(10000);
    }
    bool match = (resp == cnt);
    if (out_slave_count) *out_slave_count = cnt;
    if (out_config_valid) *out_config_valid = match && (cnt > 0);
    *out_handle = (struct motor_api_handle *)h;
    if (!match) return MA_ERR_CONFIG;
    return MA_OK;
}

/*
 * 函数: check_domain_state
 * 功能: 更新域状态快照（便于诊断）。
 */
static void check_domain_state(motor_api_handle_t *h) {
    ec_domain_state_t ds;
    ecrt_domain_state(h->domain, &ds);
    h->domain_state = ds;
}

/*
 * 函数: check_master_state
 * 功能: 更新主站状态快照（便于诊断）。
 */
static void check_master_state(motor_api_handle_t *h) {
    ec_master_state_t ms;
    ecrt_master_state(h->master, &ms);
    h->master_state = ms;
}

/*
 * 函数: check_slave_states
 * 功能: 更新从站配置状态快照（便于诊断）。
 */
static void check_slave_states(motor_api_handle_t *h) {
    ec_slave_config_state_t s;
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        ecrt_slave_config_state(h->sc[i], &s);
        h->sc_state[i] = s;
    }
}

/*
 * 函数: motor_api_create
 * 功能: 创建主站与域、注册 PDO、配置 DC，同步周期与 ENI。
 */
EXTERNFUNC ma_status_t motor_api_create(const char *eni_path,
                                        uint32_t cycle_us,
                                        uint16_t *out_slave_count,
                                        struct motor_api_handle **out_handle) {
    if (!out_handle || cycle_us == 0) return MA_ERR_PARAM;
    motor_api_handle_t *h = (motor_api_handle_t *)calloc(1, sizeof(*h)); if (!h) return MA_ERR_RUNTIME;
    h->cycle_us = cycle_us; h->dc_sync0_period_ns = (uint64_t)cycle_us * 1000ULL;
    pthread_mutex_init(&h->cmd_mutex, NULL);
    h->master = ecrt_request_master(0); if (!h->master) { free(h); return MA_ERR_INIT; }
    h->domain = ecrt_master_create_domain(h->master); if (!h->domain) { ecrt_release_master(h->master); free(h); return MA_ERR_INIT; }

    uint16_t cnt = 0; uint32_t vids[MA_MAX_SLAVES] = {0}, prods[MA_MAX_SLAVES] = {0}; uint16_t poss[MA_MAX_SLAVES] = {0};
    ma_eni_slave_t *eni_slaves = NULL;
    if (eni_path) {
        ma_status_t rc = motor_api_read_eni(eni_path, vids, prods, poss, MA_MAX_SLAVES, &cnt, &eni_slaves);
        if (rc != MA_OK || cnt == 0) {
            fprintf(stderr, "[ERROR] ENI parse failed or zero slaves: path=%s rc=%d cnt=%u\n", eni_path, rc, cnt);
            ecrt_release_master(h->master); free(h); return MA_ERR_CONFIG;
        }
        printf("[INFO] ENI parsed slaves=%u\n", cnt);
    } else {
        cnt = 3; vids[0] = vids[1] = vids[2] = 0x000116c7; prods[0] = prods[1] = prods[2] = 0x003e0402; poss[0] = 0; poss[1] = 1; poss[2] = 2;
        printf("[WARN] No ENI provided, using default 3 slaves\n");
    }
    h->slave_count = cnt; for (uint16_t i=0;i<cnt;i++){ h->vendor_id[i]=vids[i]; h->product_code[i]=prods[i]; h->position[i]=poss[i]; }
    for (uint16_t i = 0; i < cnt; ++i) {
        h->out[i].controlWord = UINT_MAX;
        h->out[i].workModeOut = UINT_MAX;
        h->out[i].targetPosition = UINT_MAX;
        h->out[i].touchProbeFunc = UINT_MAX;
        h->out[i].interpolationCtrl = UINT_MAX;
        h->in[i].statusword = UINT_MAX;
        h->in[i].workModeIn = UINT_MAX;
        h->in[i].actualPosition = UINT_MAX;
        h->in[i].actualVelocity = UINT_MAX;
        h->in[i].actualTorque = UINT_MAX;
        h->in[i].errorCode = UINT_MAX;
        h->in[i].followingError = UINT_MAX;
        h->in[i].digitalInputs = UINT_MAX;
        h->in[i].touchProbeStatus = UINT_MAX;
        h->in[i].touchProbePos = UINT_MAX;
        h->in[i].servoErrorCode = UINT_MAX;
        h->in[i].brakeDelay = UINT_MAX;
    }

    for (uint16_t i = 0; i < cnt; ++i) {
        h->sc[i] = ecrt_master_slave_config(h->master, 0, poss[i], vids[i], prods[i]);
        if (!h->sc[i]) { if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt); ecrt_release_master(h->master); free(h); return MA_ERR_INIT; }
        uint8_t period_ms = (uint8_t)(cycle_us / 1000U);
        if (vids[i] == 0x000116C7) {
            (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 2, (uint8_t)-3);
            (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 1, period_ms);
            (void)ecrt_slave_config_sdo32(h->sc[i], 0x6081, 0, 100000);
            (void)ecrt_slave_config_sdo32(h->sc[i], 0x6083, 0, 50000);
            (void)ecrt_slave_config_sdo32(h->sc[i], 0x6084, 0, 50000);
        } else if (vids[i] == 0x00001097) {
            (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 2, (uint8_t)-3);
            (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 1, 1);
            (void)ecrt_slave_config_sdo8(h->sc[i], 0x6060, 0, (uint8_t)MA_MODE_CSP);
        }
    }

    if (eni_slaves) {
        for (uint16_t i = 0; i < cnt; ++i) {
            unsigned int rx_n = eni_slaves[i].rx_pdo_count;
            unsigned int tx_n = eni_slaves[i].tx_pdo_count;
            ec_pdo_info_t *rx_infos = rx_n ? (ec_pdo_info_t *)calloc(rx_n, sizeof(*rx_infos)) : NULL;
            ec_pdo_info_t *tx_infos = tx_n ? (ec_pdo_info_t *)calloc(tx_n, sizeof(*tx_infos)) : NULL;
            ec_pdo_entry_info_t **rx_entries = rx_n ? (ec_pdo_entry_info_t **)calloc(rx_n, sizeof(*rx_entries)) : NULL;
            ec_pdo_entry_info_t **tx_entries = tx_n ? (ec_pdo_entry_info_t **)calloc(tx_n, sizeof(*tx_entries)) : NULL;

            for (unsigned int p = 0; p < rx_n; ++p) {
                unsigned int ecnt = eni_slaves[i].rx_pdos[p].entry_count;
                rx_entries[p] = ecnt ? (ec_pdo_entry_info_t *)calloc(ecnt, sizeof(*rx_entries[p])) : NULL;
                for (unsigned int e = 0; e < ecnt; ++e) {
                    rx_entries[p][e].index = eni_slaves[i].rx_pdos[p].entries[e].index;
                    rx_entries[p][e].subindex = eni_slaves[i].rx_pdos[p].entries[e].subindex;
                    rx_entries[p][e].bit_length = eni_slaves[i].rx_pdos[p].entries[e].bitlen;
                }
                rx_infos[p].index = eni_slaves[i].rx_pdos[p].pdo_index;
                rx_infos[p].entries = rx_entries[p];
                rx_infos[p].n_entries = ecnt;
            }
            for (unsigned int p = 0; p < tx_n; ++p) {
                unsigned int ecnt = eni_slaves[i].tx_pdos[p].entry_count;
                tx_entries[p] = ecnt ? (ec_pdo_entry_info_t *)calloc(ecnt, sizeof(*tx_entries[p])) : NULL;
                for (unsigned int e = 0; e < ecnt; ++e) {
                    tx_entries[p][e].index = eni_slaves[i].tx_pdos[p].entries[e].index;
                    tx_entries[p][e].subindex = eni_slaves[i].tx_pdos[p].entries[e].subindex;
                    tx_entries[p][e].bit_length = eni_slaves[i].tx_pdos[p].entries[e].bitlen;
                }
                tx_infos[p].index = eni_slaves[i].tx_pdos[p].pdo_index;
                tx_infos[p].entries = tx_entries[p];
                tx_infos[p].n_entries = ecnt;
            }

            ec_sync_info_t syncs[] = {
                {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                {2, EC_DIR_OUTPUT, (uint8_t)rx_n, rx_infos, EC_WD_ENABLE},
                {3, EC_DIR_INPUT, (uint8_t)tx_n, tx_infos, EC_WD_DISABLE},
                {0xFF, (ec_direction_t)0, 0, NULL, EC_WD_DISABLE}
            };

            if (ecrt_slave_config_pdos(h->sc[i], EC_END, syncs)) {
                for (unsigned int p = 0; p < rx_n; ++p) free(rx_entries[p]);
                for (unsigned int p = 0; p < tx_n; ++p) free(tx_entries[p]);
                free(rx_entries); free(tx_entries); free(rx_infos); free(tx_infos);
                motor_api_free_eni_slaves(eni_slaves, cnt);
                ecrt_release_master(h->master); free(h); return MA_ERR_CONFIG;
            }
            for (unsigned int p = 0; p < rx_n; ++p) free(rx_entries[p]);
            for (unsigned int p = 0; p < tx_n; ++p) free(tx_entries[p]);
            free(rx_entries);
            free(tx_entries);
            free(rx_infos);
            free(tx_infos);
        }
    } else {
        static ec_pdo_entry_info_t device_pdo_entries[] = {
            {0x6040, 0x00, 16}, {0x6060, 0x00, 8}, {0x607A, 0x00, 32}, {0x60B8, 0x00, 16},
            {0x603F, 0x00, 16}, {0x6041, 0x00, 16}, {0x6064, 0x00, 32}, {0x6061, 0x00, 8}, {0x60B9, 0x00, 16}, {0x60BA, 0x00, 32}, {0x60F4, 0x00, 32}, {0x60FD, 0x00, 32}, {0x213F, 0x00, 16},
        };
        static ec_pdo_info_t device_pdos[] = {
            {0x1600, 4, device_pdo_entries + 0},
            {0x1A00, 9, device_pdo_entries + 4},
        };
        static ec_sync_info_t device_syncs[] = {
            {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
            {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
            {2, EC_DIR_OUTPUT, 1, device_pdos + 0, EC_WD_ENABLE},
            {3, EC_DIR_INPUT, 1, device_pdos + 1, EC_WD_DISABLE},
            {0xFF, (ec_direction_t)0, 0, NULL, EC_WD_DISABLE}
        };
        for (uint16_t i = 0; i < cnt; ++i) {
            if (ecrt_slave_config_pdos(h->sc[i], EC_END, device_syncs)) { if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt); ecrt_release_master(h->master); free(h); return MA_ERR_CONFIG; }
        }
    }

    ec_pdo_entry_reg_t regs[MA_MAX_SLAVES * 24 + 1]; memset(regs, 0, sizeof(regs)); size_t r = 0;
    for (uint16_t i = 0; i < cnt; ++i) {
        const ma_eni_slave_t *s = eni_slaves ? &eni_slaves[i] : NULL;
        if (!s || motor_api_eni_has_rx_entry(s, 0x6040, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6040, .subindex = 0x00, .offset = &h->out[i].controlWord };
        if (!s || motor_api_eni_has_rx_entry(s, 0x6060, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6060, .subindex = 0x00, .offset = &h->out[i].workModeOut };
        if (!s || motor_api_eni_has_rx_entry(s, 0x607A, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x607A, .subindex = 0x00, .offset = &h->out[i].targetPosition };
        if (!s || motor_api_eni_has_rx_entry(s, 0x60B8, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60B8, .subindex = 0x00, .offset = &h->out[i].touchProbeFunc };
        if (!s || motor_api_eni_has_rx_entry(s, 0x60C2, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60C2, .subindex = 0x00, .offset = &h->out[i].interpolationCtrl };
        if (!s || motor_api_eni_has_tx_entry(s, 0x6041, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6041, .subindex = 0x00, .offset = &h->in[i].statusword };
        if (!s || motor_api_eni_has_tx_entry(s, 0x6064, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6064, .subindex = 0x00, .offset = &h->in[i].actualPosition };
        if (!s || motor_api_eni_has_tx_entry(s, 0x606C, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x606C, .subindex = 0x00, .offset = &h->in[i].actualVelocity };
        if (!s || motor_api_eni_has_tx_entry(s, 0x6077, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6077, .subindex = 0x00, .offset = &h->in[i].actualTorque };
        if (!s || motor_api_eni_has_tx_entry(s, 0x6061, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6061, .subindex = 0x00, .offset = &h->in[i].workModeIn };
        if (!s || motor_api_eni_has_tx_entry(s, 0x603F, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x603F, .subindex = 0x00, .offset = &h->in[i].errorCode };
        if (!s || motor_api_eni_has_tx_entry(s, 0x60F4, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60F4, .subindex = 0x00, .offset = &h->in[i].followingError };
        if (!s || motor_api_eni_has_tx_entry(s, 0x60FD, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60FD, .subindex = 0x00, .offset = &h->in[i].digitalInputs };
        if (!s || motor_api_eni_has_tx_entry(s, 0x60B9, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60B9, .subindex = 0x00, .offset = &h->in[i].touchProbeStatus };
        if (!s || motor_api_eni_has_tx_entry(s, 0x60BA, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60BA, .subindex = 0x00, .offset = &h->in[i].touchProbePos };
        if (!s || motor_api_eni_has_tx_entry(s, 0x213F, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x213F, .subindex = 0x00, .offset = &h->in[i].servoErrorCode };
        if (!s || motor_api_eni_has_tx_entry(s, 0x2026, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x2026, .subindex = 0x00, .offset = &h->in[i].brakeDelay };
    }
    regs[r] = (ec_pdo_entry_reg_t){0};
    if (ecrt_domain_reg_pdo_entry_list(h->domain, regs)) { if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt); ecrt_release_master(h->master); free(h); return MA_ERR_CONFIG; }

    /* DC 配置：选 0 号从站为参考时钟，统一 Sync0 周期 */
    ecrt_master_select_reference_clock(h->master, h->sc[0]);
    for (uint16_t i = 0; i < cnt; ++i) {
        if (h->vendor_id[i] == 0x000116C7) {
            (void)ecrt_slave_config_dc(h->sc[i], 0x0300, h->dc_sync0_period_ns, 0, 0, 0);
        }
    }

    if (ecrt_master_activate(h->master)) { ecrt_release_master(h->master); free(h); return MA_ERR_INIT; }
    h->domain_pd = ecrt_domain_data(h->domain); if (!h->domain_pd) { ecrt_release_master(h->master); free(h); return MA_ERR_INIT; }
    h->barrier_armed = 0; h->barrier_start_ns = 0; h->barrier_delay_ns = 1000000000ULL; h->motion_started = 0;
    memset(h->seen_enabled, 0, sizeof(h->seen_enabled));
    /* 创建后打印已注册 PDO 列表 */
    for (uint16_t i = 0; i < cnt; ++i) {
        printf("[PDO] Slave position=%u vid=0x%08X pid=0x%08X\n", h->position[i], h->vendor_id[i], h->product_code[i]);
        if (eni_slaves) {
            printf("  Rx:");
            unsigned int rx_n = eni_slaves[i].rx_pdo_count;
            for (unsigned int p = 0; p < rx_n; ++p) {
                uint16_t pidx = eni_slaves[i].rx_pdos[p].pdo_index;
                unsigned int ecnt = eni_slaves[i].rx_pdos[p].entry_count;
                printf(" [0x%04X]", pidx);
                for (unsigned int e = 0; e < ecnt; ++e) {
                    uint16_t ix = eni_slaves[i].rx_pdos[p].entries[e].index;
                    uint8_t  si = eni_slaves[i].rx_pdos[p].entries[e].subindex;
                    uint8_t  bl = eni_slaves[i].rx_pdos[p].entries[e].bitlen;
                    printf(" 0x%04X:%u %u", ix, si, bl);
                }
            }
            printf("\n  Tx:");
            unsigned int tx_n = eni_slaves[i].tx_pdo_count;
            for (unsigned int p = 0; p < tx_n; ++p) {
                uint16_t pidx = eni_slaves[i].tx_pdos[p].pdo_index;
                unsigned int ecnt = eni_slaves[i].tx_pdos[p].entry_count;
                printf(" [0x%04X]", pidx);
                for (unsigned int e = 0; e < ecnt; ++e) {
                    uint16_t ix = eni_slaves[i].tx_pdos[p].entries[e].index;
                    uint8_t  si = eni_slaves[i].tx_pdos[p].entries[e].subindex;
                    uint8_t  bl = eni_slaves[i].tx_pdos[p].entries[e].bitlen;
                    printf(" 0x%04X:%u %u", ix, si, bl);
                }
            }
            printf("\n");
        } else {
            printf("  Rx: 0x6040:0 16, 0x6060:0 8, 0x607A:0 32, 0x60B8:0 16\n");
            printf("  Tx: 0x6041:0 16, 0x6064:0 32, 0x6061:0 8, 0x603F:0 16, 0x60F4:0 32, 0x60FD:0 32, 0x60B9:0 16, 0x60BA:0 32, 0x213F:0 16\n");
        }
        printf("[OFFS%d] out:6040=%u 6060=%u 607A=%u 60C2=%u | in:6041=%u 6064=%u 6061=%u 606C=%u 6077=%u 603F=%u 2026=%u\n",
               i,
               h->out[i].controlWord, h->out[i].workModeOut, h->out[i].targetPosition, h->out[i].interpolationCtrl,
               h->in[i].statusword, h->in[i].actualPosition, h->in[i].workModeIn, h->in[i].actualVelocity, h->in[i].actualTorque, h->in[i].errorCode, h->in[i].brakeDelay);
    }
    *out_handle = (struct motor_api_handle *)h; if (out_slave_count) *out_slave_count = cnt; if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
    return MA_OK;
}

/*
 * 函数: motor_api_destroy
 * 功能: 释放主站资源与句柄。
 */
EXTERNFUNC ma_status_t motor_api_destroy(struct motor_api_handle *handle) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle; if (!h) return MA_ERR_PARAM;
    ecrt_release_master(h->master);
    pthread_mutex_destroy(&h->cmd_mutex);
    free(h);
    return MA_OK;
}

/*
 * format_diag
 * 功能：从当前 domain 数据区读取各轴关键诊断项，并拼装为 JSON 字符串。
 *
 * 输出字段含义（数组按轴索引排列）：
 * - status：0x6041 状态字
 * - mode：0x6061 当前模式
 * - followingErr：0x60F4 跟随误差
 * - err：0x603F 错误码
 * - servoErr：0x213F 厂商自定义错误码
 * - din：0x60FD 数字输入
 * - tpst：0x60B9 触发探针状态
 * - tpp：0x60BA 触发探针位置
 * - tgt：0x607A 目标位置（主站当前写入值）
 * - act：0x6064 实际位置
 *
 * 注意：
 * - 这里为了保持简洁，输出 JSON 固定按前三轴格式化；当 slave_count > 3 时，
 *   后续轴的数据不会进入 JSON（如需扩展可改为动态拼接）。
 */
static ma_status_t format_diag(motor_api_handle_t *h, char *buf, size_t buf_size) {
    if (!h || !buf || buf_size < 64) return MA_ERR_PARAM;
    uint16_t sw[MA_MAX_SLAVES] = {0};
    int8_t md[MA_MAX_SLAVES] = {0};
    int32_t fe[MA_MAX_SLAVES] = {0};
    uint16_t ec[MA_MAX_SLAVES] = {0};
    uint16_t sec[MA_MAX_SLAVES] = {0};
    uint32_t di[MA_MAX_SLAVES] = {0};
    uint16_t tpst[MA_MAX_SLAVES] = {0};
    int32_t tpp[MA_MAX_SLAVES] = {0};
    int32_t tgt[MA_MAX_SLAVES] = {0};
    int32_t act[MA_MAX_SLAVES] = {0};
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        sw[i] = MA_RD_U16(h, h->in[i].statusword);
        md[i] = MA_RD_S8(h, h->in[i].workModeIn);
        fe[i] = MA_RD_S32(h, h->in[i].followingError);
        ec[i] = MA_RD_U16(h, h->in[i].errorCode);
        sec[i] = MA_RD_U16(h, h->in[i].servoErrorCode);
        di[i] = MA_RD_U32(h, h->in[i].digitalInputs);
        tpst[i] = MA_RD_U16(h, h->in[i].touchProbeStatus);
        tpp[i] = MA_RD_S32(h, h->in[i].touchProbePos);
        tgt[i] = MA_RD_S32(h, h->out[i].targetPosition);
        act[i] = MA_RD_S32(h, h->in[i].actualPosition);
    }
    int n = snprintf(buf, buf_size,
                     "{\"status\":[%u,%u,%u],\"mode\":[%d,%d,%d],\"followingErr\":[%d,%d,%d],\"err\":[%u,%u,%u],\"servoErr\":[%u,%u,%u],\"din\":[%u,%u,%u],\"tpst\":[%u,%u,%u],\"tpp\":[%d,%d,%d],\"tgt\":[%d,%d,%d],\"act\":[%d,%d,%d]}",
                     sw[0], sw[1], sw[2],
                     md[0], md[1], md[2],
                     fe[0], fe[1], fe[2],
                     ec[0], ec[1], ec[2],
                     sec[0], sec[1], sec[2],
                     di[0], di[1], di[2],
                     tpst[0], tpst[1], tpst[2],
                     tpp[0], tpp[1], tpp[2],
                     tgt[0], tgt[1], tgt[2],
                     act[0], act[1], act[2]);
    return (n < 0) ? MA_ERR_RUNTIME : MA_OK;
}

/*
 * 函数: motor_api_format_diag_json
 * 功能: 诊断信息格式化为 JSON。
 */
EXTERNFUNC ma_status_t motor_api_format_diag_json(struct motor_api_handle *handle, char *buf, size_t buf_size) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle; if (!h) return MA_ERR_PARAM;
    return format_diag(h, buf, buf_size);
}

/*
 * 函数: motor_api_run_once
 * 功能: 周期性控制入口，包含状态机推进、目标更新、同步栅栏与调试输出。
 * 注意: 需以固定周期调用（例如 4ms）。
 */
EXTERNFUNC ma_status_t motor_api_run_once(struct motor_api_handle *handle) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle; if (!h) return MA_ERR_PARAM;
    ecrt_master_application_time(h->master, motor_api_monotonic_ns());
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
        MA_WR_S8(h, h->out[i].interpolationCtrl, (int8_t)1);
    }
    ecrt_master_receive(h->master);
    ecrt_domain_process(h->domain);
    ecrt_master_sync_slave_clocks(h->master);
    check_domain_state(h);
    check_master_state(h);
    check_slave_states(h);
    static int dbg_tick = 0; dbg_tick++;
    /* 逐轴推进状态机与写入控制字/模式 */
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
        MA_WR_S8(h, h->out[i].interpolationCtrl, (int8_t)1);
        uint16_t status_i = MA_RD_U16(h, h->in[i].statusword);
        if ((status_i & 0x6F) == 0x27) h->seen_enabled[i] = true; else h->seen_enabled[i] = false;
        uint16_t control_i = 0x06;
        if (!h->servo_enabled[i]) {
            /* 依据 CiA-402 标准用状态字低位掩码推进控制字序列 */
            switch (status_i & 0x6F) {
                case 0x00: control_i = 0x06; break;
                case 0x40: control_i = 0x06; break;
                case 0x21:
                    control_i = 0x07;
                    h->csp_target[i] = MA_RD_S32(h, h->in[i].actualPosition);
                    MA_WR_S32(h, h->out[i].targetPosition, h->csp_target[i]);
                    break;
                case 0x23: control_i = 0x0F; break;
                case 0x27:
                    control_i = 0x0F;
                    if (!h->servo_enabled[i]) {
                        h->servo_enabled[i] = true;
                        if (dbg_tick % 100 == 0) {
                            int32_t ap = MA_RD_S32(h, h->in[i].actualPosition);
                            printf("[ENABLED%d] sw:0x%04X act:%d\n", i, status_i, ap);
                        }
                    }
                    h->csp_warmup[i] = 10;
                    h->csp_target[i] = MA_RD_S32(h, h->in[i].actualPosition);
                    break;
                default: control_i = 0x06; break;
            }
            /* 检测故障位并执行快速复位（0x0080） */
            if ((status_i & 0x0040) && !(status_i & 0x0001)) {
                MA_WR_U16(h, h->out[i].controlWord, 0x0000);
                MA_WR_U16(h, h->out[i].controlWord, 0x0080);
            }
            MA_WR_U16(h, h->out[i].controlWord, control_i);
            MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
            MA_WR_S8(h, h->out[i].interpolationCtrl, (int8_t)1);
            if (dbg_tick % 500 == 0) {
                int ack = (status_i & 0x1000) ? 1 : 0;
                int trg = (status_i & 0x0400) ? 1 : 0;
                int32_t ap = MA_RD_S32(h, h->in[i].actualPosition);
                printf("[EN%d] sw:0x%04X ctrl:0x%04X mode:%d ack12:%d trg10:%d act:%d\n", i, status_i, control_i, MA_RD_S8(h, h->in[i].workModeIn), ack, trg, ap);
            }
        } else {
            h->time_cnt[i]++;
            /* 延迟栅栏未触发：保位到实际位置，写 0x0F 与模式 */
            if (!h->motion_started) {
                h->csp_target[i] = MA_RD_S32(h, h->in[i].actualPosition);
                MA_WR_S32(h, h->out[i].targetPosition, h->csp_target[i]);
                MA_WR_U16(h, h->out[i].controlWord, 0x0F);
                MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
                MA_WR_S8(h, h->out[i].interpolationCtrl, (int8_t)1);
                h->last_actual_pos[i] = MA_RD_S32(h, h->in[i].actualPosition);
                if (dbg_tick % 100 == 0) {
                    printf("[GATE%d] hold tgt:%d act:%d sw:0x%04X mode:%d\n", i,
                           MA_RD_S32(h, h->out[i].targetPosition),
                           h->last_actual_pos[i], status_i,
                           MA_RD_S8(h, h->in[i].workModeIn));
                }
            } else {
                /* 延迟栅栏已触发：按命令增量推进目标（限幅与预热） */
                pthread_mutex_lock(&h->cmd_mutex);
                /* 优先使用单轴指令，如果单轴指令有效（run=true），则覆盖全局指令；否则叠加或仅用全局 */
                /* 策略：如果单轴 run=true，则使用单轴参数；否则使用全局参数 */
                bool run = h->axis_run[i] ? true : h->cmd_run;
                int dir  = h->axis_run[i] ? h->axis_dir[i] : h->cmd_dir;
                int step = h->axis_run[i] ? h->axis_step[i] : h->cmd_step;
                pthread_mutex_unlock(&h->cmd_mutex);

                int delta = run ? (dir * step) : 0;
                if (delta > MA_MAX_DELTA_PER_CYCLE) delta = MA_MAX_DELTA_PER_CYCLE;
                if (delta < -MA_MAX_DELTA_PER_CYCLE) delta = -MA_MAX_DELTA_PER_CYCLE;
                
                /* HCFA 从站 1-3 特殊处理：如果状态机刚进入 enabled 且尚未收到运动指令，保持当前位置 */
                /* 已经在 csp_warmup 处理了初始跳变，这里主要确保 delta 能够正确作用 */
                
                if (h->csp_warmup[i] > 0) { 
                    h->csp_target[i] = MA_RD_S32(h, h->in[i].actualPosition); 
                    h->csp_warmup[i]--; 
                } else { 
                    h->csp_target[i] += delta; 
                }
                
                /* 调试：当有运动指令时，打印 delta 和 target，帮助排查为何不转动 */
                if (run && (dbg_tick % 100 == 0)) {
                     printf("[DEBUG%d] run=%d dir=%d step=%d delta=%d warm=%d tgt=%d act=%d\n", 
                            i, run, dir, step, delta, h->csp_warmup[i], h->csp_target[i], MA_RD_S32(h, h->in[i].actualPosition));
                }

                MA_WR_S32(h, h->out[i].targetPosition, h->csp_target[i]);
                MA_WR_U16(h, h->out[i].controlWord, 0x0F);
                MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
                MA_WR_S8(h, h->out[i].interpolationCtrl, (int8_t)1);
                h->last_actual_pos[i] = MA_RD_S32(h, h->in[i].actualPosition);
                if (dbg_tick % 500 == 0) {
                    printf("[RUN%d] tgt:%d act:%d sw:0x%04X mode:%d\n", i,
                           MA_RD_S32(h, h->out[i].targetPosition),
                           h->last_actual_pos[i], status_i,
                           MA_RD_S8(h, h->in[i].workModeIn));
                }
            }
        }
    }
    {
        /* 栅栏逻辑：检测全轴使能后武装，延时 1s 后统一开始运动 */
        pthread_mutex_lock(&h->cmd_mutex); 
        bool run = h->cmd_run;
        /* 只要有任意一个轴单独运行，或者全局运行，都触发栅栏逻辑 */
        for(uint16_t i=0; i<h->slave_count; ++i) if(h->axis_run[i]) run = true;
        pthread_mutex_unlock(&h->cmd_mutex);
        
        /* 消除未使用变量警告 */
        (void)run;

        int all_enabled = 1; for (uint16_t i = 0; i < h->slave_count; ++i) all_enabled = all_enabled && h->seen_enabled[i];
        if (!h->motion_started) {
             /* 即使没有全局 run，只要 all_enabled 就允许进入 armed 状态，以便单轴控制随时启动 */
             /* 修改逻辑：只要全轴使能，就自动 ARM 并 FIRE，不再等待 run 指令 */
             /* 这样可以解除 "必须先发 run 才能动" 的限制，单轴控制可以直接生效 */
            if (!h->barrier_armed && all_enabled) {
                h->barrier_armed = 1; h->barrier_start_ns = motor_api_monotonic_ns();
                printf("[BARRIER_ARM] all at 0x027 (enabled), wait 1s\n");
            }
            if (h->barrier_armed) {
                uint64_t now = motor_api_monotonic_ns();
                if (now - h->barrier_start_ns >= h->barrier_delay_ns) {
                    for (uint16_t i = 0; i < h->slave_count; ++i) {
                        h->csp_target[i] = MA_RD_S32(h, h->in[i].actualPosition);
                        MA_WR_S32(h, h->out[i].targetPosition, h->csp_target[i]);
                        MA_WR_U16(h, h->out[i].controlWord, 0x0F);
                        MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
                    }
                    printf("[BARRIER_FIRE] synchronized motion start after 1s (enabled), slaves=%u\n", h->slave_count);
                    h->motion_started = 1; h->barrier_armed = 0;
                }
            }
        }
    }
    /* 提交域数据并发送到主站 */
    ecrt_domain_queue(h->domain);
    ecrt_master_send(h->master);
    return MA_OK;
}
