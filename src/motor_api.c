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
ma_status_t motor_api_create_base(const char *eni_path,
                                        uint32_t cycle_us,
                                        uint16_t *out_slave_count,
                                        struct motor_api_handle **out_handle,
                                        const ma_axis_map_t *axis_override,
                                        int axis_override_count) {
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
    // 初始化 out/in 数组的所有字段为 UINT_MAX
    for (int i = 0; i < MA_MAX_AXES; ++i) {
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

    /* 轴配置逻辑：优先使用 Override，否则自动发现 */
    bool use_override = (axis_override && axis_override_count > 0);
    h->axis_count = 0;
    if (use_override) {
        printf("[INFO] Using manual axis configuration (count=%d)\n", axis_override_count);
        for (int k = 0; k < axis_override_count; k++) {
            if (h->axis_count >= MA_MAX_AXES) break;
            h->axis_map[h->axis_count] = axis_override[k];
            h->axis_count++;
        }
    }

    /* 遍历从站，初始化 SC 并（可选）进行轴发现 */
    for (uint16_t i = 0; i < cnt; ++i) {
        h->vendor_id[i] = vids[i];
        h->product_code[i] = prods[i];
        h->position[i] = poss[i];

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

        if (!use_override) {
            const ma_eni_slave_t *s = eni_slaves ? &eni_slaves[i] : NULL;
        if (!s) {
            // 无 ENI 信息，默认为单轴 CiA402，偏移 0x0000
            if (h->axis_count < MA_MAX_AXES) {
                ma_axis_map_t *ax = &h->axis_map[h->axis_count];
                ax->active = true;
                ax->slave_idx = i;
                ax->type = MA_AXIS_TYPE_CIA402;
                ax->base_offset = 0x0000;
                ax->scale_pos = 1.0;
                ax->scale_vel = 1.0;
                h->axis_count++;
            }
        } else {
            // 扫描 ENI，探测轴
            // 规则 1：检查标准 CiA402 (0x6040)
            if (motor_api_eni_has_rx_entry(s, 0x6040, 0)) {
                if (h->axis_count < MA_MAX_AXES) {
                    ma_axis_map_t *ax = &h->axis_map[h->axis_count];
                    ax->active = true;
                    ax->slave_idx = i;
                    ax->type = MA_AXIS_TYPE_CIA402;
                    ax->base_offset = 0x0000; // 基址偏移 0
                    ax->scale_pos = 1.0;
                    ax->scale_vel = 1.0;
                    h->axis_count++;
                }
            }
            // 规则 2：检查 Hans Robot 风格第二轴 (0x6840)
            if (motor_api_eni_has_rx_entry(s, 0x6840, 0)) {
                if (h->axis_count < MA_MAX_AXES) {
                    ma_axis_map_t *ax = &h->axis_map[h->axis_count];
                    ax->active = true;
                    ax->slave_idx = i;
                    ax->type = MA_AXIS_TYPE_CIA402;
                    ax->base_offset = 0x0800; // 基址偏移 0x800 (6840 - 6040)
                    ax->scale_pos = 1.0;
                    ax->scale_vel = 1.0;
                    h->axis_count++;
                }
            }
            // 规则 3：如果既没有 0x6040 也没有 0x6840，但有 IO (如 0x70xx/0x60xx IO port)，可以作为 IO 轴
            if (!motor_api_eni_has_rx_entry(s, 0x6040, 0) && !motor_api_eni_has_rx_entry(s, 0x6840, 0)) {
                 // 简单探测：若有 0x7000 (Output) 或 0x6000 (Input)
                 bool has_io_out = motor_api_eni_has_rx_entry(s, 0x7000, 1) || motor_api_eni_has_rx_entry(s, 0x7010, 1);
                 bool has_io_in  = motor_api_eni_has_tx_entry(s, 0x6000, 1) || motor_api_eni_has_tx_entry(s, 0x6010, 1);
                 
                 if (has_io_out || has_io_in) {
                    if (h->axis_count < MA_MAX_AXES) {
                        ma_axis_map_t *ax = &h->axis_map[h->axis_count];
                        ax->active = true;
                        ax->slave_idx = i;
                        ax->type = MA_AXIS_TYPE_IO;
                        ax->base_offset = 0x0000;
                        ax->scale_pos = 1.0;
                        ax->scale_vel = 1.0;
                        h->axis_count++;
                        printf("[INFO] Slave %u detected as IO device\n", i);
                    }
                 }
            }
        }
        }
    }
    if (!use_override) printf("[INFO] Auto-detected axes count: %u\n", h->axis_count);

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

    ec_pdo_entry_reg_t regs[MA_MAX_AXES * 24 + 1]; memset(regs, 0, sizeof(regs)); size_t r = 0;
    for (uint16_t j = 0; j < h->axis_count; ++j) {
        ma_axis_map_t *ax = &h->axis_map[j];
        uint16_t i = ax->slave_idx;
        uint16_t base = ax->base_offset;
        const ma_eni_slave_t *s = eni_slaves ? &eni_slaves[i] : NULL;

        if (ax->type == MA_AXIS_TYPE_IO) {
            // IO 设备映射：Input -> digitalInputs(0x60FD), Output -> digitalOutputs (reuse controlWord?)
            // 暂时只映射 Input 到 digitalInputs (0x60FD local storage), Output 到 controlWord (0x6040 local storage)
            // 注意：这里需要根据实际 IO 模块的对象字典调整。
            // 假设 Input 在 0x6000:01, Output 在 0x7000:01
            if (!s || motor_api_eni_has_tx_entry(s, 0x6000, 1)) 
                regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6000, .subindex = 0x01, .offset = &h->in[j].digitalInputs };
            if (!s || motor_api_eni_has_rx_entry(s, 0x7000, 1))
                regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x7000, .subindex = 0x01, .offset = &h->out[j].controlWord }; // 借用 controlWord
            continue;
        }

        // CiA402 轴映射
        if (!s || motor_api_eni_has_rx_entry(s, 0x6040 + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6040 + base, .subindex = 0x00, .offset = &h->out[j].controlWord };
        if (!s || motor_api_eni_has_rx_entry(s, 0x6060 + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6060 + base, .subindex = 0x00, .offset = &h->out[j].workModeOut };
        if (!s || motor_api_eni_has_rx_entry(s, 0x607A + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x607A + base, .subindex = 0x00, .offset = &h->out[j].targetPosition };
        if (!s || motor_api_eni_has_rx_entry(s, 0x60B8 + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60B8 + base, .subindex = 0x00, .offset = &h->out[j].touchProbeFunc };
        if (!s || motor_api_eni_has_rx_entry(s, 0x60C2 + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60C2 + base, .subindex = 0x00, .offset = &h->out[j].interpolationCtrl };
        
        if (!s || motor_api_eni_has_tx_entry(s, 0x6041 + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6041 + base, .subindex = 0x00, .offset = &h->in[j].statusword };
        if (!s || motor_api_eni_has_tx_entry(s, 0x6064 + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6064 + base, .subindex = 0x00, .offset = &h->in[j].actualPosition };
        if (!s || motor_api_eni_has_tx_entry(s, 0x606C + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x606C + base, .subindex = 0x00, .offset = &h->in[j].actualVelocity };
        if (!s || motor_api_eni_has_tx_entry(s, 0x6077 + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6077 + base, .subindex = 0x00, .offset = &h->in[j].actualTorque };
        if (!s || motor_api_eni_has_tx_entry(s, 0x6061 + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6061 + base, .subindex = 0x00, .offset = &h->in[j].workModeIn };
        if (!s || motor_api_eni_has_tx_entry(s, 0x603F + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x603F + base, .subindex = 0x00, .offset = &h->in[j].errorCode };
        if (!s || motor_api_eni_has_tx_entry(s, 0x60F4 + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60F4 + base, .subindex = 0x00, .offset = &h->in[j].followingError };
        if (!s || motor_api_eni_has_tx_entry(s, 0x60FD + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60FD + base, .subindex = 0x00, .offset = &h->in[j].digitalInputs };
        if (!s || motor_api_eni_has_tx_entry(s, 0x60B9 + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60B9 + base, .subindex = 0x00, .offset = &h->in[j].touchProbeStatus };
        if (!s || motor_api_eni_has_tx_entry(s, 0x60BA + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60BA + base, .subindex = 0x00, .offset = &h->in[j].touchProbePos };
        if (!s || motor_api_eni_has_tx_entry(s, 0x213F + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x213F + base, .subindex = 0x00, .offset = &h->in[j].servoErrorCode };
        if (!s || motor_api_eni_has_tx_entry(s, 0x2026 + base, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x2026 + base, .subindex = 0x00, .offset = &h->in[j].brakeDelay };
    }
     regs[r] = (ec_pdo_entry_reg_t){0};

     if (ecrt_domain_reg_pdo_entry_list(h->domain, regs)) { 
         if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt); 
         ecrt_release_master(h->master); 
         free(h); 
         return MA_ERR_CONFIG; 
     }

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
    printf("[INFO] Registered axes: %u\n", h->axis_count);
    for (uint16_t j = 0; j < h->axis_count; ++j) {
        ma_axis_map_t *ax = &h->axis_map[j];
        uint16_t i = ax->slave_idx;
        printf("[AXIS%d] slave:%u type:%d base:0x%04X\n", j, i, ax->type, ax->base_offset);
    }

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
        printf("[OFFS_SLAVE%d] (use axis view to see data)\n", i);
    }
    *out_handle = (struct motor_api_handle *)h; if (out_slave_count) *out_slave_count = cnt; if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
    return MA_OK;
}

/*
 * 函数: motor_api_config_axis
 * 功能: 配置轴的机械参数，用于内部单位转换。
 */
EXTERNFUNC ma_status_t motor_api_config_axis(struct motor_api_handle *handle,
                                             uint16_t axis_idx,
                                             uint32_t encoder_res,
                                             double gear_ratio,
                                             double unit_per_rev) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle;
    if (!h || axis_idx >= h->axis_count) return MA_ERR_PARAM;
    if (encoder_res == 0) encoder_res = 1; 
    if (unit_per_rev == 0.0) unit_per_rev = 1.0; 

    double scale = (double)encoder_res * gear_ratio / unit_per_rev;
    h->axis_map[axis_idx].scale_pos = scale;
    h->axis_map[axis_idx].scale_vel = scale; 
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

EXTERNFUNC ma_status_t motor_api_create(const char *eni_path,
                                        uint32_t cycle_us,
                                        uint16_t *out_slave_count,
                                        struct motor_api_handle **out_handle) {
    return motor_api_create_base(eni_path, cycle_us, out_slave_count, out_handle, NULL, 0);
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
    uint16_t sw[MA_MAX_AXES] = {0};
    int8_t md[MA_MAX_AXES] = {0};
    int32_t fe[MA_MAX_AXES] = {0};
    uint16_t ec[MA_MAX_AXES] = {0};
    uint16_t sec[MA_MAX_AXES] = {0};
    uint32_t di[MA_MAX_AXES] = {0};
    uint16_t tpst[MA_MAX_AXES] = {0};
    int32_t tpp[MA_MAX_AXES] = {0};
    int32_t tgt[MA_MAX_AXES] = {0};
    int32_t act[MA_MAX_AXES] = {0};
    // 只输出前3轴，避免buffer溢出，或者根据 buf_size 动态调整？
    // 这里保持原逻辑，但遍历 axis_count
    uint16_t n_print = h->axis_count > MA_MAX_SLAVES ? MA_MAX_SLAVES : h->axis_count; 

    for (uint16_t i = 0; i < n_print; ++i) {
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
 * 函数: motor_api_clear_error
 * 功能: 请求对指定轴（或全轴）执行 CiA-402 Fault Reset/清错序列。
 * 参数:
 * - handle: motor_api_create 创建的句柄
 * - axis_idx: 0-based 轴索引；-1 表示所有轴
 * 返回:
 * - MA_OK 成功；MA_ERR_PARAM 参数非法
 * 说明:
 * - 本函数只设置内部“清错请求计数”，真正的控制字脉冲在 motor_api_run_once 周期里执行。
 */
EXTERNFUNC ma_status_t motor_api_clear_error(struct motor_api_handle *handle, int axis_idx) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle; if (!h) return MA_ERR_PARAM;
    pthread_mutex_lock(&h->cmd_mutex);
    if (axis_idx == -1) {
        for (uint16_t i = 0; i < h->axis_count; ++i) {
            h->fault_reset_cycles[i] = 2;
            h->servo_enabled[i] = false;
            h->csp_warmup[i] = 10;
        }
        h->motion_started = 0;
        h->barrier_armed = 0;
    } else {
        if (axis_idx < 0 || axis_idx >= (int)h->axis_count) { 
            pthread_mutex_unlock(&h->cmd_mutex); 
            return MA_ERR_PARAM; 
        }
        h->fault_reset_cycles[axis_idx] = 2;
        h->servo_enabled[axis_idx] = false;
        h->csp_warmup[axis_idx] = 10;
    }
    pthread_mutex_unlock(&h->cmd_mutex);
    return MA_OK;
}

/*
 * motor_api_write_csp_defaults
 * 功能: 写入 CSP 模式与插补控制的默认值，确保周期内驱动处于期望控制模式。
 */
static inline void motor_api_write_csp_defaults(motor_api_handle_t *h, uint16_t axis_idx) {
    MA_WR_S8(h, h->out[axis_idx].workModeOut, (int8_t)MA_MODE_CSP);
    MA_WR_S8(h, h->out[axis_idx].interpolationCtrl, (int8_t)1);
}

/*
 * motor_api_snapshot_fault_reset_cycles
 * 功能: 读取 fault reset 请求计数的快照，避免在轴循环中反复加锁。
 */
static void motor_api_snapshot_fault_reset_cycles(motor_api_handle_t *h, uint8_t fr_cycles[MA_MAX_SLAVES]) {
    memset(fr_cycles, 0, sizeof(uint8_t) * MA_MAX_SLAVES);
    pthread_mutex_lock(&h->cmd_mutex);
    memcpy(fr_cycles, h->fault_reset_cycles, sizeof(h->fault_reset_cycles));
    pthread_mutex_unlock(&h->cmd_mutex);
}

/*
 * motor_api_axis_apply_fault_reset
 * 功能: 对单轴执行清错脉冲（先 0x0080，下一周期 0x0006）。
 * 返回: true 表示本周期已处理清错并短路该轴后续控制；false 表示无需清错。
 */
static bool motor_api_axis_apply_fault_reset(motor_api_handle_t *h, uint16_t axis_idx, uint8_t fr_cycle) {
    if (fr_cycle == 0) return false;
    int32_t ap = MA_RD_S32(h, h->in[axis_idx].actualPosition);
    h->csp_target[axis_idx] = ap;
    MA_WR_S32(h, h->out[axis_idx].targetPosition, ap);
    MA_WR_U16(h, h->out[axis_idx].controlWord, (fr_cycle == 2) ? 0x0080 : 0x0006);
    motor_api_write_csp_defaults(h, axis_idx);
    return true;
}

/*
 * motor_api_axis_step_enable
 * 功能: 未进入 servo_enabled 时，根据状态字推进 CiA-402 使能序列并写控制字。
 */
static void motor_api_axis_step_enable(motor_api_handle_t *h, uint16_t axis_idx, uint16_t status_i, int dbg_tick) {
    uint16_t control_i = 0x06;
    switch (status_i & 0x6F) {
        case 0x00: control_i = 0x06; break;
        case 0x40: control_i = 0x06; break;
        case 0x21:
            control_i = 0x07;
            h->csp_target[axis_idx] = MA_RD_S32(h, h->in[axis_idx].actualPosition);
            MA_WR_S32(h, h->out[axis_idx].targetPosition, h->csp_target[axis_idx]);
            break;
        case 0x23: control_i = 0x0F; break;
        case 0x27:
            control_i = 0x0F;
            if (!h->servo_enabled[axis_idx]) {
                h->servo_enabled[axis_idx] = true;
                if (dbg_tick % 100 == 0) {
                    int32_t ap = MA_RD_S32(h, h->in[axis_idx].actualPosition);
                    printf("[ENABLED%d] sw:0x%04X act:%d\n", axis_idx, status_i, ap);
                }
            }
            h->csp_warmup[axis_idx] = 10;
            h->csp_target[axis_idx] = MA_RD_S32(h, h->in[axis_idx].actualPosition);
            break;
        default: control_i = 0x06; break;
    }
    if ((status_i & 0x0040) && !(status_i & 0x0001)) {
        MA_WR_U16(h, h->out[axis_idx].controlWord, 0x0000);
        MA_WR_U16(h, h->out[axis_idx].controlWord, 0x0080);
    }
    MA_WR_U16(h, h->out[axis_idx].controlWord, control_i);
    motor_api_write_csp_defaults(h, axis_idx);
    if (dbg_tick % 500 == 0) {
        int ack = (status_i & 0x1000) ? 1 : 0;
        int trg = (status_i & 0x0400) ? 1 : 0;
        int32_t ap = MA_RD_S32(h, h->in[axis_idx].actualPosition);
        printf("[EN%d] sw:0x%04X ctrl:0x%04X mode:%d ack12:%d trg10:%d act:%d\n",
               axis_idx, status_i, control_i, MA_RD_S8(h, h->in[axis_idx].workModeIn), ack, trg, ap);
    }
}

/*
 * motor_api_axis_hold_before_motion
 * 功能: 栅栏未触发时保位到实际位置，避免统一起动前发生漂移。
 */
static void motor_api_axis_hold_before_motion(motor_api_handle_t *h, uint16_t axis_idx, uint16_t status_i, int dbg_tick) {
    h->csp_target[axis_idx] = MA_RD_S32(h, h->in[axis_idx].actualPosition);
    MA_WR_S32(h, h->out[axis_idx].targetPosition, h->csp_target[axis_idx]);
    MA_WR_U16(h, h->out[axis_idx].controlWord, 0x0F);
    motor_api_write_csp_defaults(h, axis_idx);
    h->last_actual_pos[axis_idx] = MA_RD_S32(h, h->in[axis_idx].actualPosition);
    if (dbg_tick % 100 == 0) {
        printf("[GATE%d] hold tgt:%d act:%d sw:0x%04X mode:%d\n", axis_idx,
               MA_RD_S32(h, h->out[axis_idx].targetPosition),
               h->last_actual_pos[axis_idx], status_i,
               MA_RD_S8(h, h->in[axis_idx].workModeIn));
    }
}

/*
 * motor_api_axis_run_motion
 * 功能: 栅栏触发后执行运动控制：读取指令、限幅、预热处理，并写入目标位置。
 */
static void motor_api_axis_run_motion(motor_api_handle_t *h, uint16_t axis_idx, uint16_t status_i, int dbg_tick) {
    pthread_mutex_lock(&h->cmd_mutex);
    bool run = h->axis_run[axis_idx] ? true : h->cmd_run;
    int dir  = h->axis_run[axis_idx] ? h->axis_dir[axis_idx] : h->cmd_dir;
    int step = h->axis_run[axis_idx] ? h->axis_step[axis_idx] : h->cmd_step;
    pthread_mutex_unlock(&h->cmd_mutex);

    int delta = run ? (int)(dir * step * h->axis_map[axis_idx].scale_pos) : 0;
    if (delta > MA_MAX_DELTA_PER_CYCLE) delta = MA_MAX_DELTA_PER_CYCLE;
    if (delta < -MA_MAX_DELTA_PER_CYCLE) delta = -MA_MAX_DELTA_PER_CYCLE;
    
    if (h->csp_warmup[axis_idx] > 0) { 
        h->csp_target[axis_idx] = MA_RD_S32(h, h->in[axis_idx].actualPosition); 
        h->csp_warmup[axis_idx]--; 
    } else { 
        h->csp_target[axis_idx] += delta; 
    }
    
    if (run && (dbg_tick % 100 == 0)) {
         printf("[DEBUG%d] run=%d dir=%d step=%d delta=%d warm=%d tgt=%d act=%d\n", 
                axis_idx, run, dir, step, delta, h->csp_warmup[axis_idx], h->csp_target[axis_idx],
                MA_RD_S32(h, h->in[axis_idx].actualPosition));
    }

    MA_WR_S32(h, h->out[axis_idx].targetPosition, h->csp_target[axis_idx]);
    MA_WR_U16(h, h->out[axis_idx].controlWord, 0x0F);
    motor_api_write_csp_defaults(h, axis_idx);
    h->last_actual_pos[axis_idx] = MA_RD_S32(h, h->in[axis_idx].actualPosition);
    if (dbg_tick % 500 == 0) {
        printf("[RUN%d] tgt:%d act:%d sw:0x%04X mode:%d\n", axis_idx,
               MA_RD_S32(h, h->out[axis_idx].targetPosition),
               h->last_actual_pos[axis_idx], status_i,
               MA_RD_S8(h, h->in[axis_idx].workModeIn));
    }
}

/*
 * motor_api_commit_fault_reset_cycles
 * 功能: 周期末提交并递减 fault reset 计数（只对已请求的轴递减）。
 */
static void motor_api_commit_fault_reset_cycles(motor_api_handle_t *h, const uint8_t fr_cycles[MA_MAX_AXES]) {
    pthread_mutex_lock(&h->cmd_mutex);
    for (uint16_t i = 0; i < h->axis_count; ++i) {
        if (fr_cycles[i] > 0) h->fault_reset_cycles[i] = (uint8_t)(fr_cycles[i] - 1);
    }
    pthread_mutex_unlock(&h->cmd_mutex);
}

/*
 * motor_api_update_barrier
 * 功能: 栅栏逻辑：全轴进入 enabled 后 ARM，延时到期后统一 FIRE 并标记 motion_started。
 */
static void motor_api_update_barrier(motor_api_handle_t *h) {
    int all_enabled = 1;
    // 只检查 CiA402 轴
    int cia_axes_count = 0;
    for (uint16_t i = 0; i < h->axis_count; ++i) {
        if (h->axis_map[i].type == MA_AXIS_TYPE_CIA402) {
            all_enabled = all_enabled && h->seen_enabled[i];
            cia_axes_count++;
        }
    }
    // 如果全是 IO 轴，直接标记 started? 或者不需要 barrier
    if (cia_axes_count == 0) {
        h->motion_started = 1; 
        return; 
    }

    if (h->motion_started) return;

    if (!h->barrier_armed && all_enabled) {
        h->barrier_armed = 1;
        h->barrier_start_ns = motor_api_monotonic_ns();
        printf("[BARRIER_ARM] all CiA402 axes at 0x027 (enabled), wait 1s\n");
    }
    if (!h->barrier_armed) return;

    uint64_t now = motor_api_monotonic_ns();
    if (now - h->barrier_start_ns < h->barrier_delay_ns) return;

    for (uint16_t i = 0; i < h->axis_count; ++i) {
        if (h->axis_map[i].type == MA_AXIS_TYPE_CIA402) {
            h->csp_target[i] = MA_RD_S32(h, h->in[i].actualPosition);
            MA_WR_S32(h, h->out[i].targetPosition, h->csp_target[i]);
            MA_WR_U16(h, h->out[i].controlWord, 0x0F);
            MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
        }
    }
    printf("[BARRIER_FIRE] synchronized motion start after 1s (enabled), axes=%u\n", h->axis_count);
    h->motion_started = 1;
    h->barrier_armed = 0;
}

/*
 * motor_api_cycle_begin
 * 功能: 一次周期开始的 EtherCAT 收包与状态快照更新。
 */
static void motor_api_cycle_begin(motor_api_handle_t *h) {
    ecrt_master_receive(h->master);
    ecrt_domain_process(h->domain);
    ecrt_master_sync_slave_clocks(h->master);
    check_domain_state(h);
    check_master_state(h);
    check_slave_states(h);
}

/*
 * motor_api_cycle_end
 * 功能: 一次周期结束的数据提交与发送。
 */
static void motor_api_cycle_end(motor_api_handle_t *h) {
    ecrt_domain_queue(h->domain);
    ecrt_master_send(h->master);
}

/*
 * motor_api_axis_process
 * 功能: 单轴周期控制：更新状态、处理清错、推进使能或运动控制。
 */
static void motor_api_axis_process(motor_api_handle_t *h, uint16_t axis_idx, uint8_t fr_cycle, int dbg_tick) {
    if (axis_idx >= h->axis_count) return;
    if (h->axis_map[axis_idx].type == MA_AXIS_TYPE_IO) {
        // IO 轴无需状态机，仅透传数据（已在 domain process 中完成）
        return;
    }

    uint16_t status_i = MA_RD_U16(h, h->in[axis_idx].statusword);
    h->seen_enabled[axis_idx] = (((status_i & 0x6F) == 0x27) ? true : false);

    if (motor_api_axis_apply_fault_reset(h, axis_idx, fr_cycle)) return;

    if (!h->servo_enabled[axis_idx]) {
        motor_api_axis_step_enable(h, axis_idx, status_i, dbg_tick);
        return;
    }

    h->time_cnt[axis_idx]++;
    if (!h->motion_started) {
        motor_api_axis_hold_before_motion(h, axis_idx, status_i, dbg_tick);
    } else {
        motor_api_axis_run_motion(h, axis_idx, status_i, dbg_tick);
    }
}

/*
 * 函数: motor_api_run_once
 * 功能: 周期性控制入口，包含状态机推进、目标更新、同步栅栏与调试输出。
 * 注意: 需以固定周期调用（例如 4ms）。
 */
EXTERNFUNC ma_status_t motor_api_run_once(struct motor_api_handle *handle) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle; if (!h) return MA_ERR_PARAM;
    ecrt_master_application_time(h->master, motor_api_monotonic_ns());
    motor_api_cycle_begin(h);
    static int dbg_tick = 0; dbg_tick++;
    uint8_t fr_cycles[MA_MAX_AXES];
    motor_api_snapshot_fault_reset_cycles(h, fr_cycles);
    for (uint16_t i = 0; i < h->axis_count; ++i) {
        motor_api_axis_process(h, i, fr_cycles[i], dbg_tick);
    }
    motor_api_commit_fault_reset_cycles(h, fr_cycles);
    motor_api_update_barrier(h);
    motor_api_cycle_end(h);
    return MA_OK;
}
