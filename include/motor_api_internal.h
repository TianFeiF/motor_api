/*
 * motor_api_internal.h
 *
 * 说明：
 * - 仅供 motor_api 库内部 .c 文件使用的“内部头文件”；不安装、不对外保证 ABI 稳定；
 * - 聚合内部句柄结构体 motor_api_handle_t、PDO 偏移缓存、以及对域数据的读写封装；
 * - 该文件包含 ecrt.h（内部需要访问 IgH EtherCAT Master 的类型/读写宏）。
 *
 * 注意：外部用户请只包含 include/motor_api.h。
 */

#ifndef MOTOR_API_INTERNAL_H
#define MOTOR_API_INTERNAL_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <signal.h>
#include <limits.h>

#include "motor_api.h"
#include "ecrt.h"

/*
 * ma_output_offsets_t
 * 每个逻辑轴的“输出方向(Rx)”PDO entry 在 domain 数据区中的偏移（单位：字节）。
 * 说明：ecrt_domain_reg_pdo_entry_list 注册后，offset 会被填充为具体偏移；
 *      运行周期中通过偏移快速写入，避免字符串查表/重复注册开销。
 */
typedef struct {
    unsigned int controlWord;
    unsigned int workModeOut;
    unsigned int targetPosition;
    unsigned int touchProbeFunc;
    unsigned int interpolationCtrl;
} ma_output_offsets_t;

/*
 * ma_input_offsets_t
 * 每个逻辑轴的“输入方向(Tx)”PDO entry 在 domain 数据区中的偏移（单位：字节）。
 * 同 ma_output_offsets_t，用于周期中快速读取。
 */
typedef struct {
    unsigned int statusword;
    unsigned int workModeIn;
    unsigned int actualPosition;
    unsigned int actualVelocity;
    unsigned int actualTorque;
    unsigned int errorCode;
    unsigned int followingError;
    unsigned int digitalInputs;
    unsigned int touchProbeStatus;
    unsigned int touchProbePos;
    unsigned int servoErrorCode;
    unsigned int brakeDelay;
} ma_input_offsets_t;

/*
 * ma_axis_map_t
 * 描述一个逻辑轴与物理从站及对象字典基地址的映射关系。
 */
typedef struct {
    bool active;
    uint16_t slave_idx;     // 归属的物理从站索引
    ma_axis_type_t type;    // 轴类型（CiA402 / IO）
    uint16_t base_offset;   // 对象字典基地址偏移（如 0x6000 或 0x6800）
    double scale_pos;       // 位置比例因子（用户单位 -> 脉冲）
    double scale_vel;       // 速度比例因子
} ma_axis_map_t;

/*
 * motor_api_handle_t
 * 库内部句柄（对外通过 struct motor_api_handle 不透明指针暴露）。
 *
 * 字段分组说明：
 * - EtherCAT 资源：master/domain/sc[] 及其状态快照；
 * - 配置信息：从站数量、VID/PID/Position、周期与 DC 参数；
 * - 轴映射：axis_map[] / out[] / in[]；
 * - 网络线程：http_thread/http_port/stop；
 * - 指令与状态：互斥保护的全局/单轴命令，以及状态机/调试用缓存。
 */
typedef struct motor_api_handle {
    ec_master_t *master;
    ec_domain_t *domain;
    ec_master_state_t master_state;
    ec_domain_state_t domain_state;
    ec_slave_config_t *sc[MA_MAX_SLAVES];
    ec_slave_config_state_t sc_state[MA_MAX_SLAVES];
    uint8_t *domain_pd;

    uint16_t slave_count;
    uint32_t vendor_id[MA_MAX_SLAVES];
    uint32_t product_code[MA_MAX_SLAVES];
    uint16_t position[MA_MAX_SLAVES];

    uint16_t axis_count;
    ma_axis_map_t axis_map[MA_MAX_AXES];
    ma_output_offsets_t out[MA_MAX_AXES];
    ma_input_offsets_t in[MA_MAX_AXES];
    
    /* 运行状态 */
    bool servo_enabled[MA_MAX_AXES];
    bool motion_started;
    
    /* 调试/指令缓存 */
    bool cmd_run;
    int cmd_dir;
    int cmd_step;
    
    bool axis_run[MA_MAX_AXES];
    int axis_dir[MA_MAX_AXES];
    int axis_step[MA_MAX_AXES];

    /* 互斥锁 */
    pthread_mutex_t cmd_mutex;

    /* 内部状态机缓存 */
    int32_t last_actual_pos[MA_MAX_AXES];
    uint32_t time_cnt[MA_MAX_AXES];
    int32_t csp_target[MA_MAX_AXES];
    int csp_warmup[MA_MAX_AXES];
    
    /* 故障复位处理 */
    uint8_t fault_reset_cycles[MA_MAX_AXES];

    /* 栅栏同步 */
    int barrier_armed;
    uint64_t barrier_start_ns;
    uint64_t barrier_delay_ns;

    /* DC 参数 */
    uint32_t cycle_us;
    uint64_t dc_sync0_period_ns;
    
    pthread_t http_thread;
    int http_port;
    volatile sig_atomic_t stop;

    /* 诊断快照 */
    bool seen_enabled[MA_MAX_AXES];
} motor_api_handle_t;

/* 内部基础创建函数，支持轴映射覆盖 */
ma_status_t motor_api_create_base(const char *eni_path,
                                  uint32_t cycle_us,
                                  uint16_t *out_slave_count,
                                  struct motor_api_handle **out_handle,
                                  const ma_axis_map_t *axis_override,
                                  int axis_override_count);

/*
 * MA_RD_* / MA_WR_*
 * 功能：基于“域数据基址(domain_pd) + offset”进行 PDO 数据读写。
 * 细节：
 * - offset 为 UINT_MAX 表示该 entry 未注册/不存在，此时读返回 0，写直接忽略；
 * - EC_READ_* / EC_WRITE_* 宏由 ecrt.h 提供，负责按 EtherCAT 字节序访问。
 */
static inline uint16_t MA_RD_U16(motor_api_handle_t *h, unsigned int off) {
    return off != UINT_MAX ? EC_READ_U16(h->domain_pd + off) : 0;
}
static inline uint32_t MA_RD_U32(motor_api_handle_t *h, unsigned int off) {
    return off != UINT_MAX ? EC_READ_U32(h->domain_pd + off) : 0;
}
static inline int32_t MA_RD_S32(motor_api_handle_t *h, unsigned int off) {
    return off != UINT_MAX ? EC_READ_S32(h->domain_pd + off) : 0;
}
static inline int8_t MA_RD_S8(motor_api_handle_t *h, unsigned int off) {
    return off != UINT_MAX ? EC_READ_S8(h->domain_pd + off) : 0;
}
static inline void MA_WR_U16(motor_api_handle_t *h, unsigned int off, uint16_t v) {
    if (off != UINT_MAX) EC_WRITE_U16(h->domain_pd + off, v);
}
static inline void MA_WR_S32(motor_api_handle_t *h, unsigned int off, int32_t v) {
    if (off != UINT_MAX) EC_WRITE_S32(h->domain_pd + off, v);
}
static inline void MA_WR_S8(motor_api_handle_t *h, unsigned int off, int8_t v) {
    if (off != UINT_MAX) EC_WRITE_S8(h->domain_pd + off, v);
}

#endif
