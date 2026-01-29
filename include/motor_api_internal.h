/*
 * motor_api_internal.h
 *
 * Internal structures and macros.
 */

#ifndef MOTOR_API_INTERNAL_H
#define MOTOR_API_INTERNAL_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <signal.h>
#include <limits.h>

#include "motor_api.h"
#include "motor_driver.h" // Include the new driver interface
#include "ecrt.h"

#define MA_MAX_SLAVES 16
#define MA_MAX_AXES 32

/*
 * Per-Axis Runtime Data
 * This struct replaces the old monolithic arrays (out[], in[], servo_enabled[]...)
 * Each axis_idx maps to one of these.
 */
typedef struct {
    bool active;
    uint16_t slave_idx;     // Physical slave index
    ma_axis_type_t type;
    
    // Scaling
    double scale_pos;
    double scale_vel;
    
    // State Machine / Logic State
    bool servo_enabled;
    int32_t last_actual_pos;
    uint32_t time_cnt;
    int32_t csp_target;
    int csp_warmup;
    uint8_t fault_reset_cycles;
    
    // Command Buffer (from API/Net)
    bool cmd_run;
    int cmd_dir;
    int cmd_step;
    
    // PDO Offsets (Specific to driver type)
    // To keep it simple, we can use a union or a generic void* 
    // But since we are C, let's keep the big structs for now or move them to driver private?
    // For performance, having them here is fine.
    // Let's keep the generic "Offsets" concept but maybe flexible.
    // For now, to support legacy code migration, we keep the specific structs but 
    // drivers will fill them.
    
    struct {
        unsigned int controlWord;
        unsigned int workModeOut;
        unsigned int targetPosition;
        unsigned int touchProbeFunc;
        unsigned int interpolationCtrl;
    } out;

    struct {
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
    } in;
    
    // For IO driver
    struct {
        unsigned int output_offset; // RxPDO offset
        unsigned int input_offset;  // TxPDO offset
        uint8_t size_out;           // bytes
        uint8_t size_in;            // bytes
    } io;

} ma_axis_data_t;


/*
 * Main Handle
 */
typedef struct motor_api_handle {
    // EtherCAT Core
    ec_master_t *master;
    ec_domain_t *domain;
    ec_master_state_t master_state;
    ec_domain_state_t domain_state;
    ec_slave_config_t *sc[MA_MAX_SLAVES];
    ec_slave_config_state_t sc_state[MA_MAX_SLAVES];
    uint8_t *domain_pd;

    // Config
    uint16_t slave_count;
    uint32_t vendor_id[MA_MAX_SLAVES];
    uint32_t product_code[MA_MAX_SLAVES];
    uint16_t position[MA_MAX_SLAVES];
    
    // Drivers
    const ma_driver_t *drivers[MA_MAX_SLAVES];

    // Axes
    uint16_t axis_count;
    ma_axis_data_t axes[MA_MAX_AXES];

    // Global State
    bool motion_started;
    int barrier_armed;
    uint64_t barrier_start_ns;
    uint64_t barrier_delay_ns;
    bool seen_enabled[MA_MAX_AXES];

    // DC
    uint32_t cycle_us;
    uint64_t dc_sync0_period_ns;
    
    // Threading / Net
    pthread_mutex_t cmd_mutex;
    pthread_t http_thread;
    int http_port;
    volatile sig_atomic_t stop;
    
    // Global Command (Broadcast)
    bool global_cmd_run;
    int global_cmd_dir;
    int global_cmd_step;

} motor_api_handle_t;


// --- Helper Macros ---
static inline uint16_t MA_RD_U16(const uint8_t *base, unsigned int off) {
    return off != UINT_MAX ? EC_READ_U16(base + off) : 0;
}
static inline uint32_t MA_RD_U32(const uint8_t *base, unsigned int off) {
    return off != UINT_MAX ? EC_READ_U32(base + off) : 0;
}
static inline int32_t MA_RD_S32(const uint8_t *base, unsigned int off) {
    return off != UINT_MAX ? EC_READ_S32(base + off) : 0;
}
static inline int8_t MA_RD_S8(const uint8_t *base, unsigned int off) {
    return off != UINT_MAX ? EC_READ_S8(base + off) : 0;
}
static inline void MA_WR_U16(uint8_t *base, unsigned int off, uint16_t v) {
    if (off != UINT_MAX) EC_WRITE_U16(base + off, v);
}
static inline void MA_WR_U32(uint8_t *base, unsigned int off, uint32_t v) {
    if (off != UINT_MAX) EC_WRITE_U32(base + off, v);
}
static inline void MA_WR_S32(uint8_t *base, unsigned int off, int32_t v) {
    if (off != UINT_MAX) EC_WRITE_S32(base + off, v);
}
static inline void MA_WR_S8(uint8_t *base, unsigned int off, int8_t v) {
    if (off != UINT_MAX) EC_WRITE_S8(base + off, v);
}

// --- Core Internal Functions ---
ma_status_t motor_api_create_base(const char *eni_path,
                                  uint32_t cycle_us,
                                  uint16_t *out_slave_count,
                                  struct motor_api_handle **out_handle,
                                  const ma_axis_map_t *axis_override,
                                  int axis_override_count);
void ma_core_register_driver(const ma_driver_t *drv);

#endif // MOTOR_API_INTERNAL_H
