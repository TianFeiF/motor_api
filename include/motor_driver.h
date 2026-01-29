/*
 * motor_driver.h
 *
 * Abstract Device Driver Interface
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "ecrt.h"
#include "motor_api_types.h"

// Forward declaration
struct motor_api_handle;
typedef struct motor_api_handle motor_api_handle_t;

/*
 * Driver Context
 * Each device instance (slave) will have a context.
 * Drivers can cast this to their specific private struct.
 */
typedef void* ma_driver_ctx_t;

/*
 * Driver Interface
 */
typedef struct {
    const char *name;
    
    // Check if this driver supports the given device
    bool (*match)(uint32_t vendor_id, uint32_t product_code);
    
    // Configure PDOs and register entries
    // Returns: 0 on success, < 0 on error
    // If successful, it should also register any "Axes" to the handle
    int (*setup)(motor_api_handle_t *h, uint16_t slave_idx, ec_slave_config_t *sc);
    
    // Cycle Process: Read Inputs -> Update State -> Write Outputs
    // dbg_tick: for debug prints
    void (*process)(motor_api_handle_t *h, uint16_t slave_idx, uint8_t *pd, int dbg_tick);

} ma_driver_t;

// Registry
void ma_driver_register(const ma_driver_t *drv);
const ma_driver_t *ma_driver_find(uint32_t vid, uint32_t pid);

// Driver Declarations
extern const ma_driver_t drv_cia402;
extern const ma_driver_t drv_io;
extern const ma_driver_t drv_hans;

#endif // MOTOR_DRIVER_H
