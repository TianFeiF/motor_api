/*
 * ma_core.c
 * Core EtherCAT Master Management
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "motor_api_internal.h"
#include "motor_api_common.h"

// Static list of builtin drivers
static const ma_driver_t *builtin_drivers[] = {
    &drv_hans,    // Check specific first
    &drv_io,      // Then IO
    &drv_cia402,  // Fallback generic
    NULL
};

const ma_driver_t *ma_driver_find(uint32_t vid, uint32_t pid) {
    for (int i = 0; builtin_drivers[i]; ++i) {
        if (builtin_drivers[i]->match(vid, pid)) {
            return builtin_drivers[i];
        }
    }
    return NULL;
}

ma_status_t motor_api_create_base(const char *eni_path,
                                  uint32_t cycle_us,
                                  uint16_t *out_slave_count,
                                  struct motor_api_handle **out_handle,
                                  const ma_axis_map_t *axis_override, // Deprecated support
                                  int axis_override_count) {
    if (!out_handle || cycle_us == 0) return MA_ERR_PARAM;

    motor_api_handle_t *h = (motor_api_handle_t *)calloc(1, sizeof(*h));
    if (!h) return MA_ERR_RUNTIME;

    h->cycle_us = cycle_us;
    h->dc_sync0_period_ns = (uint64_t)cycle_us * 1000ULL;
    pthread_mutex_init(&h->cmd_mutex, NULL);
    h->barrier_delay_ns = 1000000000ULL; // 1s default

    // 1. Request Master
    h->master = ecrt_request_master(0);
    if (!h->master) {
        free(h);
        return MA_ERR_INIT;
    }

    // 2. Create Domain
    h->domain = ecrt_master_create_domain(h->master);
    if (!h->domain) {
        ecrt_release_master(h->master);
        free(h);
        return MA_ERR_INIT;
    }

    // 3. Parse ENI
    ma_eni_slave_t *eni_slaves = NULL;
    uint16_t eni_count = 0;
    
    // Note: We use the existing ENI parser, but we only need VIDs/PIDs to match drivers.
    // The actual PDO config usually comes from the Driver logic or ENI.
    // For this refactor, let's stick to reading ENI to get the list of slaves.
    if (eni_path) {
        ma_status_t rc = motor_api_read_eni(eni_path, h->vendor_id, h->product_code, h->position, 
                                          MA_MAX_SLAVES, &h->slave_count, &eni_slaves);
        if (rc != MA_OK) {
            ecrt_release_master(h->master);
            free(h);
            return rc;
        }
        eni_count = h->slave_count;
    } else {
        // Fallback or manual mode not fully supported in this snippet without ENI
        // Assuming ENI is provided for now.
    }

    // 4. Configure Slaves & Match Drivers
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        printf("[CORE] Configuring Slave %d (VID=0x%08X, PID=0x%08X)...\n", 
               i, h->vendor_id[i], h->product_code[i]);

        h->sc[i] = ecrt_master_slave_config(h->master, 0, h->position[i], 
                                          h->vendor_id[i], h->product_code[i]);
        if (!h->sc[i]) {
            fprintf(stderr, "Failed to get slave config for slave %d\n", i);
            goto err_cleanup;
        }

        // Apply Common SDOs (DC, etc)
        ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 1, (uint8_t)(h->cycle_us / 1000U)); // Interpolation time period
        ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 2, (uint8_t)-3); // 10^-3 seconds (ms) ?? No, usually -3 is milli? 
        // Wait, standard is 10^x. -3 means ms. 
        
        // Find Driver
        const ma_driver_t *drv = ma_driver_find(h->vendor_id[i], h->product_code[i]);
        if (drv) {
            printf("[CORE] -> Matched Driver: %s\n", drv->name);
            h->drivers[i] = drv;
            // Setup Driver (Register PDOs, Create Axes)
            if (drv->setup(h, i, h->sc[i]) != 0) {
                fprintf(stderr, "Driver setup failed for slave %d\n", i);
                goto err_cleanup;
            }
        } else {
            printf("[CORE] -> No specific driver found (Generic Mode)\n");
        }
    }

    // Free ENI memory
    if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, eni_count);

    // 5. Activate Master
    // Configure DC Reference
    if (h->slave_count > 0) {
        ecrt_master_select_reference_clock(h->master, h->sc[0]);
    }
    
    // Apply DC config to all slaves (Simplified)
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        ecrt_slave_config_dc(h->sc[i], 0x0300, h->dc_sync0_period_ns, 0, 0, 0);
    }

    if (ecrt_master_activate(h->master)) {
        fprintf(stderr, "Failed to activate master\n");
        goto err_cleanup_no_eni;
    }

    // 6. Get Domain Pointer
    h->domain_pd = ecrt_domain_data(h->domain);
    if (!h->domain_pd) {
        fprintf(stderr, "Failed to get domain data\n");
        goto err_cleanup_no_eni;
    }

    // 7. Success
    if (out_slave_count) *out_slave_count = h->slave_count;
    *out_handle = h;
    printf("[CORE] Master Activated. Total Axes: %d\n", h->axis_count);
    return MA_OK;

err_cleanup:
    if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, eni_count);
err_cleanup_no_eni:
    ecrt_release_master(h->master);
    free(h);
    return MA_ERR_INIT;
}

EXTERNFUNC ma_status_t motor_api_destroy(struct motor_api_handle *handle) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle;
    if (!h) return MA_ERR_PARAM;
    ecrt_release_master(h->master);
    pthread_mutex_destroy(&h->cmd_mutex);
    free(h);
    return MA_OK;
}

// Forward to base
EXTERNFUNC ma_status_t motor_api_create(const char *eni_path,
                                        uint32_t cycle_us,
                                        uint16_t *out_slave_count,
                                        struct motor_api_handle **out_handle) {
    return motor_api_create_base(eni_path, cycle_us, out_slave_count, out_handle, NULL, 0);
}
