/*
 * ma_cycle.c
 * Cyclic Execution Logic
 */

#include <stdio.h>
#include "motor_api_internal.h"
#include "motor_api_common.h"

static void check_domain_state(motor_api_handle_t *h) {
    ec_domain_state_t ds;
    ecrt_domain_state(h->domain, &ds);
    h->domain_state = ds;
}

static void check_master_state(motor_api_handle_t *h) {
    ec_master_state_t ms;
    ecrt_master_state(h->master, &ms);
    h->master_state = ms;
}

EXTERNFUNC ma_status_t motor_api_run_once(struct motor_api_handle *handle) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle;
    if (!h) return MA_ERR_PARAM;

    // 1. Receive
    ecrt_master_application_time(h->master, motor_api_monotonic_ns());
    ecrt_master_receive(h->master);
    ecrt_domain_process(h->domain);
    
    // Sync logic
    check_domain_state(h);
    check_master_state(h); // Should be called periodically, not necessarily every cycle to save CPU? 
                           // For now keep it.

    static int dbg_tick = 0;
    dbg_tick++;

    // 2. Process Drivers (Slave Level)
    // In this new model, we iterate slaves and let drivers handle their axes.
    // However, the legacy logic iterated *Axes*.
    // Let's iterate Slaves for IO/Status updates, but we might need an Axis loop for Motion Planning?
    // Actually, the driver `process` can iterate its owned axes.
    
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        const ma_driver_t *drv = h->drivers[i];
        if (drv && drv->process) {
            drv->process(h, i, h->domain_pd, dbg_tick);
        }
    }

    // 3. Global Barrier Logic (Simple version)
    if (!h->motion_started) {
        bool all_ready = true;
        for (int i=0; i < h->axis_count; ++i) {
            if (h->axes[i].type == MA_AXIS_TYPE_CIA402) {
                if (!h->seen_enabled[i]) all_ready = false;
            }
        }
        
        if (all_ready && !h->barrier_armed) {
            h->barrier_armed = 1;
            h->barrier_start_ns = motor_api_monotonic_ns();
            printf("[BARRIER] All axes enabled. Waiting...\n");
        }
        
        if (h->barrier_armed) {
            if (motor_api_monotonic_ns() - h->barrier_start_ns > h->barrier_delay_ns) {
                h->motion_started = true;
                printf("[BARRIER] Motion Started!\n");
            }
        }
    }

    // 4. Send
    ecrt_domain_queue(h->domain);
    ecrt_master_send(h->master);
    
    return MA_OK;
}
