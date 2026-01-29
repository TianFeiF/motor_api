/*
 * drv_cia402.c
 * Standard Servo Driver
 */

#include "motor_api_internal.h"
#include <stdio.h>

#define MA_MAX_DELTA_PER_CYCLE 400000

static bool cia402_match(uint32_t vid, uint32_t pid) {
    // Basic heuristic: Accept generic or known servo IDs
    // For now, accept everything that ISN'T an IO or Hans? 
    // Or strictly match known servos.
    // Let's match HCFA
    if (vid == 0x000116c7) return true;
    return false;
}

static int cia402_setup(motor_api_handle_t *h, uint16_t slave_idx, ec_slave_config_t *sc) {
    if (h->axis_count >= MA_MAX_AXES) return -1;
    
    int axis_idx = h->axis_count++;
    ma_axis_data_t *ax = &h->axes[axis_idx];
    
    ax->active = true;
    ax->slave_idx = slave_idx;
    ax->type = MA_AXIS_TYPE_CIA402;
    ax->scale_pos = 1.0;
    ax->scale_vel = 1.0;
    ax->csp_warmup = 10;

    // Register PDOs (Standard mapping)
    // Note: In a real "Data Driven" approach, we would use a struct array.
    // Here we use the functional registration for simplicity in C.
    
    ec_pdo_entry_reg_t regs[] = {
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6040, 0, &ax->out.controlWord},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6060, 0, &ax->out.workModeOut},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x607A, 0, &ax->out.targetPosition},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6041, 0, &ax->in.statusword},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6064, 0, &ax->in.actualPosition},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6061, 0, &ax->in.workModeIn},
        {}
    };
    
    if (ecrt_domain_reg_pdo_entry_list(h->domain, regs)) {
        return -1;
    }
    
    // Configure default SDOs if needed? (Core already did DC)
    return 0;
}

static void cia402_process(motor_api_handle_t *h, uint16_t slave_idx, uint8_t *pd, int dbg_tick) {
    // Find the axis associated with this slave
    // (Inefficient O(N) but N is small)
    ma_axis_data_t *ax = NULL;
    int ax_idx = -1;
    for(int i=0; i<h->axis_count; ++i) {
        if (h->axes[i].slave_idx == slave_idx && h->axes[i].type == MA_AXIS_TYPE_CIA402) {
            ax = &h->axes[i];
            ax_idx = i;
            break;
        }
    }
    if (!ax) return;

    // READ Inputs
    uint16_t status = MA_RD_U16(pd, ax->in.statusword);
    int32_t act_pos = MA_RD_S32(pd, ax->in.actualPosition);
    
    // Update global "Seen Enabled"
    h->seen_enabled[ax_idx] = ((status & 0x6F) == 0x27);

    // Logic
    uint16_t ctrl = 0;
    int32_t tgt_pos = ax->csp_target;

    if (!ax->servo_enabled) {
        // Enable Sequence
        if ((status & 0x6F) == 0x27) {
            ax->servo_enabled = true;
            ax->csp_target = act_pos;
            tgt_pos = act_pos;
            ctrl = 0x0F;
            printf("[CiA402] Axis %d Enabled at %d\n", ax_idx, act_pos);
        } else if ((status & 0x4F) == 0x40) { // Switch on disabled
            ctrl = 0x06;
        } else if ((status & 0x6F) == 0x21) { // Ready to switch on
            ctrl = 0x07;
        } else if ((status & 0x6F) == 0x23) { // Switched on
            ctrl = 0x0F;
        } else if ((status & 0x08)) { // Fault
            ctrl = 0x80; // Reset
        }
    } else {
        // Motion
        if (h->motion_started) {
            // Apply Motion Command
            pthread_mutex_lock(&h->cmd_mutex);
            int dir = h->global_cmd_run ? h->global_cmd_dir : 0;
            int step = h->global_cmd_step;
            pthread_mutex_unlock(&h->cmd_mutex);
            
            int delta = (int)(dir * step * ax->scale_pos);
            if (delta > MA_MAX_DELTA_PER_CYCLE) delta = MA_MAX_DELTA_PER_CYCLE;
            if (delta < -MA_MAX_DELTA_PER_CYCLE) delta = -MA_MAX_DELTA_PER_CYCLE;
            
            if (ax->csp_warmup > 0) {
                ax->csp_warmup--;
                ax->csp_target = act_pos; // Sync
            } else {
                ax->csp_target += delta;
            }
            tgt_pos = ax->csp_target;
        } else {
            // Hold
            ax->csp_target = act_pos;
            tgt_pos = act_pos;
        }
        ctrl = 0x0F;
    }

    // WRITE Outputs
    MA_WR_U16(pd, ax->out.controlWord, ctrl);
    MA_WR_S8(pd, ax->out.workModeOut, 8); // CSP
    MA_WR_S32(pd, ax->out.targetPosition, tgt_pos);
}

const ma_driver_t drv_cia402 = {
    .name = "CiA402 Standard",
    .match = cia402_match,
    .setup = cia402_setup,
    .process = cia402_process
};
