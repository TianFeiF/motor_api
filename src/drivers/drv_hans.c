/*
 * drv_hans.c
 * Hans Robot Dual Axis Driver
 */

#include "motor_api_internal.h"
#include <stdio.h>

static bool hans_match(uint32_t vid, uint32_t pid) {
    return (vid == 0x0000001a && pid == 0x50440200);
}

static int hans_setup(motor_api_handle_t *h, uint16_t slave_idx, ec_slave_config_t *sc) {
    // This slave has 2 axes.
    if (h->axis_count + 2 > MA_MAX_AXES) return -1;

    // Axis 1
    int idx1 = h->axis_count++;
    ma_axis_data_t *ax1 = &h->axes[idx1];
    ax1->active = true;
    ax1->slave_idx = slave_idx;
    ax1->type = MA_AXIS_TYPE_CIA402;
    ax1->scale_pos = 1.0; 
    
    // Axis 2
    int idx2 = h->axis_count++;
    ma_axis_data_t *ax2 = &h->axes[idx2];
    ax2->active = true;
    ax2->slave_idx = slave_idx;
    ax2->type = MA_AXIS_TYPE_CIA402;
    ax2->scale_pos = 1.0;

    ec_pdo_entry_reg_t regs[] = {
        // Axis 1 (Standard 0x60xx)
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6040, 0, &ax1->out.controlWord},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6060, 0, &ax1->out.workModeOut},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x607A, 0, &ax1->out.targetPosition},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6041, 0, &ax1->in.statusword},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6064, 0, &ax1->in.actualPosition},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6061, 0, &ax1->in.workModeIn},
        
        // Axis 2 (Offset 0x68xx)
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6840, 0, &ax2->out.controlWord},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6860, 0, &ax2->out.workModeOut},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x687A, 0, &ax2->out.targetPosition},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6841, 0, &ax2->in.statusword},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6864, 0, &ax2->in.actualPosition},
        {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6861, 0, &ax2->in.workModeIn},
        {}
    };

    return ecrt_domain_reg_pdo_entry_list(h->domain, regs);
}

// Helper for single axis logic (Reuse from cia402 or duplicate?)
// For cleanliness, we duplicate the simple state machine or link to shared logic.
// Here we just duplicate for isolation.
static void run_axis_logic(motor_api_handle_t *h, ma_axis_data_t *ax, uint8_t *pd) {
    uint16_t status = MA_RD_U16(pd, ax->in.statusword);
    int32_t act_pos = MA_RD_S32(pd, ax->in.actualPosition);
    uint16_t ctrl = 0;
    int32_t tgt = ax->csp_target;

    // Report Enabled state
    h->seen_enabled[ax - h->axes] = ((status & 0x6F) == 0x27);

    if (!ax->servo_enabled) {
        if ((status & 0x6F) == 0x27) {
            ax->servo_enabled = true;
            ax->csp_target = act_pos;
            ctrl = 0x0F;
        } else if ((status & 0x4F) == 0x40) ctrl = 0x06;
        else if ((status & 0x6F) == 0x21) ctrl = 0x07;
        else if ((status & 0x6F) == 0x23) ctrl = 0x0F;
        else if ((status & 0x08)) ctrl = 0x80;
    } else {
        if (h->motion_started) {
             pthread_mutex_lock(&h->cmd_mutex);
             int dir = h->global_cmd_run ? h->global_cmd_dir : 0;
             int step = h->global_cmd_step;
             pthread_mutex_unlock(&h->cmd_mutex);
             // Simple motion
             if (ax->csp_warmup > 0) {
                 ax->csp_warmup--;
                 ax->csp_target = act_pos;
             } else {
                 ax->csp_target += (dir * step);
             }
             tgt = ax->csp_target;
        } else {
            tgt = act_pos;
            ax->csp_target = act_pos;
        }
        ctrl = 0x0F;
    }
    
    MA_WR_U16(pd, ax->out.controlWord, ctrl);
    MA_WR_S8(pd, ax->out.workModeOut, 8);
    MA_WR_S32(pd, ax->out.targetPosition, tgt);
}

static void hans_process(motor_api_handle_t *h, uint16_t slave_idx, uint8_t *pd, int dbg_tick) {
    // Find both axes
    for(int i=0; i<h->axis_count; ++i) {
        if (h->axes[i].slave_idx == slave_idx && h->axes[i].type == MA_AXIS_TYPE_CIA402) {
            run_axis_logic(h, &h->axes[i], pd);
        }
    }
}

const ma_driver_t drv_hans = {
    .name = "Hans Dual Axis",
    .match = hans_match,
    .setup = hans_setup,
    .process = hans_process
};
