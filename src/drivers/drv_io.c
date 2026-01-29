/*
 * drv_io.c
 * Generic IO Driver
 */

#include "motor_api_internal.h"
#include "motor_api_common.h" // For ENI helpers
#include <stdio.h>

static bool io_match(uint32_t vid, uint32_t pid) {
    if (vid == 0x00000025) return true; // Inexbot
    if (vid == 0x00201911) return true; // F2838x
    return false;
}

static int io_setup(motor_api_handle_t *h, uint16_t slave_idx, ec_slave_config_t *sc) {
    // This is the tricky part: Dynamic PDO mapping based on ENI?
    // Or we rely on the logic we just fixed in the old code.
    // For now, let's map *everything* we find in 0x7000 range as an "IO Axis"?
    // OR, we just map specific known offsets?
    
    // Strategy: 
    // We register one "IO Axis" per Output PDO entry found in ENI?
    // Or just one Axis representing the whole board?
    // The previous code mapped *every* output entry as an axis.
    
    // We need access to ENI data here. But core has freed it? 
    // Ah, `motor_api_create_base` freed ENI before calling setup? 
    // No, I moved free after setup loop in `ma_core.c`. Wait, let me check.
    // In `ma_core.c`: `if (eni_slaves) motor_api_free_eni_slaves` is AFTER the loop. Good.
    
    // We need to re-parse ENI locally? Or pass ENI info to setup?
    // `ma_driver_t` setup interface currently only gets `sc`.
    // We might need to extend `setup` to take `ma_eni_slave_t*` if available.
    
    // Workaround: We define a static list of "Known IO Layouts" or we just scan blindly.
    // But `ecrt_domain_reg_pdo_entry_list` needs specific Index/Subindex.
    
    // Let's implement the "Scan and Register" logic we added to motor_api.c recently.
    // But since we don't have the ENI pointer passed in, we can't iterate PDOs easily.
    
    // Temporary Fix: Hardcode Inexbot Layout for now, or assume 0x7000:01..09
    // To make it generic later, we should pass ENI metadata to driver setup.
    
    // Let's create axes 0..8 for Inexbot
    if (h->vendor_id[slave_idx] == 0x00000025) { // Inexbot
        for (int k=1; k<=9; ++k) {
            if (h->axis_count >= MA_MAX_AXES) break;
            int aidx = h->axis_count++;
            ma_axis_data_t *ax = &h->axes[aidx];
            ax->active = true;
            ax->slave_idx = slave_idx;
            ax->type = MA_AXIS_TYPE_IO;
            
            // Map Output
            ec_pdo_entry_reg_t regs[] = {
                {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x7000, (uint8_t)k, &ax->out.controlWord}, // Use controlWord as container
                {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x6000, 0, &ax->in.digitalInputs},
                {}
            };
            // Note: io_size_out needs to be known. 
            // 1,2,5,9 are 32bit. 3,4,6,7,8 are 16bit.
            if (k==1 || k==2 || k==5 || k==9) ax->io.size_out = 4;
            else ax->io.size_out = 2;
            
            if (ecrt_domain_reg_pdo_entry_list(h->domain, regs)) return -1;
        }
    } else {
        // Generic fallback (F28 etc): Map 0x7001:01
        int aidx = h->axis_count++;
        ma_axis_data_t *ax = &h->axes[aidx];
        ax->active = true;
        ax->slave_idx = slave_idx;
        ax->type = MA_AXIS_TYPE_IO;
        ax->io.size_out = 2;
        
        ec_pdo_entry_reg_t regs[] = {
            {0, 0, h->vendor_id[slave_idx], h->product_code[slave_idx], 0x7001, 1, &ax->out.controlWord}, 
            {}
        };
        ecrt_domain_reg_pdo_entry_list(h->domain, regs);
    }
    
    return 0;
}

static void io_process(motor_api_handle_t *h, uint16_t slave_idx, uint8_t *pd, int dbg_tick) {
    for(int i=0; i<h->axis_count; ++i) {
        if (h->axes[i].slave_idx == slave_idx && h->axes[i].type == MA_AXIS_TYPE_IO) {
            ma_axis_data_t *ax = &h->axes[i];
            
            // Output Logic: Use targetPosition or controlWord as value container?
            // API uses `motor_api_set_io_output` which likely sets `ax->out.controlWord` offset content?
            // No, the API writes to the *mapped memory*.
            // Wait, the API `motor_api_set_io_output` needs to know *where* to write.
            // In the new core, we need to provide a way to set IO.
            // The API implementation of `set_io_output` will look up `ax->out.controlWord` offset.
            
            // So here in `process`, we don't strictly need to do anything if the API writes directly to PD memory!
            // BUT, if we want to support "persistent state" or "logic", we might.
            // For raw IO, pass-through is fine.
            
            // However, `motor_api_set_io_output` writes to `h->domain_pd + offset`.
            // So `io_process` does nothing.
        }
    }
}

const ma_driver_t drv_io = {
    .name = "Generic IO",
    .match = io_match,
    .setup = io_setup,
    .process = io_process
};
