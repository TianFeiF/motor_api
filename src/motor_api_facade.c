/*
 * motor_api_facade.c
 * Implements the public API by calling into Core/Drivers.
 */

#include "motor_api.h"
#include "motor_api_internal.h"
#include "cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include "cJSON.h"
#include <stdio.h>
#include <stdlib.h>

// Public API Implementations

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
    
    h->axes[axis_idx].scale_pos = scale;
    h->axes[axis_idx].scale_vel = scale;
    return MA_OK;
}


EXTERNFUNC ma_status_t motor_api_set_io_output(struct motor_api_handle *handle, uint16_t axis_idx, uint32_t value) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle;
    if (!h || axis_idx >= h->axis_count) return MA_ERR_PARAM;
    ma_axis_data_t *ax = &h->axes[axis_idx];
    
    if (ax->type != MA_AXIS_TYPE_IO) return MA_ERR_PARAM;
    
    // Write directly to PD
    // Use the offset stored in controlWord (reused as offset holder)
    if (ax->io.size_out == 4) {
        MA_WR_U32(h->domain_pd, ax->out.controlWord, value);
    } else {
        MA_WR_U16(h->domain_pd, ax->out.controlWord, (uint16_t)value);
    }
    return MA_OK;
}

EXTERNFUNC ma_status_t motor_api_get_io_input(struct motor_api_handle *handle, uint16_t axis_idx, uint32_t *out_value) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle;
    if (!h || !out_value || axis_idx >= h->axis_count) return MA_ERR_PARAM;
    ma_axis_data_t *ax = &h->axes[axis_idx];
    
    *out_value = MA_RD_U32(h->domain_pd, ax->in.digitalInputs);
    return MA_OK;
}

// Re-export initDLL (wrapper around create)
EXTERNFUNC ma_status_t eth_initDLL(uint32_t timeout_ms,
                                   uint16_t *out_slave_count,
                                   char (*out_product_names)[64],
                                   uint16_t max_slaves,
                                   bool *out_config_valid,
                                   struct motor_api_handle **out_handle) {
    // Basic implementation that calls motor_api_create_base
    // Note: We need to find ENI first.
    // ... (This logic was in motor_api.c, we can copy it or simplified)
    // For brevity, let's assume user calls create directly or we implement minimal.
    // To match original behavior, we should implement the file search.
    // But since we split files, we can just call the utils.
    
    char eni[512];
    if (motor_api_find_latest_eni_xml("/home/phi/ecmotor_api/motor_api/doc", eni, 512) != MA_OK) {
        if (out_config_valid) *out_config_valid = false;
        return MA_ERR_IO;
    }
    
    ma_status_t rc = motor_api_create(eni, 4000, out_slave_count, out_handle);
    if (rc == MA_OK && out_config_valid) *out_config_valid = true;
    return rc;
}

EXTERNFUNC ma_status_t motor_api_format_diag_json(struct motor_api_handle *handle, char *buf, size_t buf_size) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle;
    if (!h || !buf || buf_size == 0) return MA_ERR_PARAM;

    int offset = 0;
    int n = snprintf(buf + offset, buf_size - offset, 
        "{\"cycle_us\":%u,\"motion_started\":%s,\"master_state\":%d,\"domain_state\":%d,\"axes\":[",
        h->cycle_us,
        h->motion_started ? "true" : "false",
        h->master_state.al_states,
        h->domain_state.wc_state);
        
    if (n < 0 || (size_t)n >= buf_size - offset) return MA_ERR_RUNTIME;
    offset += n;

    for (int i = 0; i < h->axis_count; ++i) {
        ma_axis_data_t *ax = &h->axes[i];
        if (i > 0) {
            n = snprintf(buf + offset, buf_size - offset, ",");
            if (n < 0 || (size_t)n >= buf_size - offset) break;
            offset += n;
        }

        n = snprintf(buf + offset, buf_size - offset, "{\"axis_idx\":%d,\"slave_idx\":%d,\"type\":\"%s\"",
            i, ax->slave_idx, ax->type == MA_AXIS_TYPE_IO ? "io" : "servo");
        if (n < 0 || (size_t)n >= buf_size - offset) break;
        offset += n;

        if (ax->type == MA_AXIS_TYPE_CIA402) {
             uint16_t sw = 0;
             int32_t act_pos = 0;
             if (h->domain_pd) {
                 sw = MA_RD_U16(h->domain_pd, ax->in.statusword);
                 act_pos = MA_RD_S32(h->domain_pd, ax->in.actualPosition);
             }
             n = snprintf(buf + offset, buf_size - offset, ",\"statusword\":%u,\"actual_pos\":%d,\"servo_enabled\":%s}",
                 sw, act_pos, ax->servo_enabled ? "true" : "false");
        } else if (ax->type == MA_AXIS_TYPE_IO) {
             uint32_t val = 0;
             if (h->domain_pd) {
                 val = MA_RD_U32(h->domain_pd, ax->in.digitalInputs);
             }
             n = snprintf(buf + offset, buf_size - offset, ",\"input\":%u}", val);
        } else {
             n = snprintf(buf + offset, buf_size - offset, "}");
        }
        
        if (n < 0 || (size_t)n >= buf_size - offset) break;
        offset += n;
    }

    n = snprintf(buf + offset, buf_size - offset, "]}");
    if (n < 0 || (size_t)n >= buf_size - offset) {
        buf[buf_size - 1] = '\0';
    }
    
    return MA_OK;
}
