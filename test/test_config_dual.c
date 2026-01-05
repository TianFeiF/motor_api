#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include "motor_api.h"
#include "motor_api_internal.h" // For accessing internal handle structure to verify mapping

static volatile sig_atomic_t g_stop = 0;
static void sig_handler(int s) { (void)s; g_stop = 1; }

int main(int argc, char **argv) {
    const char *cfg_path = "../config/config_dual.json";
    if (argc > 1) cfg_path = argv[1];
    
    printf("Loading dual-axis config from: %s\n", cfg_path);
    
    struct motor_api_handle *h = NULL;
    ma_status_t rc = motor_api_create_from_config(cfg_path, &h);
    if (rc != MA_OK) {
        fprintf(stderr, "Failed to create from config: %d\n", rc);
        return 1;
    }
    
    // Access internal structure to verify mapping
    motor_api_handle_t *ih = (motor_api_handle_t *)h;
    printf("Verifying Axis Mapping:\n");
    for (int i = 0; i < ih->axis_count; i++) {
        ma_axis_map_t *ax = &ih->axis_map[i];
        printf("  Axis %d: Slave=%d, Offset=0x%04X (%d), ScalePos=%.2f\n", 
               i, ax->slave_idx, ax->base_offset, ax->base_offset, ax->scale_pos);
    }

    /* 
     * Note: We expect:
     * Axis 0: Slave=0, Offset=0x0000
     * Axis 1: Slave=0, Offset=0x0800 (2048)
     * Axis 2: Slave=1, Offset=0x0000
     */

    printf("Motor API initialized. Running loop...\n");
    
    signal(SIGINT, sig_handler);
    
    /* Run loop */
    while (!g_stop) {
        motor_api_run_once(h);
        usleep(4000);
    }
    
    motor_api_destroy(h);
    return 0;
}
