#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include "motor_api.h"
#include "motor_api_internal.h" // For internal struct access

static volatile sig_atomic_t g_stop = 0;
static void sig_handler(int s) { (void)s; g_stop = 1; }

int main(int argc, char **argv) {
    const char *cfg_path = "../config/config.json";
    if (argc > 1) cfg_path = argv[1];
    
    printf("Loading config from: %s\n", cfg_path);
    
    struct motor_api_handle *h = NULL;
    ma_status_t rc = motor_api_create_from_config(cfg_path, &h);
    if (rc != MA_OK) {
        fprintf(stderr, "Failed to create from config: %d\n", rc);
        return 1;
    }
    
    printf("Motor API initialized from config.\n");
    
    signal(SIGINT, sig_handler);
    
    motor_api_handle_t *ih = (motor_api_handle_t *)h;
    int tick = 0;
    uint32_t io_val = 0;

    /* Run loop */
    while (!g_stop) {
        motor_api_run_once(h);
        
        /* Every 2s (500 * 4ms = 2000ms) */
        if (tick % 500 == 0) {
            io_val = (io_val == 0) ? 0xFFFFFFFF : 0;
            printf("\n--- 2s Tick: Toggling IO to 0x%08X ---\n", io_val);
            
            for (int i = 0; i < ih->axis_count; ++i) {
                ma_axis_map_t *ax = &ih->axis_map[i];
                if (ax->type == MA_AXIS_TYPE_IO) {
                    /* Set Output (RxPDO) */
                    motor_api_set_io_output(h, i, io_val);
                    
                    /* Read Input (TxPDO) */
                    uint32_t in_val = 0;
                    motor_api_get_io_input(h, i, &in_val);
                    
                    printf("[Axis %d IO] RxPDO(Out): 0x%08X | TxPDO(In): 0x%08X\n", i, io_val, in_val);
                } else {
                    /* For Servo, print key RxPDO values (ControlWord, TargetPosition) */
                    /* Note: MA_RD_* macros read from domain memory, representing what is queued/sent */
                    uint16_t ctrl_wd = MA_RD_U16(ih, ih->out[i].controlWord);
                    int32_t tgt_pos = MA_RD_S32(ih, ih->out[i].targetPosition);
                    int8_t mode = MA_RD_S8(ih, ih->out[i].workModeOut);
                    
                    printf("[Axis %d Servo] RxPDO: Ctrl=0x%04X Mode=%d Tgt=%d\n", i, ctrl_wd, mode, tgt_pos);
                }
            }
        }

        usleep(4000);
        tick++;
    }
    
    motor_api_destroy(h);
    return 0;
}
