#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include "motor_api.h"

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
    
    /* Run loop */
    while (!g_stop) {
        motor_api_run_once(h);
        usleep(4000);
    }
    
    motor_api_destroy(h);
    return 0;
}
