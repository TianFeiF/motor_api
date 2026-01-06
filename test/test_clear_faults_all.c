#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>

#include "motor_api.h"
#include "motor_api_internal.h"

static volatile sig_atomic_t g_stop = 0;

static void sig_handler(int sig) {
    (void)sig;
    g_stop = 1;
}

static void print_axis_diag(motor_api_handle_t *h) {
    for (uint16_t i = 0; i < h->axis_count; ++i) {
        uint16_t sw = MA_RD_U16(h, h->in[i].statusword);
        uint16_t ec = MA_RD_U16(h, h->in[i].errorCode);
        uint16_t sec = MA_RD_U16(h, h->in[i].servoErrorCode);
        printf("axis=%u sw=0x%04X err=0x%04X servoErr=0x%04X\n", i, sw, ec, sec);
    }
}

int main(int argc, char **argv) {
    const char *cfg_path = "../config/config.json";
    int duration_s = 5;
    if (argc > 1) cfg_path = argv[1];
    if (argc > 2) duration_s = atoi(argv[2]);
    if (duration_s < 1) duration_s = 1;

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    struct motor_api_handle *handle = NULL;
    ma_status_t st = motor_api_create_from_config(cfg_path, &handle);
    if (st != MA_OK || !handle) {
        fprintf(stderr, "motor_api_create_from_config failed: %d\n", st);
        return 1;
    }

    motor_api_handle_t *h = (motor_api_handle_t *)handle;
    if (h->cycle_us == 0) h->cycle_us = 4000;

    printf("axis_count=%u cycle_us=%u\n", h->axis_count, h->cycle_us);
    printf("before clear:\n");
    print_axis_diag(h);

    st = motor_api_clear_error(handle, -1);
    if (st != MA_OK) {
        fprintf(stderr, "motor_api_clear_error(-1) failed: %d\n", st);
        motor_api_destroy(handle);
        return 1;
    }

    uint64_t loops = (uint64_t)duration_s * 1000000ULL / (uint64_t)h->cycle_us;
    if (loops < 1) loops = 1;
    uint64_t print_every = 1000000ULL / (uint64_t)h->cycle_us;
    if (print_every < 1) print_every = 1;

    for (uint64_t i = 0; i < loops && !g_stop; ++i) {
        motor_api_run_once(handle);
        if ((i % print_every) == 0) {
            printf("t=%.1fs\n", (double)i / (double)print_every);
            print_axis_diag(h);
        }
        usleep(h->cycle_us);
    }

    printf("after clear:\n");
    print_axis_diag(h);

    motor_api_destroy(handle);
    return 0;
}

