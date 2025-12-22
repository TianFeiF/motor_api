#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "motor_api.h"

int main(int argc, char *argv[]) {
    uint32_t timeout_ms = 5000;
    if (argc > 1) {
        long t = strtol(argv[1], NULL, 10);
        if (t > 0 && t < 600000) timeout_ms = (uint32_t)t;
    }

    struct motor_api_handle *handle = NULL;
    uint16_t slave_count = 0;
    bool config_valid = false;
    char product_names[16][64];
    memset(product_names, 0, sizeof(product_names));

    ma_status_t st = eth_initDLL(timeout_ms, &slave_count, product_names, 16, &config_valid, &handle);
    if (st != MA_OK) {
        printf("eth_initDLL failed: status=%d\n", st);
        return 1;
    }

    printf("eth_initDLL ok. slaves=%u config_valid=%s\n", slave_count, config_valid ? "true" : "false");
    for (uint16_t i = 0; i < slave_count && i < 16; ++i) {
        printf("  slave[%u] product='%s'\n", i, product_names[i][0] ? product_names[i] : "(unknown)");
    }

    if (handle) {
        motor_api_destroy(handle);
    }
    return 0;
}
