#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include "motor_api.h"

static volatile sig_atomic_t stop = 0;
static void sig_handler(int s){ (void)s; stop = 1; }

int main(int argc, char **argv) {
    const char *eni = "motor_api/doc/HCFAX3E.xml";
    if (argc > 1) eni = argv[1];
    if (access(eni, R_OK) != 0) {
        const char *fallback1 = "../doc/HCFAX3E.xml";
        const char *fallback2 = "./doc/HCFAX3E.xml";
        if (access(fallback1, R_OK) == 0) {
            eni = fallback1;
        } else if (access(fallback2, R_OK) == 0) {
            eni = fallback2;
        } else {
            fprintf(stderr, "ENI not readable: tried '%s', '%s', '%s' (errno=%d)\n", "motor_api/doc/HCFAX3E.xml", fallback1, fallback2, errno);
        }
    }
    uint16_t cnt = 0; uint32_t vids[16] = {0}, prods[16] = {0}; uint16_t poss[16] = {0}; ma_eni_slave_t *slaves_info = NULL;
    uint32_t cycle_us = 4000;
    if (motor_api_read_eni(eni, vids, prods, poss, 16, &cnt, &slaves_info) == MA_OK && cnt > 0) {
        for (uint16_t i = 0; i < cnt; ++i) {
            if (prods[i] == 0x00002406) { cycle_us = 1000; break; }
        }
        motor_api_free_eni_slaves(slaves_info, cnt);
    }
    struct motor_api_handle *h = NULL; uint16_t slaves = 0;
    if (motor_api_create(eni, cycle_us, &slaves, &h) != MA_OK || !h) { fprintf(stderr, "motor_api_create failed\n"); return 1; }
    printf("motor_api created, slaves=%u, eni=%s, cycle_us=%u\n", slaves, eni, (unsigned)cycle_us);
    /* 已在创建流程打印了每个从站的PDO映射，这里仅提示 */
    printf("[INFO] PDO list printed above based on ENI parsing.\n");
    /* 固定500步长，正向运行 */
    motor_api_set_command(h, true, 1, 500);
    signal(SIGINT, sig_handler); signal(SIGTERM, sig_handler);
    while (!stop) { motor_api_run_once(h); usleep(cycle_us); }
    /* 退出前停止并销毁 */
    motor_api_set_command(h, false, 0, 0);
    motor_api_destroy(h);
    return 0;
}
