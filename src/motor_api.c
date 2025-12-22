/*
 * 版权所有 (C) 2025 phi
 * 文件名称: motor_api.c
 * 版本信息: v1.0.0
 * 文件说明: 通用电机控制库实现文件，封装 EtherCAT 主站生命周期、ENI 解析、
 *           DC 同步、CiA-402 状态机、CSP/CSV 运行、HTTP 控制与诊断等逻辑。
 * 模块关系: 与头文件 motor_api.h 配套；示例程序 example_csp.c 调用本模块 API。
 * 修改历史:
 *   - 2025-11-28: 初始实现，支持 ENI 读取、PDO 注册、DC 配置、HTTP 服务。
 *   - 2025-11-28: 增加“全轴使能(0x27)后延时 1s 同步起动”的栅栏机制与调试输出。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <ctype.h>
#include <time.h>
#include <pthread.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <dirent.h>
#include <sys/stat.h>
#include <limits.h>

#include "motor_api.h"
#include "ecrt.h"

#define MA_MAX_SLAVES 16
#define MA_MAX_DELTA_PER_CYCLE 400000

/*
 * 结构: ma_output_offsets_t
 * 功能: 保存每个从站的输出 PDO 偏移（域内地址），用于高效写入。
 * 字段:
 *   - controlWord: 0x6040 控制字
 *   - workModeOut: 0x6060 操作模式输出
 *   - targetPosition: 0x607A 目标位置
 *   - touchProbeFunc: 0x60B8 触发探针功能
 */
typedef struct {
    unsigned int controlWord;
    unsigned int workModeOut;
    unsigned int targetPosition;
    unsigned int touchProbeFunc;
    unsigned int interpolationCtrl;
} ma_output_offsets_t;

/*
 * 结构: ma_input_offsets_t
 * 功能: 保存每个从站的输入 PDO 偏移（域内地址），用于高效读取。
 * 字段:
 *   - statusword: 0x6041 状态字
 *   - workModeIn: 0x6061 操作模式输入
 *   - actualPosition: 0x6064 实际位置
 *   - errorCode: 0x603F 错误码
 *   - followingError: 0x60F4 跟随误差
 *   - digitalInputs: 0x60FD 数字量输入
 *   - touchProbeStatus: 0x60B9 探针状态
 *   - touchProbePos: 0x60BA 探针位置
 *   - servoErrorCode: 0x213F 伺服错误码（厂商自定义对象）
 */
typedef struct {
    unsigned int statusword;
    unsigned int workModeIn;
    unsigned int actualPosition;
    unsigned int actualVelocity;
    unsigned int actualTorque;
    unsigned int errorCode;
    unsigned int followingError;
    unsigned int digitalInputs;
    unsigned int touchProbeStatus;
    unsigned int touchProbePos;
    unsigned int servoErrorCode;
    unsigned int brakeDelay;
} ma_input_offsets_t;

/*
 * 结构: motor_api_handle
 * 功能: 库内部句柄，封装主站/域/从站配置、周期控制状态、命令与调试信息。
 */
typedef struct motor_api_handle {
    ec_master_t *master;
    ec_domain_t *domain;
    ec_master_state_t master_state;
    ec_domain_state_t domain_state;
    ec_slave_config_t *sc[MA_MAX_SLAVES];
    ec_slave_config_state_t sc_state[MA_MAX_SLAVES];
    uint8_t *domain_pd;

    uint16_t slave_count;
    uint32_t vendor_id[MA_MAX_SLAVES];
    uint32_t product_code[MA_MAX_SLAVES];
    uint16_t position[MA_MAX_SLAVES];

    uint32_t cycle_us;
    uint64_t dc_sync0_period_ns;

    ma_output_offsets_t out[MA_MAX_SLAVES];
    ma_input_offsets_t in[MA_MAX_SLAVES];

    pthread_t http_thread;
    int http_port;
    volatile sig_atomic_t stop;

    pthread_mutex_t cmd_mutex; /* 命令互斥，保护 run/dir/step */
    bool cmd_run;              /* 运行标志 */
    int cmd_dir;               /* 方向：-1/0/1 */
    int cmd_step;              /* 步长/速度（内部限制范围） */

    int32_t last_actual_pos[MA_MAX_SLAVES]; /* 上次实际位置快照 */
    uint32_t time_cnt[MA_MAX_SLAVES];       /* 轴内时间计数（调试/预热） */
    bool servo_enabled[MA_MAX_SLAVES];      /* 轴使能标志（到达 0x27 后置位） */
    int csp_warmup[MA_MAX_SLAVES];          /* CSP 预热计数，避免首次跳变 */
    int32_t csp_target[MA_MAX_SLAVES];      /* CSP 目标位置 */
    bool seen_enabled[MA_MAX_SLAVES];       /* 观察到 0x27（enabled） */
    int barrier_armed;                      /* 延迟栅栏已武装 */
    uint64_t barrier_start_ns;              /* 延迟起始时间 */
    uint64_t barrier_delay_ns;              /* 延迟时长（ns） */
    int motion_started;                     /* 延迟结束后开始运动 */
} motor_api_handle_t;

/*
 * 函数: monotonic_ns
 * 功能: 获取单调时钟当前时间（纳秒），用于 DC 同步与延时栅栏计时。
 */
static uint64_t monotonic_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/*
 * 函数: strncasestr
 * 功能: 在限定长度的缓冲区中进行不区分大小写的子串查找。
 */
static const char *strncasestr(const char *hay, size_t hay_len, const char *needle) {
    size_t nlen = strlen(needle);
    if (nlen == 0 || hay_len < nlen) return NULL;
    for (size_t i = 0; i + nlen <= hay_len; ++i) {
        size_t j = 0;
        for (; j < nlen; ++j) {
            char a = (char)tolower((unsigned char)hay[i + j]);
            char b = (char)tolower((unsigned char)needle[j]);
            if (a != b) break;
        }
        if (j == nlen) return hay + i;
    }
    return NULL;
}

/* 统一十六进制/十进制解析（支持 #x...、0x...、x... 以及纯数字） */
static inline int parse_hex_or_dec(const char *s) {
    const char *p = s;
    while (*p == ' ' || *p == '"') ++p;
    if (*p == '#') {
        ++p;
        if (*p == 'x' || *p == 'X') ++p;
        return (int)strtol(p, NULL, 16);
    }
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) return (int)strtol(p + 2, NULL, 16);
    if (p[0] == 'x' || p[0] == 'X') return (int)strtol(p + 1, NULL, 16);
    return (int)strtol(p, NULL, 0);
}

static inline uint16_t MA_RD_U16(motor_api_handle_t *h, unsigned int off) {
    return off != UINT_MAX ? EC_READ_U16(h->domain_pd + off) : 0;
}
static inline uint32_t MA_RD_U32(motor_api_handle_t *h, unsigned int off) {
    return off != UINT_MAX ? EC_READ_U32(h->domain_pd + off) : 0;
}
static inline int32_t MA_RD_S32(motor_api_handle_t *h, unsigned int off) {
    return off != UINT_MAX ? EC_READ_S32(h->domain_pd + off) : 0;
}
static inline int8_t MA_RD_S8(motor_api_handle_t *h, unsigned int off) {
    return off != UINT_MAX ? EC_READ_S8(h->domain_pd + off) : 0;
}
static inline void MA_WR_U16(motor_api_handle_t *h, unsigned int off, uint16_t v) {
    if (off != UINT_MAX) EC_WRITE_U16(h->domain_pd + off, v);
}
static inline void MA_WR_S32(motor_api_handle_t *h, unsigned int off, int32_t v) {
    if (off != UINT_MAX) EC_WRITE_S32(h->domain_pd + off, v);
}
static inline void MA_WR_S8(motor_api_handle_t *h, unsigned int off, int8_t v) {
    if (off != UINT_MAX) EC_WRITE_S8(h->domain_pd + off, v);
}

static int eni_has_rx_entry(const ma_eni_slave_t *s, uint16_t index, uint8_t subindex) {
    if (!s) return 0;
    for (unsigned int p = 0; p < s->rx_pdo_count; ++p) {
        for (unsigned int e = 0; e < s->rx_pdos[p].entry_count; ++e) {
            if (s->rx_pdos[p].entries[e].index == index && s->rx_pdos[p].entries[e].subindex == subindex) {
                return 1;
            }
        }
    }
    return 0;
}
static int eni_has_tx_entry(const ma_eni_slave_t *s, uint16_t index, uint8_t subindex) {
    if (!s) return 0;
    for (unsigned int p = 0; p < s->tx_pdo_count; ++p) {
        for (unsigned int e = 0; e < s->tx_pdos[p].entry_count; ++e) {
            if (s->tx_pdos[p].entries[e].index == index && s->tx_pdos[p].entries[e].subindex == subindex) {
                return 1;
            }
        }
    }
    return 0;
}

static int parse_attr_int_range(const char *beg, const char *end, const char *key) {
    if (!beg || !end || !key) return -1;
    const char *k = strncasestr(beg, (size_t)(end - beg), key);
    if (!k || k >= end) return -1;
    const char *eq = strchr(k, '=');
    if (!eq || eq >= end) return -1;
    const char *v = eq + 1;
    while (v < end && (*v == ' ' || *v == '"')) ++v;
    const char *stop = v;
    while (stop < end && *stop != ' ' && *stop != '>' && *stop != '"' && *stop != '/') ++stop;
    char bufv[64];
    size_t n = (size_t)(stop - v);
    if (n > sizeof(bufv) - 1) n = sizeof(bufv) - 1;
    memcpy(bufv, v, n);
    bufv[n] = '\0';
    return parse_hex_or_dec(bufv);
}

static int parse_attr_str_range(const char *beg, const char *end, const char *key, char *out, size_t out_len) {
    if (!beg || !end || !key || !out || out_len == 0) return -1;
    const char *k = strncasestr(beg, (size_t)(end - beg), key);
    if (!k || k >= end) return -1;
    const char *eq = strchr(k, '=');
    if (!eq || eq >= end) return -1;
    const char *v = eq + 1;
    while (v < end && (*v == ' ')) ++v;
    if (v >= end || *v != '"') return -1;
    v++;
    const char *stop = v;
    while (stop < end && *stop != '"') ++stop;
    size_t n = (size_t)(stop - v);
    if (n >= out_len) n = out_len - 1;
    memcpy(out, v, n);
    out[n] = '\0';
    return 0;
}

static int parse_tag_text_range(const char *beg, const char *end, const char *tag, char *out, size_t out_len) {
    if (!beg || !end || !tag || !out || out_len == 0) return -1;
    char open[64];
    char close[64];
    snprintf(open, sizeof(open), "<%s", tag);
    snprintf(close, sizeof(close), "</%s>", tag);
    const char *tb = strncasestr(beg, (size_t)(end - beg), open);
    if (!tb || tb >= end) return -2;
    const char *gt = strchr(tb, '>');
    if (!gt || gt >= end) return -3;
    const char *te = strncasestr(gt + 1, (size_t)(end - (gt + 1)), close);
    if (!te || te <= gt) return -4;
    const char *v = gt + 1;
    if (v + 9 < te && strncmp(v, "<![CDATA[", 9) == 0) {
        v += 9;
        const char *ce = strncasestr(v, (size_t)(te - v), "]]>");
        if (ce && ce <= te) te = ce;
    }
    size_t n = (size_t)(te - v);
    if (n >= out_len) n = out_len - 1;
    memcpy(out, v, n);
    out[n] = '\0';
    return 0;
}

static int scan_unique_xml(const char *dir, char *out_path, size_t out_size) {
    DIR *d = opendir(dir);
    if (!d) return -1;
    struct dirent *de;
    char sel[512] = {0};
    time_t sel_mt = 0;
    int cnt = 0;
    while ((de = readdir(d)) != NULL) {
        const char *n = de->d_name;
        size_t nl = strlen(n);
        if (nl >= 4) {
            const char *ext = n + nl - 4;
            char e0 = (char)tolower((unsigned char)ext[0]);
            char e1 = (char)tolower((unsigned char)ext[1]);
            char e2 = (char)tolower((unsigned char)ext[2]);
            char e3 = (char)tolower((unsigned char)ext[3]);
            if (e0 == '.' && e1 == 'x' && e2 == 'm' && e3 == 'l') {
                char full[512];
                snprintf(full, sizeof(full), "%s/%s", dir, n);
                struct stat st;
                if (stat(full, &st) == 0) {
                    cnt++;
                    if (st.st_mtime >= sel_mt) {
                        sel_mt = st.st_mtime;
                        snprintf(sel, sizeof(sel), "%s", full);
                    }
                }
            }
        }
    }
    closedir(d);
    if (cnt == 0) return -2;
    if (sel[0] == '\0') return -3;
    snprintf(out_path, out_size, "%s", sel);
    return cnt;
}

EXTERNFUNC ma_status_t eth_initDLL(uint32_t timeout_ms,
                                   uint16_t *out_slave_count,
                                   char (*out_product_names)[64],
                                   uint16_t max_slaves,
                                   bool *out_config_valid,
                                   struct motor_api_handle **out_handle) {
    const char *docdir = "/home/phi/ecmotor_api/motor_api/doc";
    uint32_t tmo = timeout_ms ? timeout_ms : 5000;
    char eni[512];
    int xmlcnt = scan_unique_xml(docdir, eni, sizeof(eni));
    if (xmlcnt < 1) { if (out_config_valid) { *out_config_valid = false; } return MA_ERR_IO; }
    uint32_t vids[MA_MAX_SLAVES] = {0};
    uint32_t prods[MA_MAX_SLAVES] = {0};
    uint16_t poss[MA_MAX_SLAVES] = {0};
    uint16_t cnt = 0;
    ma_eni_slave_t *eni_slaves = NULL;
    ma_status_t rc = motor_api_read_eni(eni, vids, prods, poss, (uint16_t)max_slaves, &cnt, &eni_slaves);
    if (rc != MA_OK || cnt == 0) { if (out_config_valid) { *out_config_valid = false; } return MA_ERR_IO; }
    if (out_product_names) {
        FILE *fp = fopen(eni, "rb");
        if (fp) {
            fseek(fp, 0, SEEK_END);
            long len = ftell(fp);
            fseek(fp, 0, SEEK_SET);
            char *buf = (char *)malloc((size_t)len + 1);
            if (buf) {
                size_t rd = fread(buf, 1, (size_t)len, fp);
                buf[rd] = '\0';
                uint32_t map_pcodes[MA_MAX_SLAVES] = {0};
                char map_names[MA_MAX_SLAVES][64];
                for (uint16_t mi = 0; mi < MA_MAX_SLAVES; ++mi) {
                    map_names[mi][0] = '\0';
                }
                uint16_t map_n = 0;
                const char *scan_map = buf;
                while (1) {
                    const char *ibeg = strncasestr(scan_map, rd - (size_t)(scan_map - buf), "<EtherCATInfo");
                    if (!ibeg) break;
                    const char *itag_end = strchr(ibeg, '>');
                    if (!itag_end) break;
                    const char *iend = strncasestr(itag_end, rd - (size_t)(itag_end - buf), "</EtherCATInfo>");
                    if (!iend) break;
                    const char *p = itag_end;
                    while (1) {
                        const char *dbeg = strncasestr(p, (size_t)(iend - p), "<Device");
                        if (!dbeg) break;
                        const char *dtag_end = strchr(dbeg, '>');
                        if (!dtag_end || dtag_end > iend) break;
                        const char *dend = strncasestr(dtag_end, (size_t)(iend - dtag_end), "</Device>");
                        if (!dend) break;
                        const char *type_beg = strncasestr(dtag_end, (size_t)(dend - dtag_end), "<Type");
                        const char *type_gt = type_beg ? strchr(type_beg, '>') : NULL;
                        int pcv = -1;
                        if (type_beg && type_gt && type_gt <= dend) {
                            pcv = parse_attr_int_range(type_beg, type_gt, "ProductCode");
                        }
                        char name[64];
                        name[0] = '\0';
                        int okn = 0;
                        if (parse_tag_text_range(dtag_end, dend, "Name", name, sizeof(name)) == 0) okn = 1;
                        if (!okn) {
                            char ttext[64];
                            ttext[0] = '\0';
                            if (type_gt && parse_tag_text_range(type_gt, dend, "Type", ttext, sizeof(ttext)) == 0) {
                                snprintf(name, sizeof(name), "%s", ttext);
                                okn = 1;
                            }
                        }
                        if (pcv >= 0 && okn && map_n < MA_MAX_SLAVES) {
                            map_pcodes[map_n] = (uint32_t)pcv;
                            snprintf(map_names[map_n], 64, "%s", name);
                            map_n++;
                        }
                        p = dend;
                    }
                    scan_map = iend;
                }
                const char *list_beg = strncasestr(buf, rd, "<SlaveList");
                const char *list_end = list_beg ? strncasestr(list_beg, (size_t)(rd - (list_beg - buf)), "</SlaveList>") : NULL;
                if (list_beg && list_end) {
                    const char *sp = list_beg;
                    uint16_t idx = 0;
                    while (idx < cnt) {
                        const char *sbeg = strncasestr(sp, (size_t)(list_end - sp), "<Slave");
                        if (!sbeg) break;
                        const char *stag_end = strchr(sbeg, '>');
                        if (!stag_end || stag_end > list_end) break;
                        const char *send = strncasestr(stag_end, (size_t)(list_end - stag_end), "</Slave>");
                        if (!send) send = stag_end;
                        char name[64];
                        name[0] = '\0';
                        int ok = 0;
                        if (parse_attr_str_range(sbeg, stag_end, "Name", name, sizeof(name)) == 0) ok = 1;
                        if (!ok && parse_attr_str_range(sbeg, stag_end, "ProductName", name, sizeof(name)) == 0) ok = 1;
                        if (!ok && parse_tag_text_range(stag_end, send, "Name", name, sizeof(name)) == 0) ok = 1;
                        if (!ok && parse_tag_text_range(stag_end, send, "ProductName", name, sizeof(name)) == 0) ok = 1;
                        if (!ok) snprintf(name, sizeof(name), "PID_0x%08X", prods[idx]);
                        snprintf(out_product_names[idx], 64, "%s", name);
                        sp = stag_end;
                        idx++;
                    }
                } else {
                    const char *scan = buf;
                    uint16_t didx = 0;
                    while (didx < cnt) {
                        const char *ibeg = strncasestr(scan, rd - (size_t)(scan - buf), "<EtherCATInfo");
                        if (!ibeg) break;
                        const char *itag_end = strchr(ibeg, '>');
                        if (!itag_end) break;
                        const char *iend = strncasestr(itag_end, rd - (size_t)(itag_end - buf), "</EtherCATInfo>");
                        if (!iend) break;
                        const char *dbeg = strncasestr(itag_end, (size_t)(iend - itag_end), "<Device");
                        if (!dbeg) {
                            scan = iend;
                            continue;
                        }
                        const char *dtag_end = strchr(dbeg, '>');
                        if (!dtag_end || dtag_end > iend) {
                            scan = iend;
                            continue;
                        }
                        const char *dend = strncasestr(dtag_end, (size_t)(iend - dtag_end), "</Device>");
                        if (!dend) {
                            scan = iend;
                            continue;
                        }
                        char name[64];
                        name[0] = '\0';
                        int ok2 = 0;
                        if (parse_tag_text_range(dtag_end, dend, "Name", name, sizeof(name)) == 0) ok2 = 1;
                        if (!ok2) {
                            char type[64];
                            type[0] = '\0';
                            if (parse_tag_text_range(dtag_end, dend, "Type", type, sizeof(type)) == 0) {
                                snprintf(name, sizeof(name), "%s", type);
                                ok2 = 1;
                            }
                        }
                        if (!ok2) snprintf(name, sizeof(name), "PID_0x%08X", prods[didx]);
                        snprintf(out_product_names[didx], 64, "%s", name);
                        scan = iend;
                        didx++;
                    }
                }
                for (uint16_t i = 0; i < cnt && i < max_slaves; ++i) {
                    for (uint16_t k = 0; k < map_n; ++k) {
                        if (map_pcodes[k] == prods[i]) {
                            snprintf(out_product_names[i], 64, "%s", map_names[k]);
                            break;
                        }
                    }
                    if (out_product_names[i][0] == '\0') {
                        snprintf(out_product_names[i], 64, "PID_0x%08X", prods[i]);
                    }
                }
                free(buf);
            }
            fclose(fp);
        } else {
            for (uint16_t i = 0; i < cnt && i < max_slaves; ++i) {
                snprintf(out_product_names[i], 64, "PID_0x%08X", prods[i]);
            }
        }
    }
    motor_api_handle_t *h = (motor_api_handle_t *)calloc(1, sizeof(*h));
    if (!h) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_RUNTIME;
    }
    pthread_mutex_init(&h->cmd_mutex, NULL);
    h->master = ecrt_request_master(0);
    if (!h->master) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        free(h);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_INIT;
    }
    h->domain = ecrt_master_create_domain(h->master);
    if (!h->domain) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        ecrt_release_master(h->master);
        free(h);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_INIT;
    }
    h->slave_count = cnt;
    for (uint16_t i = 0; i < cnt; ++i) {
        h->vendor_id[i] = vids[i];
        h->product_code[i] = prods[i];
        h->position[i] = poss[i];
    }
    for (uint16_t i = 0; i < cnt; ++i) {
        h->sc[i] = ecrt_master_slave_config(h->master, 0, poss[i], vids[i], prods[i]);
        if (!h->sc[i]) {
            if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
            ecrt_release_master(h->master);
            free(h);
            if (out_config_valid) { *out_config_valid = false; }
            return MA_ERR_INIT;
        }
        (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 2, (uint8_t)-3);
        (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 1, (uint8_t)(h->cycle_us ? (h->cycle_us/1000U) : 4));
        (void)ecrt_slave_config_sdo32(h->sc[i], 0x6081, 0, 100000);
        (void)ecrt_slave_config_sdo32(h->sc[i], 0x6083, 0, 50000);
        (void)ecrt_slave_config_sdo32(h->sc[i], 0x6084, 0, 50000);
    }
    if (eni_slaves) {
        for (uint16_t i = 0; i < cnt; ++i) {
            unsigned int rx_n = eni_slaves[i].rx_pdo_count;
            unsigned int tx_n = eni_slaves[i].tx_pdo_count;
            ec_pdo_info_t *rx_infos = rx_n ? (ec_pdo_info_t *)calloc(rx_n, sizeof(*rx_infos)) : NULL;
            ec_pdo_info_t *tx_infos = tx_n ? (ec_pdo_info_t *)calloc(tx_n, sizeof(*tx_infos)) : NULL;
            ec_pdo_entry_info_t **rx_entries = rx_n ? (ec_pdo_entry_info_t **)calloc(rx_n, sizeof(*rx_entries)) : NULL;
            ec_pdo_entry_info_t **tx_entries = tx_n ? (ec_pdo_entry_info_t **)calloc(tx_n, sizeof(*tx_entries)) : NULL;
            for (unsigned int p = 0; p < rx_n; ++p) {
                unsigned int ecnt = eni_slaves[i].rx_pdos[p].entry_count;
                rx_entries[p] = ecnt ? (ec_pdo_entry_info_t *)calloc(ecnt, sizeof(*rx_entries[p])) : NULL;
                for (unsigned int e = 0; e < ecnt; ++e) {
                    rx_entries[p][e].index = eni_slaves[i].rx_pdos[p].entries[e].index;
                    rx_entries[p][e].subindex = eni_slaves[i].rx_pdos[p].entries[e].subindex;
                    rx_entries[p][e].bit_length = eni_slaves[i].rx_pdos[p].entries[e].bitlen;
                }
                rx_infos[p].index = eni_slaves[i].rx_pdos[p].pdo_index;
                rx_infos[p].entries = rx_entries[p];
                rx_infos[p].n_entries = ecnt;
            }
            for (unsigned int p = 0; p < tx_n; ++p) {
                unsigned int ecnt = eni_slaves[i].tx_pdos[p].entry_count;
                tx_entries[p] = ecnt ? (ec_pdo_entry_info_t *)calloc(ecnt, sizeof(*tx_entries[p])) : NULL;
                for (unsigned int e = 0; e < ecnt; ++e) {
                    tx_entries[p][e].index = eni_slaves[i].tx_pdos[p].entries[e].index;
                    tx_entries[p][e].subindex = eni_slaves[i].tx_pdos[p].entries[e].subindex;
                    tx_entries[p][e].bit_length = eni_slaves[i].tx_pdos[p].entries[e].bitlen;
                }
                tx_infos[p].index = eni_slaves[i].tx_pdos[p].pdo_index;
                tx_infos[p].entries = tx_entries[p];
                tx_infos[p].n_entries = ecnt;
            }
            ec_sync_info_t syncs[] = {
                {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                {2, EC_DIR_OUTPUT, (uint8_t)rx_n, rx_infos, EC_WD_ENABLE},
                {3, EC_DIR_INPUT, (uint8_t)tx_n, tx_infos, EC_WD_DISABLE},
                {0xFF, (ec_direction_t)0, 0, NULL, EC_WD_DISABLE}
            };
            if (ecrt_slave_config_pdos(h->sc[i], EC_END, syncs)) {
                for (unsigned int p = 0; p < rx_n; ++p) free(rx_entries[p]);
                for (unsigned int p = 0; p < tx_n; ++p) free(tx_entries[p]);
                free(rx_entries);
                free(tx_entries);
                free(rx_infos);
                free(tx_infos);
                motor_api_free_eni_slaves(eni_slaves, cnt);
                ecrt_release_master(h->master);
                free(h);
                if (out_config_valid) { *out_config_valid = false; }
                return MA_ERR_CONFIG;
            }
            for (unsigned int p = 0; p < rx_n; ++p) free(rx_entries[p]);
            for (unsigned int p = 0; p < tx_n; ++p) free(tx_entries[p]);
            free(rx_entries);
            free(tx_entries);
            free(rx_infos);
            free(tx_infos);
        }
    }
    ec_pdo_entry_reg_t regs[MA_MAX_SLAVES * 24 + 1];
    memset(regs, 0, sizeof(regs));
    size_t r = 0;
    for (uint16_t i = 0; i < cnt; ++i) {
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6040, .subindex = 0x00, .offset = &h->out[i].controlWord };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6060, .subindex = 0x00, .offset = &h->out[i].workModeOut };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x607A, .subindex = 0x00, .offset = &h->out[i].targetPosition };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60B8, .subindex = 0x00, .offset = &h->out[i].touchProbeFunc };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6041, .subindex = 0x00, .offset = &h->in[i].statusword };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6064, .subindex = 0x00, .offset = &h->in[i].actualPosition };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6061, .subindex = 0x00, .offset = &h->in[i].workModeIn };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x603F, .subindex = 0x00, .offset = &h->in[i].errorCode };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60F4, .subindex = 0x00, .offset = &h->in[i].followingError };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60FD, .subindex = 0x00, .offset = &h->in[i].digitalInputs };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60B9, .subindex = 0x00, .offset = &h->in[i].touchProbeStatus };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60BA, .subindex = 0x00, .offset = &h->in[i].touchProbePos };
        regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x213F, .subindex = 0x00, .offset = &h->in[i].servoErrorCode };
    }
    regs[r] = (ec_pdo_entry_reg_t){0};
    if (ecrt_domain_reg_pdo_entry_list(h->domain, regs)) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        ecrt_release_master(h->master);
        free(h);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_CONFIG;
    }
    h->cycle_us = 4000;
    h->dc_sync0_period_ns = (uint64_t)h->cycle_us * 1000ULL;
    ecrt_master_select_reference_clock(h->master, h->sc[0]);
    for (uint16_t i = 0; i < cnt; ++i) {
        (void)ecrt_slave_config_dc(h->sc[i], 0x0300, h->dc_sync0_period_ns, 0, 0, 0);
    }
    if (ecrt_master_activate(h->master)) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        ecrt_release_master(h->master);
        free(h);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_INIT;
    }
    h->domain_pd = ecrt_domain_data(h->domain);
    if (!h->domain_pd) {
        if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
        ecrt_release_master(h->master);
        free(h);
        if (out_config_valid) { *out_config_valid = false; }
        return MA_ERR_INIT;
    }
    uint64_t start = monotonic_ns();
    uint64_t end_ns = start + (uint64_t)tmo * 1000000ULL;
    uint16_t resp = 0;
    while (monotonic_ns() < end_ns) {
        ec_master_state_t ms;
        ecrt_master_state(h->master, &ms);
        resp = (uint16_t)ms.slaves_responding;
        if (resp >= cnt) break;
        usleep(10000);
    }
    bool match = (resp == cnt);
    if (out_slave_count) *out_slave_count = cnt;
    if (out_config_valid) *out_config_valid = match && (cnt > 0);
    *out_handle = (struct motor_api_handle *)h;
    if (!match) return MA_ERR_CONFIG;
    return MA_OK;
}

/*
 * 函数: set_cmd_locked
 * 功能: 在互斥保护下更新运行命令，限制参数合法范围。
 */
static void set_cmd_locked(motor_api_handle_t *h, bool run, int dir, int step) {
    if (!h) return;
    if (step < 1) step = 1;
    if (step > 100000) step = 100000;
    if (dir != -1 && dir != 0 && dir != 1) dir = 0;
    pthread_mutex_lock(&h->cmd_mutex);
    h->cmd_run = run;
    h->cmd_dir = dir;
    h->cmd_step = step;
    pthread_mutex_unlock(&h->cmd_mutex);
}

/*
 * 函数: send_all
 * 功能: 发送完整 HTTP 响应，避免短写。
 */
static int send_all(int fd, const char *buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = send(fd, buf + off, len - off, 0);
        if (n <= 0) return -1;
        off += (size_t)n;
    }
    return 0;
}

/*
 * 函数: http_send
 * 功能: 构造并发送 HTTP 响应头与主体。
 */
static void http_send(int fd, const char *status, const char *ctype, const char *body) {
    char header[512];
    int blen = body ? (int)strlen(body) : 0;
    int hlen = snprintf(
        header,
        sizeof(header),
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s; charset=utf-8\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n",
        status ? status : "200 OK",
        ctype ? ctype : "text/plain",
        blen
    );
    send_all(fd, header, (size_t)hlen);
    if (blen > 0) {
        send_all(fd, body, (size_t)blen);
    }
}

/*
 * 函数: parse_control_json
 * 功能: 解析 POST /control 的简易 JSON（不依赖第三方库）。
 */
static int parse_control_json(const char *body, int *out_dir, int *out_step) {
    if (!body || !out_dir || !out_step) return -1;
    const char *dkey = strstr(body, "\"direction\""); if (!dkey) return -2;
    const char *dcolon = strchr(dkey, ':'); if (!dcolon) return -3;
    const char *dquote1 = strchr(dcolon, '"'); if (!dquote1) return -4;
    const char *dquote2 = strchr(dquote1 + 1, '"'); if (!dquote2) return -5;
    int dir = 0; size_t dlen = (size_t)(dquote2 - (dquote1 + 1)); if (dlen > 32) return -6;
    char dval[40]; memcpy(dval, dquote1 + 1, dlen); dval[dlen] = '\0';
    for (size_t i = 0; i < dlen; ++i) dval[i] = (char)tolower((unsigned char)dval[i]);
    if (strcmp(dval, "forward") == 0) dir = 1; else if (strcmp(dval, "reverse") == 0) dir = -1; else return -7;
    const char *skey = strstr(body, "\"step\""); if (!skey) return -8;
    const char *scolon = strchr(skey, ':'); if (!scolon) return -9;
    long step = strtol(scolon + 1, NULL, 10); if (step <= 0 || step > 100000000) return -10;
    *out_dir = dir; *out_step = (int)step; return 0;
}

/*
 * 函数: format_diag
 * 功能: 汇总各轴关键诊断数据并生成 JSON 字符串。
 */
static ma_status_t format_diag(motor_api_handle_t *h, char *buf, size_t buf_size) {
    if (!h || !buf || buf_size < 64) return MA_ERR_PARAM;
    uint16_t sw[MA_MAX_SLAVES] = {0}; int8_t md[MA_MAX_SLAVES] = {0}; int32_t fe[MA_MAX_SLAVES] = {0};
    uint16_t ec[MA_MAX_SLAVES] = {0}; uint16_t sec[MA_MAX_SLAVES] = {0}; uint32_t di[MA_MAX_SLAVES] = {0};
    uint16_t tpst[MA_MAX_SLAVES] = {0}; int32_t tpp[MA_MAX_SLAVES] = {0}; int32_t tgt[MA_MAX_SLAVES] = {0}; int32_t act[MA_MAX_SLAVES] = {0};
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        sw[i] = MA_RD_U16(h, h->in[i].statusword);
        md[i] = MA_RD_S8(h, h->in[i].workModeIn);
        fe[i] = MA_RD_S32(h, h->in[i].followingError);
        ec[i] = MA_RD_U16(h, h->in[i].errorCode);
        sec[i] = MA_RD_U16(h, h->in[i].servoErrorCode);
        di[i] = MA_RD_U32(h, h->in[i].digitalInputs);
        tpst[i] = MA_RD_U16(h, h->in[i].touchProbeStatus);
        tpp[i] = MA_RD_S32(h, h->in[i].touchProbePos);
        tgt[i] = MA_RD_S32(h, h->out[i].targetPosition);
        act[i] = MA_RD_S32(h, h->in[i].actualPosition);
    }
    int n = snprintf(buf, buf_size,
                     "{\"status\":[%u,%u,%u],\"mode\":[%d,%d,%d],\"followingErr\":[%d,%d,%d],\"err\":[%u,%u,%u],\"servoErr\":[%u,%u,%u],\"din\":[%u,%u,%u],\"tpst\":[%u,%u,%u],\"tpp\":[%d,%d,%d],\"tgt\":[%d,%d,%d],\"act\":[%d,%d,%d]}",
                     sw[0], sw[1], sw[2], md[0], md[1], md[2], fe[0], fe[1], fe[2], ec[0], ec[1], ec[2], sec[0], sec[1], sec[2], di[0], di[1], di[2], tpst[0], tpst[1], tpst[2], tpp[0], tpp[1], tpp[2], tgt[0], tgt[1], tgt[2], act[0], act[1], act[2]);
    if (n < 0) {
        return MA_ERR_RUNTIME;
    }
    return MA_OK;
}

/*
 * 函数: http_thread_fn
 * 功能: HTTP 服务线程入口，处理基本控制与诊断请求。
 */
static void *http_thread_fn(void *arg) {
    motor_api_handle_t *h = (motor_api_handle_t *)arg;
    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) return NULL;
    int opt = 1;
    (void)setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)h->http_port);
    if (bind(sfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sfd);
        return NULL;
    }
    if (listen(sfd, 8) < 0) {
        close(sfd);
        return NULL;
    }
    while (!h->stop) {
        struct sockaddr_in cli;
        socklen_t cl = sizeof(cli);
        int cfd = accept(sfd, (struct sockaddr *)&cli, &cl);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            break;
        }
        char buf[4096];
        int n = recv(cfd, buf, (int)sizeof(buf) - 1, 0);
        if (n <= 0) {
            close(cfd);
            continue;
        }
        buf[n] = '\0';
        if (strncmp(buf, "GET ", 4) == 0) {
            const char *path = buf + 4;
            const char *sp = strchr(path, ' ');
            size_t plen = sp ? (size_t)(sp - path) : 0;
            if (plen == 1 && path[0] == '/') {
                http_send(cfd, "200 OK", "text/plain", "motor_api running");
                close(cfd);
                continue;
            }
            if (plen && strncmp(path, "/status", plen) == 0) {
                char out[256];
                pthread_mutex_lock(&h->cmd_mutex);
                bool run = h->cmd_run;
                int dir = h->cmd_dir;
                int step = h->cmd_step;
                pthread_mutex_unlock(&h->cmd_mutex);
                int m = snprintf(out, sizeof(out), "{\"run\":%s,\"dir\":%d,\"step\":%d}", run ? "true" : "false", dir, step);
                (void)m;
                http_send(cfd, "200 OK", "application/json", out);
                close(cfd);
                continue;
            }
            if (plen && strncmp(path, "/diag", plen) == 0) {
                char out[1024];
                if (format_diag(h, out, sizeof(out)) == MA_OK) {
                    http_send(cfd, "200 OK", "application/json", out);
                } else {
                    http_send(cfd, "500 Internal Server Error", "text/plain", "format error");
                }
                close(cfd);
                continue;
            }
            http_send(cfd, "404 Not Found", "text/plain", "not found");
            close(cfd);
            continue;
        } else if (strncmp(buf, "POST ", 5) == 0) {
            const char *path = buf + 5;
            const char *sp = strchr(path, ' ');
            size_t plen = sp ? (size_t)(sp - path) : 0;
            const char *hdr_end = strstr(buf, "\r\n\r\n");
            const char *body = hdr_end ? (hdr_end + 4) : NULL;
            if (plen && strncmp(path, "/control", plen) == 0) {
                int dir = 0, step = 0;
                int rc = parse_control_json(body, &dir, &step);
                if (rc == 0) {
                    set_cmd_locked(h, true, dir, step);
                    http_send(cfd, "200 OK", "application/json", "{\"ok\":true}");
                } else {
                    http_send(cfd, "400 Bad Request", "application/json", "{\"ok\":false}\n");
                }
                close(cfd);
                continue;
            }
            if (plen && strncmp(path, "/stop", plen) == 0) {
                set_cmd_locked(h, false, 0, 0);
                http_send(cfd, "200 OK", "application/json", "{\"ok\":true}");
                close(cfd);
                continue;
            }
            if (plen && strncmp(path, "/shutdown", plen) == 0) {
                h->stop = 1;
                http_send(cfd, "200 OK", "application/json", "{\"ok\":true}");
                close(cfd);
                continue;
            }
            http_send(cfd, "404 Not Found", "text/plain", "not found");
            close(cfd);
            continue;
        } else {
            http_send(cfd, "405 Method Not Allowed", "text/plain", "method not allowed");
            close(cfd);
        }
    }
    close(sfd);
    return NULL;
}

/*
 * 函数: check_domain_state
 * 功能: 更新域状态快照（便于诊断）。
 */
static void check_domain_state(motor_api_handle_t *h) {
    ec_domain_state_t ds;
    ecrt_domain_state(h->domain, &ds);
    h->domain_state = ds;
}

/*
 * 函数: check_master_state
 * 功能: 更新主站状态快照（便于诊断）。
 */
static void check_master_state(motor_api_handle_t *h) {
    ec_master_state_t ms;
    ecrt_master_state(h->master, &ms);
    h->master_state = ms;
}

/*
 * 函数: check_slave_states
 * 功能: 更新从站配置状态快照（便于诊断）。
 */
static void check_slave_states(motor_api_handle_t *h) {
    ec_slave_config_state_t s;
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        ecrt_slave_config_state(h->sc[i], &s);
        h->sc_state[i] = s;
    }
}

/*
 * 函数: motor_api_read_eni
 * 功能: 简易 ENI 解析，容错提取常见从站属性。
 */
/* 删除重复定义，保持唯一的解析函数 */

EXTERNFUNC ma_status_t motor_api_read_eni(const char *eni_path,
                                          uint32_t *vendor_ids,
                                          uint32_t *product_codes,
                                          uint16_t *positions,
                                          uint16_t max_slaves,
                                          uint16_t *out_count,
                                          ma_eni_slave_t **out_slaves) {
    if (!eni_path || !out_count) return MA_ERR_PARAM;
    FILE *fp = fopen(eni_path, "rb"); if (!fp) return MA_ERR_IO;
    fseek(fp, 0, SEEK_END); long len = ftell(fp); fseek(fp, 0, SEEK_SET);
    char *buf = (char *)malloc((size_t)len + 1); if (!buf) { fclose(fp); return MA_ERR_RUNTIME; }
    size_t rd = fread(buf, 1, (size_t)len, fp); fclose(fp); buf[rd] = '\0';

    uint16_t count = 0; ma_eni_slave_t *slaves = NULL;

    const char *p = buf; size_t remaining = rd;

    /* 解析 <SlaveList>/<Slave> 布局及其内的 Rx/TxPdo/CoePdo */
    {
        const char *list_beg = strncasestr(buf, rd, "<SlaveList");
        const char *list_end = list_beg ? strncasestr(list_beg, (size_t)(rd - (list_beg - buf)), "</SlaveList>") : NULL;
        if (list_beg && list_end) {
            const char *sp = list_beg;
            while (count < max_slaves) {
                const char *sbeg = strncasestr(sp, (size_t)(list_end - sp), "<Slave"); if (!sbeg) break;
                const char *stag_end = strchr(sbeg, '>'); if (!stag_end || stag_end > list_end) break;
                const char *send = strncasestr(stag_end, (size_t)(list_end - stag_end), "</Slave>"); if (!send) send = stag_end;
                uint16_t ps = (uint16_t)(count);
                uint32_t v = 0, pc = 0;
                int pv = parse_attr_int_range(sbeg, stag_end, "Position"); if (pv >= 0) ps = (uint16_t)pv;
                int vv = parse_attr_int_range(sbeg, stag_end, "VendorId"); if (vv < 0) vv = parse_attr_int_range(sbeg, stag_end, "VendorID"); if (vv >= 0) v = (uint32_t)vv;
                int pp = parse_attr_int_range(sbeg, stag_end, "ProductCode"); if (pp >= 0) pc = (uint32_t)pp;

                ma_eni_pdo_t *rx_pdos = NULL; unsigned int rx_cnt = 0;
                ma_eni_pdo_t *tx_pdos = NULL; unsigned int tx_cnt = 0;

                const char *blk = stag_end;
                const char *scan = blk;
                while (scan < send) {
                    const char *rx = strncasestr(scan, (size_t)(send - scan), "<RxPdo");
                    const char *tx = strncasestr(scan, (size_t)(send - scan), "<TxPdo");
                    const char *pdo = strncasestr(scan, (size_t)(send - scan), "<Pdo");
                    const char *begp = NULL; int kind = -1; /* 0=rx 1=tx 2=generic */
                    if (rx && (!tx || rx < tx) && (!pdo || rx < pdo)) { begp = rx; kind = 0; }
                    else if (tx && (!rx || tx < rx) && (!pdo || tx < pdo)) { begp = tx; kind = 1; }
                    else if (pdo) { begp = pdo; kind = 2; }
                    else break;
                    const char *endp = NULL;
                    if (kind == 0) endp = strncasestr(begp, (size_t)(send - begp), "</RxPdo>");
                    else if (kind == 1) endp = strncasestr(begp, (size_t)(send - begp), "</TxPdo>");
                    else endp = strncasestr(begp, (size_t)(send - begp), "</Pdo>");
                    if (!endp) break;
                    int pdo_index = 0;
                    {
                        const char *idx_tag = strncasestr(begp, (size_t)(endp - begp), "<Index>");
                        if (idx_tag) { const char *gt = strchr(idx_tag, '>'); const char *lt = gt ? strncasestr(gt+1, (size_t)(endp - (gt+1)), "</Index>") : NULL; if (gt && lt && lt > gt) { char num[64]; size_t n = (size_t)(lt - (gt+1)); if (n > sizeof(num)-1) n = sizeof(num)-1; memcpy(num, gt+1, n); num[n] = '\0'; pdo_index = parse_hex_or_dec(num); } }
                        if (pdo_index == 0) {
                            int iv = parse_attr_int_range(begp, strchr(begp, '>') ? strchr(begp, '>') : endp, "Index");
                            if (iv > 0) pdo_index = iv;
                        }
                    }
                    ma_eni_pdo_entry_t *ents = NULL; unsigned int ecnt = 0;
                    const char *ep = begp;
                    while (1) {
                        const char *ebeg = strncasestr(ep, (size_t)(endp - ep), "<Entry"); if (!ebeg) break;
                        const char *etag_end = strchr(ebeg, '>'); if (!etag_end || etag_end > endp) break;
                        const char *eend = strncasestr(etag_end, (size_t)(endp - etag_end), "</Entry>"); if (!eend) eend = etag_end;
                        int e_index = 0, e_sub = 0, e_bit = 0;
                        const char *it = strncasestr(ebeg, (size_t)(eend - ebeg), "<Index>");
                        if (it) { const char *gt = strchr(it, '>'); const char *lt = gt ? strncasestr(gt+1, (size_t)(eend - (gt+1)), "</Index>") : NULL; if (gt && lt && lt > gt) { char num[64]; size_t n = (size_t)(lt - (gt+1)); if (n > sizeof(num)-1) n = sizeof(num)-1; memcpy(num, gt+1, n); num[n] = '\0'; e_index = parse_hex_or_dec(num); } }
                        if (e_index == 0) { int av = parse_attr_int_range(ebeg, etag_end, "Index"); if (av > 0) e_index = av; }
                        const char *st = strncasestr(ebeg, (size_t)(eend - ebeg), "<SubIndex>");
                        if (st) { const char *gt = strchr(st, '>'); const char *lt = gt ? strncasestr(gt+1, (size_t)(eend - (gt+1)), "</SubIndex>") : NULL; if (gt && lt && lt > gt) { char num[32]; size_t n = (size_t)(lt - (gt+1)); if (n > sizeof(num)-1) n = sizeof(num)-1; memcpy(num, gt+1, n); num[n] = '\0'; e_sub = (int)strtol(num, NULL, 0); } }
                        if (e_sub == 0) { int sv = parse_attr_int_range(ebeg, etag_end, "SubIndex"); if (sv >= 0) e_sub = sv; }
                        const char *bt = strncasestr(ebeg, (size_t)(eend - ebeg), "<BitLen>");
                        if (bt) { const char *gt = strchr(bt, '>'); const char *lt = gt ? strncasestr(gt+1, (size_t)(eend - (gt+1)), "</BitLen>") : NULL; if (gt && lt && lt > gt) { char num[32]; size_t n = (size_t)(lt - (gt+1)); if (n > sizeof(num)-1) n = sizeof(num)-1; memcpy(num, gt+1, n); num[n] = '\0'; e_bit = (int)strtol(num, NULL, 0); } }
                        if (e_bit == 0) { int bv = parse_attr_int_range(ebeg, etag_end, "BitLen"); if (bv > 0) e_bit = bv; }
                        ents = (ma_eni_pdo_entry_t *)realloc(ents, (ecnt + 1) * sizeof(*ents));
                        ents[ecnt].index = (uint16_t)e_index; ents[ecnt].subindex = (uint8_t)e_sub; ents[ecnt].bitlen = (uint8_t)e_bit; ecnt++;
                        ep = eend + 8;
                    }
                    ma_eni_pdo_t pdo_out; pdo_out.pdo_index = (uint16_t)pdo_index; pdo_out.entry_count = ecnt; pdo_out.entries = ents;
                    int dir = kind;
                    if (dir == 2) { if (pdo_index >= 0x1A00) dir = 1; else dir = 0; }
                    if (dir == 0) { rx_pdos = (ma_eni_pdo_t *)realloc(rx_pdos, (rx_cnt + 1) * sizeof(*rx_pdos)); rx_pdos[rx_cnt++] = pdo_out; }
                    else { tx_pdos = (ma_eni_pdo_t *)realloc(tx_pdos, (tx_cnt + 1) * sizeof(*tx_pdos)); tx_pdos[tx_cnt++] = pdo_out; }
                    scan = endp + 7;
                }

                slaves = (ma_eni_slave_t *)realloc(slaves, (count + 1) * sizeof(*slaves));
                slaves[count].vendor_id = v ? v : 0x000116c7;
                slaves[count].product_code = pc ? pc : 0x003e0402;
                slaves[count].position = ps;
                slaves[count].rx_pdo_count = rx_cnt;
                slaves[count].rx_pdos = rx_pdos;
                slaves[count].tx_pdo_count = tx_cnt;
                slaves[count].tx_pdos = tx_pdos;

                if (positions) positions[count] = ps;
                if (vendor_ids) vendor_ids[count] = slaves[count].vendor_id;
                if (product_codes) product_codes[count] = slaves[count].product_code;

                count++;
                sp = send + 8;
            }
        }
    }

    /* 解析 <EtherCATInfo> 布局 */
    while (count < max_slaves) {
        const char *info = strncasestr(p, remaining, "<EtherCATInfo>"); if (!info) break;
        const char *end = strncasestr(info, (size_t)(rd - (info - buf)), "</EtherCATInfo>"); if (!end) end = buf + rd;
        uint32_t v = 0, pc = 0; uint16_t ps = count;
        const char *vend = strncasestr(info, (size_t)(end - info), "<Id>");
        if (vend) { const char *gt = strchr(vend, '>'); const char *lt = gt ? strchr(gt+1, '<') : NULL; if (gt && lt && lt > gt) { char num[32]; size_t n = (size_t)(lt - (gt+1)); if (n > sizeof(num)-1) n = sizeof(num)-1; memcpy(num, gt+1, n); num[n] = '\0'; v = (uint32_t)strtoul(num, NULL, 10); } }
        const char *type = strncasestr(info, (size_t)(end - info), "ProductCode");
        if (type) { const char *eq = strchr(type, '='); if (eq && eq < end) { const char *vv = eq + 1; while (*vv==' '||*vv=='"'||*vv=='#') ++vv; if (*vv=='x' || *vv=='X') ++vv; pc = (uint32_t)strtoul(vv, NULL, 16); } }

        ma_eni_pdo_t *rx_pdos = NULL; unsigned int rx_cnt = 0;
        ma_eni_pdo_t *tx_pdos = NULL; unsigned int tx_cnt = 0;

        const char *scan = info;
        while (1) {
            const char *rx = strncasestr(scan, (size_t)(end - scan), "<RxPdo");
            const char *tx = strncasestr(scan, (size_t)(end - scan), "<TxPdo");
            if (!rx && !tx) break;
            int is_rx = 0; const char *pdo_beg = NULL;
            if (rx && (!tx || rx < tx)) { is_rx = 1; pdo_beg = rx; }
            else { is_rx = 0; pdo_beg = tx; }
            const char *pdo_end = NULL;
            if (is_rx) pdo_end = strncasestr(pdo_beg, (size_t)(end - pdo_beg), "</RxPdo>"); else pdo_end = strncasestr(pdo_beg, (size_t)(end - pdo_beg), "</TxPdo>");
            if (!pdo_end) break;
            int pdo_index = 0x0000;
            {
                const char *idx_tag = strncasestr(pdo_beg, (size_t)(pdo_end - pdo_beg), "<Index>");
                if (idx_tag) { const char *gt = strchr(idx_tag, '>'); const char *lt = strncasestr(gt ? gt+1 : pdo_beg, (size_t)(pdo_end - (gt ? gt+1 : pdo_beg)), "</Index>"); if (gt && lt && lt > gt) { char num[64]; size_t n = (size_t)(lt - (gt+1)); if (n > sizeof(num)-1) n = sizeof(num)-1; memcpy(num, gt+1, n); num[n] = '\0'; pdo_index = parse_hex_or_dec(num); } }
            }
            ma_eni_pdo_entry_t *ents = NULL; unsigned int ecnt = 0;
            const char *ep = pdo_beg;
            while (1) {
                const char *ebeg = strncasestr(ep, (size_t)(pdo_end - ep), "<Entry>"); if (!ebeg) break;
                const char *eend = strncasestr(ebeg, (size_t)(pdo_end - ebeg), "</Entry>"); if (!eend) break;
                int e_index = 0, e_sub = 0, e_bit = 0;
                const char *it = strncasestr(ebeg, (size_t)(eend - ebeg), "<Index>"); if (it) { const char *gt = strchr(it, '>'); const char *lt = strncasestr(gt ? gt+1 : ebeg, (size_t)(eend - (gt ? gt+1 : ebeg)), "</Index>"); if (gt && lt && lt > gt) { char num[64]; size_t n = (size_t)(lt - (gt+1)); if (n > sizeof(num)-1) n = sizeof(num)-1; memcpy(num, gt+1, n); num[n] = '\0'; e_index = parse_hex_or_dec(num); } }
                const char *st = strncasestr(ebeg, (size_t)(eend - ebeg), "<SubIndex>"); if (st) { const char *gt = strchr(st, '>'); const char *lt = strncasestr(gt ? gt+1 : ebeg, (size_t)(eend - (gt ? gt+1 : ebeg)), "</SubIndex>"); if (gt && lt && lt > gt) { char num[32]; size_t n = (size_t)(lt - (gt+1)); if (n > sizeof(num)-1) n = sizeof(num)-1; memcpy(num, gt+1, n); num[n] = '\0'; e_sub = (int)strtol(num, NULL, 0); } }
                const char *bt = strncasestr(ebeg, (size_t)(eend - ebeg), "<BitLen>"); if (bt) { const char *gt = strchr(bt, '>'); const char *lt = strncasestr(gt ? gt+1 : ebeg, (size_t)(eend - (gt ? gt+1 : ebeg)), "</BitLen>"); if (gt && lt && lt > gt) { char num[32]; size_t n = (size_t)(lt - (gt+1)); if (n > sizeof(num)-1) n = sizeof(num)-1; memcpy(num, gt+1, n); num[n] = '\0'; e_bit = (int)strtol(num, NULL, 0); } }
                ents = (ma_eni_pdo_entry_t *)realloc(ents, (ecnt + 1) * sizeof(*ents));
                ents[ecnt].index = (uint16_t)e_index; ents[ecnt].subindex = (uint8_t)e_sub; ents[ecnt].bitlen = (uint8_t)e_bit; ecnt++;
                ep = eend + 8;
            }
            ma_eni_pdo_t pdo; pdo.pdo_index = (uint16_t)pdo_index; pdo.entry_count = ecnt; pdo.entries = ents;
            if (is_rx) { rx_pdos = (ma_eni_pdo_t *)realloc(rx_pdos, (rx_cnt + 1) * sizeof(*rx_pdos)); rx_pdos[rx_cnt++] = pdo; }
            else { tx_pdos = (ma_eni_pdo_t *)realloc(tx_pdos, (tx_cnt + 1) * sizeof(*tx_pdos)); tx_pdos[tx_cnt++] = pdo; }
            scan = pdo_end + 7;
        }

        slaves = (ma_eni_slave_t *)realloc(slaves, (count + 1) * sizeof(*slaves));
        slaves[count].vendor_id = v ? v : 0x000116c7;
        slaves[count].product_code = pc ? pc : 0x003e0402;
        slaves[count].position = ps;
        slaves[count].rx_pdo_count = rx_cnt;
        slaves[count].rx_pdos = rx_pdos;
        slaves[count].tx_pdo_count = tx_cnt;
        slaves[count].tx_pdos = tx_pdos;

        if (positions) positions[count] = ps;
        if (vendor_ids) vendor_ids[count] = slaves[count].vendor_id;
        if (product_codes) product_codes[count] = slaves[count].product_code;

        count++;
        p = end + 15; remaining = rd - (size_t)(p - buf);
    }

    if (out_slaves) *out_slaves = slaves; else {
        if (slaves) {
            for (uint16_t i = 0; i < count; ++i) {
                for (unsigned int j = 0; j < slaves[i].rx_pdo_count; ++j) free(slaves[i].rx_pdos[j].entries);
                for (unsigned int j = 0; j < slaves[i].tx_pdo_count; ++j) free(slaves[i].tx_pdos[j].entries);
                free(slaves[i].rx_pdos);
                free(slaves[i].tx_pdos);
            }
            free(slaves);
        }
    }

    free(buf);
    *out_count = count;
    return MA_OK;
}

EXTERNFUNC void motor_api_free_eni_slaves(ma_eni_slave_t *slaves, uint16_t count) {
    if (!slaves) return;
    for (uint16_t i = 0; i < count; ++i) {
        for (unsigned int j = 0; j < slaves[i].rx_pdo_count; ++j) free(slaves[i].rx_pdos[j].entries);
        for (unsigned int j = 0; j < slaves[i].tx_pdo_count; ++j) free(slaves[i].tx_pdos[j].entries);
        free(slaves[i].rx_pdos);
        free(slaves[i].tx_pdos);
    }
    free(slaves);
}

/*
 * 函数: motor_api_create
 * 功能: 创建主站与域、注册 PDO、配置 DC，同步周期与 ENI。
 */
EXTERNFUNC ma_status_t motor_api_create(const char *eni_path,
                                        uint32_t cycle_us,
                                        uint16_t *out_slave_count,
                                        struct motor_api_handle **out_handle) {
    if (!out_handle || cycle_us == 0) return MA_ERR_PARAM;
    motor_api_handle_t *h = (motor_api_handle_t *)calloc(1, sizeof(*h)); if (!h) return MA_ERR_RUNTIME;
    h->cycle_us = cycle_us; h->dc_sync0_period_ns = (uint64_t)cycle_us * 1000ULL;
    pthread_mutex_init(&h->cmd_mutex, NULL);
    h->master = ecrt_request_master(0); if (!h->master) { free(h); return MA_ERR_INIT; }
    h->domain = ecrt_master_create_domain(h->master); if (!h->domain) { ecrt_release_master(h->master); free(h); return MA_ERR_INIT; }

    uint16_t cnt = 0; uint32_t vids[MA_MAX_SLAVES] = {0}, prods[MA_MAX_SLAVES] = {0}; uint16_t poss[MA_MAX_SLAVES] = {0};
    ma_eni_slave_t *eni_slaves = NULL;
    if (eni_path) {
        ma_status_t rc = motor_api_read_eni(eni_path, vids, prods, poss, MA_MAX_SLAVES, &cnt, &eni_slaves);
        if (rc != MA_OK || cnt == 0) {
            fprintf(stderr, "[ERROR] ENI parse failed or zero slaves: path=%s rc=%d cnt=%u\n", eni_path, rc, cnt);
            ecrt_release_master(h->master); free(h); return MA_ERR_CONFIG;
        }
        printf("[INFO] ENI parsed slaves=%u\n", cnt);
    } else {
        cnt = 3; vids[0] = vids[1] = vids[2] = 0x000116c7; prods[0] = prods[1] = prods[2] = 0x003e0402; poss[0] = 0; poss[1] = 1; poss[2] = 2;
        printf("[WARN] No ENI provided, using default 3 slaves\n");
    }
    h->slave_count = cnt; for (uint16_t i=0;i<cnt;i++){ h->vendor_id[i]=vids[i]; h->product_code[i]=prods[i]; h->position[i]=poss[i]; }
    for (uint16_t i = 0; i < cnt; ++i) {
        h->out[i].controlWord = UINT_MAX;
        h->out[i].workModeOut = UINT_MAX;
        h->out[i].targetPosition = UINT_MAX;
        h->out[i].touchProbeFunc = UINT_MAX;
        h->out[i].interpolationCtrl = UINT_MAX;
        h->in[i].statusword = UINT_MAX;
        h->in[i].workModeIn = UINT_MAX;
        h->in[i].actualPosition = UINT_MAX;
        h->in[i].actualVelocity = UINT_MAX;
        h->in[i].actualTorque = UINT_MAX;
        h->in[i].errorCode = UINT_MAX;
        h->in[i].followingError = UINT_MAX;
        h->in[i].digitalInputs = UINT_MAX;
        h->in[i].touchProbeStatus = UINT_MAX;
        h->in[i].touchProbePos = UINT_MAX;
        h->in[i].servoErrorCode = UINT_MAX;
        h->in[i].brakeDelay = UINT_MAX;
    }

    for (uint16_t i = 0; i < cnt; ++i) {
        h->sc[i] = ecrt_master_slave_config(h->master, 0, poss[i], vids[i], prods[i]);
        if (!h->sc[i]) { if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt); ecrt_release_master(h->master); free(h); return MA_ERR_INIT; }
        uint8_t period_ms = (uint8_t)(cycle_us / 1000U);
        if (vids[i] == 0x000116C7) {
            (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 2, (uint8_t)-3);
            (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 1, period_ms);
            (void)ecrt_slave_config_sdo32(h->sc[i], 0x6081, 0, 100000);
            (void)ecrt_slave_config_sdo32(h->sc[i], 0x6083, 0, 50000);
            (void)ecrt_slave_config_sdo32(h->sc[i], 0x6084, 0, 50000);
        } else if (vids[i] == 0x00001097) {
            (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 2, (uint8_t)-3);
            (void)ecrt_slave_config_sdo8(h->sc[i], 0x60C2, 1, 1);
            (void)ecrt_slave_config_sdo8(h->sc[i], 0x6060, 0, (uint8_t)MA_MODE_CSP);
        }
    }

    if (eni_slaves) {
        for (uint16_t i = 0; i < cnt; ++i) {
            unsigned int rx_n = eni_slaves[i].rx_pdo_count;
            unsigned int tx_n = eni_slaves[i].tx_pdo_count;
            ec_pdo_info_t *rx_infos = rx_n ? (ec_pdo_info_t *)calloc(rx_n, sizeof(*rx_infos)) : NULL;
            ec_pdo_info_t *tx_infos = tx_n ? (ec_pdo_info_t *)calloc(tx_n, sizeof(*tx_infos)) : NULL;
            ec_pdo_entry_info_t **rx_entries = rx_n ? (ec_pdo_entry_info_t **)calloc(rx_n, sizeof(*rx_entries)) : NULL;
            ec_pdo_entry_info_t **tx_entries = tx_n ? (ec_pdo_entry_info_t **)calloc(tx_n, sizeof(*tx_entries)) : NULL;

            for (unsigned int p = 0; p < rx_n; ++p) {
                unsigned int ecnt = eni_slaves[i].rx_pdos[p].entry_count;
                rx_entries[p] = ecnt ? (ec_pdo_entry_info_t *)calloc(ecnt, sizeof(*rx_entries[p])) : NULL;
                for (unsigned int e = 0; e < ecnt; ++e) {
                    rx_entries[p][e].index = eni_slaves[i].rx_pdos[p].entries[e].index;
                    rx_entries[p][e].subindex = eni_slaves[i].rx_pdos[p].entries[e].subindex;
                    rx_entries[p][e].bit_length = eni_slaves[i].rx_pdos[p].entries[e].bitlen;
                }
                rx_infos[p].index = eni_slaves[i].rx_pdos[p].pdo_index;
                rx_infos[p].entries = rx_entries[p];
                rx_infos[p].n_entries = ecnt;
            }
            for (unsigned int p = 0; p < tx_n; ++p) {
                unsigned int ecnt = eni_slaves[i].tx_pdos[p].entry_count;
                tx_entries[p] = ecnt ? (ec_pdo_entry_info_t *)calloc(ecnt, sizeof(*tx_entries[p])) : NULL;
                for (unsigned int e = 0; e < ecnt; ++e) {
                    tx_entries[p][e].index = eni_slaves[i].tx_pdos[p].entries[e].index;
                    tx_entries[p][e].subindex = eni_slaves[i].tx_pdos[p].entries[e].subindex;
                    tx_entries[p][e].bit_length = eni_slaves[i].tx_pdos[p].entries[e].bitlen;
                }
                tx_infos[p].index = eni_slaves[i].tx_pdos[p].pdo_index;
                tx_infos[p].entries = tx_entries[p];
                tx_infos[p].n_entries = ecnt;
            }

            ec_sync_info_t syncs[] = {
                {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                {2, EC_DIR_OUTPUT, (uint8_t)rx_n, rx_infos, EC_WD_ENABLE},
                {3, EC_DIR_INPUT, (uint8_t)tx_n, tx_infos, EC_WD_DISABLE},
                {0xFF, (ec_direction_t)0, 0, NULL, EC_WD_DISABLE}
            };

            if (ecrt_slave_config_pdos(h->sc[i], EC_END, syncs)) {
                for (unsigned int p = 0; p < rx_n; ++p) free(rx_entries[p]);
                for (unsigned int p = 0; p < tx_n; ++p) free(tx_entries[p]);
                free(rx_entries); free(tx_entries); free(rx_infos); free(tx_infos);
                motor_api_free_eni_slaves(eni_slaves, cnt);
                ecrt_release_master(h->master); free(h); return MA_ERR_CONFIG;
            }
            for (unsigned int p = 0; p < rx_n; ++p) free(rx_entries[p]);
            for (unsigned int p = 0; p < tx_n; ++p) free(tx_entries[p]);
            free(rx_entries);
            free(tx_entries);
            free(rx_infos);
            free(tx_infos);
        }
    } else {
        static ec_pdo_entry_info_t device_pdo_entries[] = {
            {0x6040, 0x00, 16}, {0x6060, 0x00, 8}, {0x607A, 0x00, 32}, {0x60B8, 0x00, 16},
            {0x603F, 0x00, 16}, {0x6041, 0x00, 16}, {0x6064, 0x00, 32}, {0x6061, 0x00, 8}, {0x60B9, 0x00, 16}, {0x60BA, 0x00, 32}, {0x60F4, 0x00, 32}, {0x60FD, 0x00, 32}, {0x213F, 0x00, 16},
        };
        static ec_pdo_info_t device_pdos[] = {
            {0x1600, 4, device_pdo_entries + 0},
            {0x1A00, 9, device_pdo_entries + 4},
        };
        static ec_sync_info_t device_syncs[] = {
            {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
            {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
            {2, EC_DIR_OUTPUT, 1, device_pdos + 0, EC_WD_ENABLE},
            {3, EC_DIR_INPUT, 1, device_pdos + 1, EC_WD_DISABLE},
            {0xFF, (ec_direction_t)0, 0, NULL, EC_WD_DISABLE}
        };
        for (uint16_t i = 0; i < cnt; ++i) {
            if (ecrt_slave_config_pdos(h->sc[i], EC_END, device_syncs)) { if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt); ecrt_release_master(h->master); free(h); return MA_ERR_CONFIG; }
        }
    }

    ec_pdo_entry_reg_t regs[MA_MAX_SLAVES * 24 + 1]; memset(regs, 0, sizeof(regs)); size_t r = 0;
    for (uint16_t i = 0; i < cnt; ++i) {
        const ma_eni_slave_t *s = eni_slaves ? &eni_slaves[i] : NULL;
        if (!s || eni_has_rx_entry(s, 0x6040, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6040, .subindex = 0x00, .offset = &h->out[i].controlWord };
        if (!s || eni_has_rx_entry(s, 0x6060, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6060, .subindex = 0x00, .offset = &h->out[i].workModeOut };
        if (!s || eni_has_rx_entry(s, 0x607A, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x607A, .subindex = 0x00, .offset = &h->out[i].targetPosition };
        if (!s || eni_has_rx_entry(s, 0x60B8, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60B8, .subindex = 0x00, .offset = &h->out[i].touchProbeFunc };
        if (!s || eni_has_rx_entry(s, 0x60C2, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60C2, .subindex = 0x00, .offset = &h->out[i].interpolationCtrl };
        if (!s || eni_has_tx_entry(s, 0x6041, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6041, .subindex = 0x00, .offset = &h->in[i].statusword };
        if (!s || eni_has_tx_entry(s, 0x6064, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6064, .subindex = 0x00, .offset = &h->in[i].actualPosition };
        if (!s || eni_has_tx_entry(s, 0x606C, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x606C, .subindex = 0x00, .offset = &h->in[i].actualVelocity };
        if (!s || eni_has_tx_entry(s, 0x6077, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6077, .subindex = 0x00, .offset = &h->in[i].actualTorque };
        if (!s || eni_has_tx_entry(s, 0x6061, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x6061, .subindex = 0x00, .offset = &h->in[i].workModeIn };
        if (!s || eni_has_tx_entry(s, 0x603F, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x603F, .subindex = 0x00, .offset = &h->in[i].errorCode };
        if (!s || eni_has_tx_entry(s, 0x60F4, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60F4, .subindex = 0x00, .offset = &h->in[i].followingError };
        if (!s || eni_has_tx_entry(s, 0x60FD, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60FD, .subindex = 0x00, .offset = &h->in[i].digitalInputs };
        if (!s || eni_has_tx_entry(s, 0x60B9, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60B9, .subindex = 0x00, .offset = &h->in[i].touchProbeStatus };
        if (!s || eni_has_tx_entry(s, 0x60BA, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x60BA, .subindex = 0x00, .offset = &h->in[i].touchProbePos };
        if (!s || eni_has_tx_entry(s, 0x213F, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x213F, .subindex = 0x00, .offset = &h->in[i].servoErrorCode };
        if (!s || eni_has_tx_entry(s, 0x2026, 0)) regs[r++] = (ec_pdo_entry_reg_t){ .alias = 0, .position = h->position[i], .vendor_id = h->vendor_id[i], .product_code = h->product_code[i], .index = 0x2026, .subindex = 0x00, .offset = &h->in[i].brakeDelay };
    }
    regs[r] = (ec_pdo_entry_reg_t){0};
    if (ecrt_domain_reg_pdo_entry_list(h->domain, regs)) { if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt); ecrt_release_master(h->master); free(h); return MA_ERR_CONFIG; }

    /* DC 配置：选 0 号从站为参考时钟，统一 Sync0 周期 */
    ecrt_master_select_reference_clock(h->master, h->sc[0]);
    for (uint16_t i = 0; i < cnt; ++i) {
        if (h->vendor_id[i] == 0x000116C7) {
            (void)ecrt_slave_config_dc(h->sc[i], 0x0300, h->dc_sync0_period_ns, 0, 0, 0);
        }
    }

    if (ecrt_master_activate(h->master)) { ecrt_release_master(h->master); free(h); return MA_ERR_INIT; }
    h->domain_pd = ecrt_domain_data(h->domain); if (!h->domain_pd) { ecrt_release_master(h->master); free(h); return MA_ERR_INIT; }
    h->barrier_armed = 0; h->barrier_start_ns = 0; h->barrier_delay_ns = 1000000000ULL; h->motion_started = 0;
    memset(h->seen_enabled, 0, sizeof(h->seen_enabled));
    /* 创建后打印已注册 PDO 列表 */
    for (uint16_t i = 0; i < cnt; ++i) {
        printf("[PDO] Slave position=%u vid=0x%08X pid=0x%08X\n", h->position[i], h->vendor_id[i], h->product_code[i]);
        if (eni_slaves) {
            printf("  Rx:");
            unsigned int rx_n = eni_slaves[i].rx_pdo_count;
            for (unsigned int p = 0; p < rx_n; ++p) {
                uint16_t pidx = eni_slaves[i].rx_pdos[p].pdo_index;
                unsigned int ecnt = eni_slaves[i].rx_pdos[p].entry_count;
                printf(" [0x%04X]", pidx);
                for (unsigned int e = 0; e < ecnt; ++e) {
                    uint16_t ix = eni_slaves[i].rx_pdos[p].entries[e].index;
                    uint8_t  si = eni_slaves[i].rx_pdos[p].entries[e].subindex;
                    uint8_t  bl = eni_slaves[i].rx_pdos[p].entries[e].bitlen;
                    printf(" 0x%04X:%u %u", ix, si, bl);
                }
            }
            printf("\n  Tx:");
            unsigned int tx_n = eni_slaves[i].tx_pdo_count;
            for (unsigned int p = 0; p < tx_n; ++p) {
                uint16_t pidx = eni_slaves[i].tx_pdos[p].pdo_index;
                unsigned int ecnt = eni_slaves[i].tx_pdos[p].entry_count;
                printf(" [0x%04X]", pidx);
                for (unsigned int e = 0; e < ecnt; ++e) {
                    uint16_t ix = eni_slaves[i].tx_pdos[p].entries[e].index;
                    uint8_t  si = eni_slaves[i].tx_pdos[p].entries[e].subindex;
                    uint8_t  bl = eni_slaves[i].tx_pdos[p].entries[e].bitlen;
                    printf(" 0x%04X:%u %u", ix, si, bl);
                }
            }
            printf("\n");
        } else {
            printf("  Rx: 0x6040:0 16, 0x6060:0 8, 0x607A:0 32, 0x60B8:0 16\n");
            printf("  Tx: 0x6041:0 16, 0x6064:0 32, 0x6061:0 8, 0x603F:0 16, 0x60F4:0 32, 0x60FD:0 32, 0x60B9:0 16, 0x60BA:0 32, 0x213F:0 16\n");
        }
        printf("[OFFS%d] out:6040=%u 6060=%u 607A=%u 60C2=%u | in:6041=%u 6064=%u 6061=%u 606C=%u 6077=%u 603F=%u 2026=%u\n",
               i,
               h->out[i].controlWord, h->out[i].workModeOut, h->out[i].targetPosition, h->out[i].interpolationCtrl,
               h->in[i].statusword, h->in[i].actualPosition, h->in[i].workModeIn, h->in[i].actualVelocity, h->in[i].actualTorque, h->in[i].errorCode, h->in[i].brakeDelay);
    }
    *out_handle = (struct motor_api_handle *)h; if (out_slave_count) *out_slave_count = cnt; if (eni_slaves) motor_api_free_eni_slaves(eni_slaves, cnt);
    return MA_OK;
}

/*
 * 函数: motor_api_destroy
 * 功能: 释放主站资源与句柄。
 */
EXTERNFUNC ma_status_t motor_api_destroy(struct motor_api_handle *handle) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle; if (!h) return MA_ERR_PARAM;
    ecrt_release_master(h->master);
    pthread_mutex_destroy(&h->cmd_mutex);
    free(h);
    return MA_OK;
}

/*
 * 函数: motor_api_start_http
 * 功能: 启动 HTTP 服务线程。
 */
EXTERNFUNC ma_status_t motor_api_start_http(struct motor_api_handle *handle, int port) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle; if (!h) return MA_ERR_PARAM;
    h->http_port = port; h->stop = 0;
    if (pthread_create(&h->http_thread, NULL, http_thread_fn, h) != 0) return MA_ERR_RUNTIME;
    return MA_OK;
}

/*
 * 函数: motor_api_stop_http
 * 功能: 停止 HTTP 服务线程并等待退出。
 */
EXTERNFUNC ma_status_t motor_api_stop_http(struct motor_api_handle *handle) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle; if (!h) return MA_ERR_PARAM;
    h->stop = 1; if (h->http_thread) pthread_join(h->http_thread, NULL);
    return MA_OK;
}

/*
 * 函数: motor_api_set_command
 * 功能: 更新运行命令（线程安全）。
 */
EXTERNFUNC ma_status_t motor_api_set_command(struct motor_api_handle *handle, bool run, int dir, int step) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle; if (!h) return MA_ERR_PARAM;
    set_cmd_locked(h, run, dir, step);
    return MA_OK;
}

/*
 * 函数: motor_api_format_diag_json
 * 功能: 诊断信息格式化为 JSON。
 */
EXTERNFUNC ma_status_t motor_api_format_diag_json(struct motor_api_handle *handle, char *buf, size_t buf_size) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle; if (!h) return MA_ERR_PARAM;
    return format_diag(h, buf, buf_size);
}

/*
 * 函数: motor_api_run_once
 * 功能: 周期性控制入口，包含状态机推进、目标更新、同步栅栏与调试输出。
 * 注意: 需以固定周期调用（例如 4ms）。
 */
EXTERNFUNC ma_status_t motor_api_run_once(struct motor_api_handle *handle) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle; if (!h) return MA_ERR_PARAM;
    ecrt_master_application_time(h->master, monotonic_ns());
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
        MA_WR_S8(h, h->out[i].interpolationCtrl, (int8_t)1);
    }
    ecrt_master_receive(h->master);
    ecrt_domain_process(h->domain);
    ecrt_master_sync_slave_clocks(h->master);
    check_domain_state(h);
    check_master_state(h);
    check_slave_states(h);
    static int dbg_tick = 0; dbg_tick++;
    /* 逐轴推进状态机与写入控制字/模式 */
    for (uint16_t i = 0; i < h->slave_count; ++i) {
        MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
        MA_WR_S8(h, h->out[i].interpolationCtrl, (int8_t)1);
        uint16_t status_i = MA_RD_U16(h, h->in[i].statusword);
        if ((status_i & 0x6F) == 0x27) h->seen_enabled[i] = true; else h->seen_enabled[i] = false;
        uint16_t control_i = 0x06;
        if (!h->servo_enabled[i]) {
            /* 依据 CiA-402 标准用状态字低位掩码推进控制字序列 */
            switch (status_i & 0x6F) {
                case 0x00: control_i = 0x06; break;
                case 0x40: control_i = 0x06; break;
                case 0x21:
                    control_i = 0x07;
                    h->csp_target[i] = MA_RD_S32(h, h->in[i].actualPosition);
                    MA_WR_S32(h, h->out[i].targetPosition, h->csp_target[i]);
                    break;
                case 0x23: control_i = 0x0F; break;
                case 0x27:
                    control_i = 0x0F;
                    if (!h->servo_enabled[i]) {
                        h->servo_enabled[i] = true;
                        if (dbg_tick % 100 == 0) {
                            int32_t ap = MA_RD_S32(h, h->in[i].actualPosition);
                            printf("[ENABLED%d] sw:0x%04X act:%d\n", i, status_i, ap);
                        }
                    }
                    h->csp_warmup[i] = 10;
                    h->csp_target[i] = MA_RD_S32(h, h->in[i].actualPosition);
                    break;
                default: control_i = 0x06; break;
            }
            /* 检测故障位并执行快速复位（0x0080） */
            if ((status_i & 0x0040) && !(status_i & 0x0001)) {
                MA_WR_U16(h, h->out[i].controlWord, 0x0000);
                MA_WR_U16(h, h->out[i].controlWord, 0x0080);
            }
            MA_WR_U16(h, h->out[i].controlWord, control_i);
            MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
            MA_WR_S8(h, h->out[i].interpolationCtrl, (int8_t)1);
            if (dbg_tick % 500 == 0) {
                int ack = (status_i & 0x1000) ? 1 : 0;
                int trg = (status_i & 0x0400) ? 1 : 0;
                int32_t ap = MA_RD_S32(h, h->in[i].actualPosition);
                printf("[EN%d] sw:0x%04X ctrl:0x%04X mode:%d ack12:%d trg10:%d act:%d\n", i, status_i, control_i, MA_RD_S8(h, h->in[i].workModeIn), ack, trg, ap);
            }
        } else {
            h->time_cnt[i]++;
            /* 延迟栅栏未触发：保位到实际位置，写 0x0F 与模式 */
            if (!h->motion_started) {
                h->csp_target[i] = MA_RD_S32(h, h->in[i].actualPosition);
                MA_WR_S32(h, h->out[i].targetPosition, h->csp_target[i]);
                MA_WR_U16(h, h->out[i].controlWord, 0x0F);
                MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
                MA_WR_S8(h, h->out[i].interpolationCtrl, (int8_t)1);
                h->last_actual_pos[i] = MA_RD_S32(h, h->in[i].actualPosition);
                if (dbg_tick % 100 == 0) {
                    printf("[GATE%d] hold tgt:%d act:%d sw:0x%04X mode:%d\n", i,
                           MA_RD_S32(h, h->out[i].targetPosition),
                           h->last_actual_pos[i], status_i,
                           MA_RD_S8(h, h->in[i].workModeIn));
                }
            } else {
                /* 延迟栅栏已触发：按命令增量推进目标（限幅与预热） */
                pthread_mutex_lock(&h->cmd_mutex);
                bool run = h->cmd_run;
                int dir = h->cmd_dir;
                int step = h->cmd_step;
                pthread_mutex_unlock(&h->cmd_mutex);
                int delta = run ? (dir * step) : 0;
                if (delta > MA_MAX_DELTA_PER_CYCLE) delta = MA_MAX_DELTA_PER_CYCLE;
                if (delta < -MA_MAX_DELTA_PER_CYCLE) delta = -MA_MAX_DELTA_PER_CYCLE;
                if (h->csp_warmup[i] > 0) { h->csp_target[i] = MA_RD_S32(h, h->in[i].actualPosition); h->csp_warmup[i]--; }
                else { h->csp_target[i] += delta; }
                MA_WR_S32(h, h->out[i].targetPosition, h->csp_target[i]);
                MA_WR_U16(h, h->out[i].controlWord, 0x0F);
                MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
                MA_WR_S8(h, h->out[i].interpolationCtrl, (int8_t)1);
                h->last_actual_pos[i] = MA_RD_S32(h, h->in[i].actualPosition);
                if (dbg_tick % 500 == 0) {
                    printf("[RUN%d] tgt:%d act:%d sw:0x%04X mode:%d\n", i,
                           MA_RD_S32(h, h->out[i].targetPosition),
                           h->last_actual_pos[i], status_i,
                           MA_RD_S8(h, h->in[i].workModeIn));
                }
            }
        }
    }
    {
        /* 栅栏逻辑：检测全轴使能后武装，延时 1s 后统一开始运动 */
        pthread_mutex_lock(&h->cmd_mutex); bool run = h->cmd_run; pthread_mutex_unlock(&h->cmd_mutex);
        int all_enabled = 1; for (uint16_t i = 0; i < h->slave_count; ++i) all_enabled = all_enabled && h->seen_enabled[i];
        if (!h->motion_started && run) {
            if (!h->barrier_armed && all_enabled) {
                h->barrier_armed = 1; h->barrier_start_ns = monotonic_ns();
                printf("[BARRIER_ARM] all at 0x027 (enabled), wait 1s\n");
            }
            if (h->barrier_armed) {
                uint64_t now = monotonic_ns();
                if (now - h->barrier_start_ns >= h->barrier_delay_ns) {
                    for (uint16_t i = 0; i < h->slave_count; ++i) {
                        h->csp_target[i] = MA_RD_S32(h, h->in[i].actualPosition);
                        MA_WR_S32(h, h->out[i].targetPosition, h->csp_target[i]);
                        MA_WR_U16(h, h->out[i].controlWord, 0x0F);
                        MA_WR_S8(h, h->out[i].workModeOut, (int8_t)MA_MODE_CSP);
                    }
                    printf("[BARRIER_FIRE] synchronized motion start after 1s (enabled), slaves=%u\n", h->slave_count);
                    h->motion_started = 1; h->barrier_armed = 0;
                }
            }
        }
    }
    /* 提交域数据并发送到主站 */
    ecrt_domain_queue(h->domain);
    ecrt_master_send(h->master);
    return MA_OK;
}
