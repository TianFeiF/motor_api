/*
 * motor_api_common.c
 *
 * 模块职责：
 * - 提供“非 EtherCAT(ecrt) 依赖”的公共能力：ENI(XML) 解析、字符串/属性容错扫描、
 *   选择最新 ENI 文件、从 ENI 获取产品名称映射、单调时钟获取等。
 *
 * 设计取舍：
 * - 解析逻辑采用“轻量级字符串扫描”，不引入第三方 XML 库，便于在最小依赖环境中构建；
 * - 为增强兼容性，对不同厂商 ENI 中属性命名差异(VendorId/VendorID 等)做了容错处理；
 * - 对外暴露的接口统一使用 ma_status_t，内部 helper 返回 int 便于局部错误定位。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <time.h>
#include <dirent.h>
#include <sys/stat.h>

#include "motor_api_common.h"

/*
 * 获取单调时钟（纳秒）
 * 说明：用于周期控制时间戳与超时等待，避免系统时间跳变导致的计时异常。
 */
uint64_t motor_api_monotonic_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/*
 * strncasestr_local
 * 功能：在限定长度的缓冲区中进行“不区分大小写”的子串查找。
 * 说明：标准库没有 strncasestr，这里提供最小实现以用于 XML 容错扫描。
 */
static const char *strncasestr_local(const char *hay, size_t hay_len, const char *needle) {
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

/*
 * parse_hex_or_dec_local
 * 功能：解析十六进制/十进制数字字符串。
 * 支持形式：
 * - "#x1A00" / "#X1A00"
 * - "0x1A00" / "0X1A00"
 * - "x1A00"  / "X1A00"
 * - "1234"
 */
static inline int parse_hex_or_dec_local(const char *s) {
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

/*
 * motor_api_eni_has_rx_entry
 * 功能：判断 ENI 从站的 RxPdo 列表是否包含指定 entry(index/subindex)。
 * 返回：1=包含，0=不包含。
 */
int motor_api_eni_has_rx_entry(const ma_eni_slave_t *s, uint16_t index, uint8_t subindex) {
    if (!s) return 0;
    for (unsigned int p = 0; p < s->rx_pdo_count; ++p) {
        for (unsigned int e = 0; e < s->rx_pdos[p].entry_count; ++e) {
            if (s->rx_pdos[p].entries[e].index == index && s->rx_pdos[p].entries[e].subindex == subindex) return 1;
        }
    }
    return 0;
}

/*
 * motor_api_eni_has_tx_entry
 * 功能：判断 ENI 从站的 TxPdo 列表是否包含指定 entry(index/subindex)。
 * 返回：1=包含，0=不包含。
 */
int motor_api_eni_has_tx_entry(const ma_eni_slave_t *s, uint16_t index, uint8_t subindex) {
    if (!s) return 0;
    for (unsigned int p = 0; p < s->tx_pdo_count; ++p) {
        for (unsigned int e = 0; e < s->tx_pdos[p].entry_count; ++e) {
            if (s->tx_pdos[p].entries[e].index == index && s->tx_pdos[p].entries[e].subindex == subindex) return 1;
        }
    }
    return 0;
}

/*
 * parse_attr_int_range_local
 * 功能：在 [beg,end) 片段内查找属性 key="..." 并将值解析为整数（支持十六进制形式）。
 * 返回：解析成功返回整数；失败返回 -1。
 */
static int parse_attr_int_range_local(const char *beg, const char *end, const char *key) {
    if (!beg || !end || !key) return -1;
    const char *k = strncasestr_local(beg, (size_t)(end - beg), key);
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
    return parse_hex_or_dec_local(bufv);
}

/*
 * parse_attr_str_range_local
 * 功能：在 [beg,end) 片段内查找属性 key="..." 并拷贝字符串到 out。
 * 返回：0=成功，负值=失败。
 */
static int parse_attr_str_range_local(const char *beg, const char *end, const char *key, char *out, size_t out_len) {
    if (!beg || !end || !key || !out || out_len == 0) return -1;
    const char *k = strncasestr_local(beg, (size_t)(end - beg), key);
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

/*
 * parse_tag_text_range_local
 * 功能：在 [beg,end) 片段内读取 <tag>...</tag> 的文本内容。
 * 兼容：支持 <![CDATA[...]]> 包裹的内容。
 * 返回：0=成功，负值=失败。
 */
static int parse_tag_text_range_local(const char *beg, const char *end, const char *tag, char *out, size_t out_len) {
    if (!beg || !end || !tag || !out || out_len == 0) return -1;
    char open[64];
    char close[64];
    snprintf(open, sizeof(open), "<%s", tag);
    snprintf(close, sizeof(close), "</%s>", tag);
    const char *tb = strncasestr_local(beg, (size_t)(end - beg), open);
    if (!tb || tb >= end) return -2;
    const char *gt = strchr(tb, '>');
    if (!gt || gt >= end) return -3;
    const char *te = strncasestr_local(gt + 1, (size_t)(end - (gt + 1)), close);
    if (!te || te <= gt) return -4;
    const char *v = gt + 1;
    if (v + 9 < te && strncmp(v, "<![CDATA[", 9) == 0) {
        v += 9;
        const char *ce = strncasestr_local(v, (size_t)(te - v), "]]>");
        if (ce && ce <= te) te = ce;
    }
    size_t n = (size_t)(te - v);
    if (n >= out_len) n = out_len - 1;
    memcpy(out, v, n);
    out[n] = '\0';
    return 0;
}

/*
 * scan_unique_xml_local
 * 功能：扫描目录内所有 .xml 文件，选择“修改时间最新”的那一个。
 * 返回：
 * - >=1：目录内 xml 文件数量，并已写出 out_path
 * - <0 ：失败（目录不可读/无 xml）
 */
static int scan_unique_xml_local(const char *dir, char *out_path, size_t out_size) {
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

/*
 * motor_api_find_latest_eni_xml
 * 对外封装：将 scan_unique_xml_local 的结果映射为 ma_status_t。
 */
ma_status_t motor_api_find_latest_eni_xml(const char *dir, char *out_path, size_t out_size) {
    if (!dir || !out_path || out_size == 0) return MA_ERR_PARAM;
    int rc = scan_unique_xml_local(dir, out_path, out_size);
    return (rc >= 1) ? MA_OK : MA_ERR_IO;
}

/*
 * motor_api_fill_product_names_from_eni
 * 功能：从 ENI 中尽量提取“产品名称”，填入 out_product_names。
 *
 * 解析策略：
 * 1) 默认先把所有 name 填成 "PID_0x%08X"；
 * 2) 扫描 <EtherCATInfo> 的 <Device> 列表，建立 ProductCode->Name 的映射；
 * 3) 若 ENI 存在 <SlaveList>，优先用 SlaveList 中的 Name/ProductName 覆盖；
 * 4) 最后再次用 ProductCode->Name 映射做补充覆盖（匹配到则替换）。
 */
ma_status_t motor_api_fill_product_names_from_eni(const char *eni_path,
                                                  const uint32_t *product_codes,
                                                  uint16_t count,
                                                  char (*out_product_names)[64],
                                                  uint16_t max_slaves) {
    if (!eni_path || !product_codes || !out_product_names || max_slaves == 0) return MA_ERR_PARAM;
    uint16_t n = count < max_slaves ? count : max_slaves;
    for (uint16_t i = 0; i < n; ++i) {
        snprintf(out_product_names[i], 64, "PID_0x%08X", product_codes[i]);
    }

    FILE *fp = fopen(eni_path, "rb");
    if (!fp) return MA_ERR_IO;
    if (fseek(fp, 0, SEEK_END) != 0) { fclose(fp); return MA_ERR_IO; }
    long len = ftell(fp);
    if (len <= 0) { fclose(fp); return MA_ERR_IO; }
    if (fseek(fp, 0, SEEK_SET) != 0) { fclose(fp); return MA_ERR_IO; }
    char *buf = (char *)malloc((size_t)len + 1);
    if (!buf) { fclose(fp); return MA_ERR_RUNTIME; }
    size_t rd = fread(buf, 1, (size_t)len, fp);
    fclose(fp);
    buf[rd] = '\0';

    uint32_t *map_pcodes = (uint32_t *)calloc(n, sizeof(uint32_t));
    char (*map_names)[64] = (char (*)[64])calloc(n, sizeof(*map_names));
    uint16_t map_n = 0;

    if (map_pcodes && map_names) {
        const char *scan_map = buf;
        while (1) {
            const char *ibeg = strncasestr_local(scan_map, rd - (size_t)(scan_map - buf), "<EtherCATInfo");
            if (!ibeg) break;
            const char *itag_end = strchr(ibeg, '>');
            if (!itag_end) break;
            const char *iend = strncasestr_local(itag_end, rd - (size_t)(itag_end - buf), "</EtherCATInfo>");
            if (!iend) break;
            const char *p = itag_end;
            while (1) {
                const char *dbeg = strncasestr_local(p, (size_t)(iend - p), "<Device");
                if (!dbeg) break;
                const char *dtag_end = strchr(dbeg, '>');
                if (!dtag_end || dtag_end > iend) break;
                const char *dend = strncasestr_local(dtag_end, (size_t)(iend - dtag_end), "</Device>");
                if (!dend) break;
                const char *type_beg = strncasestr_local(dtag_end, (size_t)(dend - dtag_end), "<Type");
                const char *type_gt = type_beg ? strchr(type_beg, '>') : NULL;
                int pcv = -1;
                if (type_beg && type_gt && type_gt <= dend) pcv = parse_attr_int_range_local(type_beg, type_gt, "ProductCode");
                char name[64];
                name[0] = '\0';
                int okn = 0;
                if (parse_tag_text_range_local(dtag_end, dend, "Name", name, sizeof(name)) == 0) okn = 1;
                if (!okn) {
                    char ttext[64];
                    ttext[0] = '\0';
                    if (type_gt && parse_tag_text_range_local(type_gt, dend, "Type", ttext, sizeof(ttext)) == 0) {
                        snprintf(name, sizeof(name), "%s", ttext);
                        okn = 1;
                    }
                }
                if (pcv >= 0 && okn && map_n < n) {
                    map_pcodes[map_n] = (uint32_t)pcv;
                    snprintf(map_names[map_n], 64, "%s", name);
                    map_n++;
                }
                p = dend;
            }
            scan_map = iend;
        }
    }

    const char *list_beg = strncasestr_local(buf, rd, "<SlaveList");
    const char *list_end = list_beg ? strncasestr_local(list_beg, (size_t)(rd - (list_beg - buf)), "</SlaveList>") : NULL;
    if (list_beg && list_end) {
        const char *sp = list_beg;
        uint16_t idx = 0;
        while (idx < n) {
            const char *sbeg = strncasestr_local(sp, (size_t)(list_end - sp), "<Slave");
            if (!sbeg) break;
            const char *stag_end = strchr(sbeg, '>');
            if (!stag_end || stag_end > list_end) break;
            const char *send = strncasestr_local(stag_end, (size_t)(list_end - stag_end), "</Slave>");
            if (!send) send = stag_end;
            char name[64];
            name[0] = '\0';
            int ok = 0;
            if (parse_attr_str_range_local(sbeg, stag_end, "Name", name, sizeof(name)) == 0) ok = 1;
            if (!ok && parse_attr_str_range_local(sbeg, stag_end, "ProductName", name, sizeof(name)) == 0) ok = 1;
            if (!ok && parse_tag_text_range_local(stag_end, send, "Name", name, sizeof(name)) == 0) ok = 1;
            if (!ok && parse_tag_text_range_local(stag_end, send, "ProductName", name, sizeof(name)) == 0) ok = 1;
            if (ok) snprintf(out_product_names[idx], 64, "%s", name);
            sp = stag_end;
            idx++;
        }
    }

    if (map_pcodes && map_names) {
        for (uint16_t i = 0; i < n; ++i) {
            for (uint16_t k = 0; k < map_n; ++k) {
                if (map_pcodes[k] == product_codes[i]) {
                    snprintf(out_product_names[i], 64, "%s", map_names[k]);
                    break;
                }
            }
        }
    }

    free(map_pcodes);
    free(map_names);
    free(buf);
    return MA_OK;
}

/*
 * motor_api_read_eni
 * 功能：解析 ENI(XML) 并返回从站数组及其 PDO 映射信息。
 *
 * 重要说明：
 * - 该实现采用字符串扫描，既能处理典型 ENI，也能容忍一定的标签/属性差异；
 * - 返回的 slaves 及其内部 entries/pdos 全部为动态分配，必须调用 motor_api_free_eni_slaves 释放；
 * - 若 out_slaves 为 NULL，则本函数会在内部解析完成后释放所有临时内存（只返回 count 与可选数组）。
 */
ma_status_t motor_api_read_eni(const char *eni_path,
                              uint32_t *vendor_ids,
                              uint32_t *product_codes,
                              uint16_t *positions,
                              uint16_t max_slaves,
                              uint16_t *out_count,
                              ma_eni_slave_t **out_slaves) {
    if (!eni_path || !out_count) return MA_ERR_PARAM;
    FILE *fp = fopen(eni_path, "rb");
    if (!fp) return MA_ERR_IO;
    fseek(fp, 0, SEEK_END);
    long len = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    char *buf = (char *)malloc((size_t)len + 1);
    if (!buf) { fclose(fp); return MA_ERR_RUNTIME; }
    size_t rd = fread(buf, 1, (size_t)len, fp);
    fclose(fp);
    buf[rd] = '\0';

    uint16_t count = 0;
    ma_eni_slave_t *slaves = NULL;

    const char *p = buf;
    size_t remaining = rd;

    {
        const char *list_beg = strncasestr_local(buf, rd, "<SlaveList");
        const char *list_end = list_beg ? strncasestr_local(list_beg, (size_t)(rd - (list_beg - buf)), "</SlaveList>") : NULL;
        if (list_beg && list_end) {
            const char *sp = list_beg;
            while (count < max_slaves) {
                const char *sbeg = strncasestr_local(sp, (size_t)(list_end - sp), "<Slave");
                if (!sbeg) break;
                const char *stag_end = strchr(sbeg, '>');
                if (!stag_end || stag_end > list_end) break;
                const char *send = strncasestr_local(stag_end, (size_t)(list_end - stag_end), "</Slave>");
                if (!send) send = stag_end;
                uint16_t ps = (uint16_t)(count);
                uint32_t v = 0, pc = 0;
                int pv = parse_attr_int_range_local(sbeg, stag_end, "Position");
                if (pv >= 0) ps = (uint16_t)pv;
                int vv = parse_attr_int_range_local(sbeg, stag_end, "VendorId");
                if (vv < 0) vv = parse_attr_int_range_local(sbeg, stag_end, "VendorID");
                if (vv >= 0) v = (uint32_t)vv;
                int pp = parse_attr_int_range_local(sbeg, stag_end, "ProductCode");
                if (pp >= 0) pc = (uint32_t)pp;

                ma_eni_pdo_t *rx_pdos = NULL;
                unsigned int rx_cnt = 0;
                ma_eni_pdo_t *tx_pdos = NULL;
                unsigned int tx_cnt = 0;

                const char *scan = stag_end;
                while (scan < send) {
                    const char *rx = strncasestr_local(scan, (size_t)(send - scan), "<RxPdo");
                    const char *tx = strncasestr_local(scan, (size_t)(send - scan), "<TxPdo");
                    const char *pdo = strncasestr_local(scan, (size_t)(send - scan), "<Pdo");
                    const char *begp = NULL;
                    int kind = -1;
                    if (rx && (!tx || rx < tx) && (!pdo || rx < pdo)) { begp = rx; kind = 0; }
                    else if (tx && (!rx || tx < rx) && (!pdo || tx < pdo)) { begp = tx; kind = 1; }
                    else if (pdo) { begp = pdo; kind = 2; }
                    else break;
                    const char *endp = NULL;
                    if (kind == 0) endp = strncasestr_local(begp, (size_t)(send - begp), "</RxPdo>");
                    else if (kind == 1) endp = strncasestr_local(begp, (size_t)(send - begp), "</TxPdo>");
                    else endp = strncasestr_local(begp, (size_t)(send - begp), "</Pdo>");
                    if (!endp) break;
                    int pdo_index = 0;
                    {
                        const char *idx_tag = strncasestr_local(begp, (size_t)(endp - begp), "<Index>");
                        if (idx_tag) {
                            const char *gt = strchr(idx_tag, '>');
                            const char *lt = gt ? strncasestr_local(gt + 1, (size_t)(endp - (gt + 1)), "</Index>") : NULL;
                            if (gt && lt && lt > gt) {
                                char num[64];
                                size_t nn = (size_t)(lt - (gt + 1));
                                if (nn > sizeof(num) - 1) nn = sizeof(num) - 1;
                                memcpy(num, gt + 1, nn);
                                num[nn] = '\0';
                                pdo_index = parse_hex_or_dec_local(num);
                            }
                        }
                        if (pdo_index == 0) {
                            const char *gt = strchr(begp, '>');
                            int iv = parse_attr_int_range_local(begp, gt ? gt : endp, "Index");
                            if (iv > 0) pdo_index = iv;
                        }
                    }
                    ma_eni_pdo_entry_t *ents = NULL;
                    unsigned int ecnt = 0;
                    const char *ep = begp;
                    while (1) {
                        const char *ebeg = strncasestr_local(ep, (size_t)(endp - ep), "<Entry");
                        if (!ebeg) break;
                        const char *etag_end = strchr(ebeg, '>');
                        if (!etag_end || etag_end > endp) break;
                        const char *eend = strncasestr_local(etag_end, (size_t)(endp - etag_end), "</Entry>");
                        if (!eend) eend = etag_end;
                        int e_index = 0, e_sub = 0, e_bit = 0;
                        const char *it = strncasestr_local(ebeg, (size_t)(eend - ebeg), "<Index>");
                        if (it) {
                            const char *gt = strchr(it, '>');
                            const char *lt = gt ? strncasestr_local(gt + 1, (size_t)(eend - (gt + 1)), "</Index>") : NULL;
                            if (gt && lt && lt > gt) {
                                char num[64];
                                size_t nn = (size_t)(lt - (gt + 1));
                                if (nn > sizeof(num) - 1) nn = sizeof(num) - 1;
                                memcpy(num, gt + 1, nn);
                                num[nn] = '\0';
                                e_index = parse_hex_or_dec_local(num);
                            }
                        }
                        if (e_index == 0) { int av = parse_attr_int_range_local(ebeg, etag_end, "Index"); if (av > 0) e_index = av; }
                        const char *st = strncasestr_local(ebeg, (size_t)(eend - ebeg), "<SubIndex>");
                        if (st) {
                            const char *gt = strchr(st, '>');
                            const char *lt = gt ? strncasestr_local(gt + 1, (size_t)(eend - (gt + 1)), "</SubIndex>") : NULL;
                            if (gt && lt && lt > gt) {
                                char num[32];
                                size_t nn = (size_t)(lt - (gt + 1));
                                if (nn > sizeof(num) - 1) nn = sizeof(num) - 1;
                                memcpy(num, gt + 1, nn);
                                num[nn] = '\0';
                                e_sub = (int)strtol(num, NULL, 0);
                            }
                        }
                        if (e_sub == 0) { int sv = parse_attr_int_range_local(ebeg, etag_end, "SubIndex"); if (sv >= 0) e_sub = sv; }
                        const char *bt = strncasestr_local(ebeg, (size_t)(eend - ebeg), "<BitLen>");
                        if (bt) {
                            const char *gt = strchr(bt, '>');
                            const char *lt = gt ? strncasestr_local(gt + 1, (size_t)(eend - (gt + 1)), "</BitLen>") : NULL;
                            if (gt && lt && lt > gt) {
                                char num[32];
                                size_t nn = (size_t)(lt - (gt + 1));
                                if (nn > sizeof(num) - 1) nn = sizeof(num) - 1;
                                memcpy(num, gt + 1, nn);
                                num[nn] = '\0';
                                e_bit = (int)strtol(num, NULL, 0);
                            }
                        }
                        if (e_bit == 0) { int bv = parse_attr_int_range_local(ebeg, etag_end, "BitLen"); if (bv > 0) e_bit = bv; }
                        ents = (ma_eni_pdo_entry_t *)realloc(ents, (ecnt + 1) * sizeof(*ents));
                        ents[ecnt].index = (uint16_t)e_index;
                        ents[ecnt].subindex = (uint8_t)e_sub;
                        ents[ecnt].bitlen = (uint8_t)e_bit;
                        ecnt++;
                        ep = eend + 8;
                    }
                    ma_eni_pdo_t pdo_out;
                    pdo_out.pdo_index = (uint16_t)pdo_index;
                    pdo_out.entry_count = ecnt;
                    pdo_out.entries = ents;
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

    while (count < max_slaves) {
        const char *info = strncasestr_local(p, remaining, "<EtherCATInfo>");
        if (!info) break;
        const char *end = strncasestr_local(info, (size_t)(rd - (info - buf)), "</EtherCATInfo>");
        if (!end) end = buf + rd;
        uint32_t v = 0, pc = 0;
        uint16_t ps = count;
        const char *vend = strncasestr_local(info, (size_t)(end - info), "<Id>");
        if (vend) {
            const char *gt = strchr(vend, '>');
            const char *lt = gt ? strchr(gt + 1, '<') : NULL;
            if (gt && lt && lt > gt) {
                char num[32];
                size_t nn = (size_t)(lt - (gt + 1));
                if (nn > sizeof(num) - 1) nn = sizeof(num) - 1;
                memcpy(num, gt + 1, nn);
                num[nn] = '\0';
                v = (uint32_t)strtoul(num, NULL, 10);
            }
        }
        const char *type = strncasestr_local(info, (size_t)(end - info), "ProductCode");
        if (type) {
            const char *eq = strchr(type, '=');
            if (eq && eq < end) {
                const char *vv = eq + 1;
                while (*vv == ' ' || *vv == '"' || *vv == '#') ++vv;
                if (*vv == 'x' || *vv == 'X') ++vv;
                pc = (uint32_t)strtoul(vv, NULL, 16);
            }
        }

        ma_eni_pdo_t *rx_pdos = NULL;
        unsigned int rx_cnt = 0;
        ma_eni_pdo_t *tx_pdos = NULL;
        unsigned int tx_cnt = 0;

        const char *scan = info;
        while (1) {
            const char *rx = strncasestr_local(scan, (size_t)(end - scan), "<RxPdo");
            const char *tx = strncasestr_local(scan, (size_t)(end - scan), "<TxPdo");
            if (!rx && !tx) break;
            int is_rx = 0;
            const char *pdo_beg = NULL;
            if (rx && (!tx || rx < tx)) { is_rx = 1; pdo_beg = rx; }
            else { is_rx = 0; pdo_beg = tx; }
            const char *pdo_end = NULL;
            if (is_rx) pdo_end = strncasestr_local(pdo_beg, (size_t)(end - pdo_beg), "</RxPdo>");
            else pdo_end = strncasestr_local(pdo_beg, (size_t)(end - pdo_beg), "</TxPdo>");
            if (!pdo_end) break;
            int pdo_index = 0x0000;
            {
                const char *idx_tag = strncasestr_local(pdo_beg, (size_t)(pdo_end - pdo_beg), "<Index>");
                if (idx_tag) {
                    const char *gt = strchr(idx_tag, '>');
                    const char *lt = strncasestr_local(gt ? gt + 1 : pdo_beg, (size_t)(pdo_end - (gt ? gt + 1 : pdo_beg)), "</Index>");
                    if (gt && lt && lt > gt) {
                        char num[64];
                        size_t nn = (size_t)(lt - (gt + 1));
                        if (nn > sizeof(num) - 1) nn = sizeof(num) - 1;
                        memcpy(num, gt + 1, nn);
                        num[nn] = '\0';
                        pdo_index = parse_hex_or_dec_local(num);
                    }
                }
            }
            ma_eni_pdo_entry_t *ents = NULL;
            unsigned int ecnt = 0;
            const char *ep = pdo_beg;
            while (1) {
                const char *ebeg = strncasestr_local(ep, (size_t)(pdo_end - ep), "<Entry>");
                if (!ebeg) break;
                const char *eend = strncasestr_local(ebeg, (size_t)(pdo_end - ebeg), "</Entry>");
                if (!eend) break;
                int e_index = 0, e_sub = 0, e_bit = 0;
                const char *it = strncasestr_local(ebeg, (size_t)(eend - ebeg), "<Index>");
                if (it) {
                    const char *gt = strchr(it, '>');
                    const char *lt = strncasestr_local(gt ? gt + 1 : ebeg, (size_t)(eend - (gt ? gt + 1 : ebeg)), "</Index>");
                    if (gt && lt && lt > gt) {
                        char num[64];
                        size_t nn = (size_t)(lt - (gt + 1));
                        if (nn > sizeof(num) - 1) nn = sizeof(num) - 1;
                        memcpy(num, gt + 1, nn);
                        num[nn] = '\0';
                        e_index = parse_hex_or_dec_local(num);
                    }
                }
                const char *st = strncasestr_local(ebeg, (size_t)(eend - ebeg), "<SubIndex>");
                if (st) {
                    const char *gt = strchr(st, '>');
                    const char *lt = strncasestr_local(gt ? gt + 1 : ebeg, (size_t)(eend - (gt ? gt + 1 : ebeg)), "</SubIndex>");
                    if (gt && lt && lt > gt) {
                        char num[32];
                        size_t nn = (size_t)(lt - (gt + 1));
                        if (nn > sizeof(num) - 1) nn = sizeof(num) - 1;
                        memcpy(num, gt + 1, nn);
                        num[nn] = '\0';
                        e_sub = (int)strtol(num, NULL, 0);
                    }
                }
                const char *bt = strncasestr_local(ebeg, (size_t)(eend - ebeg), "<BitLen>");
                if (bt) {
                    const char *gt = strchr(bt, '>');
                    const char *lt = strncasestr_local(gt ? gt + 1 : ebeg, (size_t)(eend - (gt ? gt + 1 : ebeg)), "</BitLen>");
                    if (gt && lt && lt > gt) {
                        char num[32];
                        size_t nn = (size_t)(lt - (gt + 1));
                        if (nn > sizeof(num) - 1) nn = sizeof(num) - 1;
                        memcpy(num, gt + 1, nn);
                        num[nn] = '\0';
                        e_bit = (int)strtol(num, NULL, 0);
                    }
                }
                ents = (ma_eni_pdo_entry_t *)realloc(ents, (ecnt + 1) * sizeof(*ents));
                ents[ecnt].index = (uint16_t)e_index;
                ents[ecnt].subindex = (uint8_t)e_sub;
                ents[ecnt].bitlen = (uint8_t)e_bit;
                ecnt++;
                ep = eend + 8;
            }
            ma_eni_pdo_t pdo;
            pdo.pdo_index = (uint16_t)pdo_index;
            pdo.entry_count = ecnt;
            pdo.entries = ents;
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
        p = end + 15;
        remaining = rd - (size_t)(p - buf);
    }

    if (out_slaves) *out_slaves = slaves;
    else {
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

/*
 * motor_api_free_eni_slaves
 * 功能：释放 read_eni 构造的树形动态内存（slaves -> (rx/tx_pdos -> entries)）。
 */
void motor_api_free_eni_slaves(ma_eni_slave_t *slaves, uint16_t count) {
    if (!slaves) return;
    for (uint16_t i = 0; i < count; ++i) {
        for (unsigned int j = 0; j < slaves[i].rx_pdo_count; ++j) free(slaves[i].rx_pdos[j].entries);
        for (unsigned int j = 0; j < slaves[i].tx_pdo_count; ++j) free(slaves[i].tx_pdos[j].entries);
        free(slaves[i].rx_pdos);
        free(slaves[i].tx_pdos);
    }
    free(slaves);
}
