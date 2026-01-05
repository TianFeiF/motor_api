/*
 * motor_api_net.c
 *
 * 模块职责：
 * - 提供一个轻量级 HTTP 控制服务（基于 socket），用于运行/停止指令下发与诊断查询；
 * - 将网络线程与解析逻辑从 EtherCAT 核心控制逻辑中分离，降低耦合。
 *
 * 接口与线程模型：
 * - motor_api_start_http() 创建并启动 http_thread_fn 线程；
 * - http_thread_fn 在指定端口监听 TCP 连接，逐个 accept 处理请求（串行处理，简单可靠）；
 * - motor_api_stop_http() 设置 stop 标志并 join 线程；
 * - 网络线程通过 motor_api_set_command / motor_api_set_axis_command 修改句柄内指令字段。
 *
 * 安全与限制：
 * - 解析为“最小可用”的字符串扫描，不是完整 HTTP/JSON 实现；
 * - 不处理分块传输、长连接、HTTP 管线等复杂情况；
 * - 单次 recv 读取固定缓冲 4096 字节，适合本项目短请求体场景。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "motor_api_internal.h"

/*
 * set_cmd_locked
 * 功能：在互斥保护下更新全局命令（run/dir/step），并对参数做基本限幅。
 * 说明：
 * - step 强制落在 [1,100000] 以避免异常输入造成过大增量；
 * - dir 只允许 -1/0/1，非法则归零；
 * - 该函数仅供网络线程内部使用（不对外暴露）。
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
 * send_all
 * 功能：尽量把 buf 的 len 字节全部发送出去，避免 send 短写导致响应不完整。
 * 返回：0=成功，-1=失败（连接关闭或错误）。
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
 * http_send
 * 功能：构造并发送 HTTP/1.1 响应（带 CORS 头），并写入 body。
 * 说明：
 * - Connection: close，便于简化连接管理；
 * - Content-Length 依据 body 长度计算；
 * - status/ctype 允许传 NULL，内部会使用默认值。
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
    if (blen > 0) send_all(fd, body, (size_t)blen);
}

/*
 * parse_control_json
 * 功能：解析 POST /control 的 body（非常简化的 JSON）。
 * 期望格式示例：{"direction":"forward","step":500}
 * 输出：
 * - out_dir：1 表示 forward，-1 表示 reverse
 * - out_step：整数步长
 *
 * 注意：这里只做最小解析：
 * - 必须包含 "direction" 与 "step"；
 * - direction 仅支持 "forward"/"reverse"；
 * - step 做基本范围校验；
 */
static int parse_control_json(const char *body, int *out_dir, int *out_step) {
    if (!body || !out_dir || !out_step) return -1;
    const char *dkey = strstr(body, "\"direction\"");
    if (!dkey) return -2;
    const char *dcolon = strchr(dkey, ':');
    if (!dcolon) return -3;
    const char *dquote1 = strchr(dcolon, '"');
    if (!dquote1) return -4;
    const char *dquote2 = strchr(dquote1 + 1, '"');
    if (!dquote2) return -5;
    int dir = 0;
    size_t dlen = (size_t)(dquote2 - (dquote1 + 1));
    if (dlen > 32) return -6;
    char dval[40];
    memcpy(dval, dquote1 + 1, dlen);
    dval[dlen] = '\0';
    for (size_t i = 0; i < dlen; ++i) dval[i] = (char)tolower((unsigned char)dval[i]);
    if (strcmp(dval, "forward") == 0) dir = 1;
    else if (strcmp(dval, "reverse") == 0) dir = -1;
    else return -7;
    const char *skey = strstr(body, "\"step\"");
    if (!skey) return -8;
    const char *scolon = strchr(skey, ':');
    if (!scolon) return -9;
    long step = strtol(scolon + 1, NULL, 10);
    if (step <= 0 || step > 100000000) return -10;
    *out_dir = dir;
    *out_step = (int)step;
    return 0;
}

/*
 * parse_axis_control_json
 * 功能：解析单轴控制 JSON，格式示例：{"axis":1, "direction":1, "step":500}
 * 说明：
 * - axis 采用 0-based 索引（与 motor_api_set_axis_command 一致）；
 * - direction 为 -1/0/1 的整数（与内部逻辑一致）；
 * - step 为整数；
 */
static int parse_axis_control_json(const char *json, int *axis, int *dir, int *step) {
    if (!json || !axis || !dir || !step) return -1;
    *axis = -1;
    *dir = 0;
    *step = 0;

    const char *p = strstr(json, "\"axis\"");
    if (p) {
        const char *cl = strchr(p, ':');
        if (cl) *axis = (int)strtol(cl + 1, NULL, 10);
    }

    p = strstr(json, "\"direction\"");
    if (p) {
        const char *cl = strchr(p, ':');
        if (cl) *dir = (int)strtol(cl + 1, NULL, 10);
    }

    p = strstr(json, "\"step\"");
    if (p) {
        const char *cl = strchr(p, ':');
        if (cl) *step = (int)strtol(cl + 1, NULL, 10);
    }

    return (*axis >= 0) ? 0 : -1;
}

/*
 * http_thread_fn
 * 功能：HTTP 服务线程主体。
 * 路由：
 * - GET  /          返回 control.html（调试页面）
 * - GET  /status    返回当前全局命令(run/dir/step)
 * - GET  /diag      返回诊断 JSON（由 motor_api_format_diag_json 生成）
 * - POST /control        设置全局命令并 run=true
 * - POST /stop           设置全局命令为停止
 * - POST /control_axis   设置单轴命令并 run=true
 * - POST /stop_axis      设置单轴命令为停止
 * - POST /shutdown       请求线程退出
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
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sfd, &rfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000;
        int sel = select(sfd + 1, &rfds, NULL, NULL, &tv);
        if (sel <= 0) {
            if (sel < 0 && errno != EINTR) {
                break;
            }
            continue;
        }

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
                FILE *f = fopen("test/control.html", "rb");
                if (f) {
                    fseek(f, 0, SEEK_END);
                    long fsize = ftell(f);
                    fseek(f, 0, SEEK_SET);
                    char *html_buf = malloc((size_t)fsize + 1);
                    if (html_buf) {
                        size_t r = fread(html_buf, 1, (size_t)fsize, f);
                        (void)r;
                        html_buf[fsize] = 0;
                        http_send(cfd, "200 OK", "text/html", html_buf);
                        free(html_buf);
                    } else {
                        http_send(cfd, "500 Internal Error", "text/plain", "malloc failed");
                    }
                    fclose(f);
                } else {
                    http_send(cfd, "404 Not Found", "text/plain", "control.html not found (run from project root)");
                }

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
                (void)snprintf(out, sizeof(out), "{\"run\":%s,\"dir\":%d,\"step\":%d}", run ? "true" : "false", dir, step);
                http_send(cfd, "200 OK", "application/json", out);
                close(cfd);
                continue;
            }
            if (plen && strncmp(path, "/diag", plen) == 0) {
                char out[1024];
                if (motor_api_format_diag_json((struct motor_api_handle *)h, out, sizeof(out)) == MA_OK) {
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

            if (plen && strncmp(path, "/control_axis", plen) == 0) {
                int axis = -1, dir = 0, step = 0;
                int rc = parse_axis_control_json(body, &axis, &dir, &step);
                if (rc == 0) {
                    motor_api_set_axis_command((struct motor_api_handle *)h, axis, true, dir, step);
                    http_send(cfd, "200 OK", "application/json", "{\"ok\":true}");
                } else {
                    http_send(cfd, "400 Bad Request", "application/json", "{\"ok\":false, \"msg\":\"invalid axis\"}");
                }
                close(cfd);
                continue;
            }

            if (plen && strncmp(path, "/stop_axis", plen) == 0) {
                int axis = -1, dir = 0, step = 0;
                parse_axis_control_json(body, &axis, &dir, &step);
                if (axis >= 0) {
                    motor_api_set_axis_command((struct motor_api_handle *)h, axis, false, 0, 0);
                    http_send(cfd, "200 OK", "application/json", "{\"ok\":true}");
                } else {
                    http_send(cfd, "400 Bad Request", "application/json", "{\"ok\":false}");
                }
                close(cfd);
                continue;
            }

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
 * motor_api_start_http
 * 功能：启动 HTTP 服务线程。
 * 参数：
 * - handle：库句柄（内部会保存端口并创建线程）
 * - port：监听端口，例如 8080
 */
ma_status_t motor_api_start_http(struct motor_api_handle *handle, int port) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle;
    if (!h) return MA_ERR_PARAM;
    h->http_port = port;
    h->stop = 0;
    if (pthread_create(&h->http_thread, NULL, http_thread_fn, h) != 0) return MA_ERR_RUNTIME;
    return MA_OK;
}

/*
 * motor_api_stop_http
 * 功能：停止 HTTP 服务线程并等待其退出。
 * 说明：通过设置 stop 标志触发线程跳出循环，然后 join。
 */
ma_status_t motor_api_stop_http(struct motor_api_handle *handle) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle;
    if (!h) return MA_ERR_PARAM;
    h->stop = 1;
    if (h->http_thread) {
        pthread_join(h->http_thread, NULL);
    }
    return MA_OK;
}

/*
 * motor_api_set_command
 * 功能：设置全局运行命令（线程安全）。
 * 说明：该接口既可被网络模块调用，也可被用户程序直接调用。
 */
ma_status_t motor_api_set_command(struct motor_api_handle *handle, bool run, int dir, int step) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle;
    if (!h) return MA_ERR_PARAM;
    set_cmd_locked(h, run, dir, step);
    return MA_OK;
}

/*
 * motor_api_set_axis_command
 * 功能：设置单轴独立运行命令（线程安全）。
 * 说明：
 * - axis_idx 为 0-based 索引，范围 [0, axis_count)；
 * - 若某轴 axis_run=true，则周期控制中优先使用该轴的 dir/step；
 */
ma_status_t motor_api_set_axis_command(struct motor_api_handle *handle, int axis_idx, bool run, int dir, int step) {
    motor_api_handle_t *h = (motor_api_handle_t *)handle;
    if (!h) return MA_ERR_PARAM;
    if (axis_idx < 0 || axis_idx >= (int)h->axis_count) return MA_ERR_PARAM;
    if (step < 1) step = 1;
    if (step > 100000) step = 100000;
    if (dir != -1 && dir != 0 && dir != 1) dir = 0;

    pthread_mutex_lock(&h->cmd_mutex);
    h->axis_run[axis_idx] = run;
    h->axis_dir[axis_idx] = dir;
    h->axis_step[axis_idx] = step;
    pthread_mutex_unlock(&h->cmd_mutex);
    return MA_OK;
}
