/*
 * motor_api_config.c
 *
 * 模块职责：
 * - 提供从“配置文件”创建 motor_api 句柄的入口（当前实现以 JSON 为主）；
 * - 将网络参数（ENI 路径/周期）以及从站/轴映射与机械参数读取后，转换为内部轴映射覆盖表；
 * - 通过 motor_api_create_base 复用主站创建与 PDO 注册逻辑，保持核心实现集中在 motor_api.c。
 *
 * 约束：
 * - 为了保持库依赖最小化，目前仅引入 cJSON 作为 JSON 解析实现；
 * - 配置文件语义以“逻辑轴索引”为准：axis_id 从 0 开始，决定 API 中 axis_idx 的含义。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "motor_api_internal.h"

/* Helper to read file */
static char *read_file(const char *path) {
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        return NULL;
    }

    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return NULL;
    }

    long len = ftell(fp);
    if (len < 0) {
        fclose(fp);
        return NULL;
    }

    if (fseek(fp, 0, SEEK_SET) != 0) {
        fclose(fp);
        return NULL;
    }

    char *buf = (char *)malloc((size_t)len + 1);
    if (!buf) {
        fclose(fp);
        return NULL;
    }

    size_t rd = fread(buf, 1, (size_t)len, fp);
    fclose(fp);

    if (rd != (size_t)len) {
        free(buf);
        return NULL;
    }

    buf[len] = '\0';
    return buf;
}

static ma_axis_type_t parse_slave_axis_type(const cJSON *slave_obj) {
    cJSON *j_type = cJSON_GetObjectItem((cJSON *)slave_obj, "type");
    if (cJSON_IsString(j_type) && j_type->valuestring && strcmp(j_type->valuestring, "io") == 0) {
        return MA_AXIS_TYPE_IO;
    }
    return MA_AXIS_TYPE_CIA402;
}

static int build_axis_override_from_json(const cJSON *root, ma_axis_map_t *out_list, int out_cap) {
    if (!root || !out_list || out_cap <= 0) {
        return 0;
    }

    ma_axis_map_t sparse[MA_MAX_AXES];
    memset(sparse, 0, sizeof(sparse));
    int max_axis_id = -1;

    cJSON *slaves = cJSON_GetObjectItem((cJSON *)root, "slaves");
    if (!cJSON_IsArray(slaves)) {
        return 0;
    }

    int slave_count = cJSON_GetArraySize(slaves);
    for (int i = 0; i < slave_count; ++i) {
        cJSON *slave = cJSON_GetArrayItem(slaves, i);
        if (!cJSON_IsObject(slave)) {
            continue;
        }

        int slave_id = 0;
        cJSON *j_id = cJSON_GetObjectItem(slave, "id");
        if (cJSON_IsNumber(j_id)) {
            slave_id = j_id->valueint;
        }

        ma_axis_type_t axis_type = parse_slave_axis_type(slave);

        cJSON *axes = cJSON_GetObjectItem(slave, "axes");
        if (!cJSON_IsArray(axes)) {
            continue;
        }

        int axis_items = cJSON_GetArraySize(axes);
        for (int j = 0; j < axis_items; ++j) {
            cJSON *axis = cJSON_GetArrayItem(axes, j);
            if (!cJSON_IsObject(axis)) {
                continue;
            }

            cJSON *j_aid = cJSON_GetObjectItem(axis, "axis_id");
            if (!cJSON_IsNumber(j_aid)) {
                continue;
            }

            int axis_id = j_aid->valueint;
            if (axis_id < 0 || axis_id >= MA_MAX_AXES) {
                continue;
            }

            ma_axis_map_t map;
            memset(&map, 0, sizeof(map));
            map.active = true;
            map.slave_idx = (uint16_t)slave_id;
            map.type = axis_type;

            cJSON *j_off = cJSON_GetObjectItem(axis, "offset");
            map.base_offset = (uint16_t)(cJSON_IsNumber(j_off) ? j_off->valueint : 0);

            cJSON *j_enc = cJSON_GetObjectItem(axis, "encoder_res");
            double encoder_res = cJSON_IsNumber(j_enc) ? j_enc->valuedouble : 131072.0;

            cJSON *j_gear = cJSON_GetObjectItem(axis, "gear_ratio");
            double gear_ratio = cJSON_IsNumber(j_gear) ? j_gear->valuedouble : 1.0;

            cJSON *j_unit = cJSON_GetObjectItem(axis, "unit_per_rev");
            double unit_per_rev = cJSON_IsNumber(j_unit) ? j_unit->valuedouble : 1.0;
            if (unit_per_rev == 0.0) {
                unit_per_rev = 1.0;
            }

            double scale = encoder_res * gear_ratio / unit_per_rev;
            map.scale_pos = scale;
            map.scale_vel = scale;

            sparse[axis_id] = map;
            if (axis_id > max_axis_id) {
                max_axis_id = axis_id;
            }
        }
    }

    int out_n = 0;
    for (int axis_id = 0; axis_id <= max_axis_id && out_n < out_cap; ++axis_id) {
        if (!sparse[axis_id].active) {
            continue;
        }
        out_list[out_n++] = sparse[axis_id];
    }

    return out_n;
}

/*
 * motor_api_create_from_config
 * 功能：从 JSON 配置文件创建 motor_api 句柄。
 * 参数：
 * - config_path：配置文件路径（JSON）
 * - out_handle：输出句柄
 * 返回：
 * - MA_OK：成功
 * - MA_ERR_PARAM：参数非法
 * - MA_ERR_IO：读取文件失败
 * - MA_ERR_CONFIG：JSON 解析失败或配置内容非法
 * - MA_ERR_INIT/MA_ERR_RUNTIME：主站创建或运行时错误
 */
EXTERNFUNC ma_status_t motor_api_create_from_config(const char *config_path, struct motor_api_handle **out_handle) {
    if (!config_path || !out_handle) {
        return MA_ERR_PARAM;
    }

    char *json_str = read_file(config_path);
    if (!json_str) {
        fprintf(stderr, "[ERROR] Failed to read config file: %s\n", config_path);
        return MA_ERR_IO;
    }

    cJSON *root = cJSON_Parse(json_str);
    free(json_str);
    if (!root) {
        fprintf(stderr, "[ERROR] Failed to parse JSON config\n");
        return MA_ERR_CONFIG;
    }

    const char *eni_path = "doc/HCFAX3E.xml";
    uint32_t cycle_us = 4000;

    cJSON *net = cJSON_GetObjectItem(root, "network");
    if (cJSON_IsObject(net)) {
        cJSON *j_eni = cJSON_GetObjectItem(net, "eni_path");
        if (cJSON_IsString(j_eni) && j_eni->valuestring) {
            eni_path = j_eni->valuestring;
        }

        cJSON *j_cycle = cJSON_GetObjectItem(net, "cycle_us");
        if (cJSON_IsNumber(j_cycle) && j_cycle->valueint > 0) {
            cycle_us = (uint32_t)j_cycle->valueint;
        }
    }

    ma_axis_map_t axis_override[MA_MAX_AXES];
    int axis_override_count = build_axis_override_from_json(root, axis_override, MA_MAX_AXES);

    /* Call base create with override */
    uint16_t slave_count = 0;
    ma_status_t ret = motor_api_create_base(
        eni_path,
        cycle_us,
        &slave_count,
        out_handle,
        axis_override,
        axis_override_count
    );

    cJSON_Delete(root);
    return ret;
}
