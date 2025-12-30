/*
 * 版权所有 (C) 2025 phi
 * 文件名称: motor_api.h
 * 版本信息: v1.0.0
 * 文件说明: 通用电机控制库对外头文件，提供 EtherCAT 主站初始化、
 *           ENI 读取、CSP/CSV 运行周期、HTTP 控制与诊断等 API。
 * 模块关系: 与实现文件 motor_api.c 配套使用；示例程序 example_csp.c 调用该 API。
 * 修改历史:
 *   - 2025-11-28: 初始版本，支持 ENI 读取、DC 同步、CSP 控制、HTTP 服务。
 *   - 2025-11-28: 增加“全轴使能后延时 1s 同步起动”的栅栏机制。
 */

#ifndef MOTOR_API_H
#define MOTOR_API_H

/*
 * motor_api 对外头文件（聚合头）
 *
 * 说明：
 * - motor_api_types.h：公共类型、错误码、句柄前置声明、ENI 解析结构体等。
 * - motor_api_common.h：不依赖 ecrt 的通用能力（ENI 解析/工具函数）。
 * - motor_api_net.h：网络/HTTP 控制接口（实现位于 motor_api_net.c）。
 *
 * 一般使用方式：
 * - 用户程序仅需 include "motor_api.h" 即可获得全部公开 API。
 */
#include "motor_api_types.h"
#include "motor_api_common.h"
#include "motor_api_net.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 函数: motor_api_create
 * 功能: 初始化 EtherCAT 主站与域，读取 ENI 并配置从站、PDO 映射、DC 同步；创建库句柄。
 * 参数:
 *   - eni_path: ENI XML 文件路径，可为 NULL 使用默认（示例为 motor_api/doc/HCFAX3E.xml）
 *   - cycle_us: 控制周期（微秒），典型值 4000（4ms）、10000（10ms），需与 0x60C2 插值周期匹配
 *   - out_slave_count: 输出从站数量指针，可为 NULL（返回配置生效的从站数）
 *   - out_handle: 输出库句柄指针，成功返回非 NULL
 * 返回:
 *   - MA_OK 成功；否则返回错误码（参见 ma_status_t）
 * 使用示例:
 *   struct motor_api_handle *h = NULL; uint16_t n = 0;
 *   创建 4ms 周期主站，错误时进行处理：
 *   if (motor_api_create("motor_api/doc/HCFAX3E.xml", 4000, &n, &h) != MA_OK) {
 *       // 错误处理，例如打印日志或退出
 *   }
 * 注意事项:
 *   - 若其他进程占用主站，创建可能失败（Device busy）；需先释放旧进程
 *   - ENI 的 Position/VendorId/ProductCode 应与现场设备一致，否则 PDO 注册失败
 */
EXTERNFUNC ma_status_t motor_api_create(const char *eni_path,
                                        uint32_t cycle_us,
                                        uint16_t *out_slave_count,
                                        struct motor_api_handle **out_handle);

/*
 * 函数: motor_api_destroy
 * 功能: 释放库句柄与主站资源，关闭线程与互斥量。
 * 参数:
 *   - handle: motor_api_create 创建的句柄
 * 返回:
 *   - MA_OK 成功；MA_ERR_PARAM 当 handle 为 NULL
 */
EXTERNFUNC ma_status_t motor_api_destroy(struct motor_api_handle *handle);

/*
 * 函数: motor_api_run_once
 * 功能: 执行一次周期控制，包括接收/处理域数据，推进 CiA-402 状态机，更新 CSP 目标。
 * 参数:
 *   - handle: 库句柄
 * 返回:
 *   - MA_OK 成功；MA_ERR_PARAM 当 handle 为 NULL
 * 注意事项:
 *   - 需以固定周期调用（如 4ms），并与 0x60C2 插值周期一致
 */
EXTERNFUNC ma_status_t motor_api_run_once(struct motor_api_handle *handle);

/*
 * 函数: motor_api_format_diag_json
 * 功能: 生成当前所有从站的诊断 JSON（与 test3.c 类似）。
 * 参数:
 *   - handle: 库句柄
 *   - buf: 输出缓冲区
 *   - buf_size: 缓冲区长度（字节）
 * 返回:
 *   - MA_OK 成功；MA_ERR_PARAM 当参数非法；MA_ERR_RUNTIME 当格式化失败
 */
EXTERNFUNC ma_status_t motor_api_format_diag_json(struct motor_api_handle *handle,
                                                  char *buf,
                                                  size_t buf_size);

EXTERNFUNC ma_status_t motor_api_clear_error(struct motor_api_handle *handle, int axis_idx);

EXTERNFUNC ma_status_t eth_initDLL(uint32_t timeout_ms,
                                   uint16_t *out_slave_count,
                                   char (*out_product_names)[64],
                                   uint16_t max_slaves,
                                   bool *out_config_valid,
                                   struct motor_api_handle **out_handle);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_API_H */
