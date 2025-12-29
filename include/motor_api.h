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

#ifdef __cplusplus
extern "C" {
#endif

#ifndef EXTERNFUNC
#ifdef _WIN32
#define EXTERNFUNC __declspec(dllexport)
#else
#define EXTERNFUNC
#endif
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * 返回状态枚举
 * 功能: 标识库函数返回的执行结果。
 * 成员含义:
 *   - MA_OK: 执行成功
 *   - MA_ERR_INIT: 初始化失败（主站/域/激活等）
 *   - MA_ERR_CONFIG: 配置失败（PDO 映射/寄存器等）
 *   - MA_ERR_PARAM: 参数错误（空指针/非法取值等）
 *   - MA_ERR_RUNTIME: 运行时错误（解析/格式化/线程等）
 *   - MA_ERR_IO: I/O 错误（文件打开/读写失败等）
 */
typedef enum {
    MA_OK = 0,
    MA_ERR_INIT = 1,
    MA_ERR_CONFIG = 2,
    MA_ERR_PARAM = 3,
    MA_ERR_RUNTIME = 4,
    MA_ERR_IO = 5
} ma_status_t;

/*
 * 操作模式枚举（CiA-402）
 * 说明: 与从站对象 0x6060/0x6061 对应，常用值如下：
 *   - MA_MODE_CSP: 同步位置模式（Cyclic Synchronous Position）
 *   - MA_MODE_CSV: 同步速度模式（Cyclic Synchronous Velocity）
 * 其余模式按标准定义。
 */
typedef enum {
    MA_MODE_PROFILE_POSITION = 1,
    MA_MODE_VELOCITY = 2,
    MA_MODE_PROFILE_VELOCITY = 3,
    MA_MODE_PROFILE_TORQUE = 4,
    MA_MODE_HOMING = 6,
    MA_MODE_CSP = 8,
    MA_MODE_CSV = 9,
    MA_MODE_CST = 10
} ma_operate_mode_t;

/*
 * 句柄类型前置声明
 * 说明: 所有对外 API 通过不透明句柄管理内部资源，确保线程安全与封装性。
 */
struct motor_api_handle;

/* ENI 解析结构（用于动态 PDO 注册） */
typedef struct {
    uint16_t index;
    uint8_t subindex;
    uint8_t bitlen;
} ma_eni_pdo_entry_t;

typedef struct {
    uint16_t pdo_index;
    unsigned int entry_count;
    ma_eni_pdo_entry_t *entries;
} ma_eni_pdo_t;

typedef struct {
    uint32_t vendor_id;
    uint32_t product_code;
    uint16_t position;
    unsigned int rx_pdo_count;
    ma_eni_pdo_t *rx_pdos;
    unsigned int tx_pdo_count;
    ma_eni_pdo_t *tx_pdos;
} ma_eni_slave_t;

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
 * 函数: motor_api_start_http
 * 功能: 启动 HTTP 服务线程，提供基本控制与诊断接口。
 * 端点:
 *   - GET /        健康检查
 *   - GET /status  当前运行参数（run/dir/step）
 *   - GET /diag    诊断信息（状态字/模式/位置等）
 *   - POST /control {direction:"forward|reverse", step:<int>} 运行指令
 *   - POST /stop   停止指令
 *   - POST /shutdown 关闭 HTTP 服务
 * 参数:
 *   - handle: 库句柄
 *   - port: 端口号（如 8080）
 * 返回:
 *   - MA_OK 成功；MA_ERR_PARAM 当 handle 为 NULL；MA_ERR_RUNTIME 线程创建失败
 */
EXTERNFUNC ma_status_t motor_api_start_http(struct motor_api_handle *handle, int port);

/*
 * 函数: motor_api_stop_http
 * 功能: 停止 HTTP 服务并等待线程退出。
 * 参数:
 *   - handle: 库句柄
 */
EXTERNFUNC ma_status_t motor_api_stop_http(struct motor_api_handle *handle);

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
 * 函数: motor_api_set_command
 * 功能: 设置全局运行指令（广播模式）。
 */
EXTERNFUNC ma_status_t motor_api_set_command(struct motor_api_handle *handle,
                                             bool run,
                                             int dir,
                                             int step);

/*
 * 函数: motor_api_set_axis_command
 * 功能: 设置单轴独立运行指令。
 * 参数:
 *   - axis_idx: 轴索引 (0-based)
 *   - run: 是否运动
 *   - dir: 方向 (-1, 0, 1)
 *   - step: 步长/速度
 */
EXTERNFUNC ma_status_t motor_api_set_axis_command(struct motor_api_handle *handle,
                                                  int axis_idx,
                                                  bool run,
                                                  int dir,
                                                  int step);

/*
 * 函数: motor_api_read_eni
 * 功能: 读取 ENI XML，解析从站 VendorId/ProductCode/Position 等常见属性。
 * 参数:
 *   - eni_path: ENI 路径
 *   - vendor_ids: 输出 VendorId 数组（可为 NULL）
 *   - product_codes: 输出 ProductCode 数组（可为 NULL）
 *   - positions: 输出 Position 数组（可为 NULL）
 *   - max_slaves: 输出数组容量上限
 *   - out_count: 输出解析到的从站数量
 * 返回:
 *   - MA_OK 成功；MA_ERR_IO 文件不存在；MA_ERR_PARAM 参数错误
 * 注意事项:
 *   - 解析采用容错扫描，兼容不同厂商 ENI 的属性命名与格式
 */
EXTERNFUNC ma_status_t motor_api_read_eni(const char *eni_path,
                                          uint32_t *vendor_ids,
                                          uint32_t *product_codes,
                                          uint16_t *positions,
                                          uint16_t max_slaves,
                                          uint16_t *out_count,
                                          ma_eni_slave_t **out_slaves);

EXTERNFUNC void motor_api_free_eni_slaves(ma_eni_slave_t *slaves, uint16_t count);

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
