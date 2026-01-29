/*
 * motor_api_types.h
 *
 * 说明：
 * - 本头文件只放“类型/枚举/前置声明/数据结构”等纯定义内容；
 * - 不包含任何 EtherCAT(ecrt.h) 依赖，便于被不同模块复用；
 * - motor_api.h 会作为对外“总入口”头文件包含本文件。
 */

#ifndef MOTOR_API_TYPES_H
#define MOTOR_API_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * EXTERNFUNC
 * 用途：控制符号导出（Windows 下导出 DLL 符号，Linux 下保持默认可见性）。
 * 说明：本项目主要运行在 Linux，但为了接口可移植，保留该宏。
 */
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
 * MA_MAX_SLAVES
 * 库内部支持的最大从站数量上限（静态数组容量）。
 */
#define MA_MAX_SLAVES 16

/*
 * MA_MAX_AXES
 * 库支持的最大逻辑轴数量（可能大于从站数，例如双轴驱动器）。
 */
#define MA_MAX_AXES 32

/*
 * ma_axis_type_t
 * 轴类型定义
 */
typedef enum {
    MA_AXIS_TYPE_NONE = 0,
    MA_AXIS_TYPE_CIA402 = 1,  // 标准 CiA402 伺服轴
    MA_AXIS_TYPE_IO = 2       // 纯 IO 设备（不运行状态机）
} ma_axis_type_t;

/*
 * ma_axis_map_t
 * 轴映射配置结构体（用于 JSON 配置或手动覆盖）
 */
typedef struct {
    bool active;
    uint16_t slave_idx;
    ma_axis_type_t type;
    uint16_t base_offset; // 相对 PDO 起始偏移（高级用法）
    double scale_pos;
    double scale_vel;
} ma_axis_map_t;

/*
 * ma_status_t
 * 统一返回码：所有对外 API 尽量用该枚举表述结果，避免返回 -1/-2 这类难以统一的错误码。
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
 * ma_operate_mode_t
 * CiA-402 操作模式常用值（与 0x6060/0x6061 对象对应）。
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
 * struct motor_api_handle
 * 对外不透明句柄：用户只能拿到指针并传回 API，内部实际结构体在实现层定义。
 */
struct motor_api_handle;

/*
 * ma_eni_pdo_entry_t / ma_eni_pdo_t / ma_eni_slave_t
 * 用途：存放从 ENI(XML) 解析得到的从站与 PDO 映射信息，用于动态注册 PDO。
 *
 * 约定：
 * - index/subindex/bitlen 对应 EtherCAT PDO Entry 的对象字典描述；
 * - rx_pdos/tx_pdos 分别表示主站写入(输出)与主站读取(输入)方向；
 * - entries 指向动态分配内存，释放由 motor_api_free_eni_slaves() 统一完成。
 */
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

#ifdef __cplusplus
}
#endif

#endif
