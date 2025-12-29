/*
 * motor_api_common.h
 *
 * 说明：
 * - 放置与 EtherCAT(ecrt.h) 无关的公共能力：时间、ENI(XML) 解析、辅助查询等；
 * - 该模块可以被 EtherCAT 核心模块、网络模块、测试程序共同使用；
 * - 所有接口均为纯 CPU/文件/字符串处理，不涉及 EtherCAT 资源生命周期。
 */

#ifndef MOTOR_API_COMMON_H
#define MOTOR_API_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_api_types.h"

/*
 * motor_api_monotonic_ns
 * 功能：获取单调时钟当前时间（纳秒）。
 * 场景：
 * - 用于周期控制时间戳、超时等待计时等；
 * - 单调时钟不受系统时间校准影响（相比 gettimeofday 更稳定）。
 */
EXTERNFUNC uint64_t motor_api_monotonic_ns(void);

/*
 * motor_api_find_latest_eni_xml
 * 功能：在指定目录中选择“最新修改时间”的 .xml 文件作为 ENI 路径。
 * 参数：
 * - dir：目录路径（例如 doc/）
 * - out_path/out_size：输出缓冲区与长度，返回完整路径
 * 返回：
 * - MA_OK：找到并写入 out_path
 * - MA_ERR_PARAM：参数非法
 * - MA_ERR_IO：目录不可访问或未找到 xml
 */
EXTERNFUNC ma_status_t motor_api_find_latest_eni_xml(const char *dir, char *out_path, size_t out_size);

/*
 * motor_api_fill_product_names_from_eni
 * 功能：从 ENI 文件中尽量提取产品名称(Name/ProductName)，用于展示/诊断。
 * 说明：
 * - 若解析失败，会用默认格式 "PID_0x%08X" 填充；
 * - 该函数不会解析 PDO 细节，只关注“产品码->名称”映射与 SlaveList 的 Name 字段。
 */
EXTERNFUNC ma_status_t motor_api_fill_product_names_from_eni(const char *eni_path,
                                                             const uint32_t *product_codes,
                                                             uint16_t count,
                                                             char (*out_product_names)[64],
                                                             uint16_t max_slaves);

/*
 * motor_api_eni_has_rx_entry / motor_api_eni_has_tx_entry
 * 功能：查询某个 ENI 从站描述中，Rx/Tx PDO 是否包含指定 index/subindex 的条目。
 * 返回：存在返回 1，不存在返回 0。
 */
EXTERNFUNC int motor_api_eni_has_rx_entry(const ma_eni_slave_t *s, uint16_t index, uint8_t subindex);
EXTERNFUNC int motor_api_eni_has_tx_entry(const ma_eni_slave_t *s, uint16_t index, uint8_t subindex);

/*
 * motor_api_read_eni
 * 功能：从 ENI(XML) 中解析从站列表以及 Rx/Tx PDO entry 列表。
 * 参数：
 * - eni_path：ENI 文件路径
 * - vendor_ids/product_codes/positions：可选输出数组（可为 NULL）
 * - max_slaves：输出容量上限
 * - out_count：解析得到的从站数量
 * - out_slaves：输出从站结构数组（内部使用 malloc/realloc 分配）
 * 返回：
 * - MA_OK：成功
 * - MA_ERR_PARAM：参数错误
 * - MA_ERR_IO：文件打不开/不存在
 * - MA_ERR_RUNTIME：内存分配等运行时错误
 *
 * 注意：
 * - out_slaves 非 NULL 时由调用者在使用完后调用 motor_api_free_eni_slaves 释放；
 * - 解析采用“容错扫描”策略，兼容不同厂商 ENI 中标签/属性写法差异。
 */
EXTERNFUNC ma_status_t motor_api_read_eni(const char *eni_path,
                                         uint32_t *vendor_ids,
                                         uint32_t *product_codes,
                                         uint16_t *positions,
                                         uint16_t max_slaves,
                                         uint16_t *out_count,
                                         ma_eni_slave_t **out_slaves);

/*
 * motor_api_free_eni_slaves
 * 功能：释放 motor_api_read_eni 动态分配的从站数组及其内部 entries/pdos。
 * 说明：count 必须与 read_eni 的 out_count 对应。
 */
EXTERNFUNC void motor_api_free_eni_slaves(ma_eni_slave_t *slaves, uint16_t count);

#ifdef __cplusplus
}
#endif

#endif
