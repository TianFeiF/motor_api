#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor_api.h"

/*
 * 该示例用于演示如何调用 motor_api_read_eni() 读取 EtherCAT ENI(XML) 文件，
 * 并打印出：
 * 1) 每个从站的 Vendor ID / Product Code / Position；
 * 2) motor_api 解析得到的详细从站结构体 ma_eni_slave_t；
 * 3) 从站的 RxPDO/TxPDO 及其 Entry（对象字典索引、子索引、位长度）。
 *
 * 注意：
 * - 本示例只负责“读取并打印”，不涉及主站启动、PDO 映射下发或实时通信。
 * - motor_api_read_eni() 内部可能会分配动态内存（slaves 及其内部成员），
 *   调用方需要在使用完后通过 motor_api_free_eni_slaves() 释放。
 */

/*
 * 打印 PDO 及其包含的 Entry 列表。
 *
 * 参数说明：
 * - type : 用于打印的标签字符串（例如 "RX" / "TX"），便于区分方向；
 * - pdos : PDO 数组首地址；
 * - count: PDO 数量。
 *
 * 结构体字段含义（以常见 EtherCAT 语义解释）：
 * - pdo_index          : PDO 的索引（例如 0x1600/0x1A00 等）；
 * - entries[j].index   : PDO Entry 对应的对象字典 Index；
 * - entries[j].subindex: 对象字典 SubIndex；
 * - entries[j].bitlen  : 该 Entry 映射的位长度。
 */
void print_pdo_entries(const char *type, const ma_eni_pdo_t *pdos, unsigned int count) {
    for (unsigned int i = 0; i < count; i++) {
        printf("    %s PDO Index: 0x%04X\n", type, pdos[i].pdo_index);
        for (unsigned int j = 0; j < pdos[i].entry_count; j++) {
            printf("      Entry: Index=0x%04X, Sub=0x%02X, BitLen=%d\n",
                   pdos[i].entries[j].index,
                   pdos[i].entries[j].subindex,
                   pdos[i].entries[j].bitlen);
        }
    }
}

int main(int argc, char *argv[]) {
    /*
     * ENI 文件路径：
     * - 若命令行提供 argv[1] 则使用 argv[1]；
     * - 否则使用默认路径。
     *
     * 说明：默认路径是示例作者在 Linux 环境下的路径；在 Windows/其他环境运行时，
     * 请通过命令行参数传入实际 ENI(XML) 路径。
     */
    const char *eni_path = (argc > 1) ? argv[1] : "/home/phi/ecmotor_api/motor_api/doc/HCFAX3E copy.xml";
    
    /*
     * 这三个数组用于接收 motor_api_read_eni() 输出的从站关键信息。
     *
     * 约定：数组容量由第 5 个参数传入（这里是 16），函数会写入实际解析到的数量
     * 到 slave_count，并按从站顺序填充 vendor_ids/product_codes/positions。
     */
    uint32_t vendor_ids[16];
    uint32_t product_codes[16];
    uint16_t positions[16];

    /*
     * slaves 用于接收更详细的从站结构体数组（包含 PDO 等更复杂信息）。
     * - 初始置 NULL；
     * - 成功后由库分配并返回；
     * - 用完必须调用 motor_api_free_eni_slaves(slaves, slave_count) 释放。
     */
    ma_eni_slave_t *slaves = NULL;

    /*
     * 解析到的从站数量（输出参数）。
     * - 这里初始化为 0；
     * - 成功后 motor_api_read_eni() 写入真实数量。
     */
    uint16_t slave_count = 0;

    printf("Reading ENI file: %s\n", eni_path);

    /*
     * 读取并解析 ENI 文件。
     *
     * 参数：
     * - eni_path       : ENI(XML) 路径；
     * - vendor_ids     : 输出数组（每个从站的 Vendor ID）；
     * - product_codes  : 输出数组（每个从站的 Product Code）；
     * - positions      : 输出数组（每个从站的位置/站号等信息，具体语义以库为准）；
     * - 16             : 上述三个数组的容量上限；
     * - &slave_count   : 输出，从站数量；
     * - &slaves        : 输出，详细从站结构体数组指针。
     *
     * 返回：
     * - MA_OK 表示成功；
     * - 其他值表示失败（可根据 motor_api.h 中的定义进一步定位原因）。
     */
    ma_status_t status = motor_api_read_eni(eni_path,
                                          vendor_ids,
                                          product_codes,
                                          positions,
                                          16,
                                          &slave_count,
                                          &slaves);

    /*
     * 失败时直接返回。
     * 说明：此处不需要释放 slaves，因为失败时通常不会返回有效分配；
     * 若库在失败路径也可能分配内存，应以 motor_api_read_eni() 的文档/实现为准。
     */
    if (status != MA_OK) {
        printf("Error reading ENI file. Status: %d\n", status);
        return 1;
    }

    printf("Successfully read ENI. Found %d slaves:\n", slave_count);
    for (int i = 0; i < slave_count; i++) {
        printf("Slave %d:\n", i);
        printf("  Vendor ID: 0x%08X\n", vendor_ids[i]);
        printf("  Product Code: 0x%08X\n", product_codes[i]);
        printf("  Position: %d\n", positions[i]);
        
        /*
         * 如果库返回了详细从站数组，则额外打印 ma_eni_slave_t 结构体内的信息。
         * 这里打印的 vendor_id/product_code/position 应与数组输出一致，
         * 可作为一致性检查。
         */
        if (slaves) {
            printf("  Detailed Info from ma_eni_slave_t:\n");
            printf("    Vendor ID: 0x%08X\n", slaves[i].vendor_id);
            printf("    Product Code: 0x%08X\n", slaves[i].product_code);
            printf("    Position: %d\n", slaves[i].position);
            
            /*
             * 打印该从站的 RxPDO/TxPDO 列表及其映射条目。
             * - RxPDO：通常表示主站->从站（Outputs）方向；
             * - TxPDO：通常表示从站->主站（Inputs）方向；
             * 具体以设备/ENI 定义为准。
             */
            print_pdo_entries("RX", slaves[i].rx_pdos, slaves[i].rx_pdo_count);
            print_pdo_entries("TX", slaves[i].tx_pdos, slaves[i].tx_pdo_count);
        }
        printf("----------------------------------------\n");
    }

    /*
     * 释放 motor_api_read_eni() 返回的动态内存。
     * - 传入 slaves 指针以及 slave_count；
     * - 函数内部应释放 slaves 数组及其内部的 PDO/Entry 等子结构。
     */
    if (slaves) {
        motor_api_free_eni_slaves(slaves, slave_count);
    }

    return 0;
}
