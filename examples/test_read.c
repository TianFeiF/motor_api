#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor_api.h"

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
    const char *eni_path = (argc > 1) ? argv[1] : "/home/phi/ecmotor_api/motor_api/doc/HCFAX3E.xml";
    
    uint32_t vendor_ids[16];
    uint32_t product_codes[16];
    uint16_t positions[16];
    ma_eni_slave_t *slaves = NULL;
    uint16_t slave_count = 0;

    printf("Reading ENI file: %s\n", eni_path);

    ma_status_t status = motor_api_read_eni(eni_path,
                                          vendor_ids,
                                          product_codes,
                                          positions,
                                          16,
                                          &slave_count,
                                          &slaves);

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
        
        if (slaves) {
            printf("  Detailed Info from ma_eni_slave_t:\n");
            printf("    Vendor ID: 0x%08X\n", slaves[i].vendor_id);
            printf("    Product Code: 0x%08X\n", slaves[i].product_code);
            printf("    Position: %d\n", slaves[i].position);
            
            print_pdo_entries("RX", slaves[i].rx_pdos, slaves[i].rx_pdo_count);
            print_pdo_entries("TX", slaves[i].tx_pdos, slaves[i].tx_pdo_count);
        }
        printf("----------------------------------------\n");
    }

    if (slaves) {
        motor_api_free_eni_slaves(slaves, slave_count);
    }

    return 0;
}
