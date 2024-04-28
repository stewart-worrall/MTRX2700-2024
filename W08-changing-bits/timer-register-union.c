#include <stdio.h>

#include <stdint.h>

typedef union {
    uint32_t reg;
    struct {
        uint32_t DTG     : 8;   // Bits 7:0 - Dead-time generator setup
        uint32_t LOCK    : 2;   // Bits 9:8 - Lock configuration
        uint32_t OSSI    : 1;   // Bit 10 - Off-state selection for Idle mode
        uint32_t OSSR    : 1;   // Bit 11 - Off-state selection for Run mode
        uint32_t BKE     : 1;   // Bit 12 - Break enable
        uint32_t BKP     : 1;   // Bit 13 - Break polarity
        uint32_t AOE     : 1;   // Bit 14 - Automatic output enable
        uint32_t MOE     : 1;   // Bit 15 - Main output enable
        uint32_t Reserved: 16;  // Bits 31:16 - Reserved, must be kept at reset value
    } bits;
} TIMx_BDTR_Register;


int main() {
    TIMx_BDTR_Register tim_bdtr;

    // Set register values
    tim_bdtr.bits.DTG = 0x1F;   // Dead-time generator setup
    tim_bdtr.bits.LOCK = 0x2;   // Lock configuration - Level 2
    tim_bdtr.bits.OSSI = 0x1;   // Off-state selection for Idle mode
    tim_bdtr.bits.OSSR = 0x0;   // Off-state selection for Run mode
    tim_bdtr.bits.BKE = 0x1;    // Break enable
    tim_bdtr.bits.BKP = 0x0;    // Break polarity
    tim_bdtr.bits.AOE = 0x1;    // Automatic output enable
    tim_bdtr.bits.MOE = 0x1;    // Main output enable
    tim_bdtr.bits.Reserved = 0; // Reserved bits, must be kept at reset value

    // Print the register value
    printf("TIMx_BDTR register value: 0x%08X\n", tim_bdtr.reg);

    // Toggle MOE (Main output enable) bit
    tim_bdtr.bits.MOE ^= 1;

    // Print the register value after toggling MOE
    printf("TIMx_BDTR register value after toggling MOE: 0x%08X\n", tim_bdtr.reg);

    return 0;
}
