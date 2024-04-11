#include <stdint.h>
#include <stdio.h>

typedef union {
    uint32_t word;
    struct {
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
    } bytes;
    struct {
        uint16_t reserved;
        struct {
            uint8_t enable : 1;
            uint8_t interruptEnable : 1;
            uint8_t modeSelect : 1;
            uint8_t clockSource : 1;
            uint8_t clockDivider : 3;
            uint8_t reset : 1;
            uint8_t dataDirection : 1;
            uint8_t pullUpEnable : 1;
            uint8_t powerDown : 1;
            uint8_t diagnosticEnable : 1;
            uint8_t filterEnable : 1;
            uint8_t outputDrive : 1;
            uint8_t inputInvert : 1;
            uint8_t reservedBit : 1;
        } bits;
    } config;
} RegisterBreakdown;

void printRegisterBreakdown(RegisterBreakdown reg) {
    printf("Word: 0x%08X\n", reg.word);
    
    printf("Bytes: 0x%02X 0x%02X 0x%02X 0x%02X\n", reg.bytes.byte1, 
		    reg.bytes.byte2, reg.bytes.byte3, reg.bytes.byte4);
    
    printf("Config:\n");
    
    printf("  Reserved: 0x%04X\n", reg.config.reserved);
    
    printf("  Enable: %u, InterruptEnable: %u, 
		    ModeSelect: %u, ClockSource: %u, ClockDivider: %u\n",
           reg.config.bits.enable, reg.config.bits.interruptEnable, 
	   reg.config.bits.modeSelect, reg.config.bits.clockSource, 
	   reg.config.bits.clockDivider);
    
    printf("  Reset: %u, DataDirection: %u, PullUpEnable: %u, 
		    PowerDown: %u, DiagnosticEnable: %u\n",
           reg.config.bits.reset, reg.config.bits.dataDirection, 
	   reg.config.bits.pullUpEnable, reg.config.bits.powerDown, 
	   reg.config.bits.diagnosticEnable);
    
    printf("  FilterEnable: %u, OutputDrive: %u, InputInvert: %u, 
		    ReservedBit: %u\n",
           reg.config.bits.filterEnable, reg.config.bits.outputDrive, 
	   reg.config.bits.inputInvert, reg.config.bits.reservedBit);
    
    printf("\n");
}

int main() {
    RegisterBreakdown reg = {0}; // initialise all to zero
    printf("Initial value (all zeros):\n");
    printRegisterBreakdown(reg);

    reg.word = 0xFFFFFFFF;
    printf("Initial value:\n");
    printRegisterBreakdown(reg);

    reg.bytes.byte1 = 0x11;
    printf("After modifying byte1:\n");
    printRegisterBreakdown(reg);

    reg.config.reserved = 0x1234;
    printf("After modifying reserved:\n");
    printRegisterBreakdown(reg);

    reg.config.bits.enable = 1;
    printf("After modifying enable:\n");
    printRegisterBreakdown(reg);

    return 0;
}
