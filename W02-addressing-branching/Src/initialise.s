.syntax unified
.thumb


.data

@ Clock setting register (base address and offsets)
.equ RCC, 0x40021000	@ base register for resetting and clock settings

@ registers for enabling clocks
.equ AHBENR, 0x14  @ enable peripherals
.equ APB1ENR, 0x1C
.equ APB2ENR, 0x18

@ bit positions for enabling GPIO in AHBENR
.equ GPIOA_ENABLE, 17
.equ GPIOC_ENABLE, 19
.equ GPIOE_ENABLE, 21

@ GPIO register base addresses
.equ GPIOA, 0x48000000	@ base register for GPIOA (pa0 is the button)
.equ GPIOC, 0x48000800	@ base register for GPIOC is used for UART4
.equ GPIOE, 0x48001000	@ base register for GPIOE (pe8-15 are the LEDs)

@ GPIO register offsets
.equ MODER, 0x00	@ register for setting the port mode (in/out/etc)
.equ ODR, 0x14	@ GPIO output register
.equ IDR, 0x10	@ GPIO input register

.text

@ initialise the discovery board I/O (just outputs: inputs are selected by default)
initialise_discovery_board:
	LDR R0, =GPIOE 	@ load the address of the GPIOE register into R0
	LDR R1, =0x5555  @ load the binary value of 01 (OUTPUT) for each port in the upper two bytes
					 @ as 0x5555 = 01010101 01010101
	STRH R1, [R0, #MODER + 2]   @ store the new register values in the top half word representing
								@ the MODER settings for pe8-15
	BX LR @ return from function call


@ enable the clocks for peripherals (GPIOA, C and E)
enable_peripheral_clocks:
	LDR R0, =RCC  @ load the address of the RCC address boundary (for enabling the IO clock)
	LDR R1, [R0, #AHBENR]  @ load the current value of the peripheral clock registers
	ORR R1, 1 << GPIOA_ENABLE | 1 << GPIOC_ENABLE | 1 << GPIOE_ENABLE  @ 21st bit is enable GPIOE clock, 17 is GPIOA clock
	STR R1, [R0, #AHBENR]  @ store the modified register back to the submodule
	BX LR @ return from function call

