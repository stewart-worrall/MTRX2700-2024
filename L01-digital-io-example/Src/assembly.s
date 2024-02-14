.syntax unified
.thumb

.global main


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




.data
@ Define variables
string: .ascii "this is a STRING"

.text

@ this is the entry function called from the startup file
main:

	LDR R0, =string @ comment 1
	LDR R1, =0

search_string:
	ADD R1, #1 @ comment 2
	LDRB R4, [R0, R1] @ comment 3
	AND R4, 0b0100000 @ comment 4
	CMP R4, 0x00 @ comment 5
	BGT search_string @ comment 6



.equ TIM7, 0x40001400	@ base register for timer 7

@ registers for enabling clocks
.equ AHBENR, 0x14  @ enable peripherals
.equ APB1ENR, 0x1C @ enable peripherals on bus 1
.equ APB2ENR, 0x18 @ enable peripherals on bus 2

@ bit positions for enabling GPIO in AHBENR
.equ GPIOA_ENABLE, 17
.equ GPIOC_ENABLE, 19

	LDR R0, =RCC  @ load the address of the RCC address boundary (for enabling the IO clock)
	LDR R1, [R0, #AHBENR]  @ load the current value of the peripheral clock registers
	ORR R1, 1 << 19 | 1 << 17  @ 21st bit is enable GPIOE clock, 19 is GPIOC, 17 is GPIOA clock
	STR R1, [R0, #AHBENR]  @ store the modified register back to the submodule











forever:

	@ Branch with link to set the clocks for the I/O and UART
	BL enable_peripheral_clocks

	@ Once the clocks are started, need to initialise the discovery board I/O
	BL initialise_discovery_board

	@ store the current light pattern (binary mask) in R4
	LDR R4, =0b01010101 @ load a pattern for the set of LEDs (every second one is on)

program_loop:

@ 	Look at the GPIOE offset ODR, display as hex, then as binary. Look at the manual page 239

	LDR R0, =GPIOE  @ load the address of the GPIOE register into R0
	STRB R4, [R0, #ODR + 1]   @ store this to the second byte of the ODR (bits 8-15)
	EOR R4, #0xFF	@ toggle all of the bits in the byte (1->0 0->1)

@ 	Look at the GPIOA offset IDR, display as hex, then as binary. Look at the manual page 239

@ 	task: read in the input button !
@	LDR R0, =GPIOA	@ port for the input button
@	LDR R1, [R0, IDR]

	@ If you run the code, the LEDs will blink too fast, so fast you cannot tell they
	@ are turning off and on. Uncomment the next line to make a delay

	@ BL delay_function

	B program_loop @ return to the program_loop label




@ think about how you could make a delay such that the LEDs blink at a certain frequency
delay_function:
	MOV R6, #0x03

	@ we continue to subtract one from R6 while the result is not zero,
	@ then return to where the delay_function was called
not_finished_yet:
	SUBS R6, 0x01
	BNE not_finished_yet

	BX LR @ return from function call




@ enable the clocks for peripherals (GPIOA, C and E)
enable_peripheral_clocks:
	LDR R0, =RCC  @ load the address of the RCC address boundary (for enabling the IO clock)
	LDR R1, [R0, #AHBENR]  @ load the current value of the peripheral clock registers
	ORR R1, 1 << GPIOA_ENABLE | 1 << GPIOC_ENABLE | 1 << GPIOE_ENABLE  @ 21st bit is enable GPIOE clock, 17 is GPIOA clock
	STR R1, [R0, #AHBENR]  @ store the modified register back to the submodule
	BX LR @ return from function call





@ initialise the discovery board I/O (just outputs: inputs are selected by default)
initialise_discovery_board:
	LDR R0, =GPIOE 	@ load the address of the GPIOE register into R0
	LDR R1, =0x5555  @ load the binary value of 01 (OUTPUT) for each port in the upper two bytes
					 @ as 0x5555 = 01010101 01010101
	STRH R1, [R0, #MODER + 2]   @ store the new register values in the top half word representing
								@ the MODER settings for pe8-15
	BX LR @ return from function call


