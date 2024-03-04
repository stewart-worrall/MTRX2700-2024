.syntax unified
.thumb

#include "definitions.s"

#.global enable_timer2_clock
#.global enable_peripheral_clocks
#.global initialise_discovery_board

.text
@ define code

enable_timer2_clock:

	LDR R0, =RCC	@ load the base address for the timer
	LDR R1, [R0, APB1ENR] 	@ load the peripheral clock control register
	ORR R1, 1 << TIM2EN @ store a 1 in bit for the TIM2 enable flag
	STR R1, [R0, APB1ENR] @ enable the timer
	BX LR @ return


@ function to enable the clocks for the peripherals we are using (A, C and E)
enable_peripheral_clocks:
	LDR R0, =RCC  @ load the address of the RCC address boundary (for enabling the IO clock)
	LDR R1, [R0, #AHBENR]  @ load the current value of the peripheral clock registers
	ORR R1, 1 << 21 | 1 << 19 | 1 << 17  @ 21st bit is enable GPIOE clock, 19 is GPIOC, 17 is GPIOA clock
	STR R1, [R0, #AHBENR]  @ store the modified register back to the submodule
	BX LR @ return


@ initialise the discovery board I/O (just outputs: inputs are selected by default)
initialise_discovery_board:
	LDR R0, =GPIOE 	@ load the address of the GPIOE register into R0
	LDR R1, =0x5555  @ load the binary value of 01 (OUTPUT) for each port in the upper two bytes
					 @ as 0x5555 = 01010101 01010101
	STRH R1, [R0, #MODER + 2]   @ store the new register values in the top half word representing
								@ the MODER settings for pe8-15
	BX LR @ return from function call



trigger_prescaler:

	@ Use (TIMx_EGR) instead (to reset the clock)

	@ This is a hack to get the prescaler to take affect
	@ the prescaler is not changed until the counter overflows
	@ the TIMx_ARR register sets the count at which the overflow
	@ happens. Here, the reset is triggered and the overflow
	@ occurs to make the prescaler take effect.
	@ you should use a different approach to this !

	@ In your code, you should be using the ARR register to
	@ set the maximum count for the timer

	@ store a value for the prescaler
	LDR R0, =TIM2	@ load the base address for the timer

	LDR R1, =0x1 @ make the timer overflow after counting to only 1
	STR R1, [R0, TIM_ARR] @ set the ARR register

	LDR R8, =0x00
	STR R8, [R0, TIM_CNT] @ reset the clock
	NOP
	NOP

	LDR R1, =0xffffffff @ set the ARR back to the default value
	STR R1, [R0, TIM_ARR] @ set the ARR register

	BX LR
