.syntax unified
.thumb

#include "initialise.s"

.global main


.data
@ define variables
on_time: .word 0x10000
off_time: .word 0x80000



.text
@ define code


@ this is the entry function called from the startup file
main:

	BL enable_timer2_clock
	BL enable_peripheral_clocks
	BL initialise_discovery_board

	@ start the counter running
	LDR R0, =TIM2	@ load the base address for the timer

	MOV R1, #0b1 @ store a 1 in bit zero for the CEN flag
	STR R1, [R0, TIM_CR1] @ enable the timer

	@ store a value for the prescaler
	LDR R0, =TIM2	@ load the base address for the timer
	MOV R1, #0x04 @ put a prescaler in R1
	STR R1, [R0, TIM_PSC] @ set the prescaler register

	BL trigger_prescaler

	@ questions for timed_loop
	@  what can make it run faster/slower (there are multiple ways)


pwm_loop:
	@ store the current light pattern (binary mask) in R7
	LDR R7, =0b01010101 @ load a pattern for the set of LEDs (every second one is on)

	LDR R1, =on_time
	LDR R1, [R1]

	LDR R2, =off_time
	LDR R2, [R2]


pwm_start:
	@ reset the counter
	LDR R0, =TIM2
	LDR R8, =0x00
	STR R8, [R0, TIM_CNT]

	LDR R0, =GPIOE  @ load the address of the GPIOE register into R0
	STRB R7, [R0, #ODR + 1]   @ store this to the second byte of the ODR (bits 8-15)

pwm_loop_inner:
	@ load the current time from the counter
	LDR R0, =TIM2  @ load the address of the timer 2 base address
	LDR R6, [R0, TIM_CNT]

	CMP R6, R1
	BGT pwm_turned_off

	B pwm_loop_inner




pwm_turned_off:
	LDR R0, =GPIOE  @ load the address of the GPIOE register into R0
	MOV R3, 0x00
	STRB R3, [R0, #ODR + 1]   @ store this to the second byte of the ODR (bits 8-15)

	CMP R6, R2
	BGT pwm_start

	B pwm_loop_inner



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
