.syntax unified
.thumb

#include "initialise.s"

.global main


.data
@ define variables
on_time: .word 0x10000
period: .word 0x80000



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
	MOV R1, #0x02 @ put a prescaler in R1
	STR R1, [R0, TIM_PSC] @ set the prescaler register

	@ see the notes in the trigger_prescaler function
	BL trigger_prescaler

	@ questions for timed_loop
	@  what can make it run faster/slower (there are multiple ways)


pwm_loop:
	@ store the current light pattern (binary mask) in R7
	LDR R7, =0b01010101 @ load a pattern for the set of LEDs (every second one is on)

	LDR R1, =on_time
	LDR R1, [R1]

	LDR R2, =period
	LDR R2, [R2]


pwm_on_cycle:
	@ reset the counter
	LDR R0, =TIM2
	LDR R8, =0x00
	STR R8, [R0, TIM_CNT]

	@ update the on_time
	ADD R1, #0x5000

	@ if on_time > period, reset on_time (Register 1)
	CMP R2, R1
	BGT no_reset
	MOV R1, #0

no_reset:
	LDR R0, =GPIOE  @ load the address of the GPIOE register into R0
	STRB R7, [R0, #ODR + 1]   @ store this to the second byte of the ODR (bits 8-15)

@ load the current time from the counter
@ and wait until on_time has elapsed
pwm_on_loop:
	LDR R0, =TIM2  @ load the address of the timer 2 base address
	LDR R6, [R0, TIM_CNT]	@ read current time

	@ compare current time to on_time (R1)
	CMP R6, R1
	BGT pwm_off_cycle

	B pwm_on_loop


@ turn off the LEDs and
pwm_off_cycle:
	LDR R0, =GPIOE  @ load the address of the GPIOE register into R0
	MOV R4, 0x00
	STRB R4, [R0, #ODR + 1]   @ store this to the second byte of the ODR (bits 8-15)

pwm_off_loop:
	LDR R0, =TIM2  @ load the address of the timer 2 base address
	LDR R6, [R0, TIM_CNT]	@ read current time

	@ compare current time to period (R2)
	CMP R6, R2
	BGT pwm_on_cycle

	B pwm_off_loop


