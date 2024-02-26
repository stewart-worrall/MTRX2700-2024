.syntax unified
.thumb

#include "initialise.s"

.global main


.data
@ define variables
byte_array: .byte 0, 1, 2, 3, 4, 5, 6
@.align
word_array: .word 0, 1, 2, 3, 4, 5, 6

demo_array_1: .byte 0, 1, 2, 3, 4, 5, 6
demo_array_2: .word 0, 1, 2, 3, 4, 5, 6

.text
@ define code



@ this is the entry function called from the startup file
main:

	@ Branch with link to set the clocks for the I/O and UART
	BL enable_peripheral_clocks

	@ Once the clocks are started, need to initialise the discovery board I/O
	BL initialise_discovery_board


@ Week 2 lecture - part 1
@ lets compare a MOV with a LDR
	MOV R1, #0x24  @ load hex 24 into register 1
	LDR R1, =0x24  @ same as above, but less efficient! (but look as dissassembly)

	@ interestingly, the next command works, the one after doesn't
	@MOV R1, #0x20000000
	@MOV R1, #0x20020002
	LDR R1, =0x20000000
	LDR R1, =0x20020002
	LDR R1, =0x12345678

	@ for addresses, almost always easier to just use LDR
	@	(LDRB for a single byte)
	LDR R0, =byte_array  @ load the address of byte_array

	LDRB R1, [R0] @ load the value at the address R0 into R1
	@	(like dereferencing a pointer)

	LDRB R1, [R0, #0] @ same thing, address + no offset

	LDR R1, [R0, #0] @ look what happens if you use LDR (load a word)

	LDRB R1, [R0, #2] @ address + offset of 2 (second element)

	ADD R1, #8 @ add 8 to 2, this is 0xA

	STRB R1, [R0, #2] @ Put the updated value back

	@ note, words and bytes are different lengths,
	@	also storage in memory

	@ note, the use of LDR here as these are words
	@ look at the alignment of the values in the
	@	memory browser (@20000007 vs @20000000)
	@ add the align

	LDR R0, =word_array  @ load the address of word_array
	LDR R1, [R0] @ load the value at the address R0 into R1
	@			 (like dereferencing a pointer)

	LDR R1, [R0, #0] @ same thing, address + no offset
	LDR R1, [R0, #2] @ What has happened here !
	LDR R1, [R0, #2*4] @ What has happened here !

	ADD R1, #8 @ add 8 to 2, this is 0xA
	STR R1, [R0, #2*4] @ Put the updated value back

	@ negative values
	MOV R1, #-10	@ show in the calculator
	MOV R2, #10
	ADD R2, R1

	@ load first, then increment
	LDR R0, =demo_array_1 @ address of the demo byte array
	LDRB R1, [R0], #1
	LDRB R1, [R0], #1
	LDRB R1, [R0], #1
	LDRB R1, [R0], #1

	LDR R0, =demo_array_2 @ address of the demo word array
	LDR R1, [R0], #4	@ why add 4????
	LDR R1, [R0], #4
	LDR R1, [R0], #4
	LDR R1, [R0], #4

	@ increment first, then load
	LDR R0, =demo_array_1 @ address of the demo byte array
	LDRB R1, [R0, #1]!
	LDRB R1, [R0, #1]!
	LDRB R1, [R0, #1]!
	LDRB R1, [R0, #1]!


	@ Branching
	@ Let's recreate
	@
	@ x = 1;
	@ y = 2;
	@ if (x<y) {
	@ 	x = x + y;
	@ }
	@

	LDR R0, =1
	LDR R1, =2
	CMP R0, R1
	BLT perform_subroutine
finished_subroutine:
	@ do something else here
	b next_program

perform_subroutine:
	ADD R0, R1
	B finished_subroutine


next_program:
	LDR R0, =1
	LDR R1, =2
	CMP R1, R0  @ uncomment to show the difference in the
				 @ flags
	CMP R0, R1
	@ branch greater or equal is inverse of less than
	BGE after_subroutine
	ADD R0, R1

after_subroutine:
	@ finished if statement


@Look at the digital I/O (separate file!)
loop_until_pressed:
	LDR R0, =GPIOA
	LDR R1, [R0, IDR]
	TST R1, 0x01 @ PA0 is bit 0, so test for equal to this
	BEQ loop_until_pressed @ loop while TST returns 0


loop_until_not_pressed:
	LDR R0, =GPIOA
	LDR R1, [R0, IDR]
	TST R1, 0x01 @ PA0 is bit 0, so test for equal to this
	BNE loop_until_not_pressed @ loop while TST returns 0


@call function (BL) - hint what happens if you need to do
@it twice

	MOV R0, PC @ show the address (roughly) we will need to
			   @ return to
	@ look at the link register to see what is happening
	BL returning_function

	@ the first_function calls a second function
	@ before starting, what do we hope will happen?
	@ how would this work?
	BL first_function


	@ but that was complicated right ?
	@ we need a better way to do this.
	@ see - the stack !


	LDR R0, =0x1234
	LDR R1, =0x0001

	LDR R2, =byte_array
	LDR R4, =0x00

	ADD R0, R1

forever_loop:

	LDRB R3, [R2, R4]
	ADD R3, #0x01
	STRB R3, [R2, R4]

	ADD R4, 0x01

	@ questions - what happens when we go past the end of the array?
	@  	        - what happens in c?

	@ change the value of the addition
	@ change the LDRB to LDRH (same for STR) -  what does this mean
	@ 			 make sure to change the increment pointer - what happens otherwise


	B forever_loop



returning_function:
	LDR R5, =50
	LDR R6, =1000
	MUL R6, R5
	BX LR  @ set the PC to the value in the link register
		   @ (where we came from)



first_function:
	LDR R5, =50000
	LDR R6, =1000
	UDIV R5, R6
	@MOV R8, LR
	BL second_function
	@MOV LR, R8
	BX LR


second_function:
	LDR R5, =1234
	LDR R6, =5678
	LDR R7, =10000
	MUL R5, R7
	ADD R6, R5
	BX LR
