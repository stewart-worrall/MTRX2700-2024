.syntax unified
.thumb

#include "initialise.s"

.global main


.data
@ define variables


.text
@ define text



@ discuss what a 'function' is, just a label the same way a variable is a label
@ these just define a memory address
@ it is how we use the memory address that gives it meaning
@ get the 'address' of a label for code or data


@ this is the entry function called from the c file
main:
	@ Branch with link to set the clocks for the I/O and UART
	BL enable_peripheral_clocks

	@ Once the clocks are started, need to initialise the discovery board I/O
	BL initialise_discovery_board

	@ store the current light pattern (binary mask) in R4
	LDR R4, =0b00110011 @ load a pattern for the set of LEDs (every second one is on)

@ 	Look at the GPIOE offset ODR, display as hex, then as binary. Look at the manual page 239

	LDR R0, =GPIOE  @ load the address of the GPIOE register into R0
	STRB R4, [R0, #ODR + 1]   @ store this to the second byte of the ODR (bits 8-15)
	EOR R4, #0xFF	@ toggle all of the bits in the byte (1->0 0->1)


program_loop:

@ 	task: read in the input button !
	LDR R0, =GPIOA	@ port for the input button
	LDR R1, [R0, IDR]

@ 	Look at the GPIOA offset IDR, display as hex, then as binary. Look at the manual page 239

	B program_loop @ return to the program_loop label
