.syntax unified
.thumb

.global main

#include "initialise.s"

.data
@ define variables


.align
@ can allocate as an array
@incoming_buffer: .byte 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
@ or allocate just as a block of space with this number of bytes
incoming_buffer: .space 62

@ One strategy is to keep a variable that lets you know the size of the buffer.
incoming_counter: .byte 62

@ Define a string
tx_string: .asciz "abcdefgh\r\n"
@ one way to know the length of the string is to just define it as a variable
tx_length: .byte 10


.text
@ define text


@ this is the entry function called from the c file
main:

	@ in class run through the functions to perform the config of the ports
	@ for more details on changing the UART, refer to the week 3 live lecture/tutorial session.

	BL initialise_power
	@BL change_clock_speed
	BL enable_peripheral_clocks
	BL enable_uart

	@ initialise the buffer and counter
	LDR R6, =incoming_buffer
	LDR R7, =incoming_counter
	LDRB R7, [R7]
	MOV R8, #0x00

	@ uncomment the next line to enter a transmission loop
	@B tx_loop



@ continue reading forever (NOTE: eventually it will run out of memory as we don't have a big buffer)
loop_forever:

	LDR R0, =UART @ the base address for the register to set up UART
	LDR R1, [R0, USART_ISR] @ load the status of the UART

	TST R1, 1 << UART_ORE | 1 << UART_FE  @ 'AND' the current status with the bit mask that we are interested in
						   @ NOTE, the ANDS is used so that if the result is '0' the z register flag is set

	BNE clear_error

	TST R1, 1 << UART_RXNE @ 'AND' the current status with the bit mask that we are interested in
							  @ NOTE, the ANDS is used so that if the result is '0' the z register flag is set

	BEQ loop_forever @ loop back to check status again if the flag indicates there is no byte waiting

	LDRB R3, [R0, USART_RDR] @ load the lowest byte (RDR bits [0:7] for an 8 bit read)
	STRB R3, [R6, R8]
	ADD R8, #1

	CMP R7, R8
	BGT no_reset
	MOV R8, #0


no_reset:

	@ load the status of the UART
	LDR R1, [R0, USART_RQR]
	ORR R1, 1 << UART_RXFRQ
	STR R1, [R0, USART_RQR]

	BGT loop_forever


clear_error:

	LDR R1, [R0, USART_ICR] @ load the status of the UART
	@ Clear the overrun/frame error flag (see page 897)
	ORR R1, 1 << UART_ORECF | 1 << UART_FECF
	STR R1, [R0, USART_ICR] @ load the status of the UART
	B loop_forever


tx_loop:

	LDR R0, =UART @ the base address for the register to set up UART

	LDR R3, =tx_string
	LDR R4, =tx_length
	LDR R4, [R4]


tx_uart:

	LDR R1, [R0, USART_ISR] @ load the status of the UART
	ANDS R1, 1 << UART_TXE  @ 'AND' the current status with the bit mask that we are interested in
						    @ NOTE, the ANDS is used so that if the result is '0' the z register flag is set

	@ loop back to check status again if the flag indicates there is no byte waiting
	BEQ tx_uart

	@ load the next value in the string into the transmit buffer for the specified UART
	LDRB R5, [R3], #1
	STRB R5, [R0, USART_TDR]

	@ note the use of the S on the end of the SUBS, this means that the register flags are set
	@ and this subtraction can be used to make a branch
	SUBS R4, #1

	@ keep looping while there are more characters to send
	BGT tx_uart

	@ make a delay before sending again
	BL delay_loop

	@ loop back to the start
	B tx_loop


delay_loop:
	LDR R9, =0xfffff

delay_inner:

	SUBS R9, #1
	BGT delay_inner
	BX LR @ return from function call



