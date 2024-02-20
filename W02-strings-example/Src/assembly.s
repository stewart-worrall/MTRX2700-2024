.syntax unified
.thumb

.global main


.data
@ define variables

string_buffer: .asciz "replace this text replace this text" @ Define a null-terminated string
ascii_string: .asciz "Put your name here !\0" @ Define a null-terminated string
byte_array: .byte 0, 1, 2, 3, 4, 5, 6
word_array: .word 0x00, 0x40, 0x80, 0xc0, 0x10, 0x14, 0xffffffff


.text
@ define text


@ this is the entry function called from the startup file
main:

	LDR R0, =ascii_string  @ the address of the string
	LDR R1, =string_buffer  @ the address of the string
	LDR R2, =0x00 	@ counter to the current place in the string

string_loop:

	LDRB R3, [R0, R2]	@ load the byte from the ascii_string (byte number R2)
	STRB R3, [R1, R2]	@ store the byte in the string_buffer (byte number R2)
	CMP R3, #0	@ Test to see whether this byte is zero (for null terminated)
	BEQ finished_string  @ if it was null (0) then jump out of the loop
	ADD R2, #1  @ increment the offset R2

	B string_loop  @ loop to the next byte

finished_string:

	LDR R0, =word_array  @ the address of the word array
	LDR R2, =0x00 	@ counter to the current place in the array
	LDR R4, =0xffffffff  @ this register provides the termination word (that ends the array)

word_loop:

	LDR R3, [R0, R2]	@ load the word from the word_array (byte number R2)
	CMP R3, R4	@ Test to see whether this word is 0xffffffff (for a special terminator)
	BEQ finished_everything  @ if it was the terminating character (0xffffffff) then jump out of the loop
	ADD R2, #4  @ increment the offset R2 by 4 (the word is 4 bytes, so increment to the next word

	B word_loop  @ loop to the next word


finished_everything:

	B finished_everything 	@ infinite loop here
