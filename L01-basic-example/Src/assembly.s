.syntax unified
.thumb

.global main

.data
@ define variables

byte_array_1: .byte 0,1,2,3,4,5,6,7
word_array_1: .word 0x01234567

.text
@ define code

@ this is the entry function called from the startup file
main:

	LDR R0, =byte_array_1
	LDR R0, =word_array_1

	LDR R0, =0x1234
	LDR R1, =0x0001

forever_loop :

	ADD R0 , R1
	B forever_loop
