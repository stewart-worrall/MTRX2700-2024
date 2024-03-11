.syntax unified
.thumb

.global main


.data
@ define variables

.text
@ define code


function_2:
	SUB SP, #20
	@ do something here
	@ LR is equal to after_function_2
	ADD SP, #20
	BX LR


function_1:
	PUSH {LR}
	@ do something here
	@ LR is equal to after_function_1
	BL function_2
	@ LR is now changed to after_function_2
after_function_2:
	POP {LR}
	BX LR






@ this is the entry function called from the startup file
main:

	@ the first_function calls a second function
	@ before starting, what do we hope will happen?
	@ we learnt that we can store the link register
	@ before calling the second function
	BL first_function


	@ We run a function that calls another function
	@ Instead of storing in a register, we can use the
	@ special memory location for local variables (the stack).
	LDR R0, =0x1234
	BL function_1
after_function_1:

	@ But, this doesn't work if the function is recursive,
	@ or if there are a number of functions to call.
	@ - to solve this, we can use the stack.

	LDR R0, =0x123
	LDR R1, =0x456
	LDR R2, =0x789
	BL first_function_stack_1

	@ the push and pop functions can store more than one
	@ register in the same command
	BL first_function_stack_2


	LDR R0, =4000
	LDR R1, =40
	@ the next function expects to use the R0 and R1 as
	@ input parameters
	@ this returns the value in R0
	@ R0 = R0 / R1
	BL function_r0_div_r1

	@ NOTE: the c compiler extensively uses the stack
	@ in a similar fashion


	B main



first_function:
	LDR R5, =50000
	LDR R6, =1000
	UDIV R5, R6
	MOV R8, LR
	BL second_function
	MOV LR, R8
	BX LR



second_function:

	PUSH {R0}
	PUSH {R1}
	PUSH {R2}
	PUSH {LR}

	LDR R0, =1234
	LDR R1, =5678
	LDR R2, =10000
	MUL R0, R2
	ADD R2, R0

	POP {LR}
	POP {R2}
	POP {R1}
	POP {R0}

	BX LR

first_function_stack_1:
	@ can now use R0, R1, R2 without worrying
	@ about who else would use it
	@
	@ Can also call BX without worrying
	@ as the link to the calling function
	@ is now preserved
	PUSH {R0}
	PUSH {R1}
	PUSH {R2}
	PUSH {LR}
	LDR R1, =50000
	LDR R2, =1000
	UDIV R1, R2

	BL second_function

	@ need to do this in the reverse order
	@ (think about putting the plates on a stack - LIFO)
	POP {LR}
	POP {R2}
	POP {R1}
	POP {R0}

	BX LR



first_function_stack_2:
	@ can now use R0, R1, R2 without worrying
	@ about who else would use it
	@
	@ Can also call BX without worrying
	@ as the link to the calling function
	@ is now preserved
	PUSH {R0-R2}
	PUSH {LR}
	LDR R1, =20000
	LDR R2, =100
	UDIV R1, R2

	BL second_function

	POP {LR}
	POP {R0-R2}

	BX LR



function_r0_div_r1:
	@ can now use R0, R1, R2 without worrying
	@ about who else would use it
	@
	@ Can also call BX without worrying
	@ as the link to the calling function
	@ is now preserved

	PUSH {R0-R2, LR}

	@ because the stack goes down, the things
	@ we have already added have a higher
	@ address (+ offset)
	@
	@ Current stack points to the top element
	@ different stacks have different strategies
	@ some point to the next available address
	@
	@ previous R0 = SP + 0
	@ previous R1 = SP + 4
	@ previous R2 = SP + 8
	@ previous LR = SP + 12
	LDR R5, [SP, 0]
	LDR R6, [SP, 4]
	LDR R7, [SP, 8]
	LDR R8, [SP, 12]

	UDIV R5, R6

	@ NOTE: by changing the memory in the stack that
	@ is used to store pushed registers, when we return
	@ the value of the register is changed
	STR R5, [SP, 0] @ replace the incoming R0

	BL second_function

	@ if we pop the LR to the program counter,
	@ we automatically return from the function
	POP {R0-R2, PC}

	@ don't need this anymore
	@ NOTE: the memory has not changed, just the
	@  the current stack pointer
	@BX LR

