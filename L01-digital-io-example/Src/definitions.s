
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


.equ TIM7, 0x40001400	@ base register for timer 7

@ registers for enabling clocks
.equ AHBENR, 0x14  @ enable peripherals
.equ APB1ENR, 0x1C @ enable peripherals on bus 1
.equ APB2ENR, 0x18 @ enable peripherals on bus 2

@ bit positions for enabling GPIO in AHBENR
.equ GPIOA_ENABLE, 17
.equ GPIOC_ENABLE, 19
