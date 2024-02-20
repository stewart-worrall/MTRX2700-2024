.syntax unified
.thumb


@ general purpose timer registers page 647

@ Clock setting register (base address and offsets)
.equ RCC, 0x40021000	@ base register for resetting and clock settings

@ registers for enabling clocks
.equ AHBENR, 0x14  @ enable peripherals
.equ APB1ENR, 0x1C @ enable peripherals on bus 1
.equ APB2ENR, 0x18 @ enable peripherals on bus 2

@ bit positions for enabling GPIO in AHBENR
.equ GPIOA_ENABLE, 17
.equ GPIOC_ENABLE, 19
.equ GPIOE_ENABLE, 21


@ enable the clocks for timer 2
.equ RCC, 0x40021000
.equ APB1ENR, 0x1C
.equ TIM2EN, 0


.equ GPIOA, 0x48000000	@ base register for GPIOA (pa0 is the button)
.equ GPIOC, 0x48000800	@ base register for GPIOA (pa0 is the button)
.equ GPIOE, 0x48001000	@ base register for GPIOE (pe8-15 are the LEDs)

@ GPIO register offsets
.equ MODER, 0x00	@ register for setting the port mode (in/out/etc)
.equ ODR, 0x14	@ GPIO output register

.equ GPIOx_AFRH, 0x24 @ offset for setting the alternate pin function


@ timer defined values
.equ TIM2, 0x40000000	@ base address for the general purpose timer 2 (TIM2)
.equ TIM_CR1, 0x00	@ control registers
.equ TIM_CCMR1, 0x18  @ compare capture settings register
.equ TIM_CNT, 0x24  @ The actual counter location
.equ TIM_ARR, 0x2C  @ The register for the auto-reload
.equ TIM_PSC, 0x28  @ prescaler
.equ TIM_CCER, 0x20 @ control register for output/capture
.equ TIM_CCR1, 0x34 @ capture/compare register for channel 1
.equ TIM_SR, 0x10 @ status of the timer
.equ TIM_DIER, 0x0C @ enable interrupts
