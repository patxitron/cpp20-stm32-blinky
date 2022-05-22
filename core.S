 // These instructions define attributes of our chip and
// the assembly language we'll use:
.syntax unified
.cpu cortex-m3
.fpu softvfp
.thumb

// Global memory locations.
.global vtable
.global default_interrupt_handler
.global reset_handler
.global sleep

// (Location to boot from for RAM startup)
.equ  boot_ram_base, 0xF108F85F

/*
 * The vector table.
 */
.type vtable, %object
.section .vector_table,"a",%progbits
vtable:
  .word _estack
  .word reset_handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .word WWDG_IRQHandler
  .word PVD_IRQHandler
  .word TAMPER_IRQHandler
  .word RTC_IRQHandler
  .word FLASH_IRQHandler
  .word RCC_IRQHandler
  .word EXTI0_IRQHandler
  .word EXTI1_IRQHandler
  .word EXTI2_IRQHandler
  .word EXTI3_IRQHandler
  .word EXTI4_IRQHandler
  .word DMA1_Channel1_IRQHandler
  .word DMA1_Channel2_IRQHandler
  .word DMA1_Channel3_IRQHandler
  .word DMA1_Channel4_IRQHandler
  .word DMA1_Channel5_IRQHandler
  .word DMA1_Channel6_IRQHandler
  .word DMA1_Channel7_IRQHandler
  .word ADC1_2_IRQHandler
  .word USB_HP_CAN1_TX_IRQHandler
  .word USB_LP_CAN1_RX0_IRQHandler
  .word CAN1_RX1_IRQHandler
  .word CAN1_SCE_IRQHandler
  .word EXTI9_5_IRQHandler
  .word TIM1_BRK_IRQHandler
  .word TIM1_UP_IRQHandler
  .word TIM1_TRG_COM_IRQHandler
  .word TIM1_CC_IRQHandler
  .word TIM2_IRQHandler
  .word TIM3_IRQHandler
  .word TIM4_IRQHandler
  .word I2C1_EV_IRQHandler
  .word I2C1_ER_IRQHandler
  .word I2C2_EV_IRQHandler
  .word I2C2_ER_IRQHandler
  .word SPI1_IRQHandler
  .word SPI2_IRQHandler
  .word USART1_IRQHandler
  .word USART2_IRQHandler
  .word USART3_IRQHandler
  .word EXTI15_10_IRQHandler
  .word RTC_Alarm_IRQHandler
  .word USBWakeUp_IRQHandler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word boot_ram_base

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the default_interrupt_handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak NMI_Handler
  .thumb_set NMI_Handler,default_interrupt_handler

  .weak HardFault_Handler
  .thumb_set HardFault_Handler,default_interrupt_handler

  .weak MemManage_Handler
  .thumb_set MemManage_Handler,default_interrupt_handler

  .weak BusFault_Handler
  .thumb_set BusFault_Handler,default_interrupt_handler

  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler,default_interrupt_handler

  .weak SVC_Handler
  .thumb_set SVC_Handler,default_interrupt_handler

  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler,default_interrupt_handler

  .weak PendSV_Handler
  .thumb_set PendSV_Handler,default_interrupt_handler

  .weak SysTick_Handler
  .thumb_set SysTick_Handler,default_interrupt_handler

  .weak WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,default_interrupt_handler

  .weak PVD_IRQHandler
  .thumb_set PVD_IRQHandler,default_interrupt_handler

  .weak TAMPER_IRQHandler
  .thumb_set TAMPER_IRQHandler,default_interrupt_handler

  .weak RTC_IRQHandler
  .thumb_set RTC_IRQHandler,default_interrupt_handler

  .weak FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler,default_interrupt_handler

  .weak RCC_IRQHandler
  .thumb_set RCC_IRQHandler,default_interrupt_handler

  .weak EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler,default_interrupt_handler

  .weak EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler,default_interrupt_handler

  .weak EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler,default_interrupt_handler

  .weak EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler,default_interrupt_handler

  .weak EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler,default_interrupt_handler

  .weak DMA1_Channel1_IRQHandler
  .thumb_set DMA1_Channel1_IRQHandler,default_interrupt_handler

  .weak DMA1_Channel2_IRQHandler
  .thumb_set DMA1_Channel2_IRQHandler,default_interrupt_handler

  .weak DMA1_Channel3_IRQHandler
  .thumb_set DMA1_Channel3_IRQHandler,default_interrupt_handler

  .weak DMA1_Channel4_IRQHandler
  .thumb_set DMA1_Channel4_IRQHandler,default_interrupt_handler

  .weak DMA1_Channel5_IRQHandler
  .thumb_set DMA1_Channel5_IRQHandler,default_interrupt_handler

  .weak DMA1_Channel6_IRQHandler
  .thumb_set DMA1_Channel6_IRQHandler,default_interrupt_handler

  .weak DMA1_Channel7_IRQHandler
  .thumb_set DMA1_Channel7_IRQHandler,default_interrupt_handler

  .weak ADC1_2_IRQHandler
  .thumb_set ADC1_2_IRQHandler,default_interrupt_handler

  .weak USB_HP_CAN1_TX_IRQHandler
  .thumb_set USB_HP_CAN1_TX_IRQHandler,default_interrupt_handler

  .weak USB_LP_CAN1_RX0_IRQHandler
  .thumb_set USB_LP_CAN1_RX0_IRQHandler,default_interrupt_handler

  .weak CAN1_RX1_IRQHandler
  .thumb_set CAN1_RX1_IRQHandler,default_interrupt_handler

  .weak CAN1_SCE_IRQHandler
  .thumb_set CAN1_SCE_IRQHandler,default_interrupt_handler

  .weak EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler,default_interrupt_handler

  .weak TIM1_BRK_IRQHandler
  .thumb_set TIM1_BRK_IRQHandler,default_interrupt_handler

  .weak TIM1_UP_IRQHandler
  .thumb_set TIM1_UP_IRQHandler,default_interrupt_handler

  .weak TIM1_TRG_COM_IRQHandler
  .thumb_set TIM1_TRG_COM_IRQHandler,default_interrupt_handler

  .weak TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,default_interrupt_handler

  .weak TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,default_interrupt_handler

  .weak TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler,default_interrupt_handler

  .weak TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler,default_interrupt_handler

  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler,default_interrupt_handler

  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler,default_interrupt_handler

  .weak I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler,default_interrupt_handler

  .weak I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler,default_interrupt_handler

  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,default_interrupt_handler

  .weak SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler,default_interrupt_handler

  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler,default_interrupt_handler

  .weak USART2_IRQHandler
  .thumb_set USART2_IRQHandler,default_interrupt_handler

  .weak USART3_IRQHandler
  .thumb_set USART3_IRQHandler,default_interrupt_handler

  .weak EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler,default_interrupt_handler

  .weak RTC_Alarm_IRQHandler
  .thumb_set RTC_Alarm_IRQHandler,default_interrupt_handler

  .weak USBWakeUp_IRQHandler
  .thumb_set USBWakeUp_IRQHandler,default_interrupt_handler
.size vtable, .-vtable

/*
 * A 'Default' interrupt handler. This is where interrupts
 * which are not otherwise configured will go.
 * It is an infinite loop, because...well, we weren't
 * expecting the interrupt, so what can we do?
 */
.section .text.default_interrupt_handler,"ax",%progbits
default_interrupt_handler:
  default_interrupt_loop:
    B default_interrupt_loop
.size default_interrupt_handler, .-default_interrupt_handler

/*
 * The Reset handler. Called on reset.
 */
.type reset_handler, %function
reset_handler:
  // Set the stack pointer to the end of the stack.
  // The '_estack' value is defined in our linker script.
  LDR  r0, =_estack
  MOV  sp, r0

  // Copy data from flash to RAM data init section.
  // R2 will store our progress along the sidata section.
  MOVS r0, #0
  // Load the start/end addresses of the data section,
  // and the start of the data init section.
  LDR  r1, =_sdata
  LDR  r2, =_edata
  LDR  r3, =_sidata
  B    copy_sidata_loop

copy_sidata:
  // Offset the data init section by our copy progress.
  LDR  r4, [r3, r0]
  // Copy the current word into data, and increment.
  STR  r4, [r1, r0]
  ADDS r0, r0, #4

copy_sidata_loop:
  // Unless we've copied the whole data section, copy the
  // next word from sidata->data.
  ADDS r4, r0, r1
  CMP  r4, r2
  BCC  copy_sidata

  // Once we are done copying the data section into RAM,
  // move on to filling the BSS section with 0s.
  MOVS r0, #0
  LDR  r1, =_sbss
  LDR  r2, =_ebss
  B    reset_bss_loop

// Fill the BSS segment with '0's.
reset_bss:
  // Store a 0 and increment by a word.
  STR  r0, [r1]
  ADDS r1, r1, #4

reset_bss_loop:
  // We'll use R1 to count progress here; if we aren't
  // done, reset the next word and increment.
  CMP  r1, r2
  BCC  reset_bss      // Branch to the 'main' method.
  B    main
.size reset_handler, .-reset_handler
