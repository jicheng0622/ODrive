
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

#include <stdbool.h>


/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/


void get_regs(void** stack_ptr) {
    void* volatile r0 __attribute__((unused)) = stack_ptr[0];
    void* volatile r1 __attribute__((unused)) = stack_ptr[1];
    void* volatile r2 __attribute__((unused)) = stack_ptr[2];
    void* volatile r3 __attribute__((unused)) = stack_ptr[3];

    void* volatile r12 __attribute__((unused)) = stack_ptr[4];
    void* volatile lr __attribute__((unused)) = stack_ptr[5];  // Link register
    void* volatile pc __attribute__((unused)) = stack_ptr[6];  // Program counter
    void* volatile psr __attribute__((unused)) = stack_ptr[7];  // Program status register

    volatile bool stay_looping = true;
    while(stay_looping);
}

/** @brief Entrypoint for Non maskable interrupt. */
void NMI_Handler(void) {
    // TODO: add handler
}

/** @brief Entrypoint for Hard fault interrupt. */
__attribute__((naked))
void HardFault_Handler(void) {
    __asm(
        " tst lr, #4     \n\t"
        " ite eq         \n\t"
        " mrseq r0, msp  \n\t"
        " mrsne r0, psp  \n\t"
        " b get_regs     \n\t"
    );
}

/** @brief Entrypoint for Memory management fault. */
void MemManage_Handler(void) {
    __asm( // TODO: not sure if this is a valid action in this situation
        " tst lr, #4     \n\t"
        " ite eq         \n\t"
        " mrseq r0, msp  \n\t"
        " mrsne r0, psp  \n\t"
        " b get_regs     \n\t"
    );
    while (1) {
        // TODO: add proper handling
    }
}

/** @brief Entrypoint for Pre-fetch fault, memory access fault. */
void BusFault_Handler(void) {
    __asm( // TODO: not sure if this is a valid action in this situation
        " tst lr, #4     \n\t"
        " ite eq         \n\t"
        " mrseq r0, msp  \n\t"
        " mrsne r0, psp  \n\t"
        " b get_regs     \n\t"
    );
    while (1) {
        // TODO: add proper handling
    }
}

/** @brief Entrypoint for Undefined instruction or illegal state. */
void UsageFault_Handler(void) {
    __asm( // TODO: not sure if this is a valid action in this situation
        " tst lr, #4     \n\t"
        " ite eq         \n\t"
        " mrseq r0, msp  \n\t"
        " mrsne r0, psp  \n\t"
        " b get_regs     \n\t"
    );
    while (1) {
        // TODO: add proper handling
    }
}

/** @brief Entrypoint for Debug monitor. */
void DebugMon_Handler(void) {
    // TODO: add debug support code (allows adding breakpoints while the CPU is running)
}

/** @brief Entrypoint for System tick timer. */
void SysTick_Handler(void) {
    osSystickHandler();
}
