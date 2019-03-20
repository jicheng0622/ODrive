
#include "stm32_tim.hpp"

bool STM32_Timer_t::setup(uint32_t period, MODE mode, uint32_t prescaler, uint32_t repetition_counter) {
    if (htim.Instance == TIM1)
        __HAL_RCC_TIM1_CLK_ENABLE();
    else if (htim.Instance == TIM2)
        __HAL_RCC_TIM2_CLK_ENABLE();
    else if (htim.Instance == TIM3)
        __HAL_RCC_TIM3_CLK_ENABLE();
    else if (htim.Instance == TIM4)
        __HAL_RCC_TIM4_CLK_ENABLE();
    else if (htim.Instance == TIM5)
        __HAL_RCC_TIM5_CLK_ENABLE();
    else if (htim.Instance == TIM6)
        __HAL_RCC_TIM6_CLK_ENABLE();
    else if (htim.Instance == TIM7)
        __HAL_RCC_TIM7_CLK_ENABLE();
    else if (htim.Instance == TIM8)
        __HAL_RCC_TIM8_CLK_ENABLE();
    else if (htim.Instance == TIM9)
        __HAL_RCC_TIM9_CLK_ENABLE();
    else if (htim.Instance == TIM10)
        __HAL_RCC_TIM10_CLK_ENABLE();
    else if (htim.Instance == TIM11)
        __HAL_RCC_TIM11_CLK_ENABLE();
    else if (htim.Instance == TIM12)
        __HAL_RCC_TIM12_CLK_ENABLE();
    else if (htim.Instance == TIM13)
        __HAL_RCC_TIM13_CLK_ENABLE();
    else if (htim.Instance == TIM14)
        __HAL_RCC_TIM14_CLK_ENABLE();
    else
        return false;

    htim.Init.Prescaler = prescaler;
    switch (mode) {
        case UP: htim.Init.CounterMode = TIM_COUNTERMODE_UP; break;
        case DOWN: htim.Init.CounterMode = TIM_COUNTERMODE_DOWN; break;
        case UP_DOWN: htim.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3; break;
        default: return false;
    }
    htim.Init.Period = period;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim.Init.RepetitionCounter = repetition_counter;
    if (HAL_TIM_Base_Init(&htim) != HAL_OK)
        return false;

    TIM_ClockConfigTypeDef sClockSourceConfig;
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig) != HAL_OK)
        return false;

    TIM_MasterConfigTypeDef sMasterConfig;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; // tim1 and tim8 had this on TIM_TRGO_UPDATE
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)
        return false;


    return true;
}

bool STM32_Timer_t::setup_pwm(uint32_t channel_num,
        STM32_GPIO_t* gpio_p, STM32_GPIO_t* gpio_n,
        bool active_high_p, bool active_high_n,
        uint32_t initial_val) {

    if (HAL_TIM_PWM_Init(&htim) != HAL_OK)
        return false;

    uint32_t channel_val = 0;
    if (channel_num == 1)
        channel_val = TIM_CHANNEL_1;
    else if (channel_num == 2)
        channel_val = TIM_CHANNEL_2;
    else if (channel_num == 3)
        channel_val = TIM_CHANNEL_3;
    else if (channel_num == 4)
        channel_val = TIM_CHANNEL_4;
    else
        return false;
  
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = initial_val;
    sConfigOC.OCPolarity = active_high_p ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = active_high_n ? TIM_OCNPOLARITY_HIGH : TIM_OCNPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, channel_val) != HAL_OK)
        return false;

    if (gpio_p)
        if (!gpio_p->setup_alternate_function(p_gpios[channel_num - 1], gpio_af, GPIO_t::NO_PULL, GPIO_t::SLOW))
            return false;

    if (gpio_n)
        if (!gpio_n->setup_alternate_function(n_gpios[channel_num - 1], gpio_af, GPIO_t::NO_PULL, GPIO_t::SLOW))
            return false;

    return true;
}

bool STM32_Timer_t::set_dead_time(uint32_t dead_time) {
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = dead_time;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim, &sBreakDeadTimeConfig) != HAL_OK)
        return false;
    
    return true;
}

bool STM32_Timer_t::setup_output_compare() {
    if (HAL_TIM_OC_Init(&htim) != HAL_OK)
        return false;

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    if (HAL_TIM_OC_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
        return false;

    return true;
}

bool STM32_Timer_t::config_encoder_mode(STM32_GPIO_t* gpio_ch1, STM32_GPIO_t* gpio_ch2) {
    // TODO: make more flexible
    TIM_Encoder_InitTypeDef sConfig;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 4;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 4;
    if (HAL_TIM_Encoder_Init(&htim, &sConfig) != HAL_OK)
        return false;

    if (gpio_ch1)
        if (!gpio_ch1->setup_alternate_function(p_gpios[0], gpio_af, GPIO_t::NO_PULL, GPIO_t::SLOW))
            return false;

    if (gpio_ch2)
        if (!gpio_ch2->setup_alternate_function(p_gpios[1], gpio_af, GPIO_t::NO_PULL, GPIO_t::SLOW))
            return false;

    return true;
}

bool STM32_Timer_t::config_input_compare_mode(STM32_GPIO_t* gpio_ch3, STM32_GPIO_t* gpio_ch4) {
    TIM_IC_InitTypeDef sConfigIC;
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 15;

    if (HAL_TIM_IC_Init(&htim) != HAL_OK)
        return false;

    if (gpio_ch3) {
        if (HAL_TIM_IC_ConfigChannel(&htim, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
            return false;
        if (!gpio_ch3->setup_alternate_function(p_gpios[2], gpio_af, GPIO_t::NO_PULL, GPIO_t::SLOW))
            return false;
    }

    if (gpio_ch4) {
        if (HAL_TIM_IC_ConfigChannel(&htim, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
            return false;
        if (!gpio_ch4->setup_alternate_function(p_gpios[3], gpio_af, GPIO_t::NO_PULL, GPIO_t::SLOW))
            return false;
    }

    return true;
}

bool STM32_Timer_t::get_general_irqn(IRQn_Type* irqn) {
    // note that timer 1 and timer 8 don't have a general interrupt vector,
    // but rather several specific ones.
    if (!irqn) {
        return false;
    } else if (htim.Instance == TIM2) {
        return *irqn = TIM2_IRQn, true;
    } else if (htim.Instance == TIM3) {
        return *irqn = TIM3_IRQn, true;
    } else if (htim.Instance == TIM4) {
        return *irqn = TIM4_IRQn, true;
    } else if (htim.Instance == TIM5) {
        return *irqn = TIM5_IRQn, true;
    } else if (htim.Instance == TIM6) {
        return *irqn = TIM6_DAC_IRQn, true;
    } else if (htim.Instance == TIM7) {
        return *irqn = TIM7_IRQn, true;
    } else if (htim.Instance == TIM9) {
        return *irqn = TIM1_BRK_TIM9_IRQn, true;
    } else if (htim.Instance == TIM10) {
        return *irqn = TIM1_UP_TIM10_IRQn, true;
    } else if (htim.Instance == TIM11) {
        return *irqn = TIM1_TRG_COM_TIM11_IRQn, true;
    } else if (htim.Instance == TIM12) {
        return *irqn = TIM8_BRK_TIM12_IRQn, true;
    } else if (htim.Instance == TIM13) {
        return *irqn = TIM8_UP_TIM13_IRQn, true;
    } else if (htim.Instance == TIM14) {
        return *irqn = TIM8_TRG_COM_TIM14_IRQn, true;
    } else {
        return false;
    }
};

bool STM32_Timer_t::enable_update_interrupt() {
    IRQn_Type irqn;
    if (htim.Instance == TIM1) {
        irqn = TIM1_UP_TIM10_IRQn;
    } else if (htim.Instance == TIM8) {
        irqn = TIM8_UP_TIM13_IRQn;
    } else if (!get_general_irqn(&irqn)) {
        return false;
    }

    HAL_NVIC_SetPriority(irqn, 0, 0);
    HAL_NVIC_EnableIRQ(irqn);

    return true;
}

bool STM32_Timer_t::enable_trigger_interrupt() {
    IRQn_Type irqn;
    if (htim.Instance == TIM1) {
        irqn = TIM1_TRG_COM_TIM11_IRQn;
    } else if (htim.Instance == TIM8) {
        irqn = TIM8_TRG_COM_TIM14_IRQn;
    } else if (!get_general_irqn(&irqn)) {
        return false;
    }

    HAL_NVIC_SetPriority(irqn, 0, 0);
    HAL_NVIC_EnableIRQ(irqn);

    return true;
}

bool STM32_Timer_t::enable_cc_interrupt() {
    IRQn_Type irqn;
    if (htim.Instance == TIM1) {
        irqn = TIM1_CC_IRQn;
    } else if (htim.Instance == TIM8) {
        irqn = TIM8_CC_IRQn;
    } else if (!get_general_irqn(&irqn)) {
        return false;
    }

    HAL_NVIC_SetPriority(irqn, 0, 0);
    HAL_NVIC_EnableIRQ(irqn);

    return true;
}

bool STM32_Timer_t::enable_break_interrupt() {
    IRQn_Type irqn;
    if (htim.Instance == TIM1) {
        irqn = TIM1_BRK_TIM9_IRQn;
    } else if (htim.Instance == TIM8) {
        irqn = TIM8_BRK_TIM12_IRQn;
    } else if (!get_general_irqn(&irqn)) {
        return false;
    }

    HAL_NVIC_SetPriority(irqn, 0, 0);
    HAL_NVIC_EnableIRQ(irqn);

    return true;
}

void STM32_Timer_t::handle_update_irq() {
    if (__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_UPDATE)) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim, TIM_IT_UPDATE)) {
            __HAL_TIM_CLEAR_IT(&htim, TIM_IT_UPDATE);
            on_update_.invoke();
        }
    }
}

void STM32_Timer_t::handle_trigger_irq() {
    if (__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_TRIGGER)) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim, TIM_IT_TRIGGER)) {
            __HAL_TIM_CLEAR_IT(&htim, TIM_IT_TRIGGER);
            on_trigger_.invoke();
        }
    }
}

void STM32_Timer_t::handle_cc_irq() {
    if (__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_CC1)) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim, TIM_IT_CC1)) {
            __HAL_TIM_CLEAR_IT(&htim, TIM_IT_CC1);
            on_cc_.invoke(1, htim.Instance->CCR1);
        }
    }
    if (__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_CC2)) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim, TIM_IT_CC2)) {
            __HAL_TIM_CLEAR_IT(&htim, TIM_IT_CC2);
            on_cc_.invoke(2, htim.Instance->CCR2);
        }
    }
    if (__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_CC3)) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim, TIM_IT_CC3)) {
            __HAL_TIM_CLEAR_IT(&htim, TIM_IT_CC3);
            on_cc_.invoke(3, htim.Instance->CCR3);
        }
    }
    if (__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_CC4)) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim, TIM_IT_CC4)) {
            __HAL_TIM_CLEAR_IT(&htim, TIM_IT_CC4);
            on_cc_.invoke(4, htim.Instance->CCR4);
        }
    }
}

void STM32_Timer_t::handle_break_irq() {
    if (__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_BREAK)) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim, TIM_IT_BREAK)) {
            __HAL_TIM_CLEAR_IT(&htim, TIM_IT_BREAK);
            on_break_.invoke();
        }
    }
}


/* Interrupt entrypoints -----------------------------------------------------*/

extern "C" {
void TIM1_UP_TIM10_IRQHandler(void);
void TIM1_TRG_COM_TIM11_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM1_BRK_TIM9_IRQHandler(void);

void TIM8_UP_TIM13_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void TIM8_CC_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void);

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM5_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);
}

/** @brief Entrypoint for the TIM1 update interrupt and TIM10 global interrupt. */
void TIM1_UP_TIM10_IRQHandler(void) {
    tim1.handle_update_irq();
    tim10.handle_any_irq();
}

/** @brief Entrypoint for the TIM1 trigger and commutation interrupt and TIM11 global interrupt. */
void TIM1_TRG_COM_TIM11_IRQHandler(void) {
    tim1.handle_trigger_irq();
    tim11.handle_any_irq();
}

/** @brief Entrypoint for the TIM1 capture compare interrupt. */
void TIM1_CC_IRQHandler(void) {
    tim1.handle_cc_irq();
}

/** @brief Entrypoint for the TIM1 break interrupt and TIM9 global interrupt. */
void TIM1_BRK_TIM9_IRQHandler(void) {
    tim1.handle_break_irq();
    tim9.handle_any_irq();
}

/** @brief Entrypoint for the TIM8 update interrupt and TIM13 global interrupt. */
void TIM8_UP_TIM13_IRQHandler(void) {
    tim8.handle_update_irq();
    tim13.handle_any_irq();
}

/** @brief Entrypoint for the TIM8 trigger and commutation interrupt and TIM14 global interrupt. */
void TIM8_TRG_COM_TIM14_IRQHandler(void) {
    tim8.handle_trigger_irq();
    tim14.handle_any_irq();
}

/** @brief Entrypoint for the TIM8 capture compare interrupt. */
void TIM8_CC_IRQHandler(void) {
    tim8.handle_cc_irq();
}

/** @brief Entrypoint for the TIM8 break interrupt and TIM12 global interrupt. */
void TIM8_BRK_TIM12_IRQHandler(void) {
    tim8.handle_break_irq();
    tim12.handle_any_irq();
}

/** @brief Entrypoint for the TIM2 global interrupt. */
void TIM2_IRQHandler(void) {
    tim2.handle_any_irq();
}

/** @brief Entrypoint for the TIM3 global interrupt. */
void TIM3_IRQHandler(void) {
    tim3.handle_any_irq();
}

/** @brief Entrypoint for the TIM4 global interrupt. */
void TIM4_IRQHandler(void) {
    tim4.handle_any_irq();
}

/** @brief Entrypoint for the TIM5 global interrupt. */
void TIM5_IRQHandler(void) {
    tim5.handle_any_irq();
}

/** @brief Entrypoint for the TIM6 global interrupt. */
void TIM6_DAC_IRQHandler(void) {
    tim6.handle_any_irq();
}

/** @brief Entrypoint for the TIM7 global interrupt. */
void TIM7_IRQHandler(void) {
    tim7.handle_any_irq();
}


/* Peripheral definitions ----------------------------------------------------*/

const STM32_GPIO_t* no_gpios[] = { nullptr };

const STM32_GPIO_t* tim1_ch1p_gpios[] = { &pa8, &pe9, nullptr };
const STM32_GPIO_t* tim1_ch2p_gpios[] = { &pa9, &pe11, nullptr };
const STM32_GPIO_t* tim1_ch3p_gpios[] = { &pa10, &pe13, nullptr };
const STM32_GPIO_t* tim1_ch4p_gpios[] = { &pa11, &pe14, nullptr };
const STM32_GPIO_t* tim1_ch1n_gpios[] = { &pa6, &pb13, &pe8, nullptr };
const STM32_GPIO_t* tim1_ch2n_gpios[] = { &pb0, &pb14, &pe10, nullptr };
const STM32_GPIO_t* tim1_ch3n_gpios[] = { &pb1, &pb15, &pe12, nullptr };
const STM32_GPIO_t* tim1_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim1_bkin_gpios[] = { &pa6, &pb12, &pe15, nullptr };
const STM32_GPIO_t* tim1_etr_gpios[] = { &pa12, &pe7, nullptr };

const STM32_GPIO_t* tim2_ch1p_gpios[] = { &pa0, &pa5, nullptr };
const STM32_GPIO_t* tim2_ch2p_gpios[] = { &pa1, &pb3, nullptr };
const STM32_GPIO_t* tim2_ch3p_gpios[] = { &pa2, &pb10, nullptr };
const STM32_GPIO_t* tim2_ch4p_gpios[] = { &pa3, &pb11, nullptr };
const STM32_GPIO_t* tim2_ch1n_gpios[] = { nullptr };
const STM32_GPIO_t* tim2_ch2n_gpios[] = { nullptr };
const STM32_GPIO_t* tim2_ch3n_gpios[] = { nullptr };
const STM32_GPIO_t* tim2_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim2_bkin_gpios[] = { nullptr };
const STM32_GPIO_t* tim2_etr_gpios[] = { &pa0, &pa5, nullptr };

const STM32_GPIO_t* tim3_ch1p_gpios[] = { &pa6, &pb4, &pc6, nullptr };
const STM32_GPIO_t* tim3_ch2p_gpios[] = { &pa7, &pb5, &pc7, nullptr };
const STM32_GPIO_t* tim3_ch3p_gpios[] = { &pb0, &pc8, nullptr };
const STM32_GPIO_t* tim3_ch4p_gpios[] = { &pb1, &pc9, nullptr };
const STM32_GPIO_t* tim3_ch1n_gpios[] = { nullptr };
const STM32_GPIO_t* tim3_ch2n_gpios[] = { nullptr };
const STM32_GPIO_t* tim3_ch3n_gpios[] = { nullptr };
const STM32_GPIO_t* tim3_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim3_bkin_gpios[] = { nullptr };
const STM32_GPIO_t* tim3_etr_gpios[] = { &pd2, nullptr };

const STM32_GPIO_t* tim4_ch1p_gpios[] = { &pb6, &pd12, nullptr };
const STM32_GPIO_t* tim4_ch2p_gpios[] = { &pb7, &pd13, nullptr };
const STM32_GPIO_t* tim4_ch3p_gpios[] = { &pb8, &pd14, nullptr };
const STM32_GPIO_t* tim4_ch4p_gpios[] = { &pb9, &pd15, nullptr };
const STM32_GPIO_t* tim4_ch1n_gpios[] = { nullptr };
const STM32_GPIO_t* tim4_ch2n_gpios[] = { nullptr };
const STM32_GPIO_t* tim4_ch3n_gpios[] = { nullptr };
const STM32_GPIO_t* tim4_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim4_bkin_gpios[] = { nullptr };
const STM32_GPIO_t* tim4_etr_gpios[] = { &pe0, nullptr };

const STM32_GPIO_t* tim5_ch1p_gpios[] = { &pa0, &ph10, nullptr };
const STM32_GPIO_t* tim5_ch2p_gpios[] = { &pa1, &ph11, nullptr };
const STM32_GPIO_t* tim5_ch3p_gpios[] = { &pa2, &ph12, nullptr };
const STM32_GPIO_t* tim5_ch4p_gpios[] = { &pa3, &pi0, nullptr };
const STM32_GPIO_t* tim5_ch1n_gpios[] = { nullptr };
const STM32_GPIO_t* tim5_ch2n_gpios[] = { nullptr };
const STM32_GPIO_t* tim5_ch3n_gpios[] = { nullptr };
const STM32_GPIO_t* tim5_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim5_bkin_gpios[] = { nullptr };
const STM32_GPIO_t* tim5_etr_gpios[] = { nullptr };

const STM32_GPIO_t* tim8_ch1p_gpios[] = { &pc6, &pi5, nullptr };
const STM32_GPIO_t* tim8_ch2p_gpios[] = { &pc7, &pi6, nullptr };
const STM32_GPIO_t* tim8_ch3p_gpios[] = { &pc8, &pi7, nullptr };
const STM32_GPIO_t* tim8_ch4p_gpios[] = { &pc9, &pi2, nullptr };
const STM32_GPIO_t* tim8_ch1n_gpios[] = { &pa5, &pa7, &ph13, nullptr };
const STM32_GPIO_t* tim8_ch2n_gpios[] = { &pb0, &pb14, &ph14, nullptr };
const STM32_GPIO_t* tim8_ch3n_gpios[] = { &pb1, &pb15, &ph15, nullptr };
const STM32_GPIO_t* tim8_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim8_bkin_gpios[] = { &pa6, &pi4, nullptr };
const STM32_GPIO_t* tim8_etr_gpios[] = { &pa0, &pi3, nullptr };

const STM32_GPIO_t* tim9_ch1p_gpios[] = { &pa2, &pe5, nullptr };
const STM32_GPIO_t* tim9_ch2p_gpios[] = { &pa3, &pe6, nullptr };
const STM32_GPIO_t* tim9_ch3p_gpios[] = { nullptr };
const STM32_GPIO_t* tim9_ch4p_gpios[] = { nullptr };
const STM32_GPIO_t* tim9_ch1n_gpios[] = { nullptr };
const STM32_GPIO_t* tim9_ch2n_gpios[] = { nullptr };
const STM32_GPIO_t* tim9_ch3n_gpios[] = { nullptr };
const STM32_GPIO_t* tim9_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim9_bkin_gpios[] = { nullptr };
const STM32_GPIO_t* tim9_etr_gpios[] = { nullptr };

const STM32_GPIO_t* tim10_ch1p_gpios[] = { &pb8, &pf6, nullptr };
const STM32_GPIO_t* tim10_ch2p_gpios[] = { nullptr };
const STM32_GPIO_t* tim10_ch3p_gpios[] = { nullptr };
const STM32_GPIO_t* tim10_ch4p_gpios[] = { nullptr };
const STM32_GPIO_t* tim10_ch1n_gpios[] = { nullptr };
const STM32_GPIO_t* tim10_ch2n_gpios[] = { nullptr };
const STM32_GPIO_t* tim10_ch3n_gpios[] = { nullptr };
const STM32_GPIO_t* tim10_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim10_bkin_gpios[] = { nullptr };
const STM32_GPIO_t* tim10_etr_gpios[] = { nullptr };

const STM32_GPIO_t* tim11_ch1p_gpios[] = { &pb9, &pf7, nullptr };
const STM32_GPIO_t* tim11_ch2p_gpios[] = { nullptr };
const STM32_GPIO_t* tim11_ch3p_gpios[] = { nullptr };
const STM32_GPIO_t* tim11_ch4p_gpios[] = { nullptr };
const STM32_GPIO_t* tim11_ch1n_gpios[] = { nullptr };
const STM32_GPIO_t* tim11_ch2n_gpios[] = { nullptr };
const STM32_GPIO_t* tim11_ch3n_gpios[] = { nullptr };
const STM32_GPIO_t* tim11_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim11_bkin_gpios[] = { nullptr };
const STM32_GPIO_t* tim11_etr_gpios[] = { nullptr };

const STM32_GPIO_t* tim12_ch1p_gpios[] = { &pb14, &ph6, nullptr };
const STM32_GPIO_t* tim12_ch2p_gpios[] = { &pb15, &ph9, nullptr };
const STM32_GPIO_t* tim12_ch3p_gpios[] = { nullptr };
const STM32_GPIO_t* tim12_ch4p_gpios[] = { nullptr };
const STM32_GPIO_t* tim12_ch1n_gpios[] = { nullptr };
const STM32_GPIO_t* tim12_ch2n_gpios[] = { nullptr };
const STM32_GPIO_t* tim12_ch3n_gpios[] = { nullptr };
const STM32_GPIO_t* tim12_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim12_bkin_gpios[] = { nullptr };
const STM32_GPIO_t* tim12_etr_gpios[] = { nullptr };

const STM32_GPIO_t* tim13_ch1p_gpios[] = { &pa6, &pf8, nullptr };
const STM32_GPIO_t* tim13_ch2p_gpios[] = { nullptr };
const STM32_GPIO_t* tim13_ch3p_gpios[] = { nullptr };
const STM32_GPIO_t* tim13_ch4p_gpios[] = { nullptr };
const STM32_GPIO_t* tim13_ch1n_gpios[] = { nullptr };
const STM32_GPIO_t* tim13_ch2n_gpios[] = { nullptr };
const STM32_GPIO_t* tim13_ch3n_gpios[] = { nullptr };
const STM32_GPIO_t* tim13_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim13_bkin_gpios[] = { nullptr };
const STM32_GPIO_t* tim13_etr_gpios[] = { nullptr };

const STM32_GPIO_t* tim14_ch1p_gpios[] = { &pa7, &pf9, nullptr };
const STM32_GPIO_t* tim14_ch2p_gpios[] = { nullptr };
const STM32_GPIO_t* tim14_ch3p_gpios[] = { nullptr };
const STM32_GPIO_t* tim14_ch4p_gpios[] = { nullptr };
const STM32_GPIO_t* tim14_ch1n_gpios[] = { nullptr };
const STM32_GPIO_t* tim14_ch2n_gpios[] = { nullptr };
const STM32_GPIO_t* tim14_ch3n_gpios[] = { nullptr };
const STM32_GPIO_t* tim14_ch4n_gpios[] = { nullptr };
const STM32_GPIO_t* tim14_bkin_gpios[] = { nullptr };
const STM32_GPIO_t* tim14_etr_gpios[] = { nullptr };

STM32_Timer_t tim1(TIM1,
        tim1_ch1p_gpios, tim1_ch2p_gpios, tim1_ch3p_gpios, tim1_ch4p_gpios,
        tim1_ch1n_gpios, tim1_ch2n_gpios, tim1_ch3n_gpios, tim1_ch4n_gpios,
        GPIO_AF1_TIM1);

STM32_Timer_t tim2(TIM2,
        tim2_ch1p_gpios, tim2_ch2p_gpios, tim2_ch3p_gpios, tim2_ch4p_gpios,
        tim2_ch1n_gpios, tim2_ch2n_gpios, tim2_ch3n_gpios, tim2_ch4n_gpios,
        GPIO_AF1_TIM2);

STM32_Timer_t tim3(TIM3,
        tim3_ch1p_gpios, tim3_ch2p_gpios, tim3_ch3p_gpios, tim3_ch4p_gpios,
        tim3_ch1n_gpios, tim3_ch2n_gpios, tim3_ch3n_gpios, tim3_ch4n_gpios,
        GPIO_AF2_TIM3);

STM32_Timer_t tim4(TIM4,
        tim4_ch1p_gpios, tim4_ch2p_gpios, tim4_ch3p_gpios, tim4_ch4p_gpios,
        tim4_ch1n_gpios, tim4_ch2n_gpios, tim4_ch3n_gpios, tim4_ch4n_gpios,
        GPIO_AF2_TIM4);

STM32_Timer_t tim5(TIM5,
        tim5_ch1p_gpios, tim5_ch2p_gpios, tim5_ch3p_gpios, tim5_ch4p_gpios,
        tim5_ch1n_gpios, tim5_ch2n_gpios, tim5_ch3n_gpios, tim5_ch4n_gpios,
        GPIO_AF2_TIM5);

STM32_Timer_t tim6(TIM6,
        no_gpios, no_gpios, no_gpios, no_gpios,
        no_gpios, no_gpios, no_gpios, no_gpios,
        0);

STM32_Timer_t tim7(TIM7,
        no_gpios, no_gpios, no_gpios, no_gpios,
        no_gpios, no_gpios, no_gpios, no_gpios,
        0);

STM32_Timer_t tim8(TIM8,
        tim8_ch1p_gpios, tim8_ch2p_gpios, tim8_ch3p_gpios, tim8_ch4p_gpios,
        tim8_ch1n_gpios, tim8_ch2n_gpios, tim8_ch3n_gpios, tim8_ch4n_gpios,
        GPIO_AF3_TIM8);

STM32_Timer_t tim9(TIM9,
        tim9_ch1p_gpios, tim9_ch2p_gpios, tim9_ch3p_gpios, tim9_ch4p_gpios,
        tim9_ch1n_gpios, tim9_ch2n_gpios, tim9_ch3n_gpios, tim9_ch4n_gpios,
        GPIO_AF3_TIM9);

STM32_Timer_t tim10(TIM10,
        tim10_ch1p_gpios, tim10_ch2p_gpios, tim10_ch3p_gpios, tim10_ch4p_gpios,
        tim10_ch1n_gpios, tim10_ch2n_gpios, tim10_ch3n_gpios, tim10_ch4n_gpios,
        GPIO_AF3_TIM10);

STM32_Timer_t tim11(TIM11,
        tim11_ch1p_gpios, tim11_ch2p_gpios, tim11_ch3p_gpios, tim11_ch4p_gpios,
        tim11_ch1n_gpios, tim11_ch2n_gpios, tim11_ch3n_gpios, tim11_ch4n_gpios,
        GPIO_AF3_TIM11);

STM32_Timer_t tim12(TIM12,
        tim12_ch1p_gpios, tim12_ch2p_gpios, tim12_ch3p_gpios, tim12_ch4p_gpios,
        tim12_ch1n_gpios, tim12_ch2n_gpios, tim12_ch3n_gpios, tim12_ch4n_gpios,
        GPIO_AF9_TIM12);

STM32_Timer_t tim13(TIM13,
        tim13_ch1p_gpios, tim13_ch2p_gpios, tim13_ch3p_gpios, tim13_ch4p_gpios,
        tim13_ch1n_gpios, tim13_ch2n_gpios, tim13_ch3n_gpios, tim13_ch4n_gpios,
        GPIO_AF9_TIM13);

STM32_Timer_t tim14(TIM14,
        tim14_ch1p_gpios, tim14_ch2p_gpios, tim14_ch3p_gpios, tim14_ch4p_gpios,
        tim14_ch1n_gpios, tim14_ch2n_gpios, tim14_ch3n_gpios, tim14_ch4n_gpios,
        GPIO_AF9_TIM14);
