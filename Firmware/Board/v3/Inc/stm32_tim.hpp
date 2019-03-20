#ifndef __STM32_TIM_HPP
#define __STM32_TIM_HPP

#include <stm32_gpio.hpp>
#include <subscriber.hpp>

class STM32_Timer_t {
public:
    enum MODE {
        UP,
        DOWN,
        UP_DOWN
    };

    TIM_HandleTypeDef htim;

    const STM32_GPIO_t** p_gpios[4];
    const STM32_GPIO_t** n_gpios[4];
    uint8_t gpio_af;

    /** @brief Triggered on a timer update interrupt */
    Subscriber<> on_update_;

    /** @brief Triggered on a timer trigger interrupt */
    Subscriber<> on_trigger_;

    /** @brief Triggered on a timer cc interrupt */
    Subscriber<uint32_t, uint32_t> on_cc_;

    /** @brief Triggered on a timer break interrupt */
    Subscriber<> on_break_;

    STM32_Timer_t(TIM_TypeDef* instance,
            const STM32_GPIO_t** ch1p_gpios, const STM32_GPIO_t** ch2p_gpios, const STM32_GPIO_t** ch3p_gpios, const STM32_GPIO_t** ch4p_gpios,
            const STM32_GPIO_t** ch1n_gpios, const STM32_GPIO_t** ch2n_gpios, const STM32_GPIO_t** ch3n_gpios, const STM32_GPIO_t** ch4n_gpios,
            uint8_t gpio_af) :
        htim { .Instance = instance },
        p_gpios{ch1p_gpios, ch2p_gpios, ch3p_gpios, ch4p_gpios},
        n_gpios{ch1n_gpios, ch2n_gpios, ch3n_gpios, ch4n_gpios},
        gpio_af(gpio_af)
    {}

    bool setup(uint32_t period, MODE mode, uint32_t prescaler = 0, uint32_t repetition_counter = 0);
    bool setup_pwm(uint32_t channel,
            STM32_GPIO_t* gpio_p, STM32_GPIO_t* gpio_n,
            bool active_high_p, bool active_high_n,
            uint32_t initial_val);
    bool set_dead_time(uint32_t dead_time);
    bool setup_output_compare();
    bool config_encoder_mode(GPIO_t* gpio_ch1, GPIO_t* gpio_ch2) {
        STM32_GPIO_t* stm32_gpio_ch1 = dynamic_cast<STM32_GPIO_t*>(gpio_ch1);
        STM32_GPIO_t* stm32_gpio_ch2 = dynamic_cast<STM32_GPIO_t*>(gpio_ch2);
        if ((gpio_ch1 && !stm32_gpio_ch1) || (gpio_ch2 && !stm32_gpio_ch2)) {
            return false;
        } else {
            return config_encoder_mode(stm32_gpio_ch1, stm32_gpio_ch2);
        }
    }
    bool config_encoder_mode(STM32_GPIO_t* gpio_ch1, STM32_GPIO_t* gpio_ch2);
    bool config_input_compare_mode(STM32_GPIO_t* gpio_ch3, STM32_GPIO_t* gpio_ch4);

    bool start() {
        // TODO: there are separate "start" functions in the HAL for each mode
        return HAL_TIM_Base_Start_IT(&htim) == HAL_OK;
    }

    bool start_encoder() {
        return HAL_TIM_Encoder_Start(&htim, TIM_CHANNEL_ALL);
    }

    bool get_general_irqn(IRQn_Type* irqn);

    /** @brief Enables the update interrupt.
     * The on_update_ subscriber should be set first. */
    bool enable_update_interrupt();

    /** @brief Enables the trigger interrupt.
     * The on_trigger_ subscriber should be set first. */
    bool enable_trigger_interrupt();

    /** @brief Enables the cc interrupt.
     * The on_cc_ subscriber should be set first. */
    bool enable_cc_interrupt();

    /** @brief Enables the break interrupt.
     * The on_break_ subscriber should be set first. */
    bool enable_break_interrupt();

    void handle_update_irq();
    void handle_trigger_irq();
    void handle_cc_irq();
    void handle_break_irq();

    void handle_any_irq() {
        handle_update_irq();
        handle_trigger_irq();
        handle_cc_irq();
        handle_break_irq();
        HAL_TIM_IRQHandler(&htim); // TODO: not sure if this is still required
    }
};

extern STM32_Timer_t tim1, tim2, tim3, tim4, tim5, tim6, tim7, tim8, tim9, tim10, tim11, tim12, tim13, tim13, tim14;

#endif // __STM32_TIM_HPP