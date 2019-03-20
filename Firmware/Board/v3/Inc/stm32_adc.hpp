#ifndef __STM32_ADC_HPP
#define __STM32_ADC_HPP

#include <stm32_gpio.hpp>
#include <stm32_tim.hpp>
#include <devices.hpp>

#include <array>

class STM32_ADCSequence_t;

class STM32_ADCChannel_t : public ADCChannel_t {
public:
    STM32_ADCSequence_t* adc;
    STM32_GPIO_t* gpio; // may be NULL
    uint32_t channel_num;

    void (*callback_)(void*) = nullptr;
    void* ctx_ = nullptr;

    STM32_ADCChannel_t(STM32_ADCSequence_t* adc, STM32_GPIO_t* gpio, uint32_t channel_num) :
        adc(adc),
        gpio(gpio),
        channel_num(channel_num) {}

    bool setup() final { return true; } // currently unused
    bool get_voltage(float *value) final;
    bool get_normalized(float *value) final;
    bool enable_updates() final;

    bool is_valid() {
        return adc && channel_num < 19;
    }
    static STM32_ADCChannel_t invalid_channel() {
        return STM32_ADCChannel_t(nullptr, nullptr, UINT32_MAX);
    }

    void handle_update() {
        on_update_.invoke();
    }
};


class STM32_ADC_t {
public:
    ADC_HandleTypeDef hadc;
    DMA_HandleTypeDef hdma_rx;
    DMA_HandleTypeDef hdma_tx;

    std::array<STM32_GPIO_t*, 16> gpios;

    STM32_ADC_t(ADC_TypeDef* instance, std::array<STM32_GPIO_t*, 16> gpios) :
            hadc{ .Instance = instance },
            gpios(gpios) { }

    bool setup();
};

class STM32_ADCSequence_t {
public:
    STM32_ADC_t* adc;

    STM32_ADCSequence_t(STM32_ADC_t* adc) : adc(adc) {}

    virtual bool set_trigger(STM32_Timer_t* timer) = 0;
    virtual bool append(STM32_ADCChannel_t* channel) = 0;
    virtual bool apply() = 0;
    virtual bool get_raw_value(size_t channel_num, uint16_t *raw_value) = 0;

    STM32_ADCChannel_t get_channel(STM32_GPIO_t* gpio) {
        if (adc) {
            for (size_t i = 0; i < adc->gpios.size(); ++i) {
                if (adc->gpios[i] == gpio) {
                    return STM32_ADCChannel_t(this, gpio, i);
                }
            }
        }

        return STM32_ADCChannel_t::invalid_channel();
    }

    STM32_ADCChannel_t get_internal_temp_channel() {
        if (adc) {
            if (adc->hadc.Instance == ADC1) {
#if defined(STM32F405xx) || defined(stm32f415xx) || defined(STM32F407xx) || defined(stm32f417xx)
                return STM32_ADCChannel_t(this, nullptr, 16); // actually applies to all STM32F40x and STM32F41x but there's no macro for that
#elif defined(STM32F425xx) || defined(STM32F435xx)
                return STM32_ADCChannel_t(this, nullptr, 18); // actually applies to all STM32F42x and STM32F43x but there's no macro for that
#else
#  error "unknown temp channel"
#endif
            }
        }
        return STM32_ADCChannel_t::invalid_channel();
    }

    STM32_ADCChannel_t get_vrefint_channel() {
        if (adc) {
            if (adc->hadc.Instance == ADC1) {
                return STM32_ADCChannel_t(this, nullptr, 17);
            }
        }
        return STM32_ADCChannel_t::invalid_channel();
    }

    STM32_ADCChannel_t get_vbat_channel() {
        if (adc) {
            if (adc->hadc.Instance == ADC1) {
                return STM32_ADCChannel_t(this, nullptr, 18);
            }
        }
        return STM32_ADCChannel_t::invalid_channel();
    }

    bool enable();
    virtual void handle_irq() = 0;
};

template<unsigned int MAX_SEQ_LENGTH>
class STM32_ADCSequence_N_t : public STM32_ADCSequence_t {
public:
    STM32_ADCChannel_t* channel_sequence[MAX_SEQ_LENGTH];
    uint32_t channel_sequence_length = 0;
    std::array<STM32_ADCChannel_t*, MAX_SEQ_LENGTH> sequence = { nullptr };
    uint16_t raw_values[MAX_SEQ_LENGTH];

    STM32_ADCSequence_N_t(STM32_ADC_t* adc) : STM32_ADCSequence_t(adc) {}

    bool append(STM32_ADCChannel_t* channel) final {
        if (!channel || !channel->is_valid())
            return false;
        if (channel_sequence_length >= MAX_SEQ_LENGTH)
            return false;
        channel_sequence[channel_sequence_length++] = channel;
        return true;
    }

    bool get_raw_value(size_t channel_num, uint16_t *raw_value) final {
        if (channel_num < channel_sequence_length) {
            if (raw_value) {
                *raw_value = raw_values[channel_num];
            }
            return true;
        } else {
            return false;
        }
    }
};


class STM32_ADCRegular_t : public STM32_ADCSequence_N_t<16> {
public:
    uint32_t trigger_source = ADC_SOFTWARE_START;
    uint32_t next_pos = 0; // TODO: ensure that this is properly synced to the ADC

    STM32_ADCRegular_t(STM32_ADC_t* adc) : STM32_ADCSequence_N_t(adc) {}

    bool set_trigger(STM32_Timer_t* timer) final;
    bool apply() final;
    void handle_irq() final;
};

class STM32_ADCInjected_t : public STM32_ADCSequence_N_t<4> {
public:
    uint32_t trigger_source = ADC_INJECTED_SOFTWARE_START;

    STM32_ADCInjected_t(STM32_ADC_t* adc) : STM32_ADCSequence_N_t(adc) {}

    bool set_trigger(STM32_Timer_t* timer) final;
    bool apply() final;
    void handle_irq() final;
};


extern STM32_ADCRegular_t adc1_regular, adc2_regular, adc3_regular;
extern STM32_ADCInjected_t adc1_injected, adc2_injected, adc3_injected;

#endif // __STM32_ADC_HPP