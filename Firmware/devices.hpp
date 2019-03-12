#ifndef __DEVICES_HPP
#define __DEVICES_HPP

struct ADCChannel_t {
    /*
    * @brief Initializes the ADC hardware.
    * Must be callable multiple times.
    */
    virtual bool setup() = 0;

    /*
    * @brief Fetches the latest reading in volts.
    * Returns true if the value could be obtained or false otherwise.
    */
    virtual bool get_voltage(float *value) = 0;

    /*
    * @brief Fetches the latest reading as a number in {0...1}.
    * Returns true if the value could be obtained or false otherwise.
    */
    virtual bool get_normalized(float *value) = 0;

    /*
    * @brief Subscribes to updates on this ADC channel.
    * This also enables updates. The underlying ADC must be configured
    * beforehand, i.e. with a timer as trigger or in continuous mode.
    */
    virtual bool subscribe(void (*callback)(void*), void* ctx) = 0;
};

class VoltageDivider_t : public ADCChannel_t {
public:
    ADCChannel_t* adc;
    float divider_ratio_;

    void (*callback_)(void*);
    void* ctx_;

    VoltageDivider_t(ADCChannel_t* adc, float divider_ratio) :
        divider_ratio_(divider_ratio) {}

    bool setup() final {
        return adc ? adc->setup() : false;
    }

    bool get_voltage(float *value) final {
        if (adc) {
            if (!adc->get_voltage(value))
                return false;
            if (!value)
                return false;
            *value *= divider_ratio_;
            return true;
        } else {
            return false;
        }
    }

    bool get_normalized(float *value) final {
        return adc ? adc->get_normalized(value) : false;
    }

    bool subscribe(void (*callback)(void*), void* ctx) {
        callback_ = nullptr;
        ctx_ = ctx;
        callback_ = callback;
        if (!adc) {
            return false;
        } else {
            adc->subscribe([](void* ctx){ if (ctx) ((VoltageDivider_t*)ctx)->handle_update(); }, this);
            return true;
        }
    }
private:
    void handle_update() {
        if (callback_) {
            callback_(ctx_);
        }
    }
};

struct SPI_t {
    /*
    * @brief Initializes the SPI peripheral.
    * Must be callable multiple times.
    */
    bool setup();
};

struct GateDriver_t {
    /*
    * @brief Initializes the gate driver hardware.
    * Must be callable multiple times.
    */
    bool setup();

    /*
    * @brief Checks for a fault condition. Returns false if the driver is in a
    * fault state and true if it is in an nominal state.
    */
    bool check_fault();
};


struct CurrentSensor_t {
    /*
    * @brief Initializes the current sensor hardware.
    * Must be callable multiple times.
    */
    bool setup(float requested_range);

    /*
    * @brief Tries to set the range of the current sensor to at least
    * {-requested_range, ..., requested_range}.
    * Returns the actual range that was set (may be smaller or larger than the 
    * requested range).
    */
    float set_range(float requested_range);

    /* @brief Returns the current current range of the sensor */
    float get_range();
};

struct OpAmp_t {
    /*
    * @brief Initializes the opamp hardware.
    * Must be callable multiple times.
    */
    bool setup();

    /*
    * @brief Tries to set the OpAmp gain to the specified value or lower.
    * Returns the actual gain that was set.
    */
    float set_gain(float max_gain);

    /*
    * @brief Returns the current gain setting
    */
    float get_gain();

    float get_max_output_swing();
};

class Shunt_t : public CurrentSensor_t {
public:
    ADCChannel_t* adc;
    OpAmp_t* opamp;
    float conductance;
    bool is_setup_ = false;

    void (*callback_)(void*);
    void* ctx_;

    Shunt_t(ADCChannel_t* adc, OpAmp_t* opamp, float conductance) :
        adc(adc), opamp(opamp), conductance(conductance) {}

    bool setup(float requested_range) {
        if (!is_setup_) {
            if (!adc->setup())
                return false;
            if (!opamp->setup())
                return false;
            set_range(requested_range);
            is_setup_ = true;
        }
    }

    float set_range(float requested_range) {
        const float max_output_swing = opamp->get_max_output_swing(); // [V] out of amplifier TODO: respect ADC max input range
        float max_unity_gain_current = max_output_swing * conductance; // [A]
        float requested_gain = max_unity_gain_current / requested_range; // [V/V]

        // set gain
        float actual_gain = opamp->set_gain(requested_gain);

        rev_gain_ = 1.0f / actual_gain;
        float actual_max_current = max_unity_gain_current * rev_gain_;
        return actual_max_current;
    }

    bool subscribe(void (*callback)(void*), void* ctx) {
        callback_ = nullptr;
        ctx_ = ctx;
        callback_ = callback;
        if (!adc) {
            return false;
        } else {
            adc->subscribe([](void* ctx){ if (ctx) ((Shunt_t*)ctx)->handle_update(); }, this);
            return true;
        }
    }

private:
    float rev_gain_;

    void handle_update() {
        // TODO: set new current
        if (callback_) {
            callback_(ctx_);
        }
    }

    //// Communication protocol definitions
    //auto make_protocol_definitions() {
    //    return make_protocol_member_list(
    //        make_protocol_property("rev_gain", &rev_gain_)
    //    );
    //}
};

#include <array>

template<size_t N>
class DerivedCurrentSensor_t : public CurrentSensor_t {
public:
    std::array<CurrentSensor_t*, N> other_current_sensors;

    DerivedCurrentSensor_t(std::array<CurrentSensor_t*, N> other_current_sensors) :
        other_current_sensors(other_current_sensors) {}

    float get_current() {
        float current = 0;
        for (size_t i = 0; i < N; ++i) {
            current -= other_current_sensors[i].get_current();
        }
        return current;
    }
};


#endif // __DEVICES_HPP