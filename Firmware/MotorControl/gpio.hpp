#ifndef __GPIO_HPP
#define __GPIO_HPP

class GPIO_t {
public:
    virtual ~GPIO_t() = default;

    enum MODE {
        INPUT,
        OUTPUT,
        OPEN_DRAIN,
    };
    enum PULL {
        PULL_UP,
        PULL_DOWN,
        NO_PULL
    };
    enum SPEED {
        SLOW,       // up to 2 MHz
        MEDIUM,     // 12.5 to 50 MHz
        FAST,       // 25 to 100 MHz
        VERY_FAST   // 50 to 200 MHz
    };

    /*
    * @brief Initializes the ADC hardware.
    * Must be callable multiple times.
    */
    bool setup(MODE mode, PULL pull, bool state = false);

    /* @brief Sets the state of the GPIO */
    void write(bool value);

    /* @brief Reads the current state of the GPIO */
    bool read();
};

#endif // __GPIO_HPP