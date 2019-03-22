

#include <stm32_system.h>
#include <stm32_adc.hpp>
#include <stm32_can.hpp>
#include <stm32_gpio.hpp>
#include <stm32_i2c.hpp>
#include <stm32_spi.hpp>
#include <stm32_usart.hpp>
#include <stm32_usb.hpp>

#include <usb_cdc.hpp>
#include <winusb_compat.hpp>

#include <devices.hpp>
#include <drv8301.hpp>

#include <odrive_main.h>

#include <freertos.hpp>

#include <inttypes.h>

struct PerChannelConfig_t {
    Motor::Config_t motor_config;
    Encoder::Config_t encoder_config;
    SensorlessEstimator::Config_t sensorless_estimator_config;
    Controller::Config_t controller_config;
    TrapezoidalTrajectory::Config_t trap_config;
    Axis::Config_t axis_config;
};


constexpr size_t AXIS_COUNT = 2; // sizeof(axes) / sizeof(axes[0]);
extern PerChannelConfig_t axis_configs[AXIS_COUNT];

const float thermistor_poly_coeffs[] =
    {363.93910201f, -462.15369634f, 307.55129571f, -27.72569531f};
const size_t thermistor_num_coeffs = sizeof(thermistor_poly_coeffs)/sizeof(thermistor_poly_coeffs[0]);

#if HW_VERSION_MAJOR == 3
#  if HW_VERSION_MINOR <= 3
#    define SHUNT_RESISTANCE (675e-6f)
#  else
#    define SHUNT_RESISTANCE (500e-6f)
#  endif
#else
#  error "unknown shunt resistance"
#endif

#if HW_VERSION_VOLTAGE >= 48
#  define VBUS_S_DIVIDER_RATIO 19.0f
#  define VBUS_OVERVOLTAGE_LEVEL 52.0f
#elif HW_VERSION_VOLTAGE == 24
#  define VBUS_S_DIVIDER_RATIO 11.0f
#  define VBUS_OVERVOLTAGE_LEVEL 26.0f
#else
#  error "unknown board voltage"
#endif

STM32_ADCChannel_t adc_vbus_sense = adc1_regular.get_channel(&pa6);
VoltageDivider_t vbus_sense(&adc_vbus_sense, VBUS_S_DIVIDER_RATIO);

STM32_GPIO_t* gpios[] = { &pa0, &pa1, &pa2, &pa3, &pc4, &pb2, &pa15, &pb3 };
const size_t num_gpios = sizeof(gpios) / sizeof(gpios[0]);

STM32_ADCChannel_t gpio_adcs[] = {
    adc1_regular.get_channel(gpios[0]),
    adc1_regular.get_channel(gpios[1]),
    adc1_regular.get_channel(gpios[2]),
    adc1_regular.get_channel(gpios[3]),
    adc1_regular.get_channel(gpios[4]),
    adc1_regular.get_channel(gpios[5]),
    adc1_regular.get_channel(gpios[6]),
    adc1_regular.get_channel(gpios[7]),
};

DRV8301_t gate_driver_m0(
    &spi3,
    &pc13, // chip select
    &pb12, // enable (shared across both motors)
    &pd2 // nFault (shared across both motors)
);

DRV8301_t gate_driver_m1(
    &spi3,
    &pc14, // chip select
    &pb12, // enable (shared across both motors)
    &pd2 // nFault (shared across both motors)
);


STM32_ADCChannel_t adc_m0_b = adc2_injected.get_channel(&pc0);
STM32_ADCChannel_t adc_m0_c = adc3_injected.get_channel(&pc1);
STM32_ADCChannel_t adc_m1_b = adc2_regular.get_channel(&pc3);
STM32_ADCChannel_t adc_m1_c = adc3_regular.get_channel(&pc2);
Shunt_t current_sensor_m0_b(&adc_m0_b, &gate_driver_m0, 1.0f / SHUNT_RESISTANCE);
Shunt_t current_sensor_m0_c(&adc_m0_c, &gate_driver_m0, 1.0f / SHUNT_RESISTANCE);
DerivedCurrentSensor_t<2> current_sensor_m0_a({&current_sensor_m0_b, &current_sensor_m0_c});
Shunt_t current_sensor_m1_b(&adc_m1_b, &gate_driver_m1, 1.0f / SHUNT_RESISTANCE);
Shunt_t current_sensor_m1_c(&adc_m1_c, &gate_driver_m1, 1.0f / SHUNT_RESISTANCE);
DerivedCurrentSensor_t<2> current_sensor_m1_a({&current_sensor_m1_b, &current_sensor_m1_c});

STM32_ADCChannel_t adc_m0_inv_temp = adc1_regular.get_channel(&pc5);
STM32_ADCChannel_t adc_m1_inv_temp = adc1_regular.get_channel(&pa4);
Thermistor_t temp_sensor_m0_inv(&adc_m0_inv_temp, thermistor_poly_coeffs, thermistor_num_coeffs);
Thermistor_t temp_sensor_m1_inv(&adc_m1_inv_temp, thermistor_poly_coeffs, thermistor_num_coeffs);

STM32_ADCChannel_t* adc_sincos_s = &gpio_adcs[2];
STM32_ADCChannel_t* adc_sincos_c = &gpio_adcs[3];

const size_t n_axes = AXIS_COUNT;

static bool board_init(Axis axes[AXIS_COUNT]) {
    new (&axes[0]) Axis(
            Motor(
                &tim1,
                &pb13, &pb14, &pb15, // pwm_{a,b,c}_l_gpio
                &pa8, &pa9, &pa10, // pwm_{a,b,c}_h_gpio
                &gate_driver_m0, // gate_driver_a
                &gate_driver_m0, // gate_driver_b
                &gate_driver_m0, // gate_driver_c
                &current_sensor_m0_a, // current_sensor_a
                &current_sensor_m0_b, // current_sensor_b
                &current_sensor_m0_c, // current_sensor_c
                &temp_sensor_m0_inv, // inverter_thermistor
                axis_configs[0].motor_config
            ),
            Encoder(
                &tim3, // counter
                &pc9, // index_gpio
                &pb4, // hallA_gpio
                &pb5, // hallB_gpio
                &pc9, // hallC_gpio (same as index pin)
                adc_sincos_s, // sincos_s (same for both encoders)
                adc_sincos_c, // sincos_c (same for both encoders)
                axis_configs[0].encoder_config
            ),
            SensorlessEstimator(axis_configs[0].sensorless_estimator_config),
            Controller(axis_configs[0].controller_config),
            TrapezoidalTrajectory(axis_configs[0].trap_config),
            (osPriority)(osPriorityHigh + (osPriority)1), // thread_priority
            axis_configs[0].axis_config
        );
    
    new (&axes[1]) Axis(
            Motor(
                &tim8,
                &pa7, &pb0, &pb1, // pwm_{a,b,c}_l_gpio
                &pc6, &pc7, &pc8, // pwm_{a,b,c}_h_gpio
                &gate_driver_m1, // gate_driver_a
                &gate_driver_m1, // gate_driver_b
                &gate_driver_m1, // gate_driver_c
                &current_sensor_m1_a, // current_sensor_a
                &current_sensor_m1_b, // current_sensor_b
                &current_sensor_m1_c, // current_sensor_c
                &temp_sensor_m1_inv, // inverter_thermistor
                axis_configs[1].motor_config
            ),
            Encoder(
                &tim4, // counter
                &pc15, // index_gpio
                &pb6, // hallA_gpio
                &pb7, // hallB_gpio
                &pc15, // hallC_gpio (same as index pin)
                adc_sincos_s, // sincos_s (same for both encoders)
                adc_sincos_c, // sincos_c (same for both encoders)
                axis_configs[1].encoder_config
            ),
            SensorlessEstimator(axis_configs[1].sensorless_estimator_config),
            Controller(axis_configs[1].controller_config),
            TrapezoidalTrajectory(axis_configs[1].trap_config),
            osPriorityHigh, // thread_priority
            axis_configs[1].axis_config
        );

    return true;
}

GPIO_t* i2c_a0_gpio = gpios[2];
GPIO_t* i2c_a1_gpio = gpios[3];
GPIO_t* i2c_a2_gpio = gpios[4];


#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 5
uint32_t default_step_gpio_nums[AXIS_COUNT] = { 1, 7 };
uint32_t default_dir_gpio_nums[AXIS_COUNT] = { 2, 8 };
#else
uint32_t default_step_gpio_nums[AXIS_COUNT] = { 1, 3 };
uint32_t default_dir_gpio_nums[AXIS_COUNT] = { 2, 4 };
#endif





STM32_USBTxEndpoint_t cdc_tx_endpoint(&usb.hUsbDeviceFS, 0x81);    /* EP1 for data IN */
STM32_USBRxEndpoint_t cdc_rx_endpoint(&usb.hUsbDeviceFS, 0x01);    /* EP1 for data OUT */
STM32_USBIntEndpoint_t cdc_cmd_endpoint(&usb.hUsbDeviceFS, 0x82);  /* EP2 for CDC commands */
STM32_USBTxEndpoint_t odrive_tx_endpoint(&usb.hUsbDeviceFS, 0x83); /* EP3 IN: ODrive device TX endpoint */
STM32_USBRxEndpoint_t odrive_rx_endpoint(&usb.hUsbDeviceFS, 0x03); /* EP3 OUT: ODrive device RX endpoint */


STM32_USART_t* comm_uart = &uart4;















uint8_t axes_obj_bufs[sizeof(Axis[AXIS_COUNT])];
Axis* axes = reinterpret_cast<Axis*>(axes_obj_bufs);




//#define __MAIN_CPP__
//#include "odrive_main.h"
#include "nvm_config.hpp"

#include "freertos_vars.h"
#include <communication/interface_usb.h>
#include <communication/interface_uart.h>
#include <communication/interface_i2c.h>

BoardConfig_t board_config;
PerChannelConfig_t axis_configs[AXIS_COUNT];
bool user_config_loaded_;

SystemStats_t system_stats_ = { 0 };
float vbus_voltage = 12.0f;


typedef Config<
    BoardConfig_t,
    PerChannelConfig_t[AXIS_COUNT]> ConfigFormat;


// @brief Returns the ADC voltage associated with the specified pin.
// GPIO_set_to_analog() must be called first to put the Pin into
// analog mode.
// Returns NaN if the pin has no associated ADC1 channel.
//
// On ODrive 3.3 and 3.4 the following pins can be used with this function:
//  GPIO_1, GPIO_2, GPIO_3, GPIO_4 and some pins that are connected to
//  on-board sensors (M0_TEMP, M1_TEMP, AUX_TEMP)
//
// The ADC values are sampled in background at ~30kHz without
// any CPU involvement.
//
// Details: each of the 16 conversion takes (15+26) ADC clock
// cycles and the ADC, so the update rate of the entire sequence is:
//  21000kHz / (15+26) / 16 = 32kHz
// The true frequency is slightly lower because of the injected vbus
// measurements
float get_adc_voltage(uint32_t gpio_num) {
    float result = 0.0f / 0.0f;
    if (gpio_num < sizeof(gpio_adcs) / sizeof(gpio_adcs[0])) {
        gpio_adcs[gpio_num].get_voltage(&result);
    }
    return result;
}

// TODO: make a BrakeResistor class
void start_brake_pwm() {
    // Start brake resistor PWM in floating output configuration
    tim2.htim.Instance->CCR3 = 0;
    tim2.htim.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    // Enable Channel3 P and Channel 4 P
    tim2.enable_pwm(false, false, false, false, true, false, true, false);
    safety_critical_arm_brake_resistor();
}


void save_configuration(void) {
    if (ConfigFormat::safe_store_config(
            &board_config,
            &axis_configs)) {
        //printf("saving configuration failed\r\n"); osDelay(5);
    } else {
        user_config_loaded_ = true;
    }
}

extern "C" int load_configuration(void) {
    // Try to load configs
    if (NVM_init() ||
        ConfigFormat::safe_load_config(
                &board_config,
                &axis_configs)) {
        //If loading failed, restore defaults
        board_config = BoardConfig_t();
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            axis_configs[i] = PerChannelConfig_t();

            // Default step/dir pins are different across hardware versions, so we need to explicitly load them
            axis_configs[i].axis_config.step_gpio_num = default_step_gpio_nums[i];
            axis_configs[i].axis_config.dir_gpio_num = default_dir_gpio_nums[i];
        }
    } else {
        user_config_loaded_ = true;
    }
    return user_config_loaded_;
}

void erase_configuration(void) {
    NVM_erase();
}

void enter_dfu_mode() {
    if ((hw_version_major == 3) && (hw_version_minor >= 5)) {
        __asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
        _reboot_cookie = 0xDEADBEEF;
        NVIC_SystemReset();
    } else {
        /*
        * DFU mode is only allowed on board version >= 3.5 because it can burn
        * the brake resistor FETs on older boards.
        * If you really want to use it on an older board, add 3.3k pull-down resistors
        * to the AUX_L and AUX_H signals and _only then_ uncomment these lines.
        */
        //__asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
        //_reboot_cookie = 0xDEADFE75;
        //NVIC_SystemReset();
    }
}

extern "C" {
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName) {
    for (;;); // TODO: safe action
}
void vApplicationIdleHook(void) {
    // TODO: reconsider how to get stack sizes
    if (system_stats_.fully_booted) {
        system_stats_.uptime = xTaskGetTickCount();
        system_stats_.min_heap_space = xPortGetMinimumEverFreeHeapSize();
        system_stats_.min_stack_space_comms = uxTaskGetStackHighWaterMark(comm_thread) * sizeof(StackType_t);
//        system_stats_.min_stack_space_axis0 = uxTaskGetStackHighWaterMark(axes[0].thread_id_) * sizeof(StackType_t);
//        system_stats_.min_stack_space_axis1 = uxTaskGetStackHighWaterMark(axes[1].thread_id_) * sizeof(StackType_t);
//        system_stats_.min_stack_space_usb = uxTaskGetStackHighWaterMark(usb_thread) * sizeof(StackType_t);
//        system_stats_.min_stack_space_uart = uxTaskGetStackHighWaterMark(uart_thread) * sizeof(StackType_t);
//        system_stats_.min_stack_space_usb_irq = uxTaskGetStackHighWaterMark(usb.irq_thread) * sizeof(StackType_t);
        system_stats_.min_stack_space_startup = uxTaskGetStackHighWaterMark(defaultTaskHandle) * sizeof(StackType_t);
    }
}
}

USBCompositeDevice composite_device;
USB_CDC_t cdc_class(&cdc_cmd_endpoint, &cdc_tx_endpoint, &cdc_rx_endpoint);
USB_ODriveNativeClass_t odrive_class(&odrive_tx_endpoint, &odrive_rx_endpoint);

int main_task(void);

int main(void) {
    if (!system_init()) {
        for (;;); // TODO: define error action
    }

    // Load persistent configuration (or defaults)
    load_configuration();

    // Load Axis objects
    board_init(reinterpret_cast<Axis*>(axes_obj_bufs));

    // Init timer for FreeRTOS scheduler

    /* Init FreeRTOS resources (in freertos.cpp) */
    freertos_init(&tim14, &main_task);

    /* Start scheduler */
    osKernelStart();

    // We should never get here as control is now taken by the scheduler
    // and all further initialization happens on main_task().
    for (;;);
}

uint64_t serial_number = 0;
int main_task(void) {
    // This procedure of building a USB serial number should be identical
    // to the way the STM's built-in USB bootloader does it. This means
    // that the device will have the same serial number in normal and DFU mode.
    uint32_t uuid0 = *(uint32_t *)(UID_BASE + 0);
    uint32_t uuid1 = *(uint32_t *)(UID_BASE + 4);
    uint32_t uuid2 = *(uint32_t *)(UID_BASE + 8);
    uint32_t uuid_mixed_part = uuid0 + uuid2;
    serial_number = ((uint64_t)uuid_mixed_part << 16) | (uint64_t)(uuid1 >> 16);

    char serial_number_str[13]; // 12 digits + null termination
    uint64_t val = serial_number;
    for (size_t i = 0; i < 12; ++i) {
        serial_number_str[i] = "0123456789ABCDEF"[(val >> (48-4)) & 0xf];
        val <<= 4;
    }
    serial_number_str[12] = 0;

    char product_str[64];
    sprintf(product_str, "ODrive %d.%d CDC Interface", HW_VERSION_MAJOR, HW_VERSION_MINOR);

    char native_interface_str[64];
    sprintf(native_interface_str, "ODrive %d.%d Native Interface", HW_VERSION_MAJOR, HW_VERSION_MINOR);


    /* Setup Communication I/O -----------------------------------------------*/

    // Set up USB device
    if (!composite_device.register_class(&cdc_class))
        goto fail;
    if (!composite_device.register_class(&odrive_class))
        goto fail;
    if (!usb.add_vendor_request_handler(&ms_request_handler)) // zero-config WinUSB support
        goto fail;
    if (!usb.add_int_endpoint(&cdc_cmd_endpoint))
        goto fail;
    if (!usb.add_rx_endpoint(&cdc_rx_endpoint))
        goto fail;
    if (!usb.add_tx_endpoint(&cdc_tx_endpoint))
        goto fail;
    if (!usb.add_rx_endpoint(&odrive_rx_endpoint))
        goto fail;
    if (!usb.add_tx_endpoint(&odrive_tx_endpoint))
        goto fail;
    if (!usb.setup(
        0x1209, 0x0D32, 1033, // VID, PID, LangID
        "ODrive Robotics", product_str, serial_number_str, "CDC Config", "CDC Interface", native_interface_str,
        &composite_device
    ))
        goto fail;

#if 0
    // M0/M1 PWM input
    tim5.setup(
        0xFFFFFFFF, // period
        STM32_Timer_t::UP
    );
    tim5.config_input_compare_mode(&pa2, &pa3);
    tim5.enable_cc_interrupt(pwm_in_cb, nullptr);

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    if (board_config.enable_i2c_instead_of_can) {
        // TODO: make
        if (i2c_a0_gpio)
            i2c_a0_gpio->setup(GPIO_t::INPUT, GPIO_t::PULL_UP);
        if (i2c_a1_gpio)
            i2c_a1_gpio->setup(GPIO_t::INPUT, GPIO_t::PULL_UP);
        if (i2c_a2_gpio)
            i2c_a2_gpio->setup(GPIO_t::INPUT, GPIO_t::PULL_UP);

        osDelay(1);
        i2c_stats_.addr = (0xD << 3);
        i2c_stats_.addr |= i2c_a0_gpio->read() ? 0x1 : 0;
        i2c_stats_.addr |= i2c_a1_gpio->read() ? 0x2 : 0;
        i2c_stats_.addr |= i2c_a2_gpio->read() ? 0x4 : 0;
        i2c1.setup_as_slave(100000, i2c_stats_.addr, &pb8, &pb9, &dma1_stream6, &dma1_stream0); // TODO: DMA stream conflict with SPI3?
    } else
#endif
    {
        can1.setup(&pb8, &pb9);
    }

    // Init general user ADC on some GPIOs.
    for (size_t i = 0; i < sizeof(gpios) / sizeof(gpios[0]); ++i) {
        // todo: only set to analog if supported
        gpios[i]->setup_analog();
    }
/*
    // Construct all objects.
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        Encoder *encoder = new Encoder(encoder_configs[i]);
        SensorlessEstimator *sensorless_estimator = new SensorlessEstimator(sensorless_configs[i]);
        Controller *controller = new Controller(controller_configs[i]);
        Motor *motor = new Motor(hw_configs[i].motor_config,
                                 motor_configs[i]);
        TrapezoidalTrajectory *trap = new TrapezoidalTrajectory(trap_configs[i]);
        axes[i] = new Axis(hw_configs[i].axis_config, axis_configs[i],
                *encoder, *sensorless_estimator, *controller, *motor, *trap);
    }*/
#endif

    // TODO: DMA is not really used by the DRV8301 driver, remove stream
    if (!spi3.setup(&pc10, &pc11, &pc12, &dma1_stream5, &dma1_stream0)) {
        goto fail;
    }

    // Diagnostics timer
    //tim13.setup(
    //    (2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR+1)) * ((float)TIM_APB1_CLOCK_HZ / (float)TIM_1_8_CLOCK_HZ) - 1, // period
    //    STM32_Timer_t::UP
    //);
    //tim13.enable_update_interrupt(); // todo: this was probably not used


    /* Set up ADC ------------------------------------------------------------*/

    if (!vbus_sense.on_update_.set<void>([](void*) {
        vbus_sense.get_voltage(&vbus_voltage);
        //if (axes[0] && !axes[0]->error_ && axes[1] && !axes[1]->error_) {
        //    if (oscilloscope_pos >= OSCILLOSCOPE_SIZE)
        //        oscilloscope_pos = 0;
        //    oscilloscope[oscilloscope_pos++] = vbus_voltage;
        //}
    }, nullptr)) {
        return false;
    }


    // AUX PWM
    tim2.setup(
        TIM_APB1_PERIOD_CLOCKS, // period
        STM32_Timer_t::UP_DOWN
    );
    tim2.setup_pwm(3,
            &pb10, nullptr,
            true, true, // active high
            0 // initial value
    ); // AUX L
    tim2.setup_pwm(4,
            &pb11, nullptr,
            true, true, // active high
            TIM_APB1_PERIOD_CLOCKS + 1 // initial value
    ); // AUX H

    // TODO: find a better place to init the ADC sequence
    if (!adc1_injected.setup(nullptr) ||
        !adc2_injected.setup(nullptr) ||
        !adc3_injected.setup(nullptr) ||
        !adc1_regular.setup(&dma2_stream0) ||
        !adc2_regular.setup(&dma2_stream2) ||
        !adc3_regular.setup(&dma2_stream1)) {
        goto fail;
    }

    if (!adc1_injected.set_trigger(&tim1) ||
        !adc2_injected.set_trigger(&tim1) ||
        !adc3_injected.set_trigger(&tim1) ||
        !adc1_regular.set_trigger(&tim8) ||
        !adc2_regular.set_trigger(&tim8) ||
        !adc3_regular.set_trigger(&tim8)) {
        goto fail;
    }

    if (!adc2_injected.append(&adc_m0_b) ||
        !adc3_injected.append(&adc_m0_c) ||
        !adc2_regular.append(&adc_m1_b) ||
        !adc3_regular.append(&adc_m1_c) ||
        !adc1_regular.append(&adc_vbus_sense)) {
        goto fail;
    }

    for (size_t i = 0; i < sizeof(gpio_adcs) / sizeof(gpio_adcs[0]); ++i) {
        if (!adc1_regular.append(&gpio_adcs[i])) {
            goto fail;
        }
    }

    // TODO: set up the remaining channels

    if (!adc1_injected.apply() ||
        !adc2_injected.apply() ||
        !adc3_injected.apply() ||
        !adc1_regular.apply() ||
        !adc2_regular.apply() ||
        !adc3_regular.apply()) {
        goto fail;
    }
    
    // Start ADC for temperature measurements and user measurements
    if (!adc1_regular.enable() ||
        !adc2_regular.enable() ||
        !adc3_regular.enable() ||
        !adc1_injected.enable() ||
        !adc2_injected.enable() ||
        !adc3_injected.enable()) {
        goto fail;
    }

    // TODO: make dynamically reconfigurable
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    if (board_config.enable_uart) {
        if (!uart4.setup(115200, gpios[0], gpios[1], &dma1_stream4, &dma1_stream2)) // Provisionally this can be changed to 921600 for faster transfers, the low power Arduinos will not keep up. 
            goto fail;
    }
#endif

#if 0
    // Start pwm-in compare modules
    // must happen after communication is initialized
    pwm_in_init();
#endif

    // Setup hardware for all components
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!axes[i].setup()) {
            goto fail;
        }
    }

    //osDelay(100);
    // Init communications (this requires the axis objects to be constructed and the config_ members to be non-null)
    init_communication();


    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        axes[i].motor_.start_updates();
    }
    // Synchronize PWM of both motors up to a 90° PWM phase shift
    sync_timers(axes[0].motor_.timer_, axes[1].motor_.timer_, TIM_CLOCKSOURCE_ITR0, TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128, nullptr);

    // Start brake resistor PWM
    start_brake_pwm();

#if 0
    // This delay serves two purposes:
    //  - Let the current sense calibration converge (the current
    //    sense interrupts are firing in background by now)
    //  - Allow a user to interrupt the code, e.g. by flashing a new code,
    //    before it does anything crazy
    // TODO make timing a function of calibration filter tau
    osDelay(1500);

    // Start state machine threads. Each thread will go through various calibration
    // procedures and then run the actual controller loops.
    // TODO: generalize for AXIS_COUNT != 2
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        axes[i].start_thread();
    }

    start_analog_thread();

#endif


    system_stats_.fully_booted = true;
    for (;;) {
        //uart4.start_tx((const uint8_t*)"a\r\n", 3, nullptr, nullptr);
        printf("hi\r\n");
//        //printf("uptime: %" PRIu32 "\r\n", system_stats_.uptime);
//        uint32_t prev_ints = all_usb_ints;
//        all_usb_ints = 0;
//        printf("USB ints: %08" PRIx32 "\r\n", prev_ints);
//        //printf("uptime: %" PRIu32 "\r\n", system_stats_.uptime);
        osDelay(100);
    }
    return 0;
    goto fail;
fail:
    //init_communication();
    for (;;) {
        printf("fail\r\n");
        osDelay(1000);
    }
    return -1;
}
