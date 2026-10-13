#include "board.hpp"

extern "C" {
#include "i2c.h"
#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include "usart.h"
}

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace h0_firmware {

namespace {

TIM_HandleTypeDef motor_1_encoder_timer{};
TIM_HandleTypeDef motor_2_encoder_timer{};

TIM_HandleTypeDef motor_1_pwm_timer{};
TIM_HandleTypeDef motor_2_rpwm_timer{};
TIM_HandleTypeDef motor_2_lpwm_timer{};

bool motor_pwm_initialized = false;

constexpr std::uint32_t pwm_frequency_hz = 20'000;
constexpr std::uint32_t pwm_period_counts = 100;
constexpr std::uint32_t drive_duty_percent = 20;
constexpr std::uint32_t drive_compare = pwm_period_counts * drive_duty_percent / 100;

std::uint32_t apb1_timer_clock_hz() {
    const std::uint32_t pclk = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) {
        return pclk * 2U;
    }

    return pclk;
}

std::uint32_t apb2_timer_clock_hz() {
    const std::uint32_t pclk = HAL_RCC_GetPCLK2Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) {
        return pclk * 2U;
    }

    return pclk;
}

bool initialize_pwm_timer(TIM_HandleTypeDef& timer, TIM_TypeDef* instance,
                          std::uint32_t timer_clock_hz) {
    const std::uint32_t denominator = pwm_frequency_hz * pwm_period_counts;
    const std::uint32_t prescaler_divisor = timer_clock_hz / denominator;

    if (prescaler_divisor == 0U) {
        return false;
    }

    timer.Instance = instance;
    timer.Init.Prescaler = prescaler_divisor - 1U;
    timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer.Init.Period = pwm_period_counts - 1U;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    return HAL_TIM_PWM_Init(&timer) == HAL_OK;
}

bool initialize_pwm_channel(TIM_HandleTypeDef& timer, std::uint32_t channel) {
    TIM_OC_InitTypeDef config{};

    config.OCMode = TIM_OCMODE_PWM1;
    config.Pulse = 0;
    config.OCPolarity = TIM_OCPOLARITY_HIGH;
    config.OCFastMode = TIM_OCFAST_DISABLE;

    return HAL_TIM_PWM_ConfigChannel(&timer, &config, channel) == HAL_OK;
}

constexpr std::uint16_t as5600_address = static_cast<std::uint16_t>(0x36U << 1U);

constexpr std::uint16_t as5600_raw_angle_register = 0x0CU;

constexpr std::uint32_t i2c_timeout_ms = 100;

constexpr std::size_t rx_buffer_size = 128;

std::array<std::uint8_t, rx_buffer_size> rx_buffer{};

volatile std::size_t rx_head = 0;
volatile std::size_t rx_tail = 0;

std::uint8_t rx_byte = 0;

constexpr std::size_t command_buffer_size = 128;

std::array<char, command_buffer_size> command_buffer{};
std::size_t command_length = 0;

bool discard_command_until_newline = false;

/*
 * TIM3 is only 16-bit.
 *
 * Extend its successive wrap-safe deltas into a signed
 * 32-bit software position.
 */
std::uint16_t motor_2_previous_hardware_count = 0;
std::int32_t motor_2_extended_count = 0;
bool motor_2_counter_initialized = false;

void initialize_motor_gpio() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*
     * EN pins remain ordinary GPIO outputs.
     *
     * Motor 1 EN = PB5
     * Motor 2 EN = PA10
     */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio{};

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_5;

    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio);

    /*
     * Motor 1:
     *
     * PA8 = TIM1_CH1 = RPWM
     * PA9 = TIM1_CH2 = LPWM
     */
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM1;

    HAL_GPIO_Init(GPIOA, &gpio);
    ;

    /*
     * Motor 2:
     *
     * PC7 = TIM8_CH2 = RPWM
     */
    gpio.Pin = GPIO_PIN_7;
    gpio.Alternate = GPIO_AF3_TIM8;

    HAL_GPIO_Init(GPIOC, &gpio);

    /*
     * Motor 2:
     *
     * PB6 = TIM4_CH1 = LPWM
     */
    gpio.Pin = GPIO_PIN_6;
    gpio.Alternate = GPIO_AF2_TIM4;

    HAL_GPIO_Init(GPIOB, &gpio);
}

void initialize_motor_pwm() {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    /*
     * Motor 1:
     *
     * TIM1_CH1 = RPWM
     * TIM1_CH2 = LPWM
     */
    if (!initialize_pwm_timer(motor_1_pwm_timer, TIM1, apb2_timer_clock_hz())) {
        Error_Handler();
    }

    if (!initialize_pwm_channel(motor_1_pwm_timer, TIM_CHANNEL_1) ||
        !initialize_pwm_channel(motor_1_pwm_timer, TIM_CHANNEL_2)) {
        Error_Handler();
    }

    /*
     * Motor 2 RPWM:
     *
     * TIM8_CH2 / PC7
     */
    if (!initialize_pwm_timer(motor_2_rpwm_timer, TIM8, apb2_timer_clock_hz()) ||
        !initialize_pwm_channel(motor_2_rpwm_timer, TIM_CHANNEL_2)) {
        Error_Handler();
    }

    /*
     * Motor 2 LPWM:
     *
     * TIM4_CH1 / PB6
     */
    if (!initialize_pwm_timer(motor_2_lpwm_timer, TIM4, apb1_timer_clock_hz()) ||
        !initialize_pwm_channel(motor_2_lpwm_timer, TIM_CHANNEL_1)) {
        Error_Handler();
    }

    /*
     * All PWM starts at zero duty
     */
    __HAL_TIM_SET_COMPARE(&motor_1_pwm_timer, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&motor_1_pwm_timer, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&motor_2_rpwm_timer, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&motor_2_lpwm_timer, TIM_CHANNEL_1, 0);

    if (HAL_TIM_PWM_Start(&motor_1_pwm_timer, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&motor_1_pwm_timer, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&motor_2_rpwm_timer, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&motor_2_lpwm_timer, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    motor_pwm_initialized = true;
}

void initialize_encoder_gpio() {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
     * Motor 1 encoder:
     *
     * PA0 = TIM2_CH1
     * PA1 = TIM2_CH2
     */
    GPIO_InitTypeDef gpio{};

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM2;

    HAL_GPIO_Init(GPIOA, &gpio);

    /*
     * Motor 2 encoder:
     *
     * PA6 = TIM3_CH1
     * PA7 = TIM3_CH2
     */
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;

    gpio.Alternate = GPIO_AF2_TIM3;

    HAL_GPIO_Init(GPIOA, &gpio);
}

void initialize_motor_1_encoder() {
    __HAL_RCC_TIM2_CLK_ENABLE();

    motor_1_encoder_timer.Instance = TIM2;

    motor_1_encoder_timer.Init.Prescaler = 0;
    motor_1_encoder_timer.Init.CounterMode = TIM_COUNTERMODE_UP;

    motor_1_encoder_timer.Init.Period = 0xFFFFFFFFU;

    motor_1_encoder_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    motor_1_encoder_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef config{};

    config.EncoderMode = TIM_ENCODERMODE_TI12;

    config.IC1Polarity = TIM_ICPOLARITY_RISING;

    config.IC1Selection = TIM_ICSELECTION_DIRECTTI;

    config.IC1Prescaler = TIM_ICPSC_DIV1;

    config.IC1Filter = 0;

    config.IC2Polarity = TIM_ICPOLARITY_RISING;

    config.IC2Selection = TIM_ICSELECTION_DIRECTTI;

    config.IC2Prescaler = TIM_ICPSC_DIV1;

    config.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(&motor_1_encoder_timer, &config) != HAL_OK) {
        force_disable_drivers();
        Error_Handler();
    }

    __HAL_TIM_SET_COUNTER(&motor_1_encoder_timer, 0);

    if (HAL_TIM_Encoder_Start(&motor_1_encoder_timer, TIM_CHANNEL_ALL) != HAL_OK) {
        force_disable_drivers();
        Error_Handler();
    }
}

void initialize_motor_2_encoder() {
    __HAL_RCC_TIM3_CLK_ENABLE();

    motor_2_encoder_timer.Instance = TIM3;

    motor_2_encoder_timer.Init.Prescaler = 0;
    motor_2_encoder_timer.Init.CounterMode = TIM_COUNTERMODE_UP;

    motor_2_encoder_timer.Init.Period = 0xFFFFU;

    motor_2_encoder_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    motor_2_encoder_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef config{};

    config.EncoderMode = TIM_ENCODERMODE_TI12;

    config.IC1Polarity = TIM_ICPOLARITY_RISING;

    config.IC1Selection = TIM_ICSELECTION_DIRECTTI;

    config.IC1Prescaler = TIM_ICPSC_DIV1;

    config.IC1Filter = 0;

    config.IC2Polarity = TIM_ICPOLARITY_RISING;

    config.IC2Selection = TIM_ICSELECTION_DIRECTTI;

    config.IC2Prescaler = TIM_ICPSC_DIV1;

    config.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(&motor_2_encoder_timer, &config) != HAL_OK) {
        force_disable_drivers();
        Error_Handler();
    }

    __HAL_TIM_SET_COUNTER(&motor_2_encoder_timer, 0);

    if (HAL_TIM_Encoder_Start(&motor_2_encoder_timer, TIM_CHANNEL_ALL) != HAL_OK) {
        force_disable_drivers();
        Error_Handler();
    }

    motor_2_previous_hardware_count = 0;
    motor_2_extended_count = 0;
    motor_2_counter_initialized = true;
}

bool read_as5600_raw(int& qraw) {
    std::array<std::uint8_t, 2> data{};

    const HAL_StatusTypeDef result =
        HAL_I2C_Mem_Read(&hi2c1, as5600_address, as5600_raw_angle_register, I2C_MEMADD_SIZE_8BIT,
                         data.data(), static_cast<std::uint16_t>(data.size()), i2c_timeout_ms);

    if (result != HAL_OK) {
        return false;
    }

    qraw = (static_cast<int>(data[0] & 0x0FU) << 8) | static_cast<int>(data[1]);

    return qraw >= 0 && qraw < 4096;
}

std::int32_t read_motor_1_encoder_count() {
    const std::uint32_t raw = __HAL_TIM_GET_COUNTER(&motor_1_encoder_timer);

    /*
     * Give the 32-bit timer a deterministic signed
     * interpretation without relying on an out-of-range
     * unsigned-to-signed cast.
     */
    if (raw <= 0x7FFFFFFFU) {
        return static_cast<std::int32_t>(raw);
    }

    return static_cast<std::int32_t>(static_cast<std::int64_t>(raw) - 0x100000000LL);
}

std::int32_t read_motor_2_encoder_count() {
    const auto hardware_count =
        static_cast<std::uint16_t>(__HAL_TIM_GET_COUNTER(&motor_2_encoder_timer));

    if (!motor_2_counter_initialized) {
        motor_2_previous_hardware_count = hardware_count;

        motor_2_extended_count = 0;

        motor_2_counter_initialized = true;

        return 0;
    }

    /*
     * Compute the shortest signed displacement between
     * successive 16-bit samples.
     */
    std::int32_t delta = static_cast<std::int32_t>(hardware_count) -
                         static_cast<std::int32_t>(motor_2_previous_hardware_count);

    if (delta > 32767) {
        delta -= 65536;
    } else if (delta < -32768) {
        delta += 65536;
    }

    motor_2_extended_count += delta;

    motor_2_previous_hardware_count = hardware_count;

    return motor_2_extended_count;
}

} // namespace

void start_command_receiver() {
    rx_head = 0;
    rx_tail = 0;

    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1) != HAL_OK) {
        force_disable_drivers();
        Error_Handler();
    }
}

void force_disable_drivers() {
    if (motor_pwm_initialized) {
        /*
         * PWM duty to zero FIRST.
         */
        __HAL_TIM_SET_COMPARE(&motor_1_pwm_timer, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&motor_1_pwm_timer, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&motor_2_rpwm_timer, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&motor_2_lpwm_timer, TIM_CHANNEL_1, 0);
    }

    /*
     * Then disable the two IBT-2s.
     */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

bool apply_drive_command(const atj::h0::MotorDriveCommand& command) {
    /*
     * Briefly remove authority while changing direction.
     */
    force_disable_drivers();

    const bool motor_1_active = command.motor_1 != atj::h0::MotorDriveDirection::off;
    const bool motor_2_active = command.motor_2 != atj::h0::MotorDriveDirection::off;

    /*
     * Motor 1 mapping confirmed during commissioning:
     *
     * WIND     -> PA9 / LPWM / TIM1_CH2
     * UNWIND   -> PA8 / RPWM / TIM1_CH1
     */
    if (command.motor_1 == atj::h0::MotorDriveDirection::wind) {
        __HAL_TIM_SET_COMPARE(&motor_1_pwm_timer, TIM_CHANNEL_2, drive_compare);
    } else if (command.motor_1 == atj::h0::MotorDriveDirection::unwind) {
        __HAL_TIM_SET_COMPARE(&motor_1_pwm_timer, TIM_CHANNEL_1, drive_compare);
    }

    /*
     * Motor 2:
     *
     * WIND     -> PC7 / RPWM / TIM8_CH2
     * UNWIND   -> PB6 / LPWM / TIM4_CH1
     */
    if (command.motor_2 == atj::h0::MotorDriveDirection::wind) {
        __HAL_TIM_SET_COMPARE(&motor_2_rpwm_timer, TIM_CHANNEL_2, drive_compare);
    } else if (command.motor_2 == atj::h0::MotorDriveDirection::unwind) {
        __HAL_TIM_SET_COMPARE(&motor_2_lpwm_timer, TIM_CHANNEL_1, drive_compare);
    }

    /*
     * Enable only after direction + duty have been set.
     */
    if (motor_1_active) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    }

    if (motor_2_active) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    }

    return true;
}

void initialize_board() {
    /*
     * EN pins are definitely LOW here.
     */
    initialize_motor_gpio();

    initialize_motor_pwm();

    force_disable_drivers();

    initialize_encoder_gpio();

    initialize_motor_1_encoder();
    initialize_motor_2_encoder();

    force_disable_drivers();
}

std::optional<atj::h0::RawHardwareSample> try_read_hardware_sample() {
    int qraw = 0;

    if (!read_as5600_raw(qraw)) {
        return std::nullopt;
    }

    const std::int32_t motor_1_count = read_motor_1_encoder_count();

    const std::int32_t motor_2_count = read_motor_2_encoder_count();

    const std::uint64_t time_us = static_cast<std::uint64_t>(HAL_GetTick()) * 1000ULL;

    return atj::h0::RawHardwareSample{.time_us = time_us,

                                      .qraw = qraw,

                                      .motor_1_raw_count = motor_1_count,

                                      .motor_2_raw_count = motor_2_count};
}

std::optional<std::string> try_read_command_line() {
    while (rx_tail != rx_head) {
        const std::uint8_t byte = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % rx_buffer_size;

        if (byte == '\r') {
            continue;
        }

        if (discard_command_until_newline) {
            if (byte == '\n') {
                discard_command_until_newline = false;
                command_length = 0;
            }

            continue;
        }

        if (byte == '\n') {
            if (command_length == 0) {
                continue;
            }

            std::string line(command_buffer.data(), command_length);

            line.push_back('\n');
            command_length = 0;
            return line;
        }

        if (command_length >= command_buffer.size() - 1) {
            command_length = 0;
            discard_command_until_newline = true;
            continue;
        }

        command_buffer[command_length] = static_cast<char>(byte);
        ++command_length;
    }

    return std::nullopt;
}

void write_serial_line(const std::string& line) {
    if (line.empty()) {
        return;
    }

    auto* bytes = reinterpret_cast<std::uint8_t*>(const_cast<char*>(line.data()));

    HAL_UART_Transmit(&huart2, bytes, static_cast<std::uint16_t>(line.size()), HAL_MAX_DELAY);
}

void wait_for_next_cycle() {
    /*
     * Observation-only dry-run rate.
     */
    HAL_Delay(8);
}

extern "C" void USART2_IRQHandler() {
    HAL_UART_IRQHandler(&huart2);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart != &huart2) {
        return;
    }

    const std::size_t next = (h0_firmware::rx_head + 1) % h0_firmware::rx_buffer_size;

    if (next != h0_firmware::rx_tail) {
        h0_firmware::rx_buffer[h0_firmware::rx_head] = h0_firmware::rx_byte;
        h0_firmware::rx_head = next;
    }

    HAL_UART_Receive_IT(&huart2, &h0_firmware::rx_byte, 1);
}

} // namespace h0_firmware
