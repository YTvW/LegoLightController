/**
 * @file pwm_driver.h
 * @brief TC-based PWM driver for SAMD11C / SAMD21
 *
 * Uses two TC hardware peripherals for efficient PWM generation,
 * leaving TCC0 free for the WS2812B DMA driver. The chip is selected
 * automatically at compile time.
 *
 * Channel to pin mapping on the SAMD11C14A (TC1/TC2):
 *   - Channel 0: PA04 (TC1/WO[0]) or PA14
 *   - Channel 1: PA05 (TC1/WO[1]) or PA15
 *   - Channel 2: PA30 (TC2/WO[0]) - SWCLK programming pin!
 *   - Channel 3: PA31 (TC2/WO[1]) - SWDIO programming pin!
 *
 * PA14/PA15 are normally taken by the WS2812B strips, and channels 2/3
 * sacrifice the SWD debug port, so PA04 and PA05 are the practical choices.
 *
 * Channel to pin mapping on the SAMD21 (TC3/TC4):
 *   - Channel 0: PA14 (TC3/WO[0]) or PA18
 *   - Channel 1: PA15 (TC3/WO[1]) or PA19
 *   - Channel 2: PA22 (TC4/WO[0]) or PB08 (G/J parts)
 *   - Channel 3: PA23 (TC4/WO[1]) or PB09 (G/J parts)
 *
 * Note: pwm_deinit() on the SAMD21 also gates the GCLK channels shared
 * with TCC2 and TC5 (which analogWrite may be using on their pins).
 *
 * TC counters run in 8-bit mode, so the PWM frequency range is
 * ~184Hz to ~188kHz (out-of-range requests are clamped).
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include <Arduino.h>
#include <sam.h>

#ifdef __cplusplus
extern "C" {
#endif

// PWM configuration
#define PWM_RESOLUTION_BITS   8
#define PWM_MAX_VALUE         ((1 << PWM_RESOLUTION_BITS) - 1)  // 255 for 8-bit
#define PWM_DEFAULT_FREQ_HZ   1000

// Channel indices
#define PWM_CHANNEL_0   0
#define PWM_CHANNEL_1   1
#define PWM_CHANNEL_2   2
#define PWM_CHANNEL_3   3
#define PWM_NUM_CHANNELS 4

/**
 * @brief PWM channel configuration structure
 */
typedef struct {
    uint8_t pin;           // Arduino pin number
    uint8_t portPin;       // Port pin (PA number)
    uint8_t wo;            // Waveform output number (0-3 or 4-7 for inverted)
    uint8_t muxFunction;   // PMUX function (E or F typically for TCC)
} pwm_channel_config_t;

/**
 * @brief Initialize the TC1/TC2 PWM driver
 *
 * @param frequencyHz PWM frequency in Hz (e.g., 1000 for 1kHz)
 * @return true if initialization successful, false otherwise
 */
bool pwm_init(uint32_t frequencyHz);

/**
 * @brief Configure a specific pin for PWM output
 * 
 * @param channel Channel index (0-3)
 * @param pin Arduino pin number
 * @return true if configuration successful, false if pin not supported
 */
bool pwm_configure_channel(uint8_t channel, uint8_t pin);

/**
 * @brief Set the duty cycle for a PWM channel
 * 
 * @param channel Channel index (0-3)
 * @param duty Duty cycle (0 to PWM_MAX_VALUE, i.e., 0-255 for 8-bit)
 */
void pwm_set_duty(uint8_t channel, uint16_t duty);

/**
 * @brief Set duty cycle as a percentage
 * 
 * @param channel Channel index (0-3)
 * @param percent Duty cycle percentage (0.0 to 100.0)
 */
void pwm_set_duty_percent(uint8_t channel, float percent);

/**
 * @brief Enable PWM output on a channel
 * 
 * @param channel Channel index (0-3)
 */
void pwm_enable_channel(uint8_t channel);

/**
 * @brief Disable PWM output on a channel (output goes low)
 * 
 * @param channel Channel index (0-3)
 */
void pwm_disable_channel(uint8_t channel);

/**
 * @brief Get current duty cycle value for a channel
 * 
 * @param channel Channel index (0-3)
 * @return Current duty cycle value
 */
uint16_t pwm_get_duty(uint8_t channel);

/**
 * @brief Set PWM frequency (affects all channels)
 * 
 * @param frequencyHz New frequency in Hz
 */
void pwm_set_frequency(uint32_t frequencyHz);

/**
 * @brief Deinitialize the PWM driver and release resources
 */
void pwm_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // PWM_DRIVER_H
