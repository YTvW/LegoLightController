/**
 * @file pwm_driver.c
 * @brief TC-based PWM driver implementation for SAMD11C
 *
 * Uses TC1 and TC2 in 8-bit PWM mode, leaving TCC0 free for the
 * WS2812B DMA driver.
 */

#include "pwm_driver.h"

// SAMD11C14A TC Pin Mapping (PMUX Function E = 0x04)
// PA04 -> TC1/WO[0] (channel 0)
// PA05 -> TC1/WO[1] (channel 1)
// PA14 -> TC1/WO[0] (channel 0, alt - normally used by WS2812B strip 2)
// PA15 -> TC1/WO[1] (channel 1, alt - normally used by WS2812B strip 1)
// PA30 -> TC2/WO[0] (channel 2, WARNING: SWCLK programming pin)
// PA31 -> TC2/WO[1] (channel 3, WARNING: SWDIO programming pin)
//
// Note: PA08/PA09 have no TC waveform outputs on the SAMD11, only TCC0.

// Pin mapping table - maps Arduino pins to TC waveform outputs
// This may need adjustment based on your specific board variant
static const struct
{
  uint8_t arduinoPin;
  uint8_t portPin; // PA pin number
  uint8_t channel; // PWM channel (0-1 = TC1 CC0/CC1, 2-3 = TC2 CC0/CC1)
  uint8_t mux;     // PMUX function
} pinMap[] = {
    {4, 4, 0, PORT_PMUX_PMUXE_E_Val},   // Arduino pin 4 = PA04 -> TC1/WO[0]
    {5, 5, 1, PORT_PMUX_PMUXE_E_Val},   // Arduino pin 5 = PA05 -> TC1/WO[1]
    {14, 14, 0, PORT_PMUX_PMUXE_E_Val}, // Arduino pin 14 = PA14 -> TC1/WO[0] (alt)
    {15, 15, 1, PORT_PMUX_PMUXE_E_Val}, // Arduino pin 15 = PA15 -> TC1/WO[1] (alt)
    {30, 30, 2, PORT_PMUX_PMUXE_E_Val}, // Arduino pin 30 = PA30 -> TC2/WO[0] (SWCLK!)
    {31, 31, 3, PORT_PMUX_PMUXE_E_Val}, // Arduino pin 31 = PA31 -> TC2/WO[1] (SWDIO!)
};
#define PIN_MAP_SIZE (sizeof(pinMap) / sizeof(pinMap[0]))

// Internal state
static bool pwmInitialized = false;
static uint16_t channelDuty[PWM_NUM_CHANNELS] = {0};
static uint8_t channelEnabled = 0; // Bitmask of enabled channels
static uint32_t currentPeriod = 0;
static uint8_t currentPrescaler = TC_CTRLA_PRESCALER_DIV1_Val;

/**
 * @brief Get the TC instance serving a channel (0-1 = TC1, 2-3 = TC2)
 */
static inline Tc *tcForChannel(uint8_t channel)
{
  return (channel < 2) ? TC1 : TC2;
}

/**
 * @brief Get the CC register index for a channel within its TC
 */
static inline uint8_t ccIndexForChannel(uint8_t channel)
{
  return channel & 1;
}

/**
 * @brief Find pin configuration in the mapping table
 */
static int findPinConfig(uint8_t pin)
{
  for (size_t i = 0; i < PIN_MAP_SIZE; i++)
  {
    if (pinMap[i].arduinoPin == pin)
    {
      return (int)i;
    }
  }
  return -1;
}

/**
 * @brief Configure port pin for TC output
 */
static void configurePinMux(uint8_t portPin, uint8_t mux)
{
  // Enable pin multiplexing
  if (portPin & 1)
  {
    // Odd pin - use PMUXO
    PORT->Group[0].PMUX[portPin >> 1].bit.PMUXO = mux;
  }
  else
  {
    // Even pin - use PMUXE
    PORT->Group[0].PMUX[portPin >> 1].bit.PMUXE = mux;
  }
  // Enable peripheral multiplexer for this pin
  PORT->Group[0].PINCFG[portPin].bit.PMUXEN = 1;
}

/**
 * @brief Disable peripheral mux for a pin (return to GPIO)
 */
static void disablePinMux(uint8_t portPin)
{
  PORT->Group[0].PINCFG[portPin].bit.PMUXEN = 0;
}

/**
 * @brief Sync wait helper for TC
 */
static inline void syncTC(Tc *tc)
{
  while (tc->COUNT8.STATUS.bit.SYNCBUSY)
  {
    // Wait for synchronization
  }
}

/**
 * @brief Calculate prescaler and 8-bit period for a target frequency
 *
 * TC in 8-bit mode has an 8-bit PER register, so the period is capped
 * at 255 (unlike TCC0's 24-bit counter). freq = 48MHz / (prescaler * (PER+1)).
 * Reachable range is ~184Hz (DIV1024, PER=255) to ~188kHz (DIV1, PER=255);
 * out-of-range requests are clamped.
 */
static void calcTimebase(uint32_t frequencyHz, uint8_t *prescalerOut, uint32_t *periodOut)
{
  uint32_t clockFreq = 48000000UL; // Assuming 48MHz GCLK0
  uint32_t prescalerValues[] = {1, 2, 4, 8, 16, 64, 256, 1024};
  uint8_t prescalerBits[] = {
      TC_CTRLA_PRESCALER_DIV1_Val,
      TC_CTRLA_PRESCALER_DIV2_Val,
      TC_CTRLA_PRESCALER_DIV4_Val,
      TC_CTRLA_PRESCALER_DIV8_Val,
      TC_CTRLA_PRESCALER_DIV16_Val,
      TC_CTRLA_PRESCALER_DIV64_Val,
      TC_CTRLA_PRESCALER_DIV256_Val,
      TC_CTRLA_PRESCALER_DIV1024_Val};

  // Fallback: slowest possible timebase
  uint8_t prescaler = TC_CTRLA_PRESCALER_DIV1024_Val;
  uint32_t period = PWM_MAX_VALUE;

  if (frequencyHz == 0)
  {
    frequencyHz = 1;
  }

  // Pick the smallest prescaler whose period fits in 8 bits,
  // maximizing duty cycle resolution
  for (int i = 0; i < 8; i++)
  {
    uint32_t testPeriod = clockFreq / prescalerValues[i] / frequencyHz;
    if (testPeriod < 2)
    {
      testPeriod = 2; // Frequency too high - clamp
    }
    if ((testPeriod - 1) <= PWM_MAX_VALUE)
    {
      prescaler = prescalerBits[i];
      period = testPeriod - 1;
      break;
    }
  }

  *prescalerOut = prescaler;
  *periodOut = period;
}

/**
 * @brief Apply mode, prescaler and period to one TC (must be called with TC disabled)
 */
static void configureTC(Tc *tc, uint8_t prescaler, uint32_t period)
{
  // Reset the TC
  tc->COUNT8.CTRLA.reg = TC_CTRLA_SWRST;
  syncTC(tc);
  while (tc->COUNT8.CTRLA.bit.SWRST)
    ;

  // 8-bit counter mode, single-slope PWM, prescaler
  // (CTRLA is enable-protected, so everything goes in one write)
  tc->COUNT8.CTRLA.reg = TC_CTRLA_MODE_COUNT8 |
                         TC_CTRLA_WAVEGEN_NPWM |
                         TC_CTRLA_PRESCALER(prescaler) |
                         TC_CTRLA_PRESCSYNC_GCLK;
  syncTC(tc);

  // Set period (TOP value)
  tc->COUNT8.PER.reg = (uint8_t)period;
  syncTC(tc);

  // Initialize both compare channels to 0 (0% duty)
  tc->COUNT8.CC[0].reg = 0;
  syncTC(tc);
  tc->COUNT8.CC[1].reg = 0;
  syncTC(tc);

  // Enable
  tc->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
  syncTC(tc);
}

bool pwm_init(uint32_t frequencyHz)
{
  if (pwmInitialized)
  {
    return true;
  }

  // TC1 and TC2 share one generic clock channel on the SAMD11.
  // Use GCLK0 (typically 48MHz from DFLL).
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC1_GCLK_ID) |
                      GCLK_CLKCTRL_GEN_GCLK0 |
                      GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Enable TC1 and TC2 in Power Manager
  PM->APBCMASK.reg |= PM_APBCMASK_TC1 | PM_APBCMASK_TC2;

  uint8_t prescaler;
  uint32_t period;
  calcTimebase(frequencyHz, &prescaler, &period);

  currentPrescaler = prescaler;
  currentPeriod = period;

  configureTC(TC1, prescaler, period);
  configureTC(TC2, prescaler, period);

  pwmInitialized = true;

  return true;
}

bool pwm_configure_channel(uint8_t channel, uint8_t pin)
{
  if (channel >= PWM_NUM_CHANNELS)
  {
    return false;
  }

  int pinIdx = findPinConfig(pin);
  if (pinIdx < 0)
  {
    return false; // Pin has no TC waveform output
  }

  if (pinMap[pinIdx].channel != channel)
  {
    // This pin doesn't map to the requested channel
    return false;
  }

  // Configure pin for TC output
  configurePinMux(pinMap[pinIdx].portPin, pinMap[pinIdx].mux);

  return true;
}

void pwm_set_duty(uint8_t channel, uint16_t duty)
{
  if (channel >= PWM_NUM_CHANNELS || !pwmInitialized)
  {
    return;
  }

  // Clamp to maximum value
  if (duty > PWM_MAX_VALUE)
  {
    duty = PWM_MAX_VALUE;
  }

  // Scale duty to current period
  uint32_t ccValue = ((uint32_t)duty * currentPeriod) / PWM_MAX_VALUE;

  // TC compare registers are not double-buffered (unlike TCC0's CCB),
  // so a mid-period update can glitch the output for one cycle.
  // Harmless for LED dimming.
  Tc *tc = tcForChannel(channel);
  tc->COUNT8.CC[ccIndexForChannel(channel)].reg = (uint8_t)ccValue;
  syncTC(tc);

  channelDuty[channel] = duty;
}

void pwm_set_duty_percent(uint8_t channel, float percent)
{
  if (percent < 0.0f)
    percent = 0.0f;
  if (percent > 100.0f)
    percent = 100.0f;

  uint16_t duty = (uint16_t)((percent * PWM_MAX_VALUE) / 100.0f);
  pwm_set_duty(channel, duty);
}

void pwm_enable_channel(uint8_t channel)
{
  if (channel >= PWM_NUM_CHANNELS)
  {
    return;
  }
  channelEnabled |= (1 << channel);
  // Restore previous duty cycle
  pwm_set_duty(channel, channelDuty[channel]);
}

void pwm_disable_channel(uint8_t channel)
{
  if (channel >= PWM_NUM_CHANNELS || !pwmInitialized)
  {
    return;
  }
  channelEnabled &= ~(1 << channel);

  // Set duty to 0 to disable output
  Tc *tc = tcForChannel(channel);
  tc->COUNT8.CC[ccIndexForChannel(channel)].reg = 0;
  syncTC(tc);
}

uint16_t pwm_get_duty(uint8_t channel)
{
  if (channel >= PWM_NUM_CHANNELS)
  {
    return 0;
  }
  return channelDuty[channel];
}

void pwm_set_frequency(uint32_t frequencyHz)
{
  if (!pwmInitialized)
  {
    return;
  }

  uint8_t prescaler;
  uint32_t period;
  calcTimebase(frequencyHz, &prescaler, &period);

  currentPrescaler = prescaler;
  currentPeriod = period;

  // Prescaler is enable-protected, so both TCs get a full reconfigure
  configureTC(TC1, prescaler, period);
  configureTC(TC2, prescaler, period);

  // Restore duty cycles on enabled channels
  for (int i = 0; i < PWM_NUM_CHANNELS; i++)
  {
    if (channelEnabled & (1 << i))
    {
      pwm_set_duty(i, channelDuty[i]);
    }
  }
}

void pwm_deinit(void)
{
  if (!pwmInitialized)
  {
    return;
  }

  // Disable and reset both TCs
  Tc *tcs[] = {TC1, TC2};
  for (int i = 0; i < 2; i++)
  {
    tcs[i]->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    syncTC(tcs[i]);
    tcs[i]->COUNT8.CTRLA.reg = TC_CTRLA_SWRST;
    syncTC(tcs[i]);
    while (tcs[i]->COUNT8.CTRLA.bit.SWRST)
      ;
  }

  // Disable the shared TC1/TC2 clock
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC1_GCLK_ID);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Disable in Power Manager
  PM->APBCMASK.reg &= ~(PM_APBCMASK_TC1 | PM_APBCMASK_TC2);

  // Reset state
  for (int i = 0; i < PWM_NUM_CHANNELS; i++)
  {
    channelDuty[i] = 0;
  }
  channelEnabled = 0;
  pwmInitialized = false;
}
