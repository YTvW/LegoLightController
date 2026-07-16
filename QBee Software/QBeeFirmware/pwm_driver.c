/**
 * @file pwm_driver.c
 * @brief TC-based PWM driver implementation for SAMD11C / SAMD21
 *
 * Uses two TC instances in 8-bit PWM mode, leaving TCC0 free for the
 * WS2812B DMA driver:
 *   - SAMD11C: TC1 (channels 0-1) and TC2 (channels 2-3)
 *   - SAMD21:  TC3 (channels 0-1) and TC4 (channels 2-3)
 */

#include "pwm_driver.h"

// The two TC instances backing the four PWM channels, and their GCLK
// channel IDs. On the SAMD11 TC1/TC2 share one GCLK channel; on the
// SAMD21 TC3 (paired with TCC2) and TC4 (paired with TC5) each sit on
// their own.
#if defined(_SAMD21_)
#define PWM_TC_A TC3
#define PWM_TC_B TC4
#define PWM_TC_A_GCLK_ID TC3_GCLK_ID
#define PWM_TC_B_GCLK_ID TC4_GCLK_ID
#define PWM_APBCMASK (PM_APBCMASK_TC3 | PM_APBCMASK_TC4)
#else // _SAMD11_
#define PWM_TC_A TC1
#define PWM_TC_B TC2
#define PWM_TC_A_GCLK_ID TC1_GCLK_ID
#define PWM_TC_B_GCLK_ID TC1_GCLK_ID // TC1 and TC2 share one GCLK channel
#define PWM_APBCMASK (PM_APBCMASK_TC1 | PM_APBCMASK_TC2)
#endif

// Pin mapping table - maps port pins to TC waveform outputs.
// Pins are matched through g_APinDescription, so Arduino pin numbers
// resolve correctly on any board variant.
static const struct
{
  uint8_t port;    // 0 = PORTA, 1 = PORTB
  uint8_t portPin; // pin number within the port
  uint8_t channel; // PWM channel (0-1 = TC A CC0/CC1, 2-3 = TC B CC0/CC1)
  uint8_t mux;     // PMUX function
} pinMap[] = {
#if defined(_SAMD21_)
    // SAMD21 TC Pin Mapping (PMUX Function E = 0x04)
    {0, 14, 0, PORT_PMUX_PMUXE_E_Val}, // PA14 -> TC3/WO[0]
    {0, 15, 1, PORT_PMUX_PMUXE_E_Val}, // PA15 -> TC3/WO[1]
    {0, 18, 0, PORT_PMUX_PMUXE_E_Val}, // PA18 -> TC3/WO[0] (alt)
    {0, 19, 1, PORT_PMUX_PMUXE_E_Val}, // PA19 -> TC3/WO[1] (alt)
    {0, 22, 2, PORT_PMUX_PMUXE_E_Val}, // PA22 -> TC4/WO[0]
    {0, 23, 3, PORT_PMUX_PMUXE_E_Val}, // PA23 -> TC4/WO[1]
    {1, 8, 2, PORT_PMUX_PMUXE_E_Val},  // PB08 -> TC4/WO[0] (alt, G/J parts)
    {1, 9, 3, PORT_PMUX_PMUXE_E_Val},  // PB09 -> TC4/WO[1] (alt, G/J parts)
#else
    // SAMD11C14A TC Pin Mapping (PMUX Function E = 0x04)
    // Note: PA08/PA09 have no TC waveform outputs on the SAMD11, only TCC0.
    {0, 4, 0, PORT_PMUX_PMUXE_E_Val},  // PA04 -> TC1/WO[0]
    {0, 5, 1, PORT_PMUX_PMUXE_E_Val},  // PA05 -> TC1/WO[1]
    {0, 14, 0, PORT_PMUX_PMUXE_E_Val}, // PA14 -> TC1/WO[0] (alt - normally WS2812B strip 2)
    {0, 15, 1, PORT_PMUX_PMUXE_E_Val}, // PA15 -> TC1/WO[1] (alt - normally WS2812B strip 1)
    {0, 30, 2, PORT_PMUX_PMUXE_E_Val}, // PA30 -> TC2/WO[0] (WARNING: SWCLK programming pin)
    {0, 31, 3, PORT_PMUX_PMUXE_E_Val}, // PA31 -> TC2/WO[1] (WARNING: SWDIO programming pin)
#endif
};
#define PIN_MAP_SIZE (sizeof(pinMap) / sizeof(pinMap[0]))

// Internal state
static bool pwmInitialized = false;
static uint16_t channelDuty[PWM_NUM_CHANNELS] = {0};
static uint8_t channelEnabled = 0; // Bitmask of enabled channels
static uint32_t currentPeriod = 0;
static uint8_t currentPrescaler = TC_CTRLA_PRESCALER_DIV1_Val;

/**
 * @brief Get the TC instance serving a channel (0-1 = TC A, 2-3 = TC B)
 */
static inline Tc *tcForChannel(uint8_t channel)
{
  return (channel < 2) ? PWM_TC_A : PWM_TC_B;
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
 *
 * Resolves the Arduino pin to its port/pin through g_APinDescription
 * (GetPort/GetPin work with every variant pin-table layout), then matches
 * against the port pin map.
 */
static int findPinConfig(uint8_t pin)
{
  uint8_t portPin = GetPin(pin);
  uint8_t portNum = GetPort(pin);

  for (size_t i = 0; i < PIN_MAP_SIZE; i++)
  {
    if (pinMap[i].port == portNum && pinMap[i].portPin == portPin)
    {
      return (int)i;
    }
  }
  return -1;
}

/**
 * @brief Configure port pin for TC output
 */
static void configurePinMux(uint8_t port, uint8_t portPin, uint8_t mux)
{
  // Enable pin multiplexing
  if (portPin & 1)
  {
    // Odd pin - use PMUXO
    PORT->Group[port].PMUX[portPin >> 1].bit.PMUXO = mux;
  }
  else
  {
    // Even pin - use PMUXE
    PORT->Group[port].PMUX[portPin >> 1].bit.PMUXE = mux;
  }
  // Enable peripheral multiplexer for this pin
  PORT->Group[port].PINCFG[portPin].bit.PMUXEN = 1;
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

  // Feed both TC GCLK channels from GCLK0 (typically 48MHz from DFLL).
  // On the SAMD11 both TCs share one channel, so the second write just
  // repeats the first; on the SAMD21 TC3 and TC4 have separate channels.
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(PWM_TC_A_GCLK_ID) |
                      GCLK_CLKCTRL_GEN_GCLK0 |
                      GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(PWM_TC_B_GCLK_ID) |
                      GCLK_CLKCTRL_GEN_GCLK0 |
                      GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Enable both TCs in Power Manager
  PM->APBCMASK.reg |= PWM_APBCMASK;

  uint8_t prescaler;
  uint32_t period;
  calcTimebase(frequencyHz, &prescaler, &period);

  currentPrescaler = prescaler;
  currentPeriod = period;

  configureTC(PWM_TC_A, prescaler, period);
  configureTC(PWM_TC_B, prescaler, period);

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
  configurePinMux(pinMap[pinIdx].port, pinMap[pinIdx].portPin, pinMap[pinIdx].mux);

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
  configureTC(PWM_TC_A, prescaler, period);
  configureTC(PWM_TC_B, prescaler, period);

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
  Tc *tcs[] = {PWM_TC_A, PWM_TC_B};
  for (int i = 0; i < 2; i++)
  {
    tcs[i]->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    syncTC(tcs[i]);
    tcs[i]->COUNT8.CTRLA.reg = TC_CTRLA_SWRST;
    syncTC(tcs[i]);
    while (tcs[i]->COUNT8.CTRLA.bit.SWRST)
      ;
  }

  // Disable both TC clock channels. Note the sharing: TC1/TC2 share one
  // channel on the SAMD11; on the SAMD21 this also stops TCC2 (paired
  // with TC3) and TC5 (paired with TC4), e.g. analogWrite on their pins.
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(PWM_TC_A_GCLK_ID);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(PWM_TC_B_GCLK_ID);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Disable in Power Manager
  PM->APBCMASK.reg &= ~PWM_APBCMASK;

  // Reset state
  for (int i = 0; i < PWM_NUM_CHANNELS; i++)
  {
    channelDuty[i] = 0;
  }
  channelEnabled = 0;
  pwmInitialized = false;
}
