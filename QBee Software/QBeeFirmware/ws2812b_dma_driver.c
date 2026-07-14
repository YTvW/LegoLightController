/**
 * @file ws2812b_dma_driver.c
 * @brief WS2812B driver using TCC0+DMA for SAMD11C / SAMD21
 *
 * Uses TCC0 in PWM mode with DMA controller for background LED updates.
 * The DMA feeds compare values into TCC0's double-buffered CCB[x] register,
 * triggered once per period by the TCC0 overflow DMA request. Hardware
 * latches CCB into CC at each period boundary (UPDATE), so the compare
 * value for a bit can never race the counter mid-period.
 *
 * Supports up to WS2812B_MAX_STRIPS independent, concurrent strips. Each
 * strip owns its own DMA channel, DMA descriptor, DMA buffer and TCC0
 * WO[x]/CC[x] compare channel - only TCC0's shared timebase (prescaler,
 * period, enable) and the DMAC's global setup are configured once, on the
 * first strip's init. Two strips can encode and transfer concurrently
 * without touching each other's state.
 *
 * NOTE: TC1's CC registers are NOT double-buffered on the SAMD11, which
 * makes TC1 unusable for this: each DMA write lands ~10-20 clocks into
 * the period it is consumed in, racing the 14-tick '0'-bit compare point,
 * and a stale OVF request left pending while TC1 free-runs causes the DMAC
 * to drain the whole bit buffer in a few microseconds at channel enable.
 */

#include "ws2812b_dma_driver.h"

// ============================================================================
// DMA Configuration
// ============================================================================

// DMA buffer size: 8 bits per color byte, plus extra for reset
#define WS2812B_DMA_BUFFER_SIZE ((WS2812B_MAX_PIXELS * 4 * 8) + 64)

// DMA descriptors (must be 16-byte aligned). The DMAC indexes this array
// by channel number; each strip's DMA channel number equals its slot.
__attribute__((aligned(16))) static DmacDescriptor _dmaDescriptors[WS2812B_MAX_STRIPS];

// Writeback descriptors (required by DMAC), same indexing as above
__attribute__((aligned(16))) static DmacDescriptor _dmaWritebacks[WS2812B_MAX_STRIPS];

// DMA buffer for compare values, one per strip slot
static uint8_t _dmaBuffers[WS2812B_MAX_STRIPS][WS2812B_DMA_BUFFER_SIZE];

// Internal pixel buffers, one per strip slot (used when ws2812b_init() is
// called with buffer = NULL)
static uint8_t _pixelBuffers[WS2812B_MAX_STRIPS][WS2812B_MAX_PIXELS * 4];

// Which strip owns each DMA channel (only used by the optional ISR)
static ws2812b_strip_t *_activeStrips[WS2812B_MAX_STRIPS] = {NULL};

// Number of strips configured so far; also the next free slot/DMA channel
static uint8_t _numStripsConfigured = 0;

// TCC0 shared timebase - configured once, by the first ws2812b_init() call
static volatile bool _tccConfigured = false;

// DMAC global setup (BASEADDR/WRBADDR/DMAENABLE) - done once
static volatile bool _dmacInitialized = false;

// Recovery counter, aggregated across all strips
static uint16_t _recoveryCount = 0;

// ============================================================================
// Forward Declarations
// ============================================================================

static bool _configureTCC0(uint8_t pin, uint8_t *woChannelOut);
static void _configureDMAChannel(uint8_t channel);
static uint16_t _encodePixels(ws2812b_strip_t *strip);

// ============================================================================
// Pin Mapping for TCC0 (per-chip: SAMD11C or SAMD21)
// ============================================================================

typedef struct
{
  uint8_t port;      // 0 = PORTA, 1 = PORTB
  uint8_t portPin;   // PA0 = 0, PA1 = 1, etc.
  uint8_t ccChannel; // TCC0 CC[x]/CCB[x] compare channel driving the pin
  uint8_t pmuxVal;   // PMUX function (E or F)
} tcc0_pin_map_t;

#if defined(_SAMD21_)

// TCC0 capable pins on SAMD21. TCC0 has 4 compare channels (CC0-CC3) and
// 8 waveform outputs; with the default output matrix WO[4+n] mirrors CC[n],
// so pins on WO[4..7] map back to CC[0..3]. Up to 4 concurrent strips are
// possible if each pin uses a distinct CC channel.
// Note: Actual Arduino pin numbers depend on board variant
static const tcc0_pin_map_t _tcc0Pins[] = {
    {0, 4, 0, PORT_PMUX_PMUXE_E_Val},   // PA04 -> TCC0/WO[0] (mux E) -> CC0
    {0, 5, 1, PORT_PMUX_PMUXE_E_Val},   // PA05 -> TCC0/WO[1] (mux E) -> CC1
    {0, 8, 0, PORT_PMUX_PMUXE_E_Val},   // PA08 -> TCC0/WO[0] (mux E) -> CC0
    {0, 9, 1, PORT_PMUX_PMUXE_E_Val},   // PA09 -> TCC0/WO[1] (mux E) -> CC1
    {0, 10, 2, PORT_PMUX_PMUXE_F_Val},  // PA10 -> TCC0/WO[2] (mux F) -> CC2
    {0, 11, 3, PORT_PMUX_PMUXE_F_Val},  // PA11 -> TCC0/WO[3] (mux F) -> CC3
    {0, 12, 2, PORT_PMUX_PMUXE_F_Val},  // PA12 -> TCC0/WO[6] (mux F) -> CC2
    {0, 13, 3, PORT_PMUX_PMUXE_F_Val},  // PA13 -> TCC0/WO[7] (mux F) -> CC3
    {0, 14, 0, PORT_PMUX_PMUXE_F_Val},  // PA14 -> TCC0/WO[4] (mux F) -> CC0
    {0, 15, 1, PORT_PMUX_PMUXE_F_Val},  // PA15 -> TCC0/WO[5] (mux F) -> CC1
    {0, 16, 2, PORT_PMUX_PMUXE_F_Val},  // PA16 -> TCC0/WO[6] (mux F) -> CC2
    {0, 17, 3, PORT_PMUX_PMUXE_F_Val},  // PA17 -> TCC0/WO[7] (mux F) -> CC3
    {0, 18, 2, PORT_PMUX_PMUXE_F_Val},  // PA18 -> TCC0/WO[2] (mux F) -> CC2
    {0, 19, 3, PORT_PMUX_PMUXE_F_Val},  // PA19 -> TCC0/WO[3] (mux F) -> CC3
    {0, 20, 2, PORT_PMUX_PMUXE_F_Val},  // PA20 -> TCC0/WO[6] (mux F) -> CC2
    {0, 21, 3, PORT_PMUX_PMUXE_F_Val},  // PA21 -> TCC0/WO[7] (mux F) -> CC3
    {0, 22, 0, PORT_PMUX_PMUXE_F_Val},  // PA22 -> TCC0/WO[4] (mux F) -> CC0
    {0, 23, 1, PORT_PMUX_PMUXE_F_Val},  // PA23 -> TCC0/WO[5] (mux F) -> CC1
    {1, 10, 0, PORT_PMUX_PMUXE_F_Val},  // PB10 -> TCC0/WO[4] (mux F) -> CC0 (G/J parts)
    {1, 11, 1, PORT_PMUX_PMUXE_F_Val},  // PB11 -> TCC0/WO[5] (mux F) -> CC1 (G/J parts)
    {1, 30, 0, PORT_PMUX_PMUXE_E_Val},  // PB30 -> TCC0/WO[0] (mux E) -> CC0 (J parts)
    {1, 31, 1, PORT_PMUX_PMUXE_E_Val},  // PB31 -> TCC0/WO[1] (mux E) -> CC1 (J parts)
};

#else // _SAMD11_

// TCC0 capable pins on SAMD11C
// Note: Actual Arduino pin numbers depend on board variant
static const tcc0_pin_map_t _tcc0Pins[] = {
    {0, 4, 0, PORT_PMUX_PMUXE_E_Val},  // PA04 -> TCC0/WO[0] (mux E) -> CC0
    {0, 5, 1, PORT_PMUX_PMUXE_E_Val},  // PA05 -> TCC0/WO[1] (mux E) -> CC1
    {0, 14, 0, PORT_PMUX_PMUXE_F_Val}, // PA14 -> TCC0/WO[0] (mux F) -> CC0
    {0, 15, 1, PORT_PMUX_PMUXE_F_Val}, // PA15 -> TCC0/WO[1] (mux F) -> CC1
};

#endif

#define TCC0_PIN_MAP_SIZE (sizeof(_tcc0Pins) / sizeof(_tcc0Pins[0]))

// ============================================================================
// TCC0 Configuration
// ============================================================================

/**
 * @brief Configure TCC0's shared timebase once, then bind one pin to its
 *        own WO[x]/CC[x] compare channel.
 *
 * Safe to call once per strip: the counter mode/period/prescaler are
 * enable-protected, so they are only touched before TCC0's first enable.
 * Binding a second pin afterwards only zeroes that pin's own CC/CCB and
 * sets its PMUX - it never resets TCC0 or disturbs another strip's
 * already-running compare channel.
 *
 * @param arduinoPin Arduino pin number to bind
 * @param woChannelOut Set to the resolved WO[x]/CC[x] index on success
 * @return true on success, false if the pin has no TCC0 output
 */
static bool _configureTCC0(uint8_t arduinoPin, uint8_t *woChannelOut)
{
  // Find the pin in our mapping
  const tcc0_pin_map_t *pinMap = NULL;

  // Map Arduino pin to port + port pin (GetPort/GetPin work with every
  // variant pin-table layout, unlike raw ulPort/ulPin field access)
  uint8_t portPin = GetPin(arduinoPin);
  uint8_t portNum = GetPort(arduinoPin);

  // Find matching TCC0 pin configuration
  for (uint8_t i = 0; i < TCC0_PIN_MAP_SIZE; i++)
  {
    if (_tcc0Pins[i].port == portNum && _tcc0Pins[i].portPin == portPin)
    {
      pinMap = &_tcc0Pins[i];
      break;
    }
  }

  if (!pinMap)
  {
    return false; // Pin not capable of TCC0 output
  }

  if (!_tccConfigured)
  {
    // Enable TCC0 clock (GCLK0 = 48MHz)
    PM->APBCMASK.reg |= PM_APBCMASK_TCC0;

    // Connect GCLK0 to TCC0
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TCC0_GCLK_ID) |
                        GCLK_CLKCTRL_CLKEN |
                        GCLK_CLKCTRL_GEN_GCLK0;
    while (GCLK->STATUS.bit.SYNCBUSY)
      ;

    // Reset TCC0
    TCC0->CTRLA.reg = TCC_CTRLA_SWRST;
    while (TCC0->SYNCBUSY.bit.SWRST || TCC0->CTRLA.bit.SWRST)
      ;

    // Prescaler DIV1 (48MHz clock)
    TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1;

    // Normal PWM: output high from 0 until CC[x] match, then low
    TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
    while (TCC0->SYNCBUSY.reg)
      ;

    // Set period for 800kHz (60 ticks @ 48MHz = 1.25µs)
    TCC0->PER.reg = WS2812B_PERIOD_TICKS - 1;
    while (TCC0->SYNCBUSY.reg)
      ;

    // Enable TCC0
    TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
    while (TCC0->SYNCBUSY.bit.ENABLE)
      ;

    _tccConfigured = true;
  }

  // Initialize this channel's compare value and its buffer to 0 (output
  // low). Safe to do on an already-running TCC0 - same mechanism
  // pwm_driver uses for live duty updates. DMA only ever writes the low
  // byte of CCB[x]; the upper bytes stay zero from this init.
  TCC0->CC[pinMap->ccChannel].reg = 0;
  while (TCC0->SYNCBUSY.reg)
    ;
  TCC0->CCB[pinMap->ccChannel].reg = 0;
  while (TCC0->SYNCBUSY.reg)
    ;

  // Configure pin for TCC0 output
  PORT->Group[portNum].PINCFG[portPin].reg = PORT_PINCFG_PMUXEN;
  if (portPin & 1)
  {
    // Odd pin: use upper nibble
    PORT->Group[portNum].PMUX[portPin >> 1].bit.PMUXO = pinMap->pmuxVal;
  }
  else
  {
    // Even pin: use lower nibble
    PORT->Group[portNum].PMUX[portPin >> 1].bit.PMUXE = pinMap->pmuxVal;
  }

  *woChannelOut = pinMap->ccChannel;
  return true;
}

// ============================================================================
// DMA Configuration
// ============================================================================

/**
 * @brief Configure one DMA channel to feed TCC0 overflow requests.
 *
 * The DMAC-wide setup (clocks, descriptor table base addresses, global
 * enable) runs once, on the first strip's init; every strip after that
 * only configures its own channel, leaving other channels' in-flight
 * transfers untouched.
 *
 * @param channel DMA channel to configure (== the strip's slot index)
 */
static void _configureDMAChannel(uint8_t channel)
{
  if (!_dmacInitialized)
  {
    // Enable DMAC clock
    PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
    PM->APBBMASK.reg |= PM_APBBMASK_DMAC;

    // Reset DMAC
    DMAC->CTRL.reg = DMAC_CTRL_SWRST;
    while (DMAC->CTRL.bit.SWRST)
      ;

    // Set descriptor base addresses (indexed by channel number)
    DMAC->BASEADDR.reg = (uint32_t)_dmaDescriptors;
    DMAC->WRBADDR.reg = (uint32_t)_dmaWritebacks;

    // Enable all priority levels
    DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xF);

    _dmacInitialized = true;
  }

  // Select and reset this channel only
  DMAC->CHID.reg = channel;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  while (DMAC->CHCTRLA.bit.SWRST)
    ;

  // Configure channel:
  // - Trigger on TCC0 overflow (once per 1.25µs bit period)
  // - Single beat per trigger
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT |
                      DMAC_CHCTRLB_TRIGSRC(TCC0_DMAC_ID_OVF) |
                      DMAC_CHCTRLB_LVL(0);
}

/**
 * @brief Encode pixel data into this strip's own DMA buffer
 *
 * Converts each bit of pixel data to a compare value:
 *   - '0' bit -> 14 (short pulse)
 *   - '1' bit -> 38 (long pulse)
 *
 * @param strip Pointer to strip structure
 * @return Number of bytes written to the strip's DMA buffer
 */
static uint16_t _encodePixels(ws2812b_strip_t *strip)
{
  uint8_t *src = strip->pixels;
  uint8_t *dstStart = _dmaBuffers[strip->_slot];
  uint8_t *dst = dstStart;
  uint16_t totalBytes = strip->numPixels * strip->bytesPerPixel;
  uint8_t brightness = strip->brightness;

  // Encode each byte of pixel data
  for (uint16_t i = 0; i < totalBytes; i++)
  {
    // Apply brightness scaling
    uint8_t pixelByte = src[i];
    if (brightness < 255)
    {
      pixelByte = (uint8_t)(((uint16_t)pixelByte * brightness) >> 8);
    }

    // Convert each bit to compare value (MSB first)
    for (int8_t bit = 7; bit >= 0; bit--)
    {
      if (pixelByte & (1 << bit))
      {
        *dst++ = WS2812B_T1H_TICKS; // '1' bit: 38 ticks high
      }
      else
      {
        *dst++ = WS2812B_T0H_TICKS; // '0' bit: 14 ticks high
      }
    }
  }

  // Add reset period (low for >50µs = ~40 cycles @ 800kHz)
  // Fill with 0s to keep output low
  for (uint8_t i = 0; i < 48; i++)
  {
    *dst++ = 0;
  }

  return (dst - dstStart);
}

/**
 * @brief Start this strip's DMA transfer on its own channel
 *
 * @param strip Pointer to strip structure
 * @param length Number of bytes to transfer
 */
static void _startDMATransfer(ws2812b_strip_t *strip, uint16_t length)
{
  uint8_t slot = strip->_slot;
  DmacDescriptor *desc = &_dmaDescriptors[slot];
  uint8_t *buf = _dmaBuffers[slot];

  // Configure this strip's own DMA descriptor. Beats are single bytes
  // written to the low byte of the 24-bit CCB[x] buffer register; the
  // upper bytes stay 0. CCB is latched into CC by hardware at each period
  // boundary, so each bit's compare value takes effect exactly one full
  // period after the overflow that fetched it - no race against the
  // running counter, and no interference with any other strip's channel.
  desc->BTCTRL.reg = DMAC_BTCTRL_VALID |
                     DMAC_BTCTRL_BEATSIZE_BYTE |
                     DMAC_BTCTRL_SRCINC | // Increment source
                     DMAC_BTCTRL_BLOCKACT_NOACT;

  desc->BTCNT.reg = length;
  desc->SRCADDR.reg = (uint32_t)(buf + length);                     // End of source
  desc->DSTADDR.reg = (uint32_t)&TCC0->CCB[strip->_woChannel].reg;  // This strip's CCB
  desc->DESCADDR.reg = 0;                                           // No linked descriptor

  // Clear the stale overflow request left over from TCC0 free-running
  // since the last transfer. Without this the DMAC sees an already
  // asserted trigger the moment the channel is enabled.
  TCC0->INTFLAG.reg = TCC_INTFLAG_OVF;

  // Select this strip's channel, clear its leftover flags, enable it
  DMAC->CHID.reg = slot;
  DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL | DMAC_CHINTFLAG_TERR | DMAC_CHINTFLAG_SUSP;
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  strip->_busyStartMs = millis();
  _activeStrips[slot] = strip;
}

/**
 * @brief Check whether this strip's transfer has finished, and recover
 *        from any abnormal end so it can never wedge in BUSY.
 *
 * Completion is normally signalled by TCMPL. Additionally treated as
 * "done": a transfer error (TERR), the channel no longer being enabled
 * without a completion flag, and a hard timeout many times the worst
 * case transfer duration (a 16-pixel RGBW frame takes ~0.7ms).
 *
 * @return true if this strip's channel is free and it was marked idle
 */
static bool _checkTransferComplete(ws2812b_strip_t *strip)
{
  DMAC->CHID.reg = strip->_slot;
  uint8_t flags = DMAC->CHINTFLAG.reg;

  bool done = (flags & (DMAC_CHINTFLAG_TCMPL | DMAC_CHINTFLAG_TERR)) != 0;

  if (!done)
  {
    // Channel silently disabled, or transfer running far too long
    if (!DMAC->CHCTRLA.bit.ENABLE ||
        (uint32_t)(millis() - strip->_busyStartMs) > WS2812B_XFER_TIMEOUT_MS)
    {
      done = true;
      _recoveryCount++;
    }
  }
  else if (flags & DMAC_CHINTFLAG_TERR)
  {
    _recoveryCount++;
  }

  if (done)
  {
    DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
    DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL | DMAC_CHINTFLAG_TERR | DMAC_CHINTFLAG_SUSP;
    strip->state = WS2812B_STATE_IDLE;
  }

  return done;
}

// ============================================================================
// Public API
// ============================================================================

bool ws2812b_init(ws2812b_strip_t *strip, uint8_t pin, uint16_t numPixels,
                  uint8_t pixelType, uint8_t *buffer)
{
  if (!strip || numPixels == 0)
  {
    return false;
  }

  if (_numStripsConfigured >= WS2812B_MAX_STRIPS)
  {
    return false; // Out of DMA channels/buffers - see WS2812B_MAX_STRIPS
  }

  // Determine bytes per pixel
  strip->bytesPerPixel = (pixelType & 0x10) ? 4 : 3;

  uint16_t bufferSize = numPixels * strip->bytesPerPixel;
  uint8_t slot = _numStripsConfigured;

  if (buffer)
  {
    strip->pixels = buffer;
  }
  else
  {
    if (bufferSize > sizeof(_pixelBuffers[slot]))
    {
      return false;
    }
    strip->pixels = _pixelBuffers[slot];
  }

  // Bind the pin to its own TCC0 compare channel before committing this
  // strip to a slot, so an unsupported pin fails without burning one.
  uint8_t woChannel;
  if (!_configureTCC0(pin, &woChannel))
  {
    return false;
  }

  strip->pin = pin;
  strip->numPixels = numPixels;
  strip->pixelType = pixelType;
  strip->brightness = 255;
  strip->state = WS2812B_STATE_IDLE;
  strip->_slot = slot;
  strip->_woChannel = woChannel;
  strip->_busyStartMs = 0;

  // Clear pixel buffer
  memset(strip->pixels, 0, bufferSize);

  // Configure this strip's own DMA channel
  _configureDMAChannel(slot);

  _numStripsConfigured++;

  return true;
}

void ws2812b_set_brightness(ws2812b_strip_t *strip, uint8_t brightness)
{
  if (strip)
  {
    strip->brightness = brightness;
  }
}

void ws2812b_set_pixel_rgb(ws2812b_strip_t *strip, uint16_t pixel,
                           uint8_t r, uint8_t g, uint8_t b)
{
  if (!strip || pixel >= strip->numPixels)
  {
    return;
  }

  uint8_t *p = &strip->pixels[pixel * strip->bytesPerPixel];
  uint8_t type = strip->pixelType & 0x0F;

  switch (type)
  {
  case WS2812B_RGB:
    p[0] = r;
    p[1] = g;
    p[2] = b;
    break;
  case WS2812B_GRB:
    p[0] = g;
    p[1] = r;
    p[2] = b;
    break;
  case WS2812B_BRG:
    p[0] = b;
    p[1] = r;
    p[2] = g;
    break;
  case WS2812B_RBG:
    p[0] = r;
    p[1] = b;
    p[2] = g;
    break;
  }
}

void ws2812b_set_pixel_rgbw(ws2812b_strip_t *strip, uint16_t pixel,
                            uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
  if (!strip || pixel >= strip->numPixels || strip->bytesPerPixel != 4)
  {
    return;
  }

  uint8_t *p = &strip->pixels[pixel * 4];
  uint8_t type = strip->pixelType & 0x0F;

  if (type == (WS2812B_RGBW & 0x0F))
  {
    p[0] = r;
    p[1] = g;
    p[2] = b;
    p[3] = w;
  }
  else
  { // GRBW
    p[0] = g;
    p[1] = r;
    p[2] = b;
    p[3] = w;
  }
}

void ws2812b_set_pixel_color(ws2812b_strip_t *strip, uint16_t pixel, uint32_t color)
{
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;

  if (strip->bytesPerPixel == 4)
  {
    uint8_t w = (color >> 24) & 0xFF;
    ws2812b_set_pixel_rgbw(strip, pixel, r, g, b, w);
  }
  else
  {
    ws2812b_set_pixel_rgb(strip, pixel, r, g, b);
  }
}

uint32_t ws2812b_get_pixel_color(ws2812b_strip_t *strip, uint16_t pixel)
{
  if (!strip || pixel >= strip->numPixels)
  {
    return 0;
  }

  uint8_t *p = &strip->pixels[pixel * strip->bytesPerPixel];
  uint8_t type = strip->pixelType & 0x0F;
  uint8_t r, g, b, w = 0;

  switch (type)
  {
  case WS2812B_RGB:
    r = p[0];
    g = p[1];
    b = p[2];
    break;
  case WS2812B_GRB:
    g = p[0];
    r = p[1];
    b = p[2];
    break;
  case WS2812B_BRG:
    b = p[0];
    r = p[1];
    g = p[2];
    break;
  case WS2812B_RBG:
    r = p[0];
    b = p[1];
    g = p[2];
    break;
  default:
    r = g = b = 0;
  }

  if (strip->bytesPerPixel == 4)
  {
    w = p[3];
    return ((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  }
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

void ws2812b_clear(ws2812b_strip_t *strip)
{
  if (!strip)
    return;
  memset(strip->pixels, 0, strip->numPixels * strip->bytesPerPixel);
}

void ws2812b_fill(ws2812b_strip_t *strip, uint8_t r, uint8_t g, uint8_t b)
{
  if (!strip)
    return;
  for (uint16_t i = 0; i < strip->numPixels; i++)
  {
    ws2812b_set_pixel_rgb(strip, i, r, g, b);
  }
}

bool ws2812b_show(ws2812b_strip_t *strip)
{
  if (!strip || !strip->pixels)
  {
    return false;
  }

  // Check if this strip's previous transfer is still in progress
  if (strip->state == WS2812B_STATE_BUSY && !_checkTransferComplete(strip))
  {
    return false; // Still busy
  }

  // Encode pixel data to this strip's own DMA buffer
  uint16_t dmaLength = _encodePixels(strip);

  // Mark as busy and start transfer on this strip's own channel
  strip->state = WS2812B_STATE_BUSY;
  _startDMATransfer(strip, dmaLength);

  return true;
}

bool ws2812b_is_ready(ws2812b_strip_t *strip)
{
  if (!strip)
    return true;

  if (strip->state == WS2812B_STATE_BUSY)
  {
    _checkTransferComplete(strip);
  }

  return (strip->state == WS2812B_STATE_IDLE);
}

uint16_t ws2812b_get_recovery_count(void)
{
  return _recoveryCount;
}

void ws2812b_wait(ws2812b_strip_t *strip)
{
  if (!strip)
    return;

  while (!ws2812b_is_ready(strip))
  {
    // Could add a small delay or yield here
  }
}

// ============================================================================
// DMA Interrupt Handler (optional - for callback support)
// ============================================================================

#ifdef WS2812B_USE_INTERRUPT
void DMAC_Handler(void)
{
  uint8_t channel = DMAC->INTPEND.bit.ID;
  if (channel >= WS2812B_MAX_STRIPS)
  {
    return;
  }

  DMAC->CHID.reg = channel;

  if (DMAC->CHINTFLAG.bit.TCMPL)
  {
    DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
    DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

    if (_activeStrips[channel])
    {
      _activeStrips[channel]->state = WS2812B_STATE_IDLE;
    }
  }
}
#endif
