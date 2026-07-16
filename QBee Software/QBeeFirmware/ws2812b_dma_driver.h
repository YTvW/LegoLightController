/**
 * @file ws2812b_dma_driver.h
 * @brief WS2812B driver using TCC0+DMA for SAMD11C / SAMD21
 *
 * Uses TCC0 in PWM mode with DMA for background LED updates.
 * Zero CPU utilization during data transmission. DMA writes go to the
 * double-buffered CCB register (latched at each period boundary), which
 * the plain TC peripherals lack - so TCC0 is required for glitch-free
 * bit timing.
 *
 * Supported chips (selected automatically at compile time):
 *   - SAMD11C: TCC0 WO[0]/WO[1] pins, up to 2 strips
 *   - SAMD21:  TCC0 CC0..CC3 pins, up to 4 strips (set WS2812B_MAX_STRIPS)
 *
 * NOTE: claims TCC0, so it cannot be combined with analogWrite() on TCC0
 * pins. pwm_driver is safe to combine: it uses TC1/TC2 (SAMD11) or
 * TC3/TC4 (SAMD21).
 *
 * Timing: 800 kHz (1.25µs period = 60 ticks @ 48MHz)
 *   - '0' bit: ~290ns high (14 ticks), ~940ns low
 *   - '1' bit: ~790ns high (38 ticks), ~460ns low
 */

#ifndef WS2812B_DMA_DRIVER_H
#define WS2812B_DMA_DRIVER_H

#include <Arduino.h>
#include <sam.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum pixels supported (each pixel needs 24 bytes in DMA buffer for RGB)
#ifndef WS2812B_MAX_PIXELS
#define WS2812B_MAX_PIXELS 16
#endif

// Maximum number of concurrent, independent strips. Each strip gets its
// own DMA channel/descriptor/buffer and its own TCC0 CC[x] compare
// channel. On the SAMD11C (QBee board) that means PA14 and PA15
// (TCC0 WO[0]/WO[1]), max 2. On the SAMD21 TCC0 has 4 compare channels,
// so this can be raised to 4 if each strip's pin uses a distinct CC.
#ifndef WS2812B_MAX_STRIPS
#define WS2812B_MAX_STRIPS 2
#endif

// TCC PWM timing constants @ 48MHz
#define WS2812B_PERIOD_TICKS    60   // 1.25µs = 800 kHz
#define WS2812B_T0H_TICKS       14   // ~290ns for '0' bit high
#define WS2812B_T1H_TICKS       38   // ~790ns for '1' bit high

// A transfer stuck in BUSY longer than this is force-recovered
// (worst case frame is ~0.7ms, so 5ms means something went wrong)
#define WS2812B_XFER_TIMEOUT_MS 5

// Pixel type flags (compatible with neopixel_driver)
#define WS2812B_RGB   0x00   // RGB byte order
#define WS2812B_GRB   0x01   // GRB byte order (most common)
#define WS2812B_BRG   0x02   // BRG byte order
#define WS2812B_RBG   0x03   // RBG byte order
#define WS2812B_RGBW  0x10   // RGBW (4 bytes per pixel)
#define WS2812B_GRBW  0x11   // GRBW (4 bytes per pixel)

/**
 * @brief WS2812B strip state
 */
typedef enum {
    WS2812B_STATE_IDLE,       // Ready for new transfer
    WS2812B_STATE_BUSY,       // DMA transfer in progress
    WS2812B_STATE_RESET       // Waiting for reset period
} ws2812b_state_t;

/**
 * @brief WS2812B strip structure
 */
typedef struct {
    uint8_t pin;               // Arduino pin number
    uint16_t numPixels;        // Number of pixels
    uint8_t pixelType;         // Color order flags
    uint8_t bytesPerPixel;     // 3 for RGB, 4 for RGBW
    uint8_t brightness;        // Global brightness (0-255)
    uint8_t *pixels;           // Pixel color buffer (RGB/RGBW data)
    volatile ws2812b_state_t state;  // Current transfer state

    // --- driver-private: set by ws2812b_init(), do not modify ---
    uint8_t _slot;              // This strip's DMA channel / buffer index
    uint8_t _woChannel;         // TCC0 CC[x]/CCB[x] channel driving its pin
    uint32_t _busyStartMs;      // millis() when its current transfer started
} ws2812b_strip_t;

/**
 * @brief Initialize WS2812B DMA driver
 *
 * Sets up TCC0 for PWM output and configures DMA channel.
 * The output pin must be capable of TCC0 WO[x] function.
 *
 * SAMD11C valid pins for TCC0:
 *   - PA04: TCC0/WO[0] mux E (Arduino pin depends on board variant)
 *   - PA05: TCC0/WO[1] mux E
 *   - PA14: TCC0/WO[0] mux F
 *   - PA15: TCC0/WO[1] mux F
 *
 * SAMD21 valid pins for TCC0 (CC channel in parentheses; concurrent
 * strips must each use a distinct CC channel):
 *   - CC0: PA04/PA08 (mux E), PA14/PA22, PB10 (mux F), PB30 (mux E)
 *   - CC1: PA05/PA09 (mux E), PA15/PA23, PB11 (mux F), PB31 (mux E)
 *   - CC2: PA10/PA12/PA16/PA18/PA20 (mux F)
 *   - CC3: PA11/PA13/PA17/PA19/PA21 (mux F)
 *
 * @param strip Pointer to strip structure
 * @param pin Arduino pin number (must support TCC0 output)
 * @param numPixels Number of pixels in strip
 * @param pixelType Pixel type flags (e.g., WS2812B_GRB)
 * @param buffer External pixel buffer (NULL to use internal buffer)
 * @return true if initialization successful, false if the pin is not
 *         TCC0-capable or WS2812B_MAX_STRIPS strips are already in use
 */
bool ws2812b_init(ws2812b_strip_t *strip, uint8_t pin, uint16_t numPixels,
                  uint8_t pixelType, uint8_t *buffer);

/**
 * @brief Set brightness for the strip
 *
 * @param strip Pointer to strip structure
 * @param brightness Brightness level (0-255)
 */
void ws2812b_set_brightness(ws2812b_strip_t *strip, uint8_t brightness);

/**
 * @brief Set a pixel color (RGB)
 *
 * @param strip Pointer to strip structure
 * @param pixel Pixel index (0 to numPixels-1)
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 */
void ws2812b_set_pixel_rgb(ws2812b_strip_t *strip, uint16_t pixel,
                           uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set a pixel color (RGBW)
 *
 * @param strip Pointer to strip structure
 * @param pixel Pixel index (0 to numPixels-1)
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @param w White component (0-255)
 */
void ws2812b_set_pixel_rgbw(ws2812b_strip_t *strip, uint16_t pixel,
                            uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/**
 * @brief Set a pixel color using packed 32-bit value
 *
 * @param strip Pointer to strip structure
 * @param pixel Pixel index
 * @param color Packed color (0x00RRGGBB or 0xWWRRGGBB for RGBW)
 */
void ws2812b_set_pixel_color(ws2812b_strip_t *strip, uint16_t pixel, uint32_t color);

/**
 * @brief Get a pixel's current color
 *
 * @param strip Pointer to strip structure
 * @param pixel Pixel index
 * @return Packed color value
 */
uint32_t ws2812b_get_pixel_color(ws2812b_strip_t *strip, uint16_t pixel);

/**
 * @brief Clear all pixels (set to black)
 *
 * @param strip Pointer to strip structure
 */
void ws2812b_clear(ws2812b_strip_t *strip);

/**
 * @brief Fill all pixels with a single color
 *
 * @param strip Pointer to strip structure
 * @param r Red component
 * @param g Green component
 * @param b Blue component
 */
void ws2812b_fill(ws2812b_strip_t *strip, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Update the LED strip (non-blocking DMA transfer)
 *
 * Encodes pixel data to DMA buffer and starts transfer.
 * Returns immediately while DMA runs in background.
 *
 * @param strip Pointer to strip structure
 * @return true if transfer started, false if still busy
 */
bool ws2812b_show(ws2812b_strip_t *strip);

/**
 * @brief Check if transfer is complete
 *
 * @param strip Pointer to strip structure
 * @return true if idle and ready for new transfer
 */
bool ws2812b_is_ready(ws2812b_strip_t *strip);

/**
 * @brief Wait for current transfer to complete
 *
 * Blocks until DMA transfer finishes.
 *
 * @param strip Pointer to strip structure
 */
void ws2812b_wait(ws2812b_strip_t *strip);

/**
 * @brief Number of transfers that ended abnormally (error/timeout)
 *
 * Increments whenever a transfer had to be force-recovered instead of
 * completing normally. Useful to print when debugging strip lockups:
 * a steadily rising count means DMA transfers are failing.
 *
 * @return Abnormal transfer count since boot
 */
uint16_t ws2812b_get_recovery_count(void);

/**
 * @brief Create a packed color value
 *
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return Packed 32-bit color value
 */
static inline uint32_t ws2812b_color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

/**
 * @brief Create a packed RGBW color value
 *
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @param w White component (0-255)
 * @return Packed 32-bit color value
 */
static inline uint32_t ws2812b_color_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    return ((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

#ifdef __cplusplus
}
#endif

#endif // WS2812B_DMA_DRIVER_H
