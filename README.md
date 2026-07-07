# QBee LED Controller

A LEGO-stud-compatible LED controller board (Rev 1.0) built around the ATSAMD11C14A,
powered over USB-C. It drives up to two WS2812B (NeoPixel) strips and three
single-color LED channels.

![Board render](lego%20light%20controllerRT.png)

## Hardware overview

| Part | Function |
|---|---|
| U1 — ATSAMD11C14A (SOIC-14) | MCU, 16KB flash / 4KB RAM, native USB |
| U2 — AP2111H-3.3 | 3.3V LDO for the MCU |
| P1 — USB-C (USB4110) | Power (5V) + USB 2.0 data; 5.1k CC pulldowns identify the board as a power sink |
| Q1–Q3 — BC847 | Low-side switches for the three LED channels (1k base, 10k pulldown) |
| R10, R11 — 470Ω | Series resistors on the WS2812B data lines |

### Connectors / IO

| Connector | Pads | Signal | MCU pin | Arduino pin |
|---|---|---|---|---|
| J1 | 1: +5V, 2: switched GND | LED1 channel | PA02 | 2 |
| J2 | 1: +5V, 2: switched GND | LED2 channel | PA04 | 4 |
| J3 | 1: +5V, 2: switched GND | LED3 channel | PA05 | 5 |
| J4 | 1: +5V, 2: data, 3: GND | WS2812B strip (DL1) | PA14 | 14 |
| J5 | 1: +5V, 2: data, 3: GND | WS2812B strip (DL2) | PA15 | 15 |
| J7 | 1: 3V3, 2: RESET, 3: SWCLK, 4: SWDIO, 5: NC, 6: GND | SWD programming header (DNP) | PA28/PA30/PA31 | — |
| TP1 | test point | EXT1 spare GPIO | PA08 | 8 |
| TP2 | test point | EXT2 spare GPIO | PA09 | 9 |

The LED channels (J1–J3) switch the LED's ground side through a transistor:
connect an LED (with its own series resistor if needed) between pad 1 (+5V)
and pad 2. A HIGH on the MCU pin turns the channel on.

Notes per pin:

- **PA02 (LED1, pin 2)** has no timer output — hardware PWM is not possible.
  `analogWrite()` on this pin uses the true 10-bit DAC; the transistor makes
  this behave as a crude analog dimmer only. Prefer channels LED2/LED3 for
  smooth PWM dimming.
- **PA04/PA05 (LED2/LED3)** have TC1 waveform outputs (WO0/WO1) — used by
  `pwm_driver` channels 0 and 1.
- **PA14/PA15 (strips)** also carry TC1 outputs; do not `analogWrite()` them.

## Arduino environment setup

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (2.x).
2. Add the Fab SAM core: **File → Preferences → Additional boards manager URLs**:

   ```
   https://raw.githubusercontent.com/qbolsee/ArduinoCore-fab-sam/master/json/package_Fab_SAM_index.json
   ```

3. **Tools → Board → Boards Manager**, search "Fab SAM", install
   **Fab SAM core for Arduino** (this project was built with 1.12.0).
4. Select **Tools → Board → Fab SAM boards → Generic D11C14A**.

### Board settings (Tools menu)

| Setting | Value | Notes |
|---|---|---|
| Clock Source | `INTERNAL_USB_CALIBRATED_OSCILLATOR` | Crystalless; calibrates 48MHz from USB. Switch to `INTERNAL_OSCILLATOR` if USB is disabled. |
| Bootloader Size | `4KB_BOOTLOADER` | Leaves 12KB for the application |
| Serial Config | `NO_UART_ONE_WIRE_ONE_SPI` | Saves ~1.5KB vs the default; USB `Serial` still works. The hardware UART (`Serial1`) is not wired to anything on this board. |
| USB Config | `CDC_ONLY` | Needed for `Serial` debugging and one-click uploads |
| Float | default | |
| Timer PWM Frequency | default (732.4Hz) | Only affects `analogWrite()` |

### Uploading

Normal uploads go over USB via the SAM-BA bootloader: just press Upload —
the IDE opens the port at 1200 baud to reset the board into the bootloader
automatically (this requires the running sketch to have USB CDC enabled).

If the chip is blank, or the sketch was built without CDC, flash via the SWD
pads (J7) with a CMSIS-DAP programmer and
[edbg](https://github.com/ataradov/edbg), or use the IDE's
*Burn Bootloader* with the Fab programmer selected. Once the bootloader is
on, USB uploads work as above.

## Firmware (`QBee Software/QBeeFirmware`)

| File | Purpose |
|---|---|
| `QBeeFirmware.ino` | Main sketch |
| `ws2812b_dma_driver.c/h` | WS2812B driver using **TCC0** PWM + DMA. Non-blocking: `ws2812b_show()` starts a hardware transfer and returns immediately. Uses TCC0's double-buffered `CCB` compare register so bit timing is glitch-free and immune to interrupts/USB load. |
| `pwm_driver.c/h` | LED-channel PWM using **TC1** (channels 0/1 → pins 4/5). TC2 exists but its outputs land on the SWD pins only. Costs no flash while unused (linker GC). |
| `neopixel_driver.c/h` | Older bit-banged WS2812B driver (blocks with interrupts off; superseded by the DMA driver) |

### Timer budget

| Peripheral | Owner |
|---|---|
| TCC0 | `ws2812b_dma_driver` (do not combine with `analogWrite`/`pwm_driver` on TCC0 pins) |
| TC1 | `pwm_driver` channels 0/1 (pins 4/5). Also claimed by `tone()` — don't use both. |
| TC2 | `pwm_driver` channels 2/3 — outputs only on SWD pins PA30/PA31, effectively unusable |
| SysTick | `millis()`/`micros()`/`delay()` (not a shared peripheral) |
| DMAC ch 0 | `ws2812b_dma_driver` |

### Flash budget

The app space is 12288 bytes. Measured levers if space runs out:

| Change | Approx. saving |
|---|---|
| Serial Config → `NO_UART...` | 1.5KB (free — `Serial1` unused) |
| USB Config → `WITHOUT_CDC` (drop `Serial` prints) | +2KB |
| USB Config → `USB_DISABLED` (also set Clock → `INTERNAL_OSCILLATOR`) | +4KB more |
| Bootloader → `NO_BOOTLOADER` (SWD uploads only) | +4KB of app space |

## Known hardware issues (Rev 1.0)

- **No level shifter on the WS2812B data lines.** Data is 3.3V while the
  strip runs at 5V; the strip's VIH spec is 0.7×VDD = 3.5V, so operation
  relies on real-world margin. Symptom: strips work on a USB-C wall supply
  but fail or glitch on some PC ports (slightly higher/noisier VBUS).
  Workaround: a silicon diode in series with the strip's +5V feed (drops
  strip VDD to ~4.4V). Proper fix for Rev 2: a 74AHCT1G125 / SN74LV1T125
  buffer on each data line, powered from 5V.
- **LED1 (PA02) has no hardware PWM** — see pin notes above.
