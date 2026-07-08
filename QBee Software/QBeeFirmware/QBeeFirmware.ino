// #include <Adafruit_NeoPixel.h>
// #include "pwm_driver.h"
// #include "neopixel_driver.h"
#include "ws2812b_dma_driver.h"

ws2812b_strip_t strip;
ws2812b_strip_t strip2;

#define PIXEL_PIN_1 15
#define PIXEL_PIN_2 14

#define PWM_PIN_2 4
#define PWM_PIN_3 5

// constants won't change. Used here to set a pin number:
const int ledPin1 = 2;
const int ledPin2 = PWM_PIN_2;
const int ledPin3 = PWM_PIN_3;

int ledState = 0;  // ledState used to set the LED

unsigned long previousMillis = 0;  // will store last time LED was updated

const long interval = 500;  // interval at which to blink (milliseconds)

// neopixel_strip_t strip;
// neopixel_strip_t strip2;

// Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(8, PIXEL_PIN_1, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  // while (!Serial) {
  //   ;  // wait for serial port to connect. Needed for native USB port only
  // }    // prints title with ending line break
  Serial.println("running set up");
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);

  ws2812b_init(&strip, PIXEL_PIN_1, 8, WS2812B_GRB, NULL);
  ws2812b_init(&strip2, PIXEL_PIN_2, 8, WS2812B_GRB, NULL);
  ws2812b_set_brightness(&strip, 64);

  ws2812b_fill(&strip, 0, 0, 0);
  ws2812b_fill(&strip2, 0, 0, 0);
  ws2812b_show(&strip);
  ws2812b_show(&strip2);
}
uint8_t ledNr = 0;
uint32_t colour =ws2812b_color(255, 255,255);
void loop() {

  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED

    previousMillis = currentMillis;
    Serial.print("ledNr: ");
    Serial.println(ledNr);
    // if the LED is off turn it on and vice-versa:
    ws2812b_set_pixel_color(&strip, ledNr, colour);
    if (ledState == 64) {
      ledState = 200;

    } else {
      


      ledState = 64;
    }

    // CPU is free while LEDs update
    if (ws2812b_is_ready(&strip)) {
      // Start next update
      Serial.println("updating leds: ");

      ws2812b_show(&strip);
    } else {
      Serial.println("leds not ready");
    }
    ledNr += 1;
    if (ledNr >= 8) {
      ledNr = 0;
      Serial.print("colour");
      Serial.println(colour);
      if (colour == 0) {
      colour = ws2812b_color(255, 0,0);
      }else {
      colour = ws2812b_color(0, 0,0);
      
      }
    }
    // neopixel_show(&strip);
    // strip1.show();
    // pwm_set_duty(PWM_CHANNEL_2, ledState);  // 50%
    // pwm_set_duty(PWM_CHANNEL_3, 64);   // 25%
    //   // set the LED with the ledState of the variable:
    // analogWrite(ledPin2, ledState);
    // analogWrite(ledPin3, 255 - ledState);
  }

  //  rainbowCycle(strip2,20);
}

// Slightly different, this makes the rainbow equally distributed throughout
// void rainbowCycle(Adafruit_NeoPixel strip, uint8_t wait) {
//   uint16_t i, j;

//   for (j = 0; j < 256 * 5; j++) {  // 5 cycles of all colors on wheel
//     for (i = 0; i < strip.numPixels(); i++) {
//       strip.setPixelColor(i, Wheel(strip, ((i * 256 / strip.numPixels()) + j) & 255));
//     }
//     strip.show();
//     delay(wait);
//   }
// }

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
// uint32_t Wheel(Adafruit_NeoPixel strip, byte WheelPos) {
//   WheelPos = 255 - WheelPos;
//   if (WheelPos < 85) {
//     return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
//   }
//   if (WheelPos < 170) {
//     WheelPos -= 85;
//     return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
//   }
//   WheelPos -= 170;
//   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
// }
