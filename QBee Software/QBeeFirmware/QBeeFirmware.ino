#include <Adafruit_NeoPixel.h>


#define PIXEL_PIN_1 15
#define PIXEL_PIN_2 14

#define PWM_PIN_1 2
#define PWM_PIN_2 4
#define PWM_PIN_3 5



// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(8, PIXEL_PIN_1, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(8, PIXEL_PIN_2, NEO_GRB + NEO_KHZ800);

// constants won't change. Used here to set a pin number:
const int ledPin1 = PWM_PIN_1;
const int ledPin2 = PWM_PIN_2;
const int ledPin3 = PWM_PIN_3;

int ledState = 0;  // ledState used to set the LED

unsigned long previousMillis = 0;  // will store last time LED was updated

const long interval = 1000;  // interval at which to blink (milliseconds)

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }    // prints title with ending line break
  Serial.println("running set up");
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  analogWrite(ledPin1, 127);
  strip1.begin();
  strip1.setBrightness(50);
  strip1.show();  // Initialize all pixels to 'off'
  strip2.begin();
  strip2.setBrightness(50);
  strip2.show();  // Initialize all pixels to 'off'
}

void loop() {

  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    Serial.println("TOGGLE OUTPUT");
    // if the LED is off turn it on and vice-versa:
    if (ledState == 0) {
      ledState = 50;
    } else {
      ledState = 0;
    }

    // set the LED with the ledState of the variable:
    analogWrite(ledPin3, ledState);
    analogWrite(ledPin1,10);
  }

  //  rainbowCycle(strip2,20);
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(Adafruit_NeoPixel strip, uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) {  // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(strip, ((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(Adafruit_NeoPixel strip, byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
