/**
 * Emulate Philips Hue Bridge ; so far the Hue app finds the emulated Bridge and gets its config
 * and switch NeoPixels with it
 **/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <TimeLib.h>
#include <NtpClientLib.h>
#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h> // instead of NeoPixelAnimator branch

#define MAX_LIGHT_HANDLERS 1
#define lightName "Hue RGB Light" // Light name, change this if you se multiple lights for easy identification

#include "LightService.h"

// these are only used in LightHandler.cpp, but it seems that the IDE only scans the .ino and real libraries for dependencies
#include "SSDP.h"
#include <aJSON.h> // Replace avm/pgmspace.h with pgmspace.h there and set #define PRINT_BUFFER_LEN 4096 ################# IMPORTANT

RgbColor red = RgbColor(COLOR_SATURATION, 0, 0);
RgbColor green = RgbColor(0, COLOR_SATURATION, 0);
RgbColor white = RgbColor(COLOR_SATURATION);
RgbColor black = RgbColor(0);

// Uncomment to use WS2812 light strips 
// instead of RGB PWM
//#define USE_WS2812_STRIP
#if defined(USE_WS2812_STRIP)
// Settings for the NeoPixels
#define NUM_PIXELS_PER_LIGHT 10 // How many physical LEDs per emulated bulb
#define pixelCount 30

#define pixelPin 2 // Strip is attached to GPIO2 on ESP-01
NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart1Ws2812xMethod> strip(MAX_LIGHT_HANDLERS * NUM_PIXELS_PER_LIGHT, pixelPin);
#else
// Defines for PWM Mode
#define NUM_PIXELS_PER_LIGHT 1
#define PWM_CHANNELS 3
#define pixelCount 1

class RGBStripe {
public:
  RGBStripe(uint8_t pinRed, uint8_t pinGreen, uint8_t pinBlue) {
    _pins[0] = pinRed;
    _pins[1] = pinGreen;
    _pins[2] = pinBlue;
  }

  void Begin()
  {
    // Setup pins for output
    for (uint8_t pin = 0; pin < PWM_CHANNELS; pin++) {
      pinMode(_pins[pin], OUTPUT);
      analogWrite(_pins[pin], 0);
    }
  }
  
  void SetPixelColor(uint16_t indexPixel, RgbColor updatedColor)
  {
    // Store current color value
    _color[0] = updatedColor.R;
    _color[1] = updatedColor.G;
    _color[2] = updatedColor.B;

    // Update outputs
    for (uint8_t pin = 0; pin < PWM_CHANNELS; pin++) {
      analogWrite(_pins[pin], (int)(_color[pin] * 4.0));
    }
  }

  void Show()
  {
  }

  RgbColor GetPixelColor(uint16_t indexPixel) const
  {
    RgbColor result(_color[0], _color[1], _color[2]);
    return result;
  }

private:
  uint8_t _pins[PWM_CHANNELS];
  uint8_t _color[PWM_CHANNELS];
};

RGBStripe strip(12, 14, 13); // red (D6), green (D5), blue (D7)
#endif
NeoPixelAnimator animator(MAX_LIGHT_HANDLERS * NUM_PIXELS_PER_LIGHT, NEO_MILLISECONDS); // NeoPixel animation management object
LightServiceClass LightService;

HsbColor getHsb(int hue, int sat, int bri) {
  float H, S, B;
  H = ((float)hue) / 182.04 / 360.0;
  S = ((float)sat) / COLOR_SATURATION;
  B = ((float)bri) / COLOR_SATURATION;
  return HsbColor(H, S, B);
}

class PixelHandler : public LightHandler {
  private:
    HueLightInfo _info;
    int16_t colorloopIndex = -1;
  public:
    void handleQuery(int lightNumber, HueLightInfo newInfo, aJsonObject* raw) {
      // define the effect to apply, in this case linear blend
      HslColor newColor = HslColor(getHsb(newInfo.hue, newInfo.saturation, newInfo.brightness));
      HslColor originalColor = strip.GetPixelColor(lightNumber);
      _info = newInfo;

      // cancel colorloop if one is running
      if (colorloopIndex >= 0) {
        animator.StopAnimation(colorloopIndex);
        colorloopIndex = -1;
      }
      if (newInfo.on) {
        if (_info.effect == EFFECT_COLORLOOP) {
          //color loop at max brightness/saturation on a 60 second cycle
          const int SIXTY_SECONDS = 60000;
          animator.StartAnimation(lightNumber, SIXTY_SECONDS, [ = ](const AnimationParam & param) {
            // save off animation index
            colorloopIndex = param.index;

            // progress will start at 0.0 and end at 1.0
            float currentHue = newColor.H + param.progress;
            if (currentHue > 1) currentHue -= 1;
            HslColor updatedColor = HslColor(currentHue, newColor.S, newColor.L);
            RgbColor currentColor = updatedColor;

            for(int i=lightNumber * NUM_PIXELS_PER_LIGHT; i < (lightNumber * NUM_PIXELS_PER_LIGHT) + NUM_PIXELS_PER_LIGHT; i++) {
              strip.SetPixelColor(i, updatedColor);
            }

            // loop the animation until canceled
            if (param.state == AnimationState_Completed) {
              // done, time to restart this position tracking animation/timer
              animator.RestartAnimation(param.index);
            }
          });
          return;
        }
        AnimUpdateCallback animUpdate = [ = ](const AnimationParam & param)
        {
          // progress will start at 0.0 and end at 1.0
          HslColor updatedColor = HslColor::LinearBlend<NeoHueBlendShortestDistance>(originalColor, newColor, param.progress);

          for(int i=lightNumber * NUM_PIXELS_PER_LIGHT; i < (lightNumber * NUM_PIXELS_PER_LIGHT) + NUM_PIXELS_PER_LIGHT; i++) {
            strip.SetPixelColor(i, updatedColor);
          }
        };
        animator.StartAnimation(lightNumber, _info.transitionTime, animUpdate);
      }
      else {
        AnimUpdateCallback animUpdate = [ = ](const AnimationParam & param)
        {
          // progress will start at 0.0 and end at 1.0
          HslColor updatedColor = HslColor::LinearBlend<NeoHueBlendShortestDistance>(originalColor, black, param.progress);
          
          for(int i=lightNumber * NUM_PIXELS_PER_LIGHT; i < (lightNumber * NUM_PIXELS_PER_LIGHT) + NUM_PIXELS_PER_LIGHT; i++) {
            strip.SetPixelColor(i, updatedColor);
          }
        };
        animator.StartAnimation(lightNumber, _info.transitionTime, animUpdate);
      }
    }

    HueLightInfo getInfo(int lightNumber) { return _info; }
};

void setup() {

  // this resets all the neopixels to an off state
  strip.Begin();
  strip.Show();

  // Show that the NeoPixels are alive
  delay(120); // Apparently needed to make the first few pixels animate correctly
  Serial.begin(115200);

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(120);
  wifiManager.autoConnect(lightName);

  while (WiFi.status() != WL_CONNECTED) {
    infoLight(red);
    delay(500);
    Serial.print(".");
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  // Sync our clock
  NTP.begin("pool.ntp.org", 0, true);

  // Show that we are connected
  infoLight(green);
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH

  LightService.begin();

  // setup pixels as lights
  for (int i = 0; i < MAX_LIGHT_HANDLERS && i < pixelCount; i++) {
    LightService.setLightHandler(i, new PixelHandler());
  }

  // We'll get the time eventually ...
  if (timeStatus() == timeSet) {
    Serial.println(NTP.getTimeDateString(now()));
  }
}

void loop() {
  ArduinoOTA.handle();
  
  LightService.update();

  static unsigned long update_strip_time = 0;  //  keeps track of pixel refresh rate... limits updates to 33 Hz
  if (millis() - update_strip_time > 30)
  {
    if ( animator.IsAnimating() ) animator.UpdateAnimations();
    strip.Show();
    update_strip_time = millis();
  }
}

void infoLight(RgbColor color) {
  // Flash the strip in the selected color. White = booted, green = WLAN connected, red = WLAN could not connect
  for (int i = 0; i < pixelCount; i++)
  {
    strip.SetPixelColor(i, color);
    strip.Show();
    delay(250);
    strip.SetPixelColor(i, black);
    strip.Show();
  }
}
