
#include <Adafruit_NeoPixel.h>
#include <Filters.h>

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#include "config.h"

// pins
const int sensorPin = A2;    // select the input pin for the potentiometer
const int excitationPin = 4;      // select the pin for the LED
const int ledPin = 13;
const int neoPixelPin = 33;
const int neoPixelCount = 9;

const int maxDepthMm = 80;
const int minDepthMm = 10;

// variable to store the value coming from the sensor
int sensorValue = 0;
// variable to store the calculated depth
int depthMm = 0;


// set up the depth_mm feed
AdafruitIO_Feed *depth_mm_feed = io.feed("h20_depth_mm");
AdafruitIO_Feed *vbatt_feed = io.feed("vbatt");
AdafruitIO_Feed *filtered_adc_feed = io.feed("filt_adc");

// Conversion factor for micro seconds to seconds
#define uS_TO_S_FACTOR 1000000ULL  
// the amount of time the sensor will go to sleep after publishing a measurement
unsigned int secondsToSleep = 600;
// low-pass filter cutoff frequency (Hz)
float lpfCutoffFreqHz = 1.0;    
// how long to sample the sensor before publishing the measurement
const unsigned long measurement_period_ms = 5000;

// create a NeoPixel strip instance
Adafruit_NeoPixel strip(neoPixelCount, neoPixelPin, NEO_RBGW + NEO_KHZ800);
// create a one pole (RC) lowpass filter
FilterOnePole filterOneLowpass( LOWPASS, lpfCutoffFreqHz );

void setup() {
  // declare the excitationPin as an output:
  pinMode(excitationPin, OUTPUT);
  // turn on the LED so we can see when the sensor wakes up
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // setup the NeoPixel strip
  strip.begin();
  strip.show();
  strip.setBrightness(50);

  // Fill along the length of the strip in various colors...
  colorWipe(strip.Color(255,   0,   0), 50); // Red

  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while (!Serial);
  
  colorWipe(strip.Color(  0, 255,   0), 50);

  // connect to io.adafruit.com
  io.connect();
  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.println(io.statusText());
    delay(500);
  }

  Serial.println();
  Serial.println(io.statusText());
  Serial.println("Sleep val=" + String(secondsToSleep));
  
  colorWipe(strip.Color(  0,   0, 255), 50); // Blue
}

void loop() {
  // read the sensor value with no excitation
  int noiseValue = analogRead(sensorPin);  
  // turn on excitation
  digitalWrite(excitationPin, HIGH);
  // let things settle
  delayMicroseconds(50);
  // read the value from the sensor:
  int drivenValue = analogRead(sensorPin);
  // make it relative to the first point to reject low-frequency noise
  sensorValue = drivenValue - noiseValue;
  // reject low frequency noise further with a LPF
  filterOneLowpass.input(sensorValue);
  int filteredValue = filterOneLowpass.output();
  // turn off excitation
  digitalWrite(excitationPin, LOW);
  // calaculate something proportional to capacitance (doesn't work very well, so the depth is just mapped to the ADC linearly below)
  int cap = 1.0f / (4095.0/filteredValue - 1)*1000;
  // calculate depth as a linear function of the ADC (not super accurate, but close enough) using hardcoded offset and sensitivity found empirically
  int depthMm = (filteredValue - 210)/23;
  updateNeopixelLevel(depthMm);
  Serial.println(String(depthMm) + "    " + String(cap) + "  " + String(filteredValue) + "  " + String(sensorValue) + "  " + String(drivenValue) + "  " + String(noiseValue));
  updateFeed(depthMm, filteredValue);
}

void updateFeed(int depth, int filteredAdcValue)
{
  static unsigned long lastRefreshTime = 0;

  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  // sample the sensor for a while to allow time for filtering before publishing the measurement
  if(millis() - lastRefreshTime >= measurement_period_ms)
  {
    lastRefreshTime += measurement_period_ms;
    // read the battery voltage
    uint batt_adc = analogRead(A13);
    float vbatt = 2 * batt_adc * 3.3 * 1.1 / 4095.0;
    Serial.println("publishing:" +  String(depth) + "mm  " + vbatt + "V (" + batt_adc + ")");

    // publish values to Adafruit IO
    vbatt_feed->save(vbatt);
    depth_mm_feed->save(depth);
    filtered_adc_feed->save(filteredAdcValue);        
    
    if(secondsToSleep == 0)
    {
      Serial.println("sleep disabled");
    }
    else
    {
      sleep();
    }
  }
}

void sleep()
{
    /*
    First we configure the wake up source
    We set our ESP32 to wake up every 5 seconds
    */
    Serial.println("sleeping for " + String(secondsToSleep) + " seconds...");
    esp_sleep_enable_timer_wakeup(secondsToSleep * uS_TO_S_FACTOR);

    /*
    Next we decide what all peripherals to shut down/keep on
    By default, ESP32 will automatically power down the peripherals
    not needed by the wakeup source, but if you want to be a poweruser
    this is for you. Read in detail at the API docs
    http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
    Left the line commented as an example of how to configure peripherals.
    The line below turns off all RTC peripherals in deep sleep.
    */
    //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    //Serial.println("Configured all RTC Peripherals to be powered down in sleep");
  
    /*
    Now that we have setup a wake cause and if needed setup the
    peripherals state in deep sleep, we can now start going to
    deep sleep.
    In the case that no wake up sources were provided but deep
    sleep was started, it will sleep forever unless hardware
    reset occurs.
    */
    Serial.println("Going to sleep now");
    Serial.flush(); 
    esp_deep_sleep_start();
}

void updateNeopixelLevel(int depth)
{
  depth = (depth < minDepthMm) ? 0 : depth - minDepthMm;
  int sublevel = map(depth, 0, maxDepthMm - minDepthMm, 0, strip.numPixels()*256);
  
  for(int i=0; i<strip.numPixels(); i++)
  {
    int pixel;

    if(sublevel > 255)
      pixel = 255;
    else
      pixel = sublevel;

    strip.setPixelColor(strip.numPixels()-i-1, strip.Color(pixel,   0,   0));
    if(sublevel > pixel)
      sublevel -= pixel;
    else
      sublevel = 0;
    
  }
  strip.show();
}

// Some functions of our own for creating animated effects -----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}
