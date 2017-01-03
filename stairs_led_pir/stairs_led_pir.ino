#include "FastLED.h"                                          // FastLED library. Please use the latest development version.

#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif
// sensor defs
#define SENSOR_BOTTOM_PIN 6
#define SENSOR_TOP_PIN 5
#define SENSOR_BOTTOM_LED_PIN 2
#define SENSOR_TOP_LED_PIN 3

#define SENSOR_ON "y"
#define SENSOR_OFF "n"
#define SENSOR_DELAY_AFTER_DETECTION 30

// LED defs
#define LED_PIN 12                                             // Data pin to connect to the strip.
#define NUM_LEDS 96 //30                                           // Number of LED's.
#define BALL_BRIGHTNESS 120
#define NORMAL_BRIGHTNESS 40
#define KEEP_ON_SEC 6

struct PIR{
  String on;
  unsigned long lastRead;
  byte pin;
  String name;
};

// Global variables can be changed on the fly.

struct CRGB leds[NUM_LEDS];                                   // Initialize our LED array.

int thisdelay = 10;
int count=0;
unsigned long READ_INTERVAL=1000;

PIR SENSOR_BOTTOM={SENSOR_OFF, 0, SENSOR_BOTTOM_PIN, "bottom"};
PIR SENSOR_TOP={SENSOR_OFF, 0, SENSOR_TOP_PIN, "top"}; 

void setup()  {
  delay(1000); 
  Serial.begin(9600);

  pinMode(SENSOR_BOTTOM.pin,INPUT);
  pinMode(SENSOR_BOTTOM_LED_PIN, OUTPUT);

  pinMode(SENSOR_TOP.pin,INPUT);
  pinMode(SENSOR_TOP_LED_PIN, OUTPUT);
  
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);  // Use this for WS2801 or APA102
  FastLED.setBrightness(BALL_BRIGHTNESS);
  set_max_power_in_volts_and_milliamps(5, 500);               // FastLED Power management set at 5V, 500mA.
}

void loop()  {
  doSensors();

  doLights();
  
  show_at_max_brightness_for_power();  
}
///////////////// LEDs ///////////////////

void doLights() {
  static boolean upFinished=false;
  static boolean downFinished= false;
  
  EVERY_N_MILLISECONDS(thisdelay) {                           // FastLED based non-blocking delay to update/display the sequence.
    if(SENSOR_BOTTOM.on == SENSOR_ON)  {
      upFinished = false;
    }
    if(SENSOR_TOP.on == SENSOR_ON)  {
      downFinished = false;
    }
    
    if (upFinished == false){
      upFinished = turnOn(true);
    } else if (downFinished == false){
      downFinished = turnOn(false);
    }
//    
//    if (upFinished==true && downFinished==true) {
////      leds[0]=CRGB::Red;
//      upFinished = false;
//      downFinished = false;
//    }
  }
  
}

uint8_t calcPixel(uint8_t position, boolean up){
  if (up == true){
    return position;
  } else {
    return NUM_LEDS -1 - position;
  }
}

uint8_t getFirst(boolean up){
  if (up == true){
    return 0;
  } else {
    return NUM_LEDS - 1;
  }
}

uint8_t getLast(boolean up){
  if (up == true){
    return NUM_LEDS - 1;
  } else {
    return 0;
  }
}

boolean turnOn(boolean up) {
  static uint8_t phase = 0;
  uint8_t first=getFirst(up);
  uint8_t last=getLast(up);
  
  // PHASE 0 : start the ball
  if (phase == 0){
    static uint8_t easeInVal_1  = 0;
    easeInVal_1++;
  
    uint8_t lerpVal = lerp8by8(0, NUM_LEDS, easeInVal_1);

    uint8_t pixel = calcPixel(lerpVal, up);
    leds[pixel] = CRGB::White;//CRGB(60,60,60);//CRGB::Green;
    
    if (pixel==last){
      easeInVal_1=0;
      phase=1;
    }
  }
  
  // PHASE 0,1 : dim leds to normal level
  if (phase == 0 || phase ==1 ){
    for (int i=0;i<NUM_LEDS; i++){
      uint8_t pixel = calcPixel(i, up);
      uint8_t avgLight = leds[pixel].getAverageLight();
      if (avgLight > NORMAL_BRIGHTNESS){
        leds[pixel].subtractFromRGB(5);
      }
    }
   
  }
  
  if (phase == 1 && leds[last].getAverageLight() <= NORMAL_BRIGHTNESS)
  {
    phase=2;
    delay(KEEP_ON_SEC*1000);
  }
  
// PHASE 2 : dim all leds after delay
  if (phase == 2){
    
    static uint8_t turnOffPixel  = 1;
    static uint8_t noOfRuns=0;
    noOfRuns++;
    if (noOfRuns >5){
      noOfRuns=0;
      if (turnOffPixel < NUM_LEDS){
        turnOffPixel++;
      }
    }

    for (int i=0;i<turnOffPixel; i++){
      uint8_t pixel = calcPixel(i, up);
      uint8_t avgLight = leds[pixel].getAverageLight();
      if (avgLight>0){
        leds[pixel].fadeToBlackBy(3);
      }
    }
    delay(20);

    if (leds[last].getAverageLight()<1){
      turnOffPixel=0;
      noOfRuns=0;
      phase=4;
    }
  }

// PHASE 4 : make all black
  if (phase == 4){
//    leds[0]=CRGB::Red;
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      phase=0;
      return true;
  }

 return false;
}

///////////////// SENSORS ///////////////
void doSensors()  {
  EVERY_N_MILLISECONDS(1000) {
    updateLed();
    readSensor(&SENSOR_BOTTOM);
    readSensor(&SENSOR_TOP);
  }
}
int readSensor(PIR *sensor){
  if(digitalRead(sensor->pin)==HIGH)  {
      Serial.println("Somebody is here. "+sensor->name);
      sensor->on = SENSOR_ON;
  } else {
      Serial.println("Nobody. "+sensor->name);
      sensor->on = SENSOR_OFF;
  }
}

int updateLed(){
    if(SENSOR_BOTTOM.on == SENSOR_ON)  {
      digitalWrite(SENSOR_BOTTOM_LED_PIN, HIGH);
    } else  {
      digitalWrite(SENSOR_BOTTOM_LED_PIN, LOW);
    }

    if(SENSOR_TOP.on == SENSOR_ON)  {
      digitalWrite(SENSOR_TOP_LED_PIN, HIGH);
    } else  {
      digitalWrite(SENSOR_TOP_LED_PIN, LOW);
    }
}

