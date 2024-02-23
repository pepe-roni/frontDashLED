#include <Wire.h> //Needed for I2C to GNSS
#include <FastLED.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 

#define LED_COUNT 144 //number of leds in that array
#define LED_PIN 32 //rgb led attached to D32
#define SERIALECHO false
#define TESTDATA false
// #define SWITCHPIN 

SFE_UBLOX_GNSS myGNSS;
CRGBArray<LED_COUNT> leds;

uint16_t hueValue = 0;
int setBrightness = 155; //max 255
// uint32_t color = leds.ColorHSV(hueValue);

void(* resetFunc) (void) = 0;

void setup()
{
  FastLED.addLeds<NEOPIXEL,LED_PIN>(leds, LED_COUNT);
  Wire.begin(22,23);
  Wire.setClock(200000);

  Serial.begin(115200);
  // while (!Serial);
  Serial.println(F("Tesla GPS DR LEDs"));

  if (myGNSS.begin() == false){ //Connect to the u-blox module using Wire port
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    resetFunc();
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  if (myGNSS.getEsfInfo()){

    Serial.print(F("Fusion Mode: "));
    Serial.println(myGNSS.packetUBXESFSTATUS->data.fusionMode);  

    if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 1){
      Serial.println(F("Fusion Mode is Initialized!"));  
    }
    else {
      Serial.println(F("Fusion Mode is either disabled or not initialized!"));  
      Serial.println(F("Please see the previous example for more information."));
    }
  }

  myGNSS.setAutoHNRATT(false); //Make sure auto HNR attitude messages are disabled
  myGNSS.setAutoHNRINS(false); //Make sure auto HNR vehicle dynamics messages are disabled
  myGNSS.setAutoHNRPVT(false); //Make sure auto HNR PVT messages are disabled

  myGNSS.setNavigationFrequency(18); //Set output to 5 times a second
  myGNSS.setHNRNavigationRate(30);
  Serial.println("Current update rate: 30Hz");
  Serial.print(myGNSS.getHour()); //universal time
  Serial.print(":");
  Serial.print(myGNSS.getMinute());
  Serial.println();
  startupAnimation();
  if(myGNSS.getHour()>2 && myGNSS.getHour()<15){
    setBrightness = 40;
  }
  else
    setBrightness = 255;
}

unsigned long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
unsigned long lastTime2 = 0;
unsigned long lastTime3 = 0;

float velocity  = 0;
float oldVelocity = 0;
float accel = 0;
bool sportMode = false;


void loop()
{
  if (TESTDATA){
    if (millis() - lastTime > 30){
      if (SERIALECHO){
        Serial.print("Lasttime: ");
        Serial.print(millis()-lastTime);      
      }
      lastTime = millis(); //Update the timer
      if (velocity < 35)
        velocity = velocity+=0.6; 
      else if (velocity < 60)
        velocity = velocity+=0.5; 
      else if (velocity < 80)
        velocity = velocity+=0.4; 
      else if (velocity < 110)
        velocity = velocity+=0.3; 
      else if (velocity < 150)
        velocity = velocity+=0.1;
      else if (velocity > 150)
        velocity = 0;
      
      if (SERIALECHO){
        Serial.print(F(" | Ground Speed (MPH): "));
        Serial.print(velocity);
        Serial.print(F(" | Lat Acc (Gs): "));
        Serial.print(accel);
        Serial.println();
      }
      
    }
  }
  else{
    if (millis() - lastTime > 10){
      if (SERIALECHO){
        Serial.print("Lasttime: ");
        Serial.print(millis()-lastTime);      
      }
      lastTime = millis(); //Update the timer

      if (myGNSS.getHNRPVT(50) == true){// Request HNR PVT data using a 50ms timeout
        velocity = myGNSS.packetUBXHNRPVT->data.gSpeed;
        if (SERIALECHO){
          Serial.print(F(" | Ground Speed (MPH): "));
          Serial.print(velocity/447.04);
          Serial.print(F(" | Lat Acc (Gs): "));
          Serial.print(accel);
        }
      }
      if(SERIALECHO)
        Serial.println();
    }
  }

  if(millis()-lastTime2 >15){
    lastTime2 = millis();
    if(TESTDATA)
      fastLEDHandler(velocity, false);
    else
      fastLEDHandler(velocity/447.04, false);
  }

  if(millis()-lastTime3 > 30){ //check acceleration every 30ms
    if(TESTDATA)
      accel = (((velocity - oldVelocity)/2.237)/((millis()-lastTime3)/1000.0))/9.81; //conv to m/s, conv to s, conv to g's
    else
      accel = (((velocity - oldVelocity)/(2.237*447.04))/((millis()-lastTime3)/1000.0))/9.81;

    oldVelocity = velocity; 
    lastTime3 = millis();
  }
  // fastLEDHandler(velocity/447.04, false);
}

void fastLEDHandler(float velocity, bool usePalette){
  int firstPixelHue;
  int pixelHue;
  float index; 

  CRGBPalette16 myPalette(
    CRGB::Navy, //0
    CRGB::Turquoise, //20
    CRGB::Yellow, //40
    CRGB::LawnGreen, //60
    CRGB::Orange, //80
    CRGB::Red, //100
    CRGB::Magenta, //120
    CRGB::DarkViolet, //140
    CRGB::Purple, //160
    CRGB::Black,
    CRGB::Black,
    CRGB::Black,
    CRGB::Black,
    CRGB::Black,
    CRGB::Black,
    CRGB::Black
  );
  
  if (usePalette)
    index = map(velocity*100.0, 0, 16000, 0, 2550)/10.0;
  else{
    if (velocity < 60){
      firstPixelHue = map(velocity*100.0, 0, 6000, 16000,8000)/100.0; //velocity to hue
    }
    else if (velocity < 100){
      firstPixelHue = map(velocity*100.0, 6000, 10000, 8000, 0)/100.0;
    }
    else{
      firstPixelHue = map(velocity*100.0, 10000, 14000, 0, -7000)/100.0;
    }
  }

  for(int i = 0; i < LED_COUNT/2+1; i++) { 
    int pixelHue = firstPixelHue + (i * 255*(0.2 + 0.2*velocity/150) / LED_COUNT);  //multiplier increase shows higher variance
    // leds.fadeToBlackBy(40);
    if (usePalette)
      leds[i] = ColorFromPalette(myPalette,index);
    else
      leds[LED_COUNT/2 - i] = CHSV(pixelHue,255,setBrightness);

    leds(LED_COUNT/2,LED_COUNT-1) = leds(LED_COUNT/2 - 1 ,0);
    // FastLED.show();
  }
  // if(velocity>59.50 && velocity<60.50)
  FastLED.show();
}

void startupAnimation(){
  int cycles = 0;
  static uint8_t hue;

  while(cycles<5){
    for(int i = 0; i < LED_COUNT/2; i++) {   
      leds.fadeToBlackBy(40);
      leds[i] = CHSV(hue++,255,255);
      leds(LED_COUNT/2,LED_COUNT-1) = leds(LED_COUNT/2 - 1 ,0);
      FastLED.delay(12-(cycles*10/5));
    }
    cycles+=1;
  }
  for(int i = 0; i < LED_COUNT; i++) {  
    leds.fadeToBlackBy(5);
    FastLED.show();
  }
}
