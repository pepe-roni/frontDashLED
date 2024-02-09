#include <Wire.h> //Needed for I2C to GNSS
#include <Adafruit_NeoPixel.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 

#define LED_COUNT 144 //number of leds in that array
#define LED_PIN 32 //rgb led attached to D3
#define SERIALECHO true

SFE_UBLOX_GNSS myGNSS;
Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB +NEO_KHZ800);

uint16_t hueValue = 0;
uint32_t color = leds.ColorHSV(hueValue);

void setup()
{
  leds.begin();
  leds.show(); 
  leds.setBrightness(155);
  Wire.begin(22,23);
  Wire.setClock(400000);
  
  Serial.begin(500000);
  while (!Serial);
  Serial.println(F("Tesla GPS DR LEDs"));
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
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
  
  myGNSS.setNavigationFrequency(20); //Set output to 5 times a second
  uint8_t rate = myGNSS.getNavigationFrequency(); //Get the update rate of this module
  
//  uint8_t fixType = myGNSS.getFixType();

  Serial.print("Current update rate: ");
  Serial.println(rate);

  // uint16_t rate = myGNSS.getMeasurementRate(); //Get the measurement rate of this module
  // Serial.print("New measurement interval (ms): ");
  // Serial.println(rate);

  // myGNSS.setMeasurementRate(50); //Produce a measurement every 1000ms
  // myGNSS.setNavigationRate(2); 
  // uint8_t rate = myGNSS.getNavigationRate(); //Get the navigation rate of this module
  // Serial.print("New navigation ratio (cycles): ");
  // Serial.print(rate); 
  // Serial.println();
  startupAnimation();
}

unsigned long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

void loop()
{
  if (millis() - lastTime > 15)
  {
    Serial.print("Lasttime: ");
    Serial.print(millis()-lastTime);
    lastTime = millis(); //Update the timer

    float velocity = myGNSS.getGroundSpeed();
    Serial.print(F(" | Ground Speed (MPH): "));
    Serial.print(velocity/447.04);
  //   if (myGNSS.getNAVVELNED())
  // {
  //   // Serial.print(F("velN: "));
  //   // Serial.print((float)myGNSS.packetUBXNAVVELNED->data.velN / 100.0, 2); // convert velN to m/s

  //   Serial.print(F(" velE: "));
  //   Serial.print((float)myGNSS.packetUBXNAVVELNED->data.velE / 100.0, 2); // convert velE to m/s

  //   // Serial.print(F(" velD: "));
  //   // Serial.print((float)myGNSS.packetUBXNAVVELNED->data.velD / 100.0, 2); // convert velD to m/s
  //   Serial.println(F(" (m/s)"));

  //   myGNSS.flushNAVVELNED(); //Mark all the data as read/stale so we get fresh data next time
  // }
  
    Serial.println();

  }
}

void startupAnimation(){
  uint16_t hue = 230;
  uint32_t fillColor;
  for(int i=0; i<250; i+=2){
    fillColor = leds.gamma32(leds.ColorHSV(hue*65536/361,250,i));
    leds.fill(fillColor);
    delay(1);
    leds.show();
  }
  for(int i=250; i>1; i-=2){
    fillColor = leds.gamma32(leds.ColorHSV(hue*65536/361,i,250));
    leds.fill(fillColor);
    delay(1);
    leds.show();
  }
  hue = 30;
   for(int i=0; i<255; i+=2){
    fillColor = leds.gamma32(leds.ColorHSV(hue*65536/361,i,250));
    leds.fill(fillColor);
    delay(1);
    leds.show();
  }
  for(int i=250; i>0; i-=2){
    fillColor = leds.gamma32(leds.ColorHSV(hue*65536/361,255,i));
    leds.fill(fillColor);
    delay(1);
    leds.show();
  }
}
