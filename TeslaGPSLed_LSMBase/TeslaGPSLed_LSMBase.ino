
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <LSM303.h>

LSM303 compass;

#define LED_COUNT 144 //number of leds in that array
#define LED_PIN 3 //rgb led attached to D3
#define GPSECHO  false
#define SERIALECHO false //speed and accel output
#define TESTDATA false //pseudo data

SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB +NEO_KHZ800);

uint16_t hueValue = 0;
uint32_t color = leds.ColorHSV(hueValue);
float Ycal = 0;
//float costheta = 1;

void setup()
{
  leds.begin();
  leds.show(); 
  leds.setBrightness(155);

  if (SERIALECHO){
    Serial.begin(115200);
    Serial.println("Begin Serial Output");
  }
  
  Wire.begin();
  GPS.begin(9600);
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  compass.init();
  compass.enableDefault();
  
  compass.read();
//  costheta = compass.a.z/16000.0*9.81/9.81;
//  Ycal = sqrt((9.81*9.81)-((compass.a.z/16000.0*9.81)*(compass.a.z/16000.0*9.81)));
  Ycal = -compass.a.y/16000.0*9.81;
  
//  startupAnimation();
  
//  Serial.print("Ycal: "); Serial.println(Ycal);
//  Serial.print("COS(Theta): "); Serial.println(costheta);
  
}

uint32_t timer = millis();
float velocity = 0;
float accel = 0;
int firstPixelHue = 0;
int value = 0;


void loop()                   
{
  int tempHue = firstPixelHue;
  int tempValue = value;
  
  char c = GPS.read();
  if ((c) && (GPSECHO))
    Serial.write(c);

  if (GPS.newNMEAreceived()) {
      if (TESTDATA){
        if (velocity < 35)
          velocity = velocity+=1.35; 
        if (velocity < 60)
          velocity = velocity+=1.1; 
        if (velocity < 80)
          velocity = velocity+=0.75; 
        if (velocity < 110)
          velocity = velocity+=0.55; 
        if (velocity < 150)
          velocity = velocity+=0.25;
        if (velocity > 150)
          velocity = 0;
      }
      else
        velocity = GPS.speed*1.151;
      if (SERIALECHO){
        Serial.print("Speed (mph): "); Serial.println(velocity);
        Serial.print("Acceleration: "); Serial.println(accel);
      }
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (millis() - timer > 10) {
//     compass.read();
     accel = 0;
     for(int j=0; j<2; j+=1){ //get 10 values
        compass.read();
        accel = (accel+-compass.a.y/16000.0*9.81)-Ycal; //conv from adc to g's to m/s^2
     }
     accel = abs(accel/2.0); //avg them
     
     velocity = velocity + accel*((millis() - timer)/1000.0); //if no datareceived from gps, estimate speed from accel
     
     
     if (velocity < 60){
        firstPixelHue = map(velocity*10.0, 0, 600, 45000,21845); //velocity to hue
        if (accel >= 10)
          accel = 10;
        value = map(accel*100.0,200,1000,100,255); //accel to brightness
     }
     else if (velocity < 100){
        firstPixelHue = map(velocity*10.0, 600, 1000, 21845, 0);
        if (accel >= 8)
          accel = 8;
        value = map(accel*100.0,100,800,100,255); //accel to brightness
     }
     else{
        firstPixelHue = map(velocity*10.0, 1000, 1400, 0, -15922);
        if (accel >= 7)
          accel = 7;
        value = map(accel*100.0,0,700,100,255); //accel to brightness
     }
     int diffHue = firstPixelHue - tempHue;
     int diffValue = value - tempValue;
  
     for(int i=0; i<(leds.numPixels()/2)+1; i++) { // For each pixel in strip...
       int pixelHue = firstPixelHue + (i * 0.2*65536L / leds.numPixels());
       leds.setPixelColor(i, leds.gamma32(leds.ColorHSV(pixelHue, 255, value)));
       leds.setPixelColor(144-i, leds.gamma32(leds.ColorHSV(pixelHue, 255, value)));
     }
  
     leds.show();
     timer = millis(); // reset the timer
  
  }

  //final write to leds
//  int diffHue = firstPixelHue - tempHue;
//  int diffValue = value - tempValue;
//  
//  for(int i=0; i<(leds.numPixels()/2)+1; i++) { // For each pixel in strip...
//     int pixelHue = firstPixelHue + (i * 0.2*65536L / leds.numPixels());
//     leds.setPixelColor(i, leds.gamma32(leds.ColorHSV(pixelHue, 255, value)));
//     leds.setPixelColor(144-i, leds.gamma32(leds.ColorHSV(pixelHue, 255, value)));
//   }
//
//   leds.show();
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

  //zero the accel
//  compass.read();

  
 }
  
