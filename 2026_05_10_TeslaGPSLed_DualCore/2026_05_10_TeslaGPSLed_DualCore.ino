#include <Wire.h> //Needed for I2C to GNSS
#include <FastLED.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
#include <U8g2lib.h>

#define LED_COUNT 144
#define LED_PIN 32
#define GPS_SDA 21
#define GPS_SCL 22
#define SERIALECHO true
#define TESTDATA false

#define FRAMETIME 16 // ~60Hz refresh rate (1000ms / 60)

SFE_UBLOX_GNSS myGNSS;
CRGBArray<LED_COUNT> leds;
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R0, /* cs= */ 5, /* dc= */ 17, /* reset= */ 16);

uint16_t hueValue = 0;
int setBrightness = 155; 

void(* resetFunc) (void) = 0;

// Shared volatile variables for multi-core thread safety
volatile float velocity = 0;
volatile float accel_long = 0; // Longitudinal acceleration (G-force)
volatile float accel_lat = 0;  // Lateral acceleration (G-force)
bool sportMode = false;

// FreeRTOS Task Handle for running LEDs on Core 0
TaskHandle_t LEDTaskHandle = NULL;

void LEDTask(void * pvParameters);

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println(F("Tesla GPS DR LEDs - Direct ESF IMU Acceleration Edition"));

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, LED_COUNT);
  
  Wire.begin(GPS_SDA, GPS_SCL);
  Wire.setClock(400000); // Fast I2C
  // u8g2.begin();
  
  if (myGNSS.begin() == false){ 
    Serial.println(F("u-blox GNSS not detected. Freezing."));
    resetFunc();
  }
  myGNSS.setESFAutoAlignment(true);
  myGNSS.setPortOutput(COM_PORT_I2C, COM_TYPE_UBX);
  myGNSS.setI2COutput(COM_TYPE_UBX); 
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); 

  // Enable the External Sensor Fusion (ESF) status monitoring
  if (myGNSS.getEsfInfo()){
    Serial.print(F("Fusion Mode: "));
    Serial.println(myGNSS.packetUBXESFSTATUS->data.fusionMode);  
    Serial.print(F("Time status on boot: "));
    if (myGNSS.getYear() > 2025) {
      Serial.println(F("HOT START READY (Time is preserved!)"));
    } else {
      Serial.println(F("COLD START (Backup battery is dead or disconnected!)"));
   }
  }

  // Disable standard message noise
  myGNSS.setAutoHNRATT(false);

  myGNSS.setAutoHNRINS(true); 
  // Keep true so the module continuously streams 30Hz PVT packets
  myGNSS.setAutoHNRPVT(true); 

  // Configure high update rate
  myGNSS.setNavigationFrequency(1); 
  myGNSS.setHNRNavigationRate(30); // 30Hz HNR rate
  Serial.println("Current update rate: 30Hz");

  startupAnimation();
  while (!myGNSS.getGnssFixOk() && myGNSS.getSIV() < 2) {
    //bad fix
    delay(5000);
    startupAnimation();
  }
  //good fix
  Serial.print("Sat Connected: ");
  Serial.println(myGNSS.getSIV());
  delay(500);
  
  if(myGNSS.getHour() > 2 && myGNSS.getHour() < 15){
    setBrightness = 80;
  } else {
    setBrightness = 255;
  }

  // Pin the LED strip updating process to Core 0 (GPS & Main Loop stay on Core 1)
  xTaskCreatePinnedToCore(
    LEDTask,           
    "LEDTask",         
    4000,              
    NULL,              
    1,                 
    &LEDTaskHandle,    
    0                  
  );
}

unsigned long lastFakeGPSTime = 0;

void loop()
{
  // --- GPS DATA ACQUISITION & OLED (CORE 1) ---
  if (TESTDATA){
    if (millis() - lastFakeGPSTime > 33){ 
      lastFakeGPSTime = millis(); 
      if (velocity < 150) velocity += 0.5;
      else velocity = 0;
      
      // Simulate some cornering and braking forces for testing
      accel_long = sin(millis() / 1000.0) * 0.8; // Swings between -0.8G and +0.8G
      accel_lat  = cos(millis() / 1500.0) * 0.6; // Swings between -0.6G and +0.6G
    }
  } 
  else {
    // Continuous Polling: checkUblox() parses incoming packets in the I2C buffer
    if (myGNSS.checkUblox()) {
      
      // 1. Update velocity from High Navigation Rate PVT (in mm/s)
      velocity = myGNSS.packetUBXHNRPVT->data.gSpeed;

      // 2. Extract Lat/Long G-forces from High Navigation Rate INS Struct
      // Units are in cm/s^2, so we divide by 100 to get m/s^2, then by 9.81 for Gs
      float rawLat = myGNSS.packetUBXHNRINS->data.xAccel; 
      accel_lat = (rawLat / 100.0) / 9.81;

      float rawLong = myGNSS.packetUBXHNRINS->data.yAccel; 
      accel_long = (rawLong / 100.0) / 9.81;
      
      // Mark the HNR INS data as read/stale so we only process fresh packets
      myGNSS.flushHNRINS();
      
      if (SERIALECHO) {
        Serial.print(F("Speed: ")); Serial.print(velocity / 447.04);
        Serial.print(F(" | LongG: ")); Serial.print(accel_long);
        Serial.print(F(" | LatG: ")); Serial.println(accel_lat);
      }

      // --- OLED DISPLAY UPDATE (LEFT: TEXT | RIGHT: G-FORCE PLOT) ---
      u8g2.clearBuffer();
      
      // 1. DRAW SPEED (Left Side)
      u8g2.setFont(u8g2_font_logisoso20_tf); 
      char speedString[8];
      dtostrf(velocity / 447.04, 3, 0, speedString); // Rounded to nearest MPH to save horizontal space
      u8g2.drawStr(0, 30, speedString);
      
      u8g2.setFont(u8g2_font_helvB08_tr);
      u8g2.drawStr(48, 25, "MPH");

      // 2. DRAW NUMERICAL Gs (Left Side Bottom)
      char gString[64];
      sprintf(gString, "La:%.2f Lo:%.2f", accel_lat, accel_long);
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0, 58, gString);

      // 3. DRAW G-FORCE PLOT (Right Side)
      int centerX = 96;  // Center X of the G-circle
      int centerY = 32;  // Center Y of the G-circle
      int maxRadius = 24; // Boundary radius of the circle
      float maxG = 1.0;   // The outer circle represents 1.0G

      // Draw the outer boundary circle
      u8g2.drawCircle(centerX, centerY, maxRadius, U8G2_DRAW_ALL);
      
      // Draw inner target crosshairs (center point indicator)
      u8g2.drawPixel(centerX, centerY);
      u8g2.drawLine(centerX - 3, centerY, centerX + 3, centerY);
      u8g2.drawLine(centerX, centerY - 3, centerX, centerY + 3);

      // Map G-Force values to pixel offsets.
      // - Lateral (X) map: Positive is Right, Negative is Left
      // - Longitudinal (Y) map: Positive is Acceleration (Up/Forward), Negative is Braking (Down/Back)
      // Note: On OLEDs, coordinate Y=0 is the TOP of the screen, so we invert the Y axis (-=).
      float targetX = centerX + (accel_lat / maxG) * maxRadius;
      float targetY = centerY - (accel_long / maxG) * maxRadius;

      // Constrain the dot to stay strictly within our max boundary circle using trigonometry (Vector Magnitude)
      float dx = targetX - centerX;
      float dy = targetY - centerY;
      float distance = sqrt(dx*dx + dy*dy);

      if (distance > maxRadius) {
        // Force the coordinate back onto the perimeter of the boundary circle
        targetX = centerX + (dx / distance) * maxRadius;
        targetY = centerY + (dy / distance) * maxRadius;
      }

      // Draw the dynamic G-force "bubble" (a filled 3px radius circle)
      u8g2.drawDisc(round(targetX), round(targetY), 3, U8G2_DRAW_ALL);

      u8g2.sendBuffer(); 
    }
  }

  // --- ACCELERATION CALCULATION (DISABLED - Now using raw IMU values above) ---
  /*
  if(millis() - lastTime3 > 30){ 
    float currentVelocity = velocity; 
    accel = (((currentVelocity - oldVelocity)/(2.237*447.04))/((millis()-lastTime3)/1000.0))/9.81;
    oldVelocity = currentVelocity; 
    lastTime3 = millis();
  }
  */
}

// --- CORE 0: DEDICATED LED ANIMATION ENGINE ---
void LEDTask(void * pvParameters){
  unsigned long lastLEDUpdate = 0;
  
  for(;;){ 
    if (millis() - lastLEDUpdate > FRAMETIME) {
      lastLEDUpdate = millis();

      float targetVelocity = velocity; 

      if (TESTDATA) {
        fastLEDHandler(targetVelocity, false);
      } else {
        fastLEDHandler(targetVelocity / 447.04, false);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // Feed the watchdog timer
  }
}

void fastLEDHandler(float currentVelocity, bool usePalette){
  int firstPixelHue;

  if (currentVelocity < 60){
    firstPixelHue = map(currentVelocity * 100.0, 0, 6000, 16000, 8000) / 100.0;
  }
  else if (currentVelocity < 100){
    firstPixelHue = map(currentVelocity * 100.0, 6000, 10000, 8000, 0) / 100.0;
  }
  else{
    firstPixelHue = map(currentVelocity * 100.0, 10000, 14000, 0, -7000) / 100.0;
  }

  for(int i = 0; i < LED_COUNT/2 + 1; i++) { 
    int pixelHue = firstPixelHue + (i * 255 * (0.2 + 0.2 * currentVelocity / 150) / LED_COUNT);
    leds[LED_COUNT/2 - i] = CHSV(pixelHue, 255, setBrightness);
    leds(LED_COUNT/2, LED_COUNT - 1) = leds(LED_COUNT/2 - 1, 0);
  }
  FastLED.show();
}

void startupAnimation(){
  int cycles = 0;
  static uint8_t hue;

  while(cycles < 5){
    for(int i = 0; i < LED_COUNT/2; i++) {   
      leds.fadeToBlackBy(40);
      leds[i] = CHSV(hue++, 255, 255);
      leds(LED_COUNT/2, LED_COUNT-1) = leds(LED_COUNT/2 - 1 , 0);
      FastLED.delay(12 - (cycles * 10/5));
    }
    cycles += 1;
  }
  for(int i = 0; i < LED_COUNT; i++) {  
    leds.fadeToBlackBy(5);
    FastLED.show();
  }
}