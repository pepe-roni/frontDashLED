#include <Wire.h> //Needed for I2C to GNSS
#include <FastLED.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
#include <U8g2lib.h>

#define LED_COUNT 144
#define LED_PIN 32
#define GPS_SDA 21
#define GPS_SCL 22
#define SERIALECHO false
#define TESTDATA false

#define NAV_RATE 1
#define HNR_RATE 30

#define STAGING_TIMER 2000 //ms for stage wait
#define ABORT_TIMER 10000// ms for idle wait
#define COMPLETE_DISMISS_TIMEOUT 20000//ms for launch completed
#define LAUNCH_TIMEOUT 10000//ms to launch timeout

#define STATUS_ROTATION_TIME 6000 // Alternate status display 
#define FRAMETIME 16 // ~60Hz refresh rate (1000ms / 60)

SFE_UBLOX_GNSS myGNSS;
CRGBArray<LED_COUNT> leds;
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R2, /* cs= */ 5, /* dc= */ 17, /* reset= */ 16);

enum LaunchState { 
  STATE_IDLE, 
  STATE_STAGING, 
  STATE_READY, 
  STATE_IN_PROGRESS, 
  STATE_60_COMPLETE,
  STATE_ABORT 
};
LaunchState currentLaunchState = STATE_IDLE;

uint32_t stoppedTimerStart = 0;
uint32_t launchTimerStart = 0;
uint32_t abortTimerStart = 0;
uint32_t completeTimerStart = 0;

float timeTo60 = 0.0f;
uint32_t hnrStartiTOW = 0;
float lastSpeed = 0.0f;
uint32_t last_iTOW = 0;

uint32_t lastLoopMicros = 0;
uint32_t loopDeltaMicros = 0;
uint32_t lastDiagnosticsUpdate = 0;
float actualHz = 0.0f;
uint32_t packetCount = 0;

uint16_t hueValue = 0;
int setBrightness = 155; 

void(* resetFunc) (void) = 0;

// Shared volatile variables for multi-core thread safety
volatile float velocity = 0;
volatile float accel_long = 0; // Longitudinal acceleration (G-force)
volatile float accel_lat = 0;  // Lateral acceleration (G-force)

// FreeRTOS Task Handle for running LEDs on Core 0
TaskHandle_t LEDTaskHandle = NULL;

void LEDTask(void * pvParameters);

void setup()
{
  Serial.begin(115200);
  delay(500);
  
  // Initialize LED strip and Screen Hardware
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, LED_COUNT);
  u8g2.begin();
  u8g2.setBusClock(8000000);
  
  // Set up a clean, readable text font for the boot log
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 12, "Booting System...");
  u8g2.sendBuffer();

  Serial.println(F("Tesla GPS DR LEDs - Direct ESF IMU Acceleration Edition"));
  Serial.println(F("Initializing LEDS and Display"));
  Wire.setBufferSize(1024); //large buffer
  Wire.setClock(400000); // Fast I2C
  Wire.begin(GPS_SDA, GPS_SCL);
  // Wire.setClock(400000); // Fast I2C
  
  if (myGNSS.begin() == false){ 
    Serial.println(F("u-blox GNSS not detected. Freezing."));
    u8g2.clearBuffer();
    u8g2.drawStr(0, 16, "ERROR: u-blox GNSS");
    u8g2.drawStr(0, 32, "not detected!");
    u8g2.drawStr(0, 48, "Freezing & Resetting...");
    u8g2.sendBuffer();
    delay(3000); // Give time to read the screen before hard-looping
    resetFunc();
  }
  else{
    Serial.println(F("u-blox GNSS connected. Setting params."));
    u8g2.drawStr(0, 26, "GNSS Connected.");
    u8g2.sendBuffer();
  }
  
  myGNSS.setESFAutoAlignment(true);
  myGNSS.setPortOutput(COM_PORT_I2C, COM_TYPE_UBX);
  myGNSS.setI2COutput(COM_TYPE_UBX); 

  myGNSS.configureMessage(0x01, 0x07, 1, 0); //NAV PVT 1 per epoch
  myGNSS.configureMessage(0x04, 0x00, 0, 0); // INF-ERROR
  myGNSS.configureMessage(0x04, 0x01, 0, 0); // INF-WARNING
  myGNSS.configureMessage(0x04, 0x02, 0, 0); // INF-NOTICE
  myGNSS.configureMessage(0x04, 0x03, 0, 0); // INF-DEBUG

  myGNSS.configureMessage(0x28, 0x00, 1, 0); // UBX_HNR_PVT -> Send every epoch (30Hz)
  myGNSS.configureMessage(0x28, 0x01, 1, 0); // UBX_HNR_ATT -> Send every epoch (30Hz)
  myGNSS.configureMessage(0x28, 0x02, 1, 0); // UBX_HNR_INS -> Send every epoch (30Hz)
  // myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); 

  // Disable standard message noise
  myGNSS.setAutoHNRATT(true);
  myGNSS.setAutoHNRINS(true); 
  myGNSS.setAutoPVT(true); 
  myGNSS.setAutoHNRPVT(true);
  myGNSS.setAutoESFSTATUS(true);
  
  // myGNSS.setAutoESFALG(true);

  delay(1000);
  myGNSS.checkUblox();
  myGNSS.getEsfInfo();

// Ensure the pointer is NOT null FIRST, then check the fusion mode value
  if (myGNSS.packetUBXESFSTATUS != NULL && myGNSS.packetUBXESFSTATUS->data.fusionMode == 1) {
    u8g2.drawStr(0, 40, "Fusion RDY.");
    u8g2.sendBuffer();
  }
  else {
    char fusString[16];
    sprintf(fusString, "Fusion NOT RDY: %d", myGNSS.packetUBXESFSTATUS->data.fusionMode);
    u8g2.drawStr(0, 40, fusString);
    u8g2.sendBuffer();
  }

  if (myGNSS.getYear() > 2025) {
      Serial.println(F("HOT START READY (Time is preserved!)"));
      u8g2.drawStr(0, 54, "Status: HOT");
    } else {
      Serial.println(F("COLD START (Backup battery is dead or disconnected!)"));
      u8g2.drawStr(0, 54, "Status: COLD");
    }
  u8g2.sendBuffer();


  // Configure high update rate safely at 1Hz background
  myGNSS.setNavigationFrequency(NAV_RATE); 
  myGNSS.setHNRNavigationRate(HNR_RATE); // 30Hz HNR rate
  Serial.print("Current update rate: ");
  Serial.println(myGNSS.getHNRNavigationRate());
  u8g2.sendBuffer();

  // delay(1500); // Hold the final diagnostic info on screen briefly before animation
  startupAnimation();
  
  // Wait for satellite geometry lock
  u8g2.clearBuffer();
  u8g2.drawStr(0, 24, "Waiting for Sat...");
  char rateString[24];
  sprintf(rateString, "Nav: %dHz | HNR: %dHz", NAV_RATE,myGNSS.getHNRNavigationRate());
  u8g2.drawStr(0, 40, rateString);
  u8g2.sendBuffer();

  while (!myGNSS.getGnssFixOk() && (myGNSS.packetUBXNAVPVT ->data.numSV < 4)) {
    delay(2000);
    // Dynamic text update while looping for lock
    u8g2.clearBuffer();
    u8g2.drawStr(0, 24, "Acquiring Satellites...");
    char sivString[16];
    sprintf(sivString, "SIV Count: %d", myGNSS.packetUBXNAVPVT ->data.numSV);
    u8g2.drawStr(0, 40, sivString);
    u8g2.sendBuffer();
    
    // startupAnimation();
  }
  
  // Satellites connected successfully
  Serial.print("Sat Connected: ");
  Serial.println(myGNSS.packetUBXNAVPVT ->data.numSV);
  delay(1000);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 24, "GNSS Lock Confirmed!");
  char finalSiv[16];
  sprintf(finalSiv, "Active SIV: %d", myGNSS.packetUBXNAVPVT ->data.numSV);
  u8g2.drawStr(0, 40, finalSiv);
  u8g2.drawStr(0, 56, "Getting TimeDate.");
  u8g2.sendBuffer();
  startupAnimation();
  
  if(myGNSS.getHour() > 2 && myGNSS.getHour() < 15){
    setBrightness = 80;
    u8g2.setContrast(100);
  } else {
    setBrightness = 255;
    u8g2.setContrast(255);
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
      //loop timer:
      static uint32_t processed_iTOW = 0;

      if (myGNSS.packetUBXHNRPVT != NULL) {
        uint32_t current_iTOW = myGNSS.packetUBXHNRPVT->data.iTOW;
        
        // If the timestamp has changed, a full 30Hz cycle has completed!
        if (current_iTOW != processed_iTOW) {
          processed_iTOW = current_iTOW;
      
          packetCount++;
          if (millis() - lastDiagnosticsUpdate >= 500) {
            actualHz = (packetCount * 1000.0f) / (millis() - lastDiagnosticsUpdate);
            packetCount = 0;
            lastDiagnosticsUpdate = millis();
          }
        }

        float pitchRad = 0;
        float rollRad  = 0;
        if (myGNSS.packetUBXHNRATT != NULL) {
            // u-blox provides these in degrees * 1e-5. Convert to Radians for sin()
          // Serial.print("gotdata");
          pitchRad = ((float)myGNSS.packetUBXHNRATT->data.pitch / 100000.0f) * (PI / 180.0f);
          rollRad  = ((float)myGNSS.packetUBXHNRATT->data.roll  / 100000.0f) * (PI / 180.0f);
        }
        // 1. Update velocity from High Navigation Rate PVT (in mm/s)
        velocity = myGNSS.packetUBXHNRPVT->data.gSpeed;
        //accel in x and y, adjusted for roll and pitch
        float rawLat = myGNSS.packetUBXHNRINS->data.yAccel; 
        accel_lat = ((rawLat / 100.0) / 9.81) + (sin(rollRad));
        float rawLong = myGNSS.packetUBXHNRINS->data.xAccel; 
        accel_long = ((rawLong / 100.0) / 9.81) - sin(pitchRad);
        float totalAccel = sqrtf((accel_lat * accel_lat)+ (accel_long * accel_long));
        float currentSpeed = velocity/447.04;
        int32_t mphTimesTen = (currentSpeed * 10);
        int32_t wholeMPH = mphTimesTen / 10;
        int32_t tenthsMPH = mphTimesTen % 10;
        
        uint8_t fusionMode = 0; 
        uint8_t alignStatus = 0;

        if (myGNSS.packetUBXESFSTATUS != NULL) {
          // 0 = Initializing, 1 = Fusion Operational, 2 = Suspended/Dead Reckoning Only
          fusionMode = myGNSS.packetUBXESFSTATUS->data.fusionMode;
        }
        // Correctly get Satellites In View using the library helper
        uint8_t satCount = 0;
        if (myGNSS.packetUBXNAVPVT != NULL){
          satCount = myGNSS.packetUBXNAVPVT ->data.numSV;
        }
        
    

        // Global Abort Check: If we are actively launching and hit negative longitudinal Gs, immediately cut it
        if (currentLaunchState == STATE_IN_PROGRESS && accel_long < 0.0f) {
            currentLaunchState = STATE_ABORT;
            abortTimerStart = millis();
            Serial.println("LAUNCH ABORTED: NEGATIVE G DETECTED");
        }

        switch (currentLaunchState) {
          
          case STATE_IDLE:
              // Waiting state. If we drop below 0.3 MPH, check if fusion is ready to move to Staging
              if (currentSpeed < 0.1f) {
                  if (fusionMode == 1) { //fusion ready
                      currentLaunchState = STATE_STAGING;
                      stoppedTimerStart = millis(); // Start tracking the 3-second window
                      Serial.println("STATUS: STAGING - FUSION READY, HOLDING FOR 3S");
                  }
              }
              break;

          case STATE_STAGING:
              // Must stay under 0.3 MPH to progress to READY
              if (currentSpeed < 0.3f) {
                  if (millis() - stoppedTimerStart >= STAGING_TIMER) {
                      currentLaunchState = STATE_READY;
                      timeTo60 = 0.0f;
                      Serial.println("STATUS: LAUNCH READY - AWAITING TRIGGER (<3MPH & >0.5G)");
                  }
              } else {
                  // If the car moves or rolls before the 3s is up, revert to IDLE
                  currentLaunchState = STATE_IDLE;
              }
              break;

          case STATE_READY:
              // Waiting for the launch trigger conditions: under 3MPH and a heavy G spike
              if (currentSpeed <= 3.0f) {
                  if (accel_long > 0.5f) {
                      launchTimerStart = millis();

                      if(myGNSS.packetUBXHNRPVT != NULL) {
                          hnrStartiTOW = myGNSS.packetUBXHNRPVT->data.iTOW;
                      }

                      currentLaunchState = STATE_IN_PROGRESS;
                      Serial.println("STATUS: LAUNCH IN PROGRESS!");
                  }
              } else {
                  // If you cruise away smoothly and cross 3MPH without hitting 0.5G, reset to IDLE
                  currentLaunchState = STATE_IDLE;
              }
              break;
          
          case STATE_IN_PROGRESS:
              // Clock is ticking. Check if we hit the finish line.
              if (currentSpeed >= 60.0f) {
                  // CALCULATE EXACT FINISH USING INTERPOLATION
                  if(myGNSS.packetUBXHNRPVT != NULL && hnrStartiTOW > 0 && last_iTOW > 0 && lastSpeed < 60.0f) {
                      uint32_t current_iTOW = myGNSS.packetUBXHNRPVT->data.iTOW;
                      
                      // 1. Find what percentage of the time gap we spent getting to exactly 60.0
                      float speedDelta = currentSpeed - lastSpeed;
                      float fraction = (60.0f - lastSpeed) / speedDelta;
                      
                      // 2. Apply that percentage to the hardware time gap
                      uint32_t timeDelta = current_iTOW - last_iTOW;
                      uint32_t exactEndiTOW = last_iTOW + (uint32_t)(fraction * timeDelta);
                      
                      // 3. Calculate final time
                      timeTo60 = (float)(exactEndiTOW - hnrStartiTOW) / 1000.0f;
                      Serial.print("Interpolated speed error: "); Serial.println(speedDelta);
                      Serial.print("Interpolated time error: "); Serial.println(timeDelta);
                  } else {
                      // Fallback to millis if packets are missing or math fails
                      timeTo60 = (float)(millis() - launchTimerStart) / 1000.0f;
                  }
                  
                  completeTimerStart = millis();
                  currentLaunchState = STATE_60_COMPLETE;
                  Serial.print("STATUS: LAUNCH 60 COMPLETE! Time: "); Serial.println(timeTo60);
              }
              else if(millis() - launchTimerStart > LAUNCH_TIMEOUT){
                  Serial.println("STATUS: LAUNCH TIMEOUT");
                  currentLaunchState = STATE_IDLE;
              }
              break;

          case STATE_60_COMPLETE:
              // Report time on screen. Once you come back to a complete stop, it resets back to IDLE
              if ((currentSpeed < 0.1f)|| (millis() - completeTimerStart > COMPLETE_DISMISS_TIMEOUT)){
                  currentLaunchState = STATE_IDLE;
                  Serial.println("STATUS: RESULTS DISMISSED -> REVERTED TO IDLE");
              }
              break;

          case STATE_ABORT:
              // Stay in abort status for 3 seconds before reverting to IDLE
              if (millis() - abortTimerStart >= ABORT_TIMER) {
                  currentLaunchState = STATE_IDLE;
                  Serial.println("STATUS: REVERTED TO IDLE FROM ABORT");
              }
              break;
      }
      // --- SAVE HISTORY FOR NEXT FRAME INTERPOLATION ---
        lastSpeed = currentSpeed;
        if (myGNSS.packetUBXHNRPVT != NULL) {
            last_iTOW = myGNSS.packetUBXHNRPVT->data.iTOW;
        }
        
        // Mark the HNR INS data as read/stale so we only process fresh packets
        myGNSS.flushHNRINS();
        
        if (SERIALECHO) {
          Serial.print(F("Speed: ")); Serial.print(velocity / 447.04);
          Serial.print(F(" | LongG: ")); Serial.print(accel_long);
          Serial.print(F(" | LatG: ")); Serial.print(accel_lat);
          Serial.print("| P: "); Serial.print(pitchRad * 180.0/PI); 
          Serial.print(" | R: "); Serial.println(rollRad * 180.0/PI);

        }

        // --- OLED DISPLAY UPDATE (LEFT: TEXT | RIGHT: G-FORCE PLOT) ---
        u8g2.clearBuffer();

        // Format string explicitly: "60.5"
        u8g2.setFont(u8g2_font_logisoso34_tn); //was 32
        char wholeString[16];
        sprintf(wholeString, "%d", wholeMPH);
        u8g2.drawStr(0, 34, wholeString);

        int nextX = u8g2.getStrWidth(wholeString);
        char tenthsString[8];
        sprintf(tenthsString, ".%d", tenthsMPH);
        u8g2.setFont(u8g2_font_logisoso16_tf); 
        u8g2.drawStr(nextX + 1, 32, tenthsString);
        
        u8g2.setFont(u8g2_font_profont10_tr); //6pt font
        u8g2.drawStr(2, 47 , "MPH");

        drawLaunchStatus();
        // 2. DRAW NUMERICAL Gs (Left Side Bottom)
        if (fusionMode == 1) {
          char gString[64];
          if (totalAccel < 1.0f) {
            sprintf(gString, ".%02d G", (int)(totalAccel * 100)); 
          } else {
            // Fallback for when you pull 1.0G or more
            sprintf(gString, "%.2f G", totalAccel); 
          }
          u8g2.setFont(u8g2_font_profont17_mn);
          u8g2.drawStr(97, 12, gString);

          // 3. DRAW G-FORCE PLOT (Right Side)
          int maxRadius = 18; // Reduced from 24 to 16 pixels
          int centerX = 109; // Shifted right from 96 to 108 to hug the screen edge cleanly
          int centerY = 35;  // Keeps the circle perfectly centered vertically
          float maxG = 0.9;   // Keeps the outer edge mapped to 1.0G
          int dotSize = 2; //px for g dot

          // Draw the outer boundary circle
          u8g2.drawCircle(centerX, centerY, maxRadius, U8G2_DRAW_ALL);
          
          // Draw inner target crosshairs (center point indicator)
          u8g2.drawPixel(centerX, centerY);
          u8g2.drawLine(centerX - 3, centerY, centerX + 3, centerY);
          u8g2.drawLine(centerX, centerY - 3, centerX, centerY + 3);

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

        
          u8g2.drawDisc(round(targetX), round(targetY), dotSize, U8G2_DRAW_ALL);
        }
        // --- SMALL BOOT/STATUS INDICATORS (Bottom Edge) ---
        u8g2.setFont(u8g2_font_04b_03_tr); // Ultra-micro 5px tall fontu8g2_font_04b_03_tr

        bool showLoopMetrics = (millis() / STATUS_ROTATION_TIME) % 2 == 0;
        if(showLoopMetrics){
          char hzString[10];
          sprintf(hzString, "%dHz", (int)actualHz);
          u8g2.drawStr(89, 64, hzString); // Sits nicely to the left of center
        }
        else{
          // Draw Satellite Count (Bottom Left, right next to your G-Force string)
          char satString[10];
          sprintf(satString, " ST:%d", satCount);
          u8g2.drawStr(85, 64, satString);
        }

        // Draw Fusion Status Token (Bottom Middle/Right depending on layout space)
        // We map modes to quick strings: INIT, FIX (Operational), or DR (Dead Reckoning Only)
        if (fusionMode == 1) {
          u8g2.drawStr(111, 64, "LOCK");
        } else if (fusionMode == 2) {
          u8g2.drawStr(111, 64, "IMU");
        } else {
          u8g2.drawStr(111, 64, "INIT");
        }
        

        u8g2.sendBuffer(); 
      }
    }
  }
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
  firstPixelHue = map(currentVelocity * 100.0, 0, 6000, 16500, -9600) / 100.0;

  for(int i = 0; i < LED_COUNT/2 + 1; i++) { 
    int pixelHue = firstPixelHue + (i * 255 * (0.225 + 0.85 * currentVelocity / 150) / LED_COUNT);
    leds[LED_COUNT/2 - i] = CHSV(pixelHue, 255, setBrightness);
    leds(LED_COUNT/2, LED_COUNT - 1) = leds(LED_COUNT/2 - 1, 0);
  }
  FastLED.show();
}

void drawLaunchStatus() {
  static int statusX = 20;
  static int statusY = 47;

  // Clear space or draw a separation line if necessary
  switch (currentLaunchState) {
    
    case STATE_IDLE:
      // u8g2.setFont(u8g2_font_profont10_tr); //6pt font
      // u8g2.drawStr(statusX, statusY, "IDLE");
      // Keep screen clean or show a subtle status if you like. 
      // If we are moving normally, we leave this blank so the speedometer takes priority.
      break;

    case STATE_STAGING:
      u8g2.setFont(u8g2_font_profont10_tr); //6pt font
      // Visual progress bar showing how close you are to the 3-second hold mark
      {
        long elapsed = millis() - stoppedTimerStart;
        if(elapsed>STAGING_TIMER*0.75){
          u8g2.drawStr(statusX, statusY, "STAGING");  
        }
        else if(elapsed>STAGING_TIMER*0.5){
          u8g2.drawStr(statusX, statusY, "STAGING |");
        }
        else if(elapsed>STAGING_TIMER*0.25){
          u8g2.drawStr(statusX, statusY, "STAGING ||");
        }
        else{
          u8g2.drawStr(statusX, statusY, "STAGING |||");
        }
      }
      break;

    case STATE_READY:
      {
      // High-visibility inverted text box so it pops out at the driver
      u8g2.setFont(u8g2_font_profont10_tr); //6pt font
      // u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawBox(statusX, statusY - 9, 62, 11); 
      
      u8g2.setDrawColor(0); // Set color to black (to draw inside white box)
      u8g2.drawStr(statusX+5, statusY, "LAUNCH RDY");
      u8g2.setDrawColor(1); // Reset back to white
      u8g2.setFont(u8g2_font_profont17_mn); 
      u8g2.drawStr(0, 64, "0.000s");
      }
      break;

    case STATE_IN_PROGRESS:
      {
      u8g2.setFont(u8g2_font_profont10_tr); //6pt font
      if ((millis() / 200) % 2 == 0) {
        u8g2.drawStr(statusX, statusY, "LAUNCHING");
      }
      // u8g2.setFont(u8g2_font_7x14_tf); // Use a slightly chunkier, readable font
      
      float liveTimer = (float)(millis() - launchTimerStart) / 1000.0f;
      u8g2.setFont(u8g2_font_profont17_tr); 
      char liveStr[16];
      sprintf(liveStr, "%.3fs", liveTimer);
      u8g2.drawStr(0, 64, liveStr);
      }
      break;

    case STATE_60_COMPLETE:
      u8g2.setFont(u8g2_font_profont10_tr); //6pt font
      u8g2.drawStr(statusX, statusY, "LAUNCH OK");

      char resultStr[32];
      sprintf(resultStr, "%.3fs", timeTo60);
      if ((millis() / 500) % 2 == 0) {
        u8g2.setFont(u8g2_font_profont17_tr); 
        u8g2.drawStr(0, 64, resultStr);
        u8g2.setFont(u8g2_font_profont10_tr); 
        u8g2.drawStr(64, 64,"to 60");
      }
      // Optional: draw a tiny trophy icon or border around the time
      // u8g2.drawFrame(0, yPos - 11, 128, 13);
      break;

    case STATE_ABORT:
      // Flashing alert using millis() tracking
      if ((millis() / 500) % 2 == 0) {
        u8g2.setFont(u8g2_font_profont10_tr); //6pt font
        u8g2.drawStr(statusX, statusY, "ABORTED");
      }
      break;
  }
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