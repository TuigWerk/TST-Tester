#undef min  // to fix issue: defining min() as this is vulnerable to the double evaluation problem and considered bad practice.
#include <M5Unified.h>
#include "Arduino.h"
#include <RunningAverage.h>
#include <Preferences.h>

float Sav[3];
float PrevSav[3];
float MiMa[6];
float MiMaAdjust[6];
float Xzero, Yzero, Zzero;
float Xfactor, Yfactor, Zfactor;

const int buttonPin = 37; // KEY1 on M5Stick S3

const unsigned long readInterval = 100;
unsigned long lastReadTime = 0;  // Last read time
unsigned long pressTime;
unsigned long debounceInterval = 500;
bool buttonState = false;
bool prevButtonState;

int bufferSize = 10;
float measureThreshold = 0.85;
float accX, accY, accZ;

int menuIndex = 0;

Preferences preferences;

RunningAverage XRA(bufferSize);
RunningAverage YRA(bufferSize);
RunningAverage ZRA(bufferSize);

void printValues() {
  Serial.print("Zero values:             ");
  Serial.print("X: ");
  Serial.print(Xzero);
  Serial.print(" Y: ");
  Serial.print(Yzero);
  Serial.print(" Z: ");
  Serial.print(Zzero);
  Serial.println();

  Serial.print("Multiplication factors:  ");
  Serial.print("X: ");
  Serial.print(Xfactor);
  Serial.print(" Y: ");
  Serial.print(Yfactor);
  Serial.print(" Z: ");
  Serial.print(Zfactor);
  Serial.println();
  Serial.println();
}

void setup() {
  // initialize the serial communications:
  Serial.begin(115200);
  preferences.begin("Calibration", false);
  pinMode(buttonPin, INPUT);

  auto cfg = M5.config();
  M5.begin(cfg);
  M5.update();

  M5.Display.setRotation(3);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setFont(&fonts::FreeSansBold12pt7b);

  delay(2000);
  Serial.println("Starting....");

  XRA.clear();
  YRA.clear();
  ZRA.clear();

  Serial.println("Stored preferences: ");
  Serial.println();

  Xzero = preferences.getFloat("Xzero", 0.00);
  Xfactor = preferences.getFloat("Xfactor", 0.00);
  Yzero = preferences.getFloat("Yzero", 0.00);
  Yfactor = preferences.getFloat("Yfactor", 0.00);
  Zzero = preferences.getFloat("Zzero", 0.00);
  Zfactor = preferences.getFloat("Zfactor", 0.00);

  printValues();

  Serial.println("Ready to save X Min, put sensor on bottom side");
}

void loop() {
  M5.update();

  if (millis() - pressTime > debounceInterval) {  //wait a moment for debounce
    buttonState = digitalRead(buttonPin);         //check button state 1 or 0
    if (buttonState != prevButtonState) {         //compare with previous button state and act only if state has changed
      pressTime = millis();
      if (buttonState == false) {
        M5.Speaker.tone(4000, 50);
        if (menuIndex == 0) {
          Serial.println("X Min saved");
          MiMa[0] = Sav[0];
          MiMaAdjust[0] = -1 - MiMa[0];
          Serial.println("Ready to save X Max, put sensor on top side");
        }
        if (menuIndex == 1) {
          Serial.println("X Max saved");
          MiMa[1] = Sav[0];
          MiMaAdjust[1] = 1 - MiMa[1];
          Serial.println("Ready to save Y Min, put sensor on right side");
        }
        if (menuIndex == 2) {
          Serial.println("Y Min saved");
          MiMa[2] = Sav[1];
          MiMaAdjust[2] = -1 - MiMa[2];
          Serial.println("Ready to save Y Max, put sensor on left side");
        }
        if (menuIndex == 3) {
          Serial.println("Y Max saved");
          MiMa[3] = Sav[1];
          MiMaAdjust[3] = 1 - MiMa[3];
          Serial.println("Ready to save Z Min, put sensor on front side");
        }
        if (menuIndex == 4) {
          Serial.println("Z Min saved");
          MiMa[4] = Sav[2];
          MiMaAdjust[4] = -1 - MiMa[4];
          Serial.println("Ready to save Z Max, put sensor on back side");
        }

        if (menuIndex == 5) {
          Serial.println("Z Max saved");
          MiMa[5] = Sav[2];
          MiMaAdjust[5] = 1 - MiMa[5];
          Serial.println("Calibration finished");
        }

        if (menuIndex == 6) {
          menuIndex = -1;
          XRA.clear();
          YRA.clear();
          ZRA.clear();
          Serial.println("Restarting calibration...");
          Serial.println("Ready to save X Min, put sensor on bottom side");
        }

        menuIndex++;

        for (int i = 0; i < 6; i++) {
          Serial.print(MiMa[i]);
          Serial.print(", ");
        }
        Serial.println();

        if (menuIndex == 6) {
          //menuIndex = 0;

          Xzero = (MiMa[0] + MiMa[1]) / 2;
          Yzero = (MiMa[2] + MiMa[3]) / 2;
          Zzero = (MiMa[4] + MiMa[5]) / 2;
          Xfactor = 1 / (MiMa[1] - Xzero);
          Yfactor = 1 / (MiMa[3] - Yzero);
          Zfactor = 1 / (MiMa[5] - Zzero);


          Serial.println("Writing to EEPROM...");
          printValues();

          preferences.putFloat("Xzero", Xzero);
          preferences.putFloat("Yzero", Yzero);
          preferences.putFloat("Zzero", Zzero);
          preferences.putFloat("Xfactor", Xfactor);
          preferences.putFloat("Yfactor", Yfactor);
          preferences.putFloat("Zfactor", Zfactor);


          Serial.println("Repeat Calibration if necessary");
          Serial.println("Ready to save X Min, put sensor on bottom side");
        }
      }
    }
    prevButtonState = buttonState;
  }

  if (millis() - lastReadTime >= readInterval) {
    lastReadTime = millis();
    M5.Imu.update();
    auto data = M5.Imu.getImuData();
    accX = data.accel.x;
    accY = data.accel.y;
    accZ = data.accel.z;

    XRA.addValue(accX);
    YRA.addValue(accY);
    ZRA.addValue(accZ);

    Sav[0] = XRA.getAverage();
    Sav[1] = YRA.getAverage();
    Sav[2] = ZRA.getAverage();

    if (menuIndex < 6) {
      const char* instructions[] = {
        "Place on BOTTOM",
        "Place on TOP",
        "Place on RIGHT side",
        "Place on LEFT side",
        "Place on FRONT",
        "Place on BACK"
      };
      char axisChar = (menuIndex < 2) ? 'X' : (menuIndex < 4) ? 'Y' : 'Z';
      float displayVal = (menuIndex < 2) ? Sav[0] : (menuIndex < 4) ? Sav[1] : Sav[2];

      M5.Display.fillScreen(BLACK);
      M5.Display.setFont(&fonts::FreeSans9pt7b);
      M5.Display.setTextSize(1);
      M5.Display.setCursor(5, 18);
      M5.Display.printf("Step %d/6", menuIndex + 1);
      M5.Display.setCursor(5, 40);
      M5.Display.print(instructions[menuIndex]);
      M5.Display.setFont(&fonts::FreeSansBold12pt7b);
      M5.Display.setTextSize(2);
      M5.Display.setCursor(20, 100);
      M5.Display.printf("%c: %.2f", axisChar, displayVal);
    }
    if (menuIndex == 6) {
      M5.Display.clear();
      M5.Display.setFont(&fonts::FreeSans9pt7b);
      M5.Display.setTextSize(1);
      M5.Display.setCursor(10, 18);
      M5.Display.print("Calibration Done!");
      M5.Display.setCursor(10, 45);
      M5.Display.printf("X: %.2f", Sav[0] - Xzero * Xfactor);
      M5.Display.setCursor(10, 68);
      M5.Display.printf("Y: %.2f", Sav[1] - Yzero * Yfactor);
      M5.Display.setCursor(10, 91);
      M5.Display.printf("Z: %.2f", Sav[2] - Zzero * Zfactor);
      M5.Display.setCursor(10, 120);
      M5.Display.print("Btn: restart");
    }
  }
}
