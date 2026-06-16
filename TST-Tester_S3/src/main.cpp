#include <M5Unified.h>
#undef min // to fix issue: defining min() as this is vulnerable to the double evaluation problem and considered bad practice.
#include <NimBLEDevice.h>
#include "Arduino.h"
#include <Wire.h>
#include "TSTLogo.h"
#include "Arrows.h"
#include "Bluetooth.h"
#include "BatteryFull.h"
#include "BatteryHalf.h"
#include "BatteryEmpty.h"
#include "S1.h"
#include "S2.h"
#include "S3.h"
#include "S4.h"

#include <RunningAverage.h>
#include <RunningMedian.h> // Rob Tillaart's RunningMedian library
#include "FFT.h"           // Replaced ArduinoFFT with FFT.h
#include <Preferences.h>
#include <esp_sleep.h>

Preferences preferences;
Preferences deviceSettings; // Separate namespace for device settings (read-write)

const char *DEVICE_NAME[] = {"TST Sensor 1", "TST Sensor 2", "TST Sensor 3", "TST Sensor 4"};

#define SOFTWARE_VERSION "v5.8 S3"

// Change Log:
//  v5.8 S3: Port to M5Stick S3 — BMI270 IMU, button GPIO 37, IR pin GPIO 46.
//  v5.8: Combined freq+amplitude pipeline; no mode switching, both characteristics always notified.
//  v5.7: Set standard multiplication factor to 1.00 to solve 0 Hz values when device is not calibrated.
//  v5.6: Device number stored in NVS, button release to change device number, auto-off when no vibration detected
//  v5.5: NimBLE implementation for better BLE stability
//  v5.4: FFT for freq and amplitude, min/max frequencies, running median
//  v5.3: Improved battery monitoring, max 60 Hz frequency detection

#define BLE_UUID_ACC_SERVICE "b3d94f96-7fb4-11ed-a1eb-0242ac120002"
#define BLE_UUID_FREQ "57fcccde-7fb4-11ed-a1eb-0242ac120002"
#define BLE_UUID_AMPX "57fcd116-7fb4-11ed-a1eb-0242ac120002"
#define BLE_UUID_AMPY "57fcd328-7fb4-11ed-a1eb-0242ac120002"
#define BLE_UUID_AMPZ "57fcd67a-7fb4-11ed-a1eb-0242ac120002"
#define BLE_UUID_MODE "57fcd83c-7fb4-11ed-a1eb-0242ac120002"

// NimBLE objects
NimBLEServer *pServer = nullptr;
NimBLEService *pAccService = nullptr;
NimBLECharacteristic *pFreqChar = nullptr;
NimBLECharacteristic *pAmpxChar = nullptr;
NimBLECharacteristic *pAmpyChar = nullptr;
NimBLECharacteristic *pAmpzChar = nullptr;
NimBLECharacteristic *pModeChar = nullptr; // stub for app backward compatibility
NimBLEAdvertising *pAdvertising = nullptr;

bool deviceConnected = false;
bool prevDeviceConnected = false;

// FFT Configuration
#define SAMPLING_FREQ 500.0
#define FFT_SIZE 1024
#define HOP_SIZE 1024  // New samples collected per cycle (75% overlap → ~0.5 s update rate)

// Amplitude filtering configuration
#define MAX_REASONABLE_AMPLITUDE 8.0  // Maximum physically reasonable amplitude in g
#define MIN_REASONABLE_AMPLITUDE 0.01 // Minimum meaningful amplitude in g
#define MEDIAN_FILTER_SIZE 10         // Size of median filter buffer
#define CORRECTION_FACTOR 1.00        // Amplitude adjustment for accelerometer

const uint16_t *DEVICE_IMAGES[] = {S1, S2, S3, S4};

// Battery constants
const int BATTERY_LEVEL_MID = 50;
const int BATTERY_LEVEL_LOW = 25;

// Display brightness constants
const uint8_t BRIGHTNESS_FULL = 128;
const uint8_t BRIGHTNESS_DIM = 1;
const unsigned long DIM_TIMEOUT = 30000; // Dim after 30 seconds of inactivity

// Sampling buffers — 3 axes, FFT_SIZE samples each
float measure_buffers[3][FFT_SIZE];

unsigned int sampling_period_us;

// Running average Battery
RunningAverage RaBatLvl(10);

// Amplitude filtering objects
RunningMedian ampX_median(MEDIAN_FILTER_SIZE);
RunningMedian ampY_median(MEDIAN_FILTER_SIZE);
RunningMedian ampZ_median(MEDIAN_FILTER_SIZE);

float outputAmpX = 0, outputAmpY = 0, outputAmpZ = 0;

const int buttonPin = 11; // KEY1 on M5StickS3
const int ledPin = 46;    // IR TX on M5Stick S3

const uint16_t imageWidth = 240;
const uint16_t imageHeight = 135;

int BatLevel;
int BatLvlAvg;

float Xzero, Yzero, Zzero;
float Xfactor, Yfactor, Zfactor;

unsigned long pressTime;
unsigned long pressStartTime = 0;
unsigned long debounceInterval = 200;
unsigned long refreshTime;
unsigned long refreshInterval = 2000;
unsigned long updateBatTime;
unsigned long updateBatInterval = 5000;
unsigned long batteryCheckTime = 0;
const unsigned long BATTERY_CHECK_INTERVAL = 30000;
unsigned long startupDelay = 5000;
unsigned long lastActivityTime;
unsigned long maxAutoOff = 300000; // Maximum auto-off time (5 minutes)
unsigned long autoOff = maxAutoOff;
float lowestFreq = 2.0;
float highestFreq = 60.0;
float outputFreq = 0.0;

const int SpeakerTone = 4000;

byte deviceNumber = 0; // Initialize to 0 (TST Sensor 1)
bool buttonState = false;
bool prevButtonState = false;
bool isDimmed = false;

// Function declarations
bool MeasureAll(void);
void refreshDisplay(void);
void updateBatteryStatus(void);
void displayBatteryIcon(void);
void displayConnectionStatus(void);
void enterDeepSleep(void);

// FFT helper functions
void parabola(float x1, float y1, float x2, float y2, float x3, float y3, float *a, float *b, float *c)
{
  // Fit parabola y = ax² + bx + c through 3 points
  // For consecutive integer x values (i-1, i, i+1), this simplifies to:
  float denom = (x1 - x2) * (x1 - x3) * (x2 - x3);
  if (abs(denom) < 1e-10)
  {
    // Degenerate case - set to simple peak
    *a = 0;
    *b = 0;
    *c = y2;
    return;
  }

  *a = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / denom;
  *b = (x3 * x3 * (y1 - y2) + x2 * x2 * (y3 - y1) + x1 * x1 * (y2 - y3)) / denom;
  *c = (x2 * x3 * (x2 - x3) * y1 + x3 * x1 * (x3 - x1) * y2 + x1 * x2 * (x1 - x2) * y3) / denom;
}

void removeDC(float *data, int size)
{
  float mean = 0.0;
  for (int i = 0; i < size; i++)
  {
    mean += data[i];
  }
  mean /= size;

  for (int i = 0; i < size; i++)
  {
    data[i] -= mean;
  }
}

void applyHanningWindow(float *data, int size, float *window_correction)
{
  float sum_window = 0.0;

  for (int i = 0; i < size; i++)
  {
    float window_coeff = 0.5 * (1.0 - cos(2.0 * PI * i / (size - 1)));
    data[i] *= window_coeff;
    sum_window += window_coeff;
  }

  *window_correction = sum_window / size;
  Serial.printf("Window correction factor: %.4f (expected ~0.5)\n", *window_correction);
}

float performFFTAnalysis(float *input_data, int size)
{
  // Allocate memory for FFT
  float *fft_input = (float *)malloc(size * sizeof(float));
  float *fft_output = (float *)malloc(size * sizeof(float));

  if (!fft_input || !fft_output)
  {
    Serial.println("FFT memory allocation failed!");
    if (fft_input)
      free(fft_input);
    if (fft_output)
      free(fft_output);
    return 0.0;
  }

  // Copy input data
  for (int i = 0; i < size; i++)
  {
    fft_input[i] = input_data[i];
  }

  // Preprocessing
  float window_correction = 1.0;
  removeDC(fft_input, size);
  applyHanningWindow(fft_input, size, &window_correction);

  // Initialize and execute FFT
  fft_config_t *fft_plan = fft_init(size, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
  if (!fft_plan)
  {
    Serial.println("FFT initialization failed!");
    free(fft_input);
    free(fft_output);
    return 0.0;
  }

  fft_execute(fft_plan);

  // Find peak frequency with parabolic interpolation
  float max_magnitude = 0;
  int peak_bin = 0;
  float peak_frequency = 0;

  // First pass: find the peak bin in machine range (5-50 Hz)
  int start_bin = max(1, (int)(lowestFreq * size / SAMPLING_FREQ));
  int end_bin = min(size / 2 - 1, (int)(highestFreq * size / SAMPLING_FREQ));

  for (int k = start_bin; k <= end_bin; k++)
  {
    float real_part = fft_plan->output[2 * k];
    float imag_part = fft_plan->output[2 * k + 1];
    float magnitude = sqrt(real_part * real_part + imag_part * imag_part);

    if (magnitude > max_magnitude)
    {
      max_magnitude = magnitude;
      peak_bin = k;
    }
  }

  // Second pass: parabolic interpolation for sub-bin precision
  if (peak_bin > start_bin && peak_bin < end_bin && max_magnitude > 0)
  {
    // Get magnitudes of peak and neighboring bins
    float mag_left, mag_center, mag_right;

    // Left neighbor
    float real_left = fft_plan->output[2 * (peak_bin - 1)];
    float imag_left = fft_plan->output[2 * (peak_bin - 1) + 1];
    mag_left = sqrt(real_left * real_left + imag_left * imag_left);

    // Center (peak)
    float real_center = fft_plan->output[2 * peak_bin];
    float imag_center = fft_plan->output[2 * peak_bin + 1];
    mag_center = sqrt(real_center * real_center + imag_center * imag_center);

    // Right neighbor
    float real_right = fft_plan->output[2 * (peak_bin + 1)];
    float imag_right = fft_plan->output[2 * (peak_bin + 1) + 1];
    mag_right = sqrt(real_right * real_right + imag_right * imag_right);

    // Fit parabola through the 3 points
    float a, b, c;
    parabola(peak_bin - 1, mag_left, peak_bin, mag_center, peak_bin + 1, mag_right, &a, &b, &c);

    // Find parabola peak for sub-bin precision
    if (abs(a) > 1e-10)
    { // Valid parabola
      float precise_bin = -b / (2.0 * a);

      // Sanity check: interpolated peak should be near the original peak
      if (precise_bin >= (peak_bin - 1) && precise_bin <= (peak_bin + 1))
      {
        peak_frequency = (precise_bin * SAMPLING_FREQ) / size;
        Serial.printf("Interpolated: bin %.3f -> %.3f Hz (was bin %d -> %.3f Hz)\n",
                      precise_bin, peak_frequency, peak_bin, (peak_bin * SAMPLING_FREQ) / size);
      }
      else
      {
        // Fall back to simple bin frequency
        peak_frequency = (peak_bin * SAMPLING_FREQ) / size;
        Serial.printf("Interpolation out of range, using bin frequency: %.3f Hz\n", peak_frequency);
      }
    }
    else
    {
      // Fall back to simple bin frequency
      peak_frequency = (peak_bin * SAMPLING_FREQ) / size;
      Serial.printf("Invalid parabola, using bin frequency: %.3f Hz\n", peak_frequency);
    }
  }
  else
  {
    // Peak at edge or no valid peak, use simple bin frequency
    peak_frequency = (peak_bin * SAMPLING_FREQ) / size;
    Serial.printf("Edge case, using bin frequency: %.3f Hz\n", peak_frequency);
  }

  fft_destroy(fft_plan);
  free(fft_input);
  free(fft_output);

  return peak_frequency;
}

float calculateFFTAmplitude(float *input_data, int size)
{
  // Similar to performFFTAnalysis but returns amplitude instead of frequency
  float *fft_input = (float *)malloc(size * sizeof(float));
  float *fft_output = (float *)malloc(size * sizeof(float));

  if (!fft_input || !fft_output)
  {
    if (fft_input)
      free(fft_input);
    if (fft_output)
      free(fft_output);
    return 0.0;
  }

  for (int i = 0; i < size; i++)
  {
    fft_input[i] = input_data[i];
  }

  float window_correction = 1.0;
  removeDC(fft_input, size);
  applyHanningWindow(fft_input, size, &window_correction);

  fft_config_t *fft_plan = fft_init(size, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
  if (!fft_plan)
  {
    free(fft_input);
    free(fft_output);
    return 0.0;
  }

  fft_execute(fft_plan);

  // Find peak amplitude in machine range with interpolation
  float max_magnitude = 0;
  int peak_bin = 0;

  // First pass: find the peak bin in machine range (5-50 Hz)
  int start_bin = max(1, (int)(lowestFreq * size / SAMPLING_FREQ));
  int end_bin = min(size / 2 - 1, (int)(highestFreq * size / SAMPLING_FREQ));

  for (int k = start_bin; k <= end_bin; k++)
  {
    float real_part = fft_plan->output[2 * k];
    float imag_part = fft_plan->output[2 * k + 1];
    float magnitude = sqrt(real_part * real_part + imag_part * imag_part);

    if (magnitude > max_magnitude)
    {
      max_magnitude = magnitude;
      peak_bin = k;
    }
  }

  // Second pass: parabolic interpolation for more accurate amplitude
  float interpolated_magnitude = max_magnitude;

  if (peak_bin > start_bin && peak_bin < end_bin && max_magnitude > 0)
  {
    // Get magnitudes of peak and neighboring bins
    float mag_left, mag_center, mag_right;

    // Left neighbor
    float real_left = fft_plan->output[2 * (peak_bin - 1)];
    float imag_left = fft_plan->output[2 * (peak_bin - 1) + 1];
    mag_left = sqrt(real_left * real_left + imag_left * imag_left);

    // Center (peak)
    float real_center = fft_plan->output[2 * peak_bin];
    float imag_center = fft_plan->output[2 * peak_bin + 1];
    mag_center = sqrt(real_center * real_center + imag_center * imag_center);

    // Right neighbor
    float real_right = fft_plan->output[2 * (peak_bin + 1)];
    float imag_right = fft_plan->output[2 * (peak_bin + 1) + 1];
    mag_right = sqrt(real_right * real_right + imag_right * imag_right);

    // Fit parabola through the 3 points
    float a, b, c;
    parabola(peak_bin - 1, mag_left, peak_bin, mag_center, peak_bin + 1, mag_right, &a, &b, &c);

    // Find parabola peak magnitude
    if (abs(a) > 1e-10)
    { // Valid parabola
      float precise_bin = -b / (2.0 * a);

      // Calculate interpolated magnitude at parabola peak
      if (precise_bin >= (peak_bin - 1) && precise_bin <= (peak_bin + 1))
      {
        interpolated_magnitude = a * precise_bin * precise_bin + b * precise_bin + c;
      }
    }
  }

  // Convert magnitude to amplitude
  float amplitude = CORRECTION_FACTOR * ((interpolated_magnitude * 2) / (size * window_correction));

  fft_destroy(fft_plan);
  free(fft_input);
  free(fft_output);

  return amplitude;
}

// Amplitude filtering function
float MedianAmplitude(float raw_amplitude, int axis)
{
  // Step 1: Physical range check
  if (raw_amplitude < MIN_REASONABLE_AMPLITUDE)
  {
    raw_amplitude = 0.0;
  }
  else if (raw_amplitude > MAX_REASONABLE_AMPLITUDE)
  {
    Serial.printf("Warning: Amplitude %.3f g exceeds reasonable limit, clamping\n", raw_amplitude);
    raw_amplitude = MAX_REASONABLE_AMPLITUDE;
  }

  // Step 2: Add to appropriate median filter and return result
  switch (axis)
  {
  case 0: // X-axis
    ampX_median.add(raw_amplitude);
    return ampX_median.getMedian();
  case 1: // Y-axis
    ampY_median.add(raw_amplitude);
    return ampY_median.getMedian();
  case 2: // Z-axis
    ampZ_median.add(raw_amplitude);
    return ampZ_median.getMedian();
  default:
    return raw_amplitude; // No filtering
  }
}

// NimBLE Server Callbacks
class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("Client connected");
  }

  void onDisconnect(NimBLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("Client disconnected");
    // Restart advertising
    NimBLEDevice::startAdvertising();
  }
};


void setup()
{
  Serial.begin(115200);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));

  // Read-only: avoids any NVS flash write before M5.begin() initialises the PSRAM/SPI bus.
  // If calibration has never been run the namespace won't exist; defaults are used silently.
  if (preferences.begin("Calibration", true)) {
    Xzero    = preferences.getFloat("Xzero",   0.00);
    Xfactor  = preferences.getFloat("Xfactor", 1.00);
    Yzero    = preferences.getFloat("Yzero",   0.00);
    Yfactor  = preferences.getFloat("Yfactor", 1.00);
    Zzero    = preferences.getFloat("Zzero",   0.00);
    Zfactor  = preferences.getFloat("Zfactor", 1.00);
    preferences.end();
  }

  // Load device number from persistent storage (read-only on startup — no flash write risk).
  if (deviceSettings.begin("DeviceSettings", true)) {
    deviceNumber = deviceSettings.getUChar("deviceNum", 0);
    deviceSettings.end();
  }
  if (deviceNumber > 3)
    deviceNumber = 0; // Sanity check
  Serial.printf("Loaded device number: %d (%s)\n", deviceNumber, DEVICE_NAME[deviceNumber]);

  // Release any I2C slave stuck mid-transaction from a prior crash (SDA held low).
  // StickS3 internal I2C: SDA=GPIO47, SCL=GPIO48. Toggle SCL 9 times then STOP.
  pinMode(47, OUTPUT_OPEN_DRAIN);
  pinMode(48, OUTPUT_OPEN_DRAIN);
  digitalWrite(47, HIGH);
  digitalWrite(48, HIGH);
  for (int i = 0; i < 9; i++) {
    digitalWrite(48, LOW); delayMicroseconds(10);
    digitalWrite(48, HIGH); delayMicroseconds(10);
  }
  digitalWrite(47, LOW); delayMicroseconds(10);  // STOP: SDA low while SCL high
  digitalWrite(47, HIGH); delayMicroseconds(10);

  auto cfg = M5.config();
  M5.begin(cfg);

  // Set BMI270 accelerometer ODR to 800 Hz via M5Unified's internal I2C driver.
  // ACC_CONF (0x40): bit7=perf_mode, bits[6:4]=normal BW (0x2), bits[3:0]=800Hz ODR (0xC)
  M5.In_I2C.writeRegister8(0x68, 0x40, 0xAC, 400000);
  Serial.println("IMU sample rate set to 800 Hz");

  M5.Display.setRotation(1);
  M5.Display.setTextColor(WHITE);
  M5.Display.setFont(&fonts::FreeSansBold9pt7b);
  M5.Display.setTextSize(1);
  M5.Display.setBrightness(BRIGHTNESS_FULL);

  pinMode(buttonPin, INPUT);
  prevButtonState = digitalRead(buttonPin); // Initialize to actual state to prevent false trigger on boot
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Initialize NimBLE
  Serial.println("Initializing NimBLE...");
  NimBLEDevice::init(DEVICE_NAME[deviceNumber]);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Max power for better range

  // Create server
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create service
  pAccService = pServer->createService(BLE_UUID_ACC_SERVICE);

  // Create characteristics
  pFreqChar = pAccService->createCharacteristic(
      BLE_UUID_FREQ,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pAmpxChar = pAccService->createCharacteristic(
      BLE_UUID_AMPX,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pAmpyChar = pAccService->createCharacteristic(
      BLE_UUID_AMPY,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pAmpzChar = pAccService->createCharacteristic(
      BLE_UUID_AMPZ,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pModeChar = pAccService->createCharacteristic(
      BLE_UUID_MODE,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

  // Set initial values
  pFreqChar->setValue(outputFreq);
  pAmpxChar->setValue(outputAmpX);
  pAmpyChar->setValue(outputAmpY);
  pAmpzChar->setValue(outputAmpZ);

  // Start service
  pAccService->start();

  // Start advertising
  pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_UUID_ACC_SERVICE);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  NimBLEDevice::startAdvertising();

  Serial.println(DEVICE_NAME[deviceNumber]);
  Serial.println("NimBLE initialized and advertising");

  M5.update();
  M5.Display.setSwapBytes(true);
  M5.Display.setRotation(3);
  M5.Display.pushImage(0, 0, imageWidth, imageHeight, TSTLogo);
  M5.Display.setTextDatum(TL_DATUM); // Top-left alignment
  M5.Display.setFont(&fonts::FreeSans9pt7b);
  M5.Display.drawString(SOFTWARE_VERSION, 120, 20);
  delay(3000);

  M5.Display.fillScreen(BLACK);
  M5.Display.pushImage(0, 0, 145, 135, Arrows);
  refreshDisplay();

  // Clear amplitude filters
  ampX_median.clear();
  ampY_median.clear();
  ampZ_median.clear();

  BatLevel = M5.Power.getBatteryLevel();
  RaBatLvl.fillValue(BatLevel, 10);
  BatLvlAvg = BatLevel;
  batteryCheckTime = millis();

  Serial.println("Stored calibration values: ");
  Serial.println();
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

  Serial.printf("FFT: %d samples at %.0f Hz, %.2f Hz resolution\n",
                FFT_SIZE, SAMPLING_FREQ, SAMPLING_FREQ / FFT_SIZE);
}

void loop()
{
  M5.update();
  M5.Imu.update();

  // Button handling: short press = change device number, long press (≥2s held) = power off
  if (millis() - pressTime > debounceInterval)
  {
    buttonState = digitalRead(buttonPin);
    if (buttonState != prevButtonState)
    {
      if (buttonState == false && prevButtonState == true && millis() > startupDelay)
      {
        // Button pressed — record start time and beep
        pressStartTime = millis();
        M5.Speaker.tone(SpeakerTone, 50);
      }

      if (buttonState == true && prevButtonState == false && millis() > startupDelay)
      {
        // Button released
        pressTime = millis();
        lastActivityTime = millis();

        // Short press only (long press is handled mid-hold below)
        if (!deviceConnected && (millis() - pressStartTime) < 2000)
        {
          if (deviceNumber < 3)
          {
            deviceNumber++;
          }
          else
          {
            deviceNumber = 0;
          }
          // Save device number to persistent storage (open R/W only when actually writing).
          deviceSettings.begin("DeviceSettings", false);
          deviceSettings.putUChar("deviceNum", deviceNumber);
          deviceSettings.end();
          Serial.printf("Saved device number: %d\n", deviceNumber);

          refreshDisplay();
          Serial.println(DEVICE_NAME[deviceNumber]);
          // Update BLE device name (requires restart advertising)
          NimBLEDevice::deinit(true);
          delay(100);
          NimBLEDevice::init(DEVICE_NAME[deviceNumber]);
          // Reinitialize BLE (simplified - in production you'd refactor this)
          pServer = NimBLEDevice::createServer();
          pServer->setCallbacks(new ServerCallbacks());
          pAccService = pServer->createService(BLE_UUID_ACC_SERVICE);
          pFreqChar = pAccService->createCharacteristic(BLE_UUID_FREQ, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
          pAmpxChar = pAccService->createCharacteristic(BLE_UUID_AMPX, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
          pAmpyChar = pAccService->createCharacteristic(BLE_UUID_AMPY, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
          pAmpzChar = pAccService->createCharacteristic(BLE_UUID_AMPZ, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
          pModeChar = pAccService->createCharacteristic(BLE_UUID_MODE, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
          pAccService->start();
          pAdvertising = NimBLEDevice::getAdvertising();
          pAdvertising->addServiceUUID(BLE_UUID_ACC_SERVICE);
          NimBLEDevice::startAdvertising();
        }
      }
    }
    prevButtonState = buttonState;
  }

  // Long press: power off while still holding after 2 seconds
  if (buttonState == false && pressStartTime > 0 && (millis() - pressStartTime) >= 2000)
  {
    pressStartTime = 0; // prevent repeated triggers
    enterDeepSleep();
  }

  // Periodic battery status check
  if (millis() - batteryCheckTime >= BATTERY_CHECK_INTERVAL)
  {
    batteryCheckTime = millis();
    updateBatteryStatus();
    unsigned long elapsed = millis() - lastActivityTime;
    unsigned long remaining = (elapsed < autoOff) ? (autoOff - elapsed) / 1000 : 0;
    Serial.println();
    Serial.printf("BatLvl: %d%%, Auto-off in %lus (timeout: %lus)\n", BatLvlAvg, remaining, autoOff / 1000);
    Serial.println();
  }

  // Periodic display refresh
  if (millis() - refreshTime >= refreshInterval)
  {
    refreshTime = millis();
    refreshDisplay();
  }

  // Display brightness control based on activity
  if (millis() - lastActivityTime > DIM_TIMEOUT)
  {
    if (!isDimmed)
    {
      M5.Display.setBrightness(BRIGHTNESS_DIM);
      isDimmed = true;
    }
  }
  else
  {
    if (isDimmed)
    {
      M5.Display.setBrightness(BRIGHTNESS_FULL);
      isDimmed = false;
    }
  }

  // Handle connection state changes
  if (deviceConnected && !prevDeviceConnected)
  {
    prevDeviceConnected = true;
    M5.Display.pushImage(0, 0, 145, 135, Arrows);
    M5.Display.pushImage(100, 0, 40, 75, Bluetooth);
    Serial.println("Device connected");
  }

  if (!deviceConnected && prevDeviceConnected)
  {
    prevDeviceConnected = false;
    autoOff = 120000; // 2 minutes after BLE disconnect
    lastActivityTime = millis();
    Serial.println("Device disconnected");
  }

  // Main measurement loop when connected
  if (deviceConnected)
  {
    if (MeasureAll())
    {
      pFreqChar->setValue(outputFreq);
      pFreqChar->notify();
      pAmpxChar->setValue(outputAmpX);
      pAmpxChar->notify();
      pAmpyChar->setValue(outputAmpY);
      pAmpyChar->notify();
      pAmpzChar->setValue(outputAmpZ);
      pAmpzChar->notify();
    }
  }

  // Auto power off (timeout varies by state: 5min default, 2min after BLE, 1min when full)
  if (millis() - lastActivityTime > autoOff)
  {
    enterDeepSleep();
  }
}

bool MeasureAll(void)
{
  static bool buffer_primed = false;
  static int priming_pos = 0;
  static unsigned long collection_start = 0;

  collection_start = millis();

  if (!buffer_primed)
  {
    // Fill buffer incrementally HOP_SIZE samples at a time until FFT_SIZE is reached
    int to_collect = min(HOP_SIZE, FFT_SIZE - priming_pos);
    for (int i = 0; i < to_collect; i++)
    {
      unsigned long newTime = micros();
      M5.Imu.update();
      auto data = M5.Imu.getImuData();
      measure_buffers[0][priming_pos + i] = (data.accel.x - Xzero) * Xfactor;
      measure_buffers[1][priming_pos + i] = (data.accel.y - Yzero) * Yfactor;
      measure_buffers[2][priming_pos + i] = (data.accel.z - Zzero) * Zfactor;
      while ((micros() - newTime) < sampling_period_us) {}
    }
    priming_pos += to_collect;
    Serial.printf("Priming: %d/%d samples (%lu ms)\n", priming_pos, FFT_SIZE, millis() - collection_start);
    if (priming_pos < FFT_SIZE)
      return false;
    buffer_primed = true;
    // Fall through to run FFT on the now-full buffer
  }
  else
  {
    // Slide buffer left by HOP_SIZE, collect HOP_SIZE new samples at the end
    for (int axis = 0; axis < 3; axis++)
    {
      memmove(measure_buffers[axis],
              measure_buffers[axis] + HOP_SIZE,
              (FFT_SIZE - HOP_SIZE) * sizeof(float));
    }
    for (int i = 0; i < HOP_SIZE; i++)
    {
      unsigned long newTime = micros();
      M5.Imu.update();
      auto data = M5.Imu.getImuData();
      int idx = FFT_SIZE - HOP_SIZE + i;
      measure_buffers[0][idx] = (data.accel.x - Xzero) * Xfactor;
      measure_buffers[1][idx] = (data.accel.y - Yzero) * Yfactor;
      measure_buffers[2][idx] = (data.accel.z - Zzero) * Zfactor;
      while ((micros() - newTime) < sampling_period_us) {}
    }
    Serial.printf("Hop collection: %lu ms\n", millis() - collection_start);
  }

  // Run amplitude FFT on all 3 axes, frequency FFT on best axis
  float raw_amps[3];
  for (int axis = 0; axis < 3; axis++)
    raw_amps[axis] = calculateFFTAmplitude(measure_buffers[axis], FFT_SIZE);

  int best_axis = 0;
  for (int i = 1; i < 3; i++)
    if (raw_amps[i] > raw_amps[best_axis]) best_axis = i;

  float detected_freq = performFFTAnalysis(measure_buffers[best_axis], FFT_SIZE);

  if (raw_amps[best_axis] >= 0.1 && detected_freq >= lowestFreq && detected_freq <= highestFreq)
  {
    outputFreq = detected_freq;
    autoOff = maxAutoOff;
    lastActivityTime = millis();
  }
  else
  {
    outputFreq = 0.0;
  }

  outputAmpX = MedianAmplitude(raw_amps[0], 0);
  outputAmpY = MedianAmplitude(raw_amps[1], 1);
  outputAmpZ = MedianAmplitude(raw_amps[2], 2);

  if (outputAmpX > 0.1 || outputAmpY > 0.1 || outputAmpZ > 0.1)
  {
    autoOff = maxAutoOff;
    lastActivityTime = millis();
  }

  Serial.printf("Freq: %.2f Hz (axis %d) | Amp X=%.3f Y=%.3f Z=%.3f | total %lu ms\n",
    outputFreq, best_axis, outputAmpX, outputAmpY, outputAmpZ, millis() - collection_start);

  refreshDisplay();
  return true;
}

void refreshDisplay(void)
{
  // Display device number using array lookup
  if (deviceNumber < 4)
  {
    M5.Display.pushImage(145, 0, 95, 85, DEVICE_IMAGES[deviceNumber]);
  }

  displayBatteryIcon();
  displayConnectionStatus();
}

void updateBatteryStatus(void)
{
  BatLevel = M5.Power.getBatteryLevel();
  RaBatLvl.addValue(BatLevel);
  BatLvlAvg = RaBatLvl.getAverage();

  if (M5.Power.isCharging())
  {
    lastActivityTime = millis();
    Serial.println("Charging detected");
  }
}

void displayBatteryIcon(void)
{
  // Select battery icon based on level
  const uint16_t *batteryIcon;
  if (BatLvlAvg > BATTERY_LEVEL_MID)
  {
    batteryIcon = BatteryFull;
  }
  else if (BatLvlAvg > BATTERY_LEVEL_LOW)
  {
    batteryIcon = BatteryHalf;
  }
  else
  {
    batteryIcon = BatteryEmpty;
  }

  M5.Display.pushImage(145, 85, 95, 50, batteryIcon);

  // Display battery percentage
  if (millis() - updateBatTime > updateBatInterval)
  {
    M5.Display.setTextColor(WHITE);
    M5.Display.setFont(&fonts::FreeSansBold9pt7b);
    M5.Display.setCursor(175, 97);
    M5.Display.printf("%3d%%", BatLvlAvg);
  }
}

void displayConnectionStatus(void)
{
  if (deviceConnected)
  {
    M5.Display.pushImage(100, 0, 40, 75, Bluetooth);
  }
  else
  {
    M5.Display.pushImage(0, 0, 145, 135, Arrows);
  }
}

void enterDeepSleep(void)
{
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextDatum(MC_DATUM);
  M5.Display.setFont(&fonts::FreeSansBold9pt7b);
  M5.Display.drawString("Sleeping...", M5.Display.width() / 2, M5.Display.height() / 2);
  delay(500);

  // Stop BLE radio — largest current draw when awake
  NimBLEDevice::deinit(true);

  // Send SLEEP_IN command to display controller and cut backlight
  M5.Display.sleep();

  // Ensure IR LED is off
  digitalWrite(ledPin, LOW);

  // Disable M5PM1 LED power (register 0x06 bit 4) to turn off the RGB status LED.
  // Note: if USB is connected the charger STAT pin drives the green LED in hardware —
  // that one cannot be turned off in software.
  M5.In_I2C.bitOff(0x6E, 0x06, 1 << 4, 100000);

  // Ensure PA (speaker amplifier) stays off: M5PM1 GPIO3 = LOW (register 0x11 bit 3)
  M5.In_I2C.bitOff(0x6E, 0x11, 1 << 3, 100000);

  // Suspend the BMI270 IMU. It runs continuously at 800 Hz in full performance
  // mode (~850 uA) and its power rail stays on through ESP32 deep sleep, so
  // without this it alone draws far more current than the whole sleeping ESP32.
  M5.In_I2C.writeRegister8(0x68, 0x7D, 0x00, 400000); // PWR_CTRL: disable acc/gyr/aux/temp
  M5.In_I2C.writeRegister8(0x68, 0x7C, 0x01, 400000); // PWR_CONF: enable advanced power save (suspend)

  // Wait for KEY1 to be released before arming the wakeup.
  // If the button is still held when esp_deep_sleep_start() is called the LOW
  // wakeup condition is already true and the device wakes up immediately.
  while (digitalRead(buttonPin) == LOW) {
    delay(10);
  }
  delay(50); // debounce after release

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_11, LOW); // KEY1 wakes device
  esp_deep_sleep_start();
}
