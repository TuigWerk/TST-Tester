#include <M5StickCPlus2.h>
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

Preferences preferences;
Preferences deviceSettings; // Separate namespace for device settings (read-write)

const char *DEVICE_NAME[] = {"TST Sensor 1", "TST Sensor 2", "TST Sensor 3", "TST Sensor 4"};

#define SOFTWARE_VERSION "v5.7 Unified"

// Change Log:
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
NimBLECharacteristic *pModeChar = nullptr;
NimBLEAdvertising *pAdvertising = nullptr;

bool deviceConnected = false;
bool prevDeviceConnected = false;
uint8_t prevModus = 255; // Track mode changes

// FFT Configuration
#define SAMPLING_FREQ 500.0
#define FFT_SIZE 1024              // Fixed sample size
#define AXIS_DETECTION_SAMPLES 200 // 0.4 seconds at 500Hz
#define MEASURE_FFT_SIZE 1024      // For amplitude measurement

// Simplified FFT variables
bool axis_detection_complete = false;
int fft_sample_count = 0;

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
const uint8_t BRIGHTNESS_DIM = 2;
const unsigned long DIM_TIMEOUT = 30000; // Dim after 30 seconds of inactivity

// Axis detection
enum Axis
{
  X_AXIS = 0,
  Y_AXIS = 1,
  Z_AXIS = 2
};

struct AxisStats
{
  float max_value;
  float min_value;
  const char *name;
};

AxisStats axis_stats[3] = {
    {-999, 999, "X"},
    {-999, 999, "Y"},
    {-999, 999, "Z"}};

Axis selected_axis = X_AXIS;

// Simplified sampling buffers
float freq_buffer[FFT_SIZE];                // Single buffer for frequency detection
float measure_buffers[3][MEASURE_FFT_SIZE]; // For amplitude measurement [axis][sample]

unsigned int sampling_period_us;

// Running average Battery
RunningAverage RaBatLvl(10);

// Amplitude filtering objects
RunningMedian ampX_median(MEDIAN_FILTER_SIZE);
RunningMedian ampY_median(MEDIAN_FILTER_SIZE);
RunningMedian ampZ_median(MEDIAN_FILTER_SIZE);

float outputAmpX = 0, outputAmpY = 0, outputAmpZ = 0;

const int buttonPin = 35;
const int ledPin = 19;

const uint16_t imageWidth = 240;
const uint16_t imageHeight = 135;

int BatLevel;
int BatLvlAvg;
int BatLvlPrevious = 0; // Battery level from 1 min ago for charge detection
unsigned long chargeCheckTime = 0;
const unsigned long CHARGE_CHECK_INTERVAL = 60000; // Compare battery level every minute

float sampleFreq = 500;

float Xzero, Yzero, Zzero;
float Xfactor, Yfactor, Zfactor;

unsigned long printTime;
unsigned long printInterval = 750;
unsigned long ledTime;
unsigned long ledInterval = 2000;
unsigned long pressTime;
unsigned long debounceInterval = 200;
unsigned long refreshTime;
unsigned long refreshInterval = 2000;
unsigned long updateBatTime;
unsigned long updateBatInterval = 5000;
unsigned long batteryCheckTime = 0;
const unsigned long BATTERY_CHECK_INTERVAL = 30000; // Check battery every 30 seconds
unsigned long startupDelay = 5000;
unsigned long connectTime;
unsigned long lastActivityTime;
unsigned long maxAutoOff = 300000; // Maximum auto-off time (5 minutes)
unsigned long autoOff = maxAutoOff;
float lowestFreq = 2.0;
float highestFreq = 60.0;
float outputFreq = 0.0;

const int SpeakerTone = 4000;

byte Modus = 0;
byte wasConnected = 0;
byte deviceNumber = 0; // Initialize to 0 (TST Sensor 1)
bool buttonState = false;
bool prevButtonState = false;
bool isDimmed = false;

// Function declarations
void FindFreq(void);
void Measure(void);
void refreshDisplay(void);
void updateBatteryStatus(void);
void displayBatteryIcon(void);
void displayConnectionStatus(void);

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

// NimBLE Characteristic Callbacks for Mode
class ModeCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0)
    {
      Modus = value[0];
      Serial.printf("Mode changed to: %d\n", Modus);
    }
  }
};

void setup()
{
  Serial.begin(115200);
  preferences.begin("Calibration", true);

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));

  Xzero = preferences.getFloat("Xzero", 0.00);
  Xfactor = preferences.getFloat("Xfactor", 1.00);
  Yzero = preferences.getFloat("Yzero", 0.00);
  Yfactor = preferences.getFloat("Yfactor", 1.00);
  Zzero = preferences.getFloat("Zzero", 0.00);
  Zfactor = preferences.getFloat("Zfactor", 1.00);

  // Load device number from persistent storage
  deviceSettings.begin("DeviceSettings", false); // false = read-write mode
  deviceNumber = deviceSettings.getUChar("deviceNum", 0);
  if (deviceNumber > 3)
    deviceNumber = 0; // Sanity check
  Serial.printf("Loaded device number: %d (%s)\n", deviceNumber, DEVICE_NAME[deviceNumber]);

  auto cfg = M5.config();
  StickCP2.begin(cfg);

  // Increase IMU sample rate to 1kHz (SMPLRT_DIV = 0)
  Wire1.beginTransmission(0x68); // MPU6886 I2C address
  Wire1.write(0x19);             // SMPLRT_DIV register
  Wire1.write(0x00);             // Divider = 0 -> 1000 Hz output rate
  Wire1.endTransmission();
  Serial.println("IMU sample rate set to 1kHz");

  StickCP2.Display.setRotation(1);
  StickCP2.Display.setTextColor(WHITE);
  StickCP2.Display.setFont(&fonts::FreeSansBold9pt7b);
  StickCP2.Display.setTextSize(1);
  StickCP2.Display.setBrightness(BRIGHTNESS_FULL);

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
  pModeChar->setCallbacks(new ModeCallbacks());

  // Set initial values
  pFreqChar->setValue(outputFreq);
  pAmpxChar->setValue(outputAmpX);
  pAmpyChar->setValue(outputAmpY);
  pAmpzChar->setValue(outputAmpZ);
  pModeChar->setValue(&Modus, 1);

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

  StickCP2.update();
  StickCP2.Display.setSwapBytes(true);
  StickCP2.Display.setRotation(3);
  StickCP2.Display.pushImage(0, 0, imageWidth, imageHeight, TSTLogo);
  StickCP2.Display.setTextDatum(TL_DATUM); // Top-left alignment
  StickCP2.Display.setFont(&fonts::FreeSans9pt7b);
  StickCP2.Display.drawString(SOFTWARE_VERSION, 120, 20);
  delay(3000);

  StickCP2.Display.fillScreen(BLACK);
  StickCP2.Display.pushImage(0, 0, 145, 135, Arrows);
  refreshDisplay();

  // Clear amplitude filters
  ampX_median.clear();
  ampY_median.clear();
  ampZ_median.clear();

  BatLevel = StickCP2.Power.getBatteryLevel();
  RaBatLvl.fillValue(BatLevel, 10);
  BatLvlAvg = BatLevel;      // Initialize average for display
  BatLvlPrevious = BatLevel; // Initialize for charge detection
  chargeCheckTime = millis();
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

  // Simple FFT initialization
  axis_detection_complete = false;
  fft_sample_count = 0;

  Serial.printf("FFT initialized: %d samples at %.0f Hz (%.2f Hz resolution)\n",
                FFT_SIZE, SAMPLING_FREQ, SAMPLING_FREQ / FFT_SIZE);

  float test_signal[MEASURE_FFT_SIZE];
  for (int i = 0; i < MEASURE_FFT_SIZE; i++)
  {
    test_signal[i] = 2.0 * sin(2.0 * PI * 10.0 * i / 500.0); // 2g amplitude at 10Hz
  }
  float test_amplitude = calculateFFTAmplitude(test_signal, MEASURE_FFT_SIZE);
  Serial.printf("Test signal amplitude: Expected=2.0g, Measured=%.3fg\n", test_amplitude);
}

void loop()
{
  StickCP2.update();
  StickCP2.Imu.update();

  // Handle button release for device number change (not press, to avoid changing when turning off)
  if (millis() - pressTime > debounceInterval)
  {
    buttonState = digitalRead(buttonPin);
    if (buttonState != prevButtonState)
    {
      if (buttonState == true && prevButtonState == false && millis() > startupDelay)
      {
        // Button released - reset activity timer to brighten display
        lastActivityTime = millis();

        // Only change device number when not connected via BLE
        if (!deviceConnected)
        {
          StickCP2.Speaker.tone(SpeakerTone, 50);
          if (deviceNumber < 3)
          {
            deviceNumber++;
          }
          else
          {
            deviceNumber = 0;
          }
          // Save device number to persistent storage
          deviceSettings.putUChar("deviceNum", deviceNumber);
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
          pModeChar->setCallbacks(new ModeCallbacks());
          pAccService->start();
          pAdvertising = NimBLEDevice::getAdvertising();
          pAdvertising->addServiceUUID(BLE_UUID_ACC_SERVICE);
          NimBLEDevice::startAdvertising();
        }
      }
    }
    prevButtonState = buttonState;
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
      StickCP2.Display.setBrightness(BRIGHTNESS_DIM);
      isDimmed = true;
    }
  }
  else
  {
    if (isDimmed)
    {
      StickCP2.Display.setBrightness(BRIGHTNESS_FULL);
      isDimmed = false;
    }
  }

  // Handle connection state changes
  if (deviceConnected && !prevDeviceConnected)
  {
    // Just connected
    connectTime = millis();
    prevDeviceConnected = true;
    StickCP2.Display.pushImage(0, 0, 145, 135, Arrows);
    StickCP2.Display.pushImage(100, 0, 40, 75, Bluetooth);
    Serial.println("Device connected - ready for commands");
  }

  if (!deviceConnected && prevDeviceConnected)
  {
    // Just disconnected
    prevDeviceConnected = false;
    Modus = 0;
    prevModus = 255;
    wasConnected = 1;
    autoOff = 120000; // 2 minutes after BLE disconnect
    lastActivityTime = millis();
    Serial.println("Device disconnected");
  }

  // Handle mode changes
  if (deviceConnected && Modus != prevModus)
  {
    prevModus = Modus;
    StickCP2.Display.pushImage(0, 0, 145, 135, Arrows);
    StickCP2.Display.pushImage(100, 0, 40, 75, Bluetooth);

    // Reset FFT state when mode changes
    axis_detection_complete = false;
    fft_sample_count = 0;

    // Reset axis stats
    for (int i = 0; i < 3; i++)
    {
      axis_stats[i].max_value = -999;
      axis_stats[i].min_value = 999;
    }

    // Clear amplitude filters
    ampX_median.clear();
    ampY_median.clear();
    ampZ_median.clear();

    Serial.printf("Mode changed to %d - FFT state reset\n", Modus);
  }

  // Main measurement loop when connected
  if (deviceConnected)
  {
    wasConnected = 1;

    if (millis() - connectTime >= startupDelay)
    {
      if (Modus == 0)
      {
        FindFreq();
        // Send frequency data
        pFreqChar->setValue(outputFreq);
        pFreqChar->notify();
        pAmpxChar->setValue(outputAmpX);
        pAmpxChar->notify();
        pAmpyChar->setValue(outputAmpY);
        pAmpyChar->notify();
        pAmpzChar->setValue(outputAmpZ);
        pAmpzChar->notify();
        // pAmpxChar->setValue(0.0f);
        // pAmpyChar->setValue(0.0f);
        // pAmpzChar->setValue(0.0f);
      }
      else
      {
        Measure();
        if (millis() - printTime >= printInterval)
        {
          printTime = millis();
          // Send amplitude data
          // pFreqChar->setValue(outputFreq);
          // pFreqChar->notify();
          pAmpxChar->setValue(outputAmpX);
          pAmpxChar->notify();
          pAmpyChar->setValue(outputAmpY);
          pAmpyChar->notify();
          pAmpzChar->setValue(outputAmpZ);
          pAmpzChar->notify();
        }
        if (millis() - ledTime >= ledInterval)
        {
          digitalWrite(ledPin, HIGH);
          delay(20);
          digitalWrite(ledPin, LOW);
          ledTime = millis();
        }
        if (millis() - refreshTime >= refreshInterval)
        {
          refreshTime = millis();
          refreshDisplay();
        }
      }
    }
  }

  // Auto power off (timeout varies by state: 5min default, 2min after BLE, 1min when full)
  if (millis() - lastActivityTime > autoOff)
  {
    StickCP2.Power.powerOff();
  }
}

// SIMPLIFIED FINDFREQ FUNCTION - Fixed timing issues!
void FindFreq(void)
{
  static unsigned long collection_start_time = 0;

  // Phase 1: Axis Detection (run once)
  if (!axis_detection_complete)
  {
    Serial.println("Starting axis detection...");

    // Collect samples rapidly for axis detection
    for (int i = 0; i < AXIS_DETECTION_SAMPLES; i++)
    {
      unsigned long newTime = micros();
      StickCP2.Imu.update();
      auto data = StickCP2.Imu.getImuData();

      float samples[3] = {
          (data.accel.x - Xzero) * Xfactor,
          (data.accel.y - Yzero) * Yfactor,
          (data.accel.z - Zzero) * Zfactor};

      // Update axis statistics
      for (int axis = 0; axis < 3; axis++)
      {
        float value = samples[axis];
        if (value > axis_stats[axis].max_value)
        {
          axis_stats[axis].max_value = value;
        }
        if (value < axis_stats[axis].min_value)
        {
          axis_stats[axis].min_value = value;
        }
      }

      // More precise timing - wait for exact interval
      while ((micros() - newTime) < sampling_period_us)
      {
        /* chill */
      }
    }

    // Select axis with highest range
    float best_range = 0;
    int best_axis = 0;

    Serial.println("Axis Detection Complete:");
    for (int axis = 0; axis < 3; axis++)
    {
      float range = axis_stats[axis].max_value - axis_stats[axis].min_value;
      Serial.printf("  %s-Axis: Range=%.3fg\n", axis_stats[axis].name, range);

      if (range > best_range)
      {
        best_range = range;
        best_axis = axis;
      }
    }

    selected_axis = (Axis)best_axis;
    axis_detection_complete = true;
    fft_sample_count = 0;
    collection_start_time = millis(); // Start timing collection phase

    Serial.printf("Selected Axis: %s (Range: %.3fg)\n",
                  axis_stats[selected_axis].name, best_range);
    Serial.printf("Starting FFT collection: %d samples at %.0f Hz (should take %.1f seconds)\n",
                  FFT_SIZE, SAMPLING_FREQ, FFT_SIZE / SAMPLING_FREQ);
    return; // Exit early during axis detection
  }

  // Phase 2: Fast sample collection in tight loop
  if (fft_sample_count == 0)
  {
    Serial.println("Starting fast sample collection...");
    collection_start_time = millis();

    // Collect ALL samples in one tight loop for best timing
    for (int i = 0; i < FFT_SIZE; i++)
    {
      unsigned long sample_start = micros();

      StickCP2.Imu.update();
      auto data = StickCP2.Imu.getImuData();

      float selected_sample;
      switch (selected_axis)
      {
      case X_AXIS:
        selected_sample = (data.accel.x - Xzero) * Xfactor;
        break;
      case Y_AXIS:
        selected_sample = (data.accel.y - Yzero) * Yfactor;
        break;
      case Z_AXIS:
        selected_sample = (data.accel.z - Zzero) * Zfactor;
        break;
      }

      freq_buffer[i] = selected_sample;

      // Precise timing - wait for exact 2ms interval (500Hz)
      while ((micros() - sample_start) < sampling_period_us)
      {
        /* tight timing loop */
      }
    }

    fft_sample_count = FFT_SIZE; // Mark collection complete
    unsigned long collection_time = millis() - collection_start_time;
    Serial.printf("Sample collection complete in %lu ms (expected: %.0f ms)\n",
                  collection_time, (FFT_SIZE * 1000.0) / SAMPLING_FREQ);
    return; // Exit to perform FFT on next call
  }

  // Phase 3: Perform FFT when buffer is full
  if (fft_sample_count >= FFT_SIZE)
  {
    Serial.printf("Performing FFT on %d samples...\n", FFT_SIZE);

    unsigned long fft_start = millis();
    float detected_freq = performFFTAnalysis(freq_buffer, FFT_SIZE);
    unsigned long fft_time = millis() - fft_start;

    // Get amplitude for the detected frequency to determine if signal is real
    float fft_amplitude = calculateFFTAmplitude(freq_buffer, FFT_SIZE);

    // Amplitude-based filtering: only output frequency if signal is strong enough
    if (fft_amplitude < 0.1)
    {
      outputFreq = 0.0; // Too weak = likely noise, output 0 Hz
      Serial.printf("Weak signal (%.3f g), frequency rejected\n", fft_amplitude);
    }
    else if (detected_freq < 1.0 || detected_freq > 60.0)
    {
      outputFreq = 0.0; // Keep basic range check for obviously wrong frequencies
      Serial.printf("Frequency %.2f Hz out of range (1-60 Hz), rejected\n", detected_freq);
    }
    else
    {
      outputFreq = detected_freq;  // Strong signal in valid range = accept
      autoOff = maxAutoOff;        // Extend timeout to maxAutoOff minutes when vibration detected
      lastActivityTime = millis(); // Reset auto-off timer
      Serial.printf("Strong signal (%.3f g), frequency: %.2f Hz accepted\n", fft_amplitude, outputFreq);
    }

    Serial.printf("FFT complete in %lu ms - Raw: %.2f Hz, Output: %.2f Hz (%.0f RPM)\n",
                  fft_time, detected_freq, outputFreq, outputFreq * 60.0);

    // Reset for next collection cycle
    fft_sample_count = 0;
    collection_start_time = 0;
  }

  // Only update display and LED when not collecting samples
  if (fft_sample_count == 0)
  {
    static unsigned long last_display_update = 0;
    if (millis() - last_display_update > 1000)
    { // Update display only once per second
      refreshDisplay();
      last_display_update = millis();
    }

    // Quick LED blink
    digitalWrite(ledPin, HIGH);
    delayMicroseconds(10000); // 10ms blink
    digitalWrite(ledPin, LOW);
  }
}

void Measure(void)
{
  static bool collection_complete = false;
  static unsigned long last_measure_start = 0;

  // Phase 1: Collect ALL samples in one tight loop
  if (!collection_complete)
  {
    last_measure_start = millis();
    Serial.println("Starting simultaneous 3-axis measurement...");

    // Collect ALL samples in one uninterrupted loop
    for (int i = 0; i < MEASURE_FFT_SIZE; i++)
    {
      unsigned long newTime = micros();
      StickCP2.Imu.update();
      auto data = StickCP2.Imu.getImuData();

      // Store samples for all three axes simultaneously
      measure_buffers[0][i] = (data.accel.x - Xzero) * Xfactor; // X-axis
      measure_buffers[1][i] = (data.accel.y - Yzero) * Yfactor; // Y-axis
      measure_buffers[2][i] = (data.accel.z - Zzero) * Zfactor; // Z-axis

      // Wait for precise timing
      while ((micros() - newTime) < sampling_period_us)
      {
        /* chill */
      }
    }

    collection_complete = true;
    unsigned long collection_time = millis() - last_measure_start;
    Serial.printf("Sample collection complete in %lu ms. Performing 3 FFTs...\n", collection_time);
    return;
  }

  // Phase 2: Process FFTs and output raw data for Serial Plotter
  if (collection_complete)
  {

    // Continue with normal FFT processing
    float amplitudes[3];
    float median_amplitudes[3];
    const char *axis_names[] = {"X", "Y", "Z"};

    for (int axis = 0; axis < 3; axis++)
    {
      amplitudes[axis] = calculateFFTAmplitude(measure_buffers[axis], MEASURE_FFT_SIZE);
      median_amplitudes[axis] = MedianAmplitude(amplitudes[axis], axis);

      Serial.printf("%s-axis: raw= %.3f g, median= %.3f g\n",
                    axis_names[axis], amplitudes[axis], median_amplitudes[axis]);
    }

    // Update amplitude values
    outputAmpX = median_amplitudes[0];
    outputAmpY = median_amplitudes[1];
    outputAmpZ = median_amplitudes[2];

    // Reset auto-off timer when significant vibration detected on any axis
    if (outputAmpX > 0.1 || outputAmpY > 0.1 || outputAmpZ > 0.1)
    {
      autoOff = maxAutoOff; // Extend timeout to maxAutoOff minutes when vibration detected
      lastActivityTime = millis();
    }

    unsigned long total_time = millis() - last_measure_start;
    Serial.printf("Complete 3-axis measurement cycle: %lu ms\n", total_time);
    Serial.println("----");

    collection_complete = false;
  }
}

void refreshDisplay(void)
{
  // Display device number using array lookup
  if (deviceNumber < 4)
  {
    StickCP2.Display.pushImage(145, 0, 95, 85, DEVICE_IMAGES[deviceNumber]);
  }

  displayBatteryIcon();
  displayConnectionStatus();
}

void updateBatteryStatus(void)
{
  // Update battery readings
  BatLevel = StickCP2.Power.getBatteryLevel();
  RaBatLvl.addValue(BatLevel);
  BatLvlAvg = RaBatLvl.getAverage();

  // Check if battery level is rising (charging detection)
  if (millis() - chargeCheckTime >= CHARGE_CHECK_INTERVAL)
  {
    if (BatLvlPrevious > 0 && BatLvlAvg > BatLvlPrevious)
    {
      // Battery level is rising - charging, keep device on
      lastActivityTime = millis();
      Serial.printf("Charging detected: battery level rose %d%%\n", BatLvlAvg - BatLvlPrevious);
    }
    BatLvlPrevious = BatLvlAvg;
    chargeCheckTime = millis();
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

  StickCP2.Display.pushImage(145, 85, 95, 50, batteryIcon);

  // Display battery percentage
  if (millis() - updateBatTime > updateBatInterval)
  {
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setFont(&fonts::FreeSansBold9pt7b);
    StickCP2.Display.setCursor(175, 97);
    StickCP2.Display.printf("%3d%%", BatLvlAvg);
  }
}

void displayConnectionStatus(void)
{
  if (deviceConnected)
  {
    StickCP2.Display.pushImage(100, 0, 40, 75, Bluetooth);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.setFont(&fonts::FreeSansBold9pt7b);
    StickCP2.Display.setCursor(80, 23);

    // Display mode indicator
    StickCP2.Display.printf("%c", (Modus == 0) ? 'F' : 'A');
  }
  else
  {
    StickCP2.Display.pushImage(0, 0, 145, 135, Arrows);
  }
}
