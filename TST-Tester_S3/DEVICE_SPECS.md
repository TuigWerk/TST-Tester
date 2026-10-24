# TST Tester S3 — Device Specifications

A battery-powered, handheld vibration analyzer built on an M5Stick S3 (ESP32-S3). It measures 3-axis acceleration, extracts the dominant vibration frequency and per-axis amplitude via FFT, and streams the results over Bluetooth Low Energy (BLE) to a companion app. Up to four units can be told apart by a selectable device profile ("TST Sensor 1–4").

## Hardware

| Item | Spec |
|---|---|
| MCU | ESP32-S3 (M5Stick S3) |
| Sensor | Bosch BMI270, 6-axis IMU (accelerometer used for measurement) |
| Sensor interface | I2C, address 0x68, 400 kHz |
| Display | 240 × 135 px color LCD |
| Connectivity | Bluetooth Low Energy (NimBLE), max TX power |
| Power | Internal rechargeable battery, USB charging |
| Controls | 1 button (mode select / power) |

## Measurement Specifications

| Parameter | Value | Notes |
|---|---|---|
| **Accelerometer range** | ±8 g | Fixed; not user-adjustable |
| **Accelerometer resolution** | 16-bit (0.244 mg/LSB) | Native BMI270 ADC resolution |
| **Sensor output data rate (ODR)** | 800 Hz | Hardware sensor rate; oversamples so fresh data is always ready |
| **Effective sampling rate** | 500 Hz | Firmware reads the sensor on a fixed 500 Hz software timer for analysis |
| **FFT size** | 1024 samples | Real FFT, Hanning-windowed |
| **Frequency resolution** | 0.488 Hz/bin | 500 Hz ÷ 1024, refined with parabolic interpolation around the peak bin |
| **Frequency range (reported)** | 2 – 60 Hz | Software-limited band of interest; Nyquist limit is 250 Hz but only 2–60 Hz is searched/reported |
| **Measurement update rate** | ~2 s | Time to refill the 1024-sample buffer at 500 Hz (buffer is fully replaced each cycle, no overlap) |
| **Amplitude range (reported)** | 0.01 – 8.0 g | Values are clamped to this window; below it reads as no signal |
| **Amplitude detection threshold** | 0.1 g | Minimum amplitude for a frequency to be considered valid/displayed |
| **Amplitude smoothing** | 10-sample running median, per axis | Trades a small amount of lag for noise rejection |
| **Axes reported** | X, Y, Z amplitude (independently) + 1 dominant frequency | Frequency is computed from whichever axis has the largest amplitude |

## Calibration

- 3-axis offset + gain calibration (zero and scale factor per axis), stored in flash (NVS).
- Without calibration, defaults (zero offset, ×1.00 gain) are used — readings will be usable but not corrected for sensor/mounting bias.
- Calibration must be (re-)run after any change in mounting orientation or before precision work.

## Power Management

- Active measurement: BMI270 draws roughly 850 µA at 800 Hz/full performance.
- Auto power-off: 5 minutes of inactivity, or 2 minutes after a BLE disconnect.
- Deep sleep: display, LED, speaker, and the IMU itself are all powered down/suspended; wakes only on button press.
- Display dims after 30 s of inactivity to save power.

## Bluetooth Interface

- One BLE service exposing: frequency, amplitude X, amplitude Y, amplitude Z (32-bit float, read/notify), plus a legacy "mode" characteristic kept for backward compatibility with older companion apps.
- Notifications are pushed continuously while a measurement is valid; no polling required by the client.

## Known Limitations

- **Narrow frequency window**: only 2–60 Hz is analyzed and reported, even though the sensor and FFT could resolve up to 250 Hz. Vibration content outside 2–60 Hz is invisible to this device as currently configured.
- **Coarse frequency resolution**: 0.488 Hz/bin before interpolation — fine for typical rotating-machinery/vibration work, but not suited to detecting sub-Hz frequency drift.
- **Amplitude floor**: signals below ~0.1 g won't produce a reported frequency, and below 0.01 g amplitude itself reads as the floor value rather than true zero.
- **Single-axis frequency**: only one dominant frequency is reported per cycle (from the strongest axis), not independent per-axis frequencies — simultaneous multi-axis tones at different frequencies won't both be captured.
- **~2-second update latency**: the FFT buffer is fully replaced (no overlapping windows) before each new result, so the device reacts to changing vibration roughly every 2 seconds, not continuously.
- **No on-device frequency unit conversion**: output is raw Hz/g; conversions (e.g. to mm/s velocity or RPM) must be done by the receiving app.
- **Fixed ±8 g range**: not adjustable; very low-amplitude, low-noise work near the resolution floor (sub-10 mg) is not this device's strength.
- **Calibration dependency**: accuracy across axes depends on calibration having been performed; an uncalibrated unit is usable but not guaranteed accurate.
