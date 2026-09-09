/**
 * @file    main.cpp
 * @brief   UAV Thrust Stand Firmware — RPI DBF Propulsion Test Bench
 *
 * Runs on an ESP32-S3 DevKitC-1. Reads three HX711 load cells (thrust, torque
 * push, torque pull), an INA228 power monitor (voltage + current over I2C),
 * and a hall-effect / optical RPM sensor via interrupt. An ESC is driven by a
 * 50 Hz PWM signal (LEDC peripheral) with a 1000–2000 µs pulse width.
 *
 * All sensor data is streamed at 50 Hz over USB CDC Serial as a CSV row:
 *   Time_ms, Thrust_g, TorquePush_Nm, TorquePull_Nm, Throttle_PWM,
 *   Push_g, Pull_g, Volts, Amps, RPM,
 *   AmbTemp_C, Humidity_pct, Pressure_hPa, IRTemp_C, Pitot_mV,
 *   GPS_Lat, GPS_Lon, GPS_Speed_mps, GPS_Fix, GPS_Sats
 *
 * The ten trailing fields are auxiliary diagnostics (BME680 ambient temp /
 * humidity / pressure, MLX90614 IR/laser object temp, raw pitot ADC millivolts,
 * and GPS position / ground speed / fix / satellite count). They are shown live
 * by the host GUI but are NOT yet part of the recorded CSV log.
 *
 * Serial commands (send as plain text + newline):
 *   "Z"        — Tare (zero) all three load cells
 *   "1000"–"2000" — Set ESC throttle in microseconds
 *
 * Pinout — updated for the "teststand" PCB rev (from KiCad netlist):
 *   GPIO  4  — HX711 DOUT (Thrust load cell, LoadCellDriver1)
 *   GPIO 17  — HX711 DOUT (Torque PUSH load cell, LoadCellDriver2)
 *   GPIO 18  — HX711 DOUT (Torque PULL load cell, LoadCellDriver3)
 *   GPIO 13  — HX711 SCK  (shared clock)
 *   GPIO  3  — I2C1 SDA  (INA228, "CVsensorDataB" — its own bus, NOT the main bus)
 *   GPIO 10  — I2C1 SCL  (INA228, "CVsensorDataY")
 *   GPIO  8  — ESC PWM output
 *   GPIO  7  — RPM signal input (rising-edge interrupt; bodge wire, RPMData
 *              is not routed on the PCB so jumper the sensor data line to GPIO7)
 *   GPIO  5  — I2C0 SDA (bus A: BME680 0x77 + MLX90614 0x5A)
 *   GPIO  6  — I2C0 SCL (bus A)
 *   GPIO  1  — Pitot analog output (ADC1_CH0; rails without a divider — see PCB notes)
 *   GPIO 44  — GPS UART RX (ESP RX <- GPS TX, 9600 baud NMEA)
 *   GPIO 43  — GPS UART TX (ESP TX -> GPS RX)
 *
 * The BME680 / MLX90614 / GPS / pitot block (bus A GPIO5/6, UART GPIO44/43,
 * ADC GPIO1) is now read and appended to the stream for live diagnostics, but
 * those fields are not written to the recorded CSV. See selftest.cpp for the
 * full per-sensor wiring check.
 *
 * @board   ESP32-S3 DevKitC-1  (PlatformIO env: esp32-s3-devkitc-1)
 * @version 3.0
 */

#include <Arduino.h>
#include <Wire.h>
#include "HX711.h"
#include <Adafruit_BME680.h>   // ambient temperature (bus A, 0x77)
#include <TinyGPSPlus.h>       // GPS NMEA parser (heading / fix / sats)

// --- CALIBRATION FACTORS ---
// These values were determined empirically with known weights.
// Re-run calibration if load cells are replaced or remounted.
const float CAL_THRUST      = 73.77;           // HX711 scale divisor for thrust cell
const float CAL_PUSH_LEFT   = 104.680247642;   // HX711 scale divisor for torque-push cell
const float CAL_PULL_RIGHT  = 110;             // HX711 scale divisor for torque-pull cell
const float TORQUE_ARM_M    = 0.06317685;      // Distance from motor shaft to load cell (metres)
const float GRAMS_TO_N      = 0.00981;         // Multiply grams × 0.00981 to get Newtons

// --- CONFIGURATION ---
#define INA228_ADDR      0x45   // I2C address set by ADDR pin wiring (A0=VCC → 0x45)
#define BME680_ADDR      0x77   // BME680 on bus A (SDO + CS tied to 3V3)
#define MLX90614_ADDR    0x5A   // MLX90614 IR thermometer on bus A
#define MOTOR_POLE_PAIRS 7      // Number of electrical pole pairs on the motor (14-pole → 7 pairs)

// --- PINS (teststand PCB rev — see header) ---
const int PIN_HX_SCK    = 13;  // HX711 shared clock line
const int PIN_LC_THRUST =  4;  // HX711 DOUT — axial thrust load cell (DT1)
const int PIN_LC_PUSH   = 17;  // HX711 DOUT — torque push (CW) load cell (DT2)
const int PIN_LC_PULL   = 18;  // HX711 DOUT — torque pull (CCW) load cell (DT3)
const int PIN_INA_SDA   =  3;  // I2C1 data  — INA228 power monitor (own bus)
const int PIN_INA_SCL   = 10;  // I2C1 clock — INA228 power monitor
const int PIN_ESC       =  8;  // PWM output to ESC signal wire
const int PIN_RPM       =  7;  // RPM pulse input (bodge: RPMData → GPIO7)
const int PIN_SDA_A     =  5;  // I2C0 data  — bus A (BME680 + MLX90614)
const int PIN_SCL_A     =  6;  // I2C0 clock — bus A
const int PIN_PITOT     =  1;  // Pitot analog output (ADC1_CH0)
const int PIN_GPS_RX    = 44;  // GPS UART: ESP RX <- GPS TX
const int PIN_GPS_TX    = 43;  // GPS UART: ESP TX -> GPS RX

// --- ESC via LEDC (Legacy Arduino-ESP32 v2.x API) ---
// In v2.x the LEDC API targets a CHANNEL number; the channel is later bound to
// a GPIO pin with ledcAttachPin().  A 14-bit timer at 50 Hz gives sub-µs
// resolution across the standard 1000–2000 µs servo/ESC pulse range.
#define ESC_LEDC_CHANNEL    0     // LEDC channel 0 (0–7 on S3)
#define ESC_LEDC_FREQ_HZ    50    // Standard RC ESC PWM frequency
#define ESC_LEDC_RESOLUTION 14    // 14-bit counter → 0–16383 ticks
#define ESC_PWM_MIN         819   // Tick count for 1000 µs (motor idle / armed)
#define ESC_PWM_MAX         1638  // Tick count for 2000 µs (full throttle)

// --- OBJECTS ---
HX711 scaleThrust, scalePush, scalePull;
Adafruit_BME680 bme(&Wire);   // BME680 ambient sensor lives on bus A (Wire)
TinyGPSPlus gps;              // NMEA parser fed from Serial1 (GPS UART)
bool inaOnline = false;
bool bmeOnline = false;

// --- STATE ---
volatile unsigned long lastPulseTimeMicros = 0;
volatile unsigned long pulseIntervalMicros = 0;
volatile bool newPulseAvailable = false;

unsigned long lastStream  = 0;
float currentRpm      = 0;
int   currentThrottle = 1000;

// --- AUX SENSOR STATE (diagnostics only; appended to stream, not logged) ---
float  ambTempC = NAN;    // BME680 ambient temperature (C)
float  humidity = NAN;    // BME680 relative humidity (%)
float  pressHpa = NAN;    // BME680 barometric pressure (hPa)
float  irTempC  = NAN;    // MLX90614 IR object ("laser") temperature
int    pitotMv  = 0;      // raw pitot ADC millivolts
double gpsLat   = NAN;    // GPS latitude (deg)
double gpsLon   = NAN;    // GPS longitude (deg)
float  gpsSpeed = NAN;    // GPS ground speed (m/s)
int    gpsSats  = 0;      // GPS satellites in view
bool   gpsFix   = false;  // GPS has a position fix
unsigned long lastBmeRead = 0;

// --- ISR ---
/**
 * @brief Interrupt Service Routine — fires on every rising edge of the RPM signal.
 *
 * Records the time between consecutive pulses in pulseIntervalMicros.  The loop()
 * converts this interval to RPM using the motor's pole-pair count.
 * Unsigned subtraction wraps correctly, so the 70-minute micros() rollover is safe.
 * Stored in IRAM so it runs even if flash cache is busy.
 */
void IRAM_ATTR handleRpmPulse() {
    unsigned long now = micros();
    // Unsigned math naturally handles the 70-minute micros() overflow
    pulseIntervalMicros = now - lastPulseTimeMicros;
    lastPulseTimeMicros = now;
    newPulseAvailable = true;
}

// --- ESC HELPER ---
/**
 * @brief  Write an ESC pulse width in microseconds (1000–2000 µs).
 * @param  us  Desired pulse width.  Values outside [1000, 2000] are clamped.
 *
 * Converts the µs value to a 14-bit LEDC tick count and writes it to the
 * ESC channel.  1000 µs = motor armed/idle; 2000 µs = full throttle.
 */
void escWriteMicroseconds(int us) {
    us = constrain(us, 1000, 2000);
    uint32_t ticks = map(us, 1000, 2000, ESC_PWM_MIN, ESC_PWM_MAX);
    // Write to the CHANNEL, which is linked to the PIN in setup
    ledcWrite(ESC_LEDC_CHANNEL, ticks);
}

// --- I2C HELPERS (Library-Free INA228) ---
// The INA228 sits on its own I2C bus (Wire1, GPIO3/10) on this PCB rev, so all
// of these talk to Wire1 — NOT the default Wire.
uint16_t readReg16(uint8_t reg) {
    Wire1.beginTransmission(INA228_ADDR);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return 0;

    // Explicit casts to resolve "ambiguous overload" error
    Wire1.requestFrom((uint16_t)INA228_ADDR, (uint8_t)2);

    if (Wire1.available() == 2) return (Wire1.read() << 8) | Wire1.read();
    return 0;
}

int32_t readReg24(uint8_t reg) {
    Wire1.beginTransmission(INA228_ADDR);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return 0;

    // Explicit casts to resolve "ambiguous overload" error
    Wire1.requestFrom((uint16_t)INA228_ADDR, (uint8_t)3);

    if (Wire1.available() == 3) {
        uint32_t val = (uint32_t)Wire1.read() << 16 | (uint32_t)Wire1.read() << 8 | Wire1.read();
        if (val & 0x800000) val |= 0xFF000000; // Sign extension
        return (int32_t)val;
    }
    return 0;
}

// --- MLX90614 IR TEMPERATURE (bus A / Wire) ---
// SMBus RAM words are LSB-first; reg 0x07 = object temp (Tobj1).  Returns the
// temperature in Celsius, or NAN if the device does not respond / flags an error.
float mlxReadObjectTemp() {
    Wire.beginTransmission(MLX90614_ADDR);
    Wire.write(0x07);
    if (Wire.endTransmission(false) != 0) return NAN;
    if (Wire.requestFrom((int)MLX90614_ADDR, 2) != 2) return NAN;
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    uint16_t raw = ((uint16_t)hi << 8) | lo;
    if (raw & 0x8000) return NAN;          // error flag set
    return raw * 0.02f - 273.15f;
}

// --- SAFE TARE HELPER ---
/**
 * @brief  Tare (zero) an HX711 scale with a configurable timeout.
 * @param  scale       Reference to the HX711 instance.
 * @param  name        Human-readable name printed to Serial for diagnostics.
 * @param  timeout_ms  Maximum ms to wait for the ADC to become ready (default 3 s).
 * @return true on success, false if the ADC did not respond in time.
 */
bool safeTare(HX711 &scale, const char* name, unsigned long timeout_ms = 3000) {
    Serial.printf("  Taring %s... ", name);
    unsigned long start = millis();
    while (!scale.is_ready()) {
        if (millis() - start > timeout_ms) {
            Serial.printf("TIMEOUT!\n");
            return false;
        }
        delay(10);
    }
    scale.tare();
    Serial.printf("OK\n");
    return true;
}

void setup() {
    Serial.begin(921600);
    
    // USB CDC Wait for ESP32-S3
    unsigned long bootWait = millis();
    while (!Serial && millis() - bootWait < 3000);
    delay(500);

    Serial.println("\n=== BOOT START ===");

    // 1. I2C Initialization (INA228 is on the secondary bus, Wire1)
    Wire1.begin(PIN_INA_SDA, PIN_INA_SCL);
    uint16_t id = readReg16(0x3F);
    if (id == 0x2280 || id == 0x2281) {
        inaOnline = true;
        Serial.println("[1/6] INA228 detected.");
    } else {
        Serial.printf("[1/6] INA228 not found (ID: 0x%04X).\n", id);
    }

    // 1b. Bus A sensors (BME680 ambient + MLX90614 IR) on Wire (GPIO5/6)
    Wire.begin(PIN_SDA_A, PIN_SCL_A);
    bmeOnline = bme.begin(BME680_ADDR, true);
    if (bmeOnline) {
        // Ambient-only config: gas heater off keeps performReading() fast (~10 ms).
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(0, 0);
        Serial.println("[2/6] BME680 ambient sensor online.");
    } else {
        Serial.println("[2/6] BME680 not found on bus A (0x77).");
    }

    // 2. Load Cell Initialization
    scaleThrust.begin(PIN_LC_THRUST, PIN_HX_SCK);
    scalePush.begin(PIN_LC_PUSH,   PIN_HX_SCK);
    scalePull.begin(PIN_LC_PULL,   PIN_HX_SCK);
    
    scaleThrust.set_scale(CAL_THRUST);
    scalePush.set_scale(CAL_PUSH_LEFT);
    scalePull.set_scale(CAL_PULL_RIGHT);

    Serial.println("[3/6] Taring scales...");
    safeTare(scaleThrust, "THRUST");
    safeTare(scalePush,   "PUSH");
    safeTare(scalePull,   "PULL");

    // 4. ESC Initialization (Legacy API)
    Serial.println("[4/6] Initializing ESC PWM...");
    ledcSetup(ESC_LEDC_CHANNEL, ESC_LEDC_FREQ_HZ, ESC_LEDC_RESOLUTION);
    ledcAttachPin(PIN_ESC, ESC_LEDC_CHANNEL);
    escWriteMicroseconds(1000);

    // 5. RPM Initialization
    Serial.println("[5/6] Attaching RPM interrupt...");
    pinMode(PIN_RPM, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), handleRpmPulse, RISING);

    // 6. GPS UART (heading / fix / satellite count) — NMEA at 9600 baud on Serial1
    Serial.println("[6/6] Starting GPS UART...");
    Serial1.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    Serial.println("=== BOOT COMPLETE ===");
    Serial.println("Time_ms,Thrust_g,TorquePush_Nm,TorquePull_Nm,Throttle_PWM,Push_g,Pull_g,Volts,Amps,RPM,AmbTemp_C,Humidity_pct,Pressure_hPa,IRTemp_C,Pitot_mV,GPS_Lat,GPS_Lon,GPS_Speed_mps,GPS_Fix,GPS_Sats");
    
    lastStream = millis();
}

void loop() {
    // Keep the NMEA parser fed every iteration so GPS heading/fix stay current.
    while (Serial1.available()) gps.encode(Serial1.read());

    // Serial Command Parser
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input == "Z") {
            safeTare(scaleThrust, "THRUST");
            safeTare(scalePush, "PUSH");
            safeTare(scalePull, "PULL");
        } else {
            int val = input.toInt();
            if (val >= 1000 && val <= 2000) {
                currentThrottle = val;
                escWriteMicroseconds(val);
            }
        }
    }

    // Scale Sampling (Non-blocking)
    static float t = 0, p = 0, l = 0;
    if (scaleThrust.is_ready()) t = scaleThrust.get_units(1);
    if (scalePush.is_ready())   p = scalePush.get_units(1);
    if (scalePull.is_ready())   l = scalePull.get_units(1);

    // Data Streaming @ 50Hz
    unsigned long now = millis();
    if (now - lastStream >= 20) {
        
        // --- RPM PERIOD CALCULATION ---
        // 1. Safely grab the volatile variables
        noInterrupts();
        unsigned long interval = pulseIntervalMicros;
        unsigned long lastPulse = lastPulseTimeMicros;
        bool hasNew = newPulseAvailable;
        newPulseAvailable = false; 
        interrupts();

        // 2. Calculate RPM if a new interval was recorded
        if (hasNew && interval > 0) {
            // 60,000,000 microseconds in a minute
            currentRpm = 60000000.0 / (interval * MOTOR_POLE_PAIRS);
        }

        // 3. Zero-RPM Timeout 
        // If the motor stops, interrupts stop firing, and the RPM would freeze 
        // at its last value. If no pulse is seen for 200ms (200,000 us), force to 0.
        if (micros() - lastPulse > 200000) {
            currentRpm = 0;
        }
        // ------------------------------

        // Torque calculation (Direct Math)
        float tPush = (abs(p) > 0.5) ? (p * GRAMS_TO_N * TORQUE_ARM_M) : 0.0f;
        float tPull = (abs(l) > 0.5) ? (-l * GRAMS_TO_N * TORQUE_ARM_M) : 0.0f;

        // INA228 Power Reading
        float v = 0, a = 0;
        if (inaOnline) {
            v = (readReg24(0x05) >> 4) * 0.0001953125f;
            a = ((readReg24(0x04) >> 4) * 0.0000003125f) / 0.0002f; // Assuming 0.2mOhm shunt
        }

        // --- AUX SENSORS (diagnostics only) ---
        // MLX IR temp, pitot mV, and GPS state are cheap to sample every cycle.
        irTempC    = mlxReadObjectTemp();
        pitotMv    = analogReadMilliVolts(PIN_PITOT);
        gpsFix     = gps.location.isValid();
        gpsLat     = gpsFix ? gps.location.lat() : NAN;
        gpsLon     = gpsFix ? gps.location.lng() : NAN;
        gpsSpeed   = gps.speed.isValid() ? gps.speed.mps() : NAN;
        gpsSats    = gps.satellites.value();
        // BME680 performReading() blocks ~10 ms, so refresh ambient at 1 Hz only
        // to protect the 50 Hz thrust/RPM streaming cadence.
        if (now - lastBmeRead >= 1000) {
            if (bmeOnline && bme.performReading()) {
                ambTempC = bme.temperature;
                humidity = bme.humidity;
                pressHpa = bme.pressure / 100.0f;   // Pa -> hPa
            }
            lastBmeRead = now;
        }

        // CSV Output (10 core fields + 10 auxiliary diagnostic fields)
        Serial.printf("%lu,%.2f,%.4f,%.4f,%d,%.2f,%.2f,%.2f,%.3f,%.0f,%.1f,%.1f,%.1f,%.1f,%d,%.6f,%.6f,%.2f,%d,%d\n",
                      now, -t, tPush, tPull, currentThrottle, p, l, v, a, currentRpm,
                      ambTempC, humidity, pressHpa, irTempC, pitotMv, gpsLat, gpsLon, gpsSpeed, gpsFix ? 1 : 0, gpsSats);

        lastStream = now;
    }
}