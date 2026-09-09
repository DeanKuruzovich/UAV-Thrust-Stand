/**
 * @file    selftest.cpp
 * @brief   PCB hardware self-test for the UAV Thrust Stand board.
 *
 * Standalone diagnostic firmware (NOT the flight/logging firmware in main.cpp).
 * It walks every component on the "teststand" PCB, confirms each one is wired
 * up and responding, and then prints LIVE values once per second:
 *   - BME680 ambient temperature / pressure / humidity
 *   - MLX90614 IR ("laser") object + ambient temperature
 *   - GPS course-over-ground (direction), fix status, satellites
 *   - 3x HX711 load-cell raw counts, pitot mV, INA228 raw regs
 *
 * Pin assignments are taken DIRECTLY from the KiCad netlist
 * (teststand.kicad_sch), not from the logging firmware — the PCB rev uses a
 * different pinout than main.cpp.
 *
 *   --- I2C bus A (Wire)  SDA=GPIO5  SCL=GPIO6 ("SLC" net) ---
 *      BME680  ambient T/P/RH/gas   addr 0x77 (SDO + CS tied to 3V3)
 *      MLX90614 IR motor temp       addr 0x5A
 *      Compass (3DR GPS/compass)    SDA only — see WARNING below
 *
 *   --- I2C bus B (Wire1) SDA=GPIO3  SCL=GPIO10 (INA "DataB/DataY") ---
 *      INA228  current / voltage    addr 0x40
 *
 *   --- Other ---
 *      HX711 x3   DT=GPIO4/17/18, shared SCK=GPIO13
 *      Pitot      analog on GPIO1 (ADC1_CH0)
 *      GPS UART   ESP RX=GPIO44, TX=GPIO43 (9600 baud NMEA)
 *      ESC        GPIO8 (driven to 1000us idle only — motor will NOT spin)
 *      RPM        GPIO7 (BODGE WIRE: RPMData is not routed on the PCB, so the
 *                 sensor's data line must be jumpered to the GPIO7 header pin)
 *
 * KNOWN SCHEMATIC ISSUES (flagged at runtime):
 *   - Compass SCL is on its own net with no ESP32 pin -> compass won't ACK.
 *   - RPMData net is unrouted on the PCB; this firmware expects it bodged to GPIO7.
 *
 * Build/flash this instead of main.cpp with the dedicated PlatformIO env:
 *     pio run -e selftest -t upload && pio device monitor -e selftest
 */

#include <Arduino.h>
#include <Wire.h>
#include "HX711.h"
#include <Adafruit_BME680.h>
#include <TinyGPSPlus.h>

// ---------------- Pin map (from netlist) ----------------
static const int PIN_SDA_A   = 5;    // I2C bus A data  (BME680, MLX90614, compass)
static const int PIN_SCL_A   = 6;    // I2C bus A clock (net "SLC")
static const int PIN_SDA_B   = 3;    // I2C bus B data  (INA228, net "CVsensorDataB")
static const int PIN_SCL_B   = 10;   // I2C bus B clock (INA228, net "CVsensorDataY")

static const int PIN_HX_DT1  = 4;    // LoadCellDriver1 (thrust)
static const int PIN_HX_DT2  = 17;   // LoadCellDriver2 (push)
static const int PIN_HX_DT3  = 18;   // LoadCellDriver3 (pull)
static const int PIN_HX_SCK  = 13;   // shared HX711 clock

static const int PIN_PITOT   = 1;    // analog pitot output (ADC1_CH0)
static const int PIN_GPS_RX  = 44;   // ESP RX  <- GPS TX
static const int PIN_GPS_TX  = 43;   // ESP TX  -> GPS RX
static const int PIN_ESC     = 8;    // ESC signal
static const int PIN_RPM     = 7;    // RPM pulse input (bodge: RPMData -> GPIO7)

// RPM sensor counts electrical pulses; divide by pole pairs to get mechanical RPM.
#define MOTOR_POLE_PAIRS 7

// ---------------- Expected I2C addresses ----------------
static const uint8_t ADDR_BME680   = 0x77;
static const uint8_t ADDR_MLX90614 = 0x5A;
static const uint8_t ADDR_INA228   = 0x45;   // ADDR pin strapped to VCC (confirmed by bus scan)

// ---------------- ESC idle (LEDC v2.x API) ----------------
#define ESC_CH   0
#define ESC_FREQ 50
#define ESC_RES  14
#define ESC_IDLE 819   // ~1000us @ 14-bit / 50Hz

// ---------------- Objects ----------------
HX711 hxThrust, hxPush, hxPull;
Adafruit_BME680 bme(&Wire);    // BME680 lives on I2C bus A (Wire)
TinyGPSPlus gps;               // NMEA parser fed from Serial1
bool bmeOnline = false;

// --- RPM interrupt state (pin bodged to GPIO7) ---
volatile unsigned long rpmLastPulseUs = 0;   // micros() of the previous edge
volatile unsigned long rpmIntervalUs  = 0;   // time between the last two edges
volatile unsigned long rpmPulseCount   = 0;  // total edges seen (proves wiring)

// Fires on each rising edge; unsigned subtraction survives the micros() rollover.
void IRAM_ATTR onRpmPulse() {
    unsigned long now = micros();
    rpmIntervalUs  = now - rpmLastPulseUs;
    rpmLastPulseUs = now;
    rpmPulseCount++;
}

// ANSI colours (most serial monitors render them; harmless if not)
#define C_RST  "\033[0m"
#define C_PASS "\033[32m"   // green
#define C_WARN "\033[33m"   // yellow
#define C_FAIL "\033[31m"   // red

static void line() { Serial.println("------------------------------------------------------------"); }

static void result(const char* tag, bool ok, const char* name, const char* detail) {
    const char* col = ok ? C_PASS : C_FAIL;
    Serial.printf("  [%s%s%s] %-18s %s\n", col, tag, C_RST, name, detail);
}

// Probe a single 7-bit address on the given bus. Returns true if the device ACKs.
static bool i2cPresent(TwoWire &bus, uint8_t addr) {
    bus.beginTransmission(addr);
    return bus.endTransmission() == 0;
}

// Read one 16-bit big-endian register from an I2C device.
static bool readReg16(TwoWire &bus, uint8_t addr, uint8_t reg, uint16_t &out) {
    bus.beginTransmission(addr);
    bus.write(reg);
    if (bus.endTransmission(false) != 0) return false;
    if (bus.requestFrom((int)addr, 2) != 2) return false;
    out = ((uint16_t)bus.read() << 8) | bus.read();
    return true;
}

// MLX90614 RAM words are LSB-first; return temp in Celsius (NAN on failure).
// reg 0x06 = ambient (Ta), 0x07 = object (Tobj1).
static float mlxReadTemp(uint8_t reg) {
    uint16_t be;
    if (!readReg16(Wire, ADDR_MLX90614, reg, be)) return NAN;
    uint16_t le = (be >> 8) | (be << 8);      // swap back to little-endian
    if (le & 0x8000) return NAN;              // error flag set
    return le * 0.02f - 273.15f;
}

// Scan a bus and print every address that ACKs.
static void scanBus(TwoWire &bus, const char* name) {
    Serial.printf("  %s scan:", name);
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2cPresent(bus, a)) {
            Serial.printf(" 0x%02X", a);
            found++;
        }
    }
    if (!found) Serial.print(" (no devices)");
    Serial.println();
}

// ---- per-sensor checks (run once at boot) ----

static void checkBME680() {
    bmeOnline = bme.begin(ADDR_BME680, true);
    if (!bmeOnline) {
        result("FAIL", false, "BME680", "begin() failed — no ACK on bus A @0x77");
        return;
    }
    // Ambient-only config: modest oversampling, gas heater off (faster, not needed here).
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(0, 0);
    if (bme.performReading()) {
        char d[64];
        snprintf(d, sizeof d, "%.1f C  %.1f hPa  %.0f%%RH",
                 bme.temperature, bme.pressure / 100.0, bme.humidity);
        result("PASS", true, "BME680 ambient", d);
    } else {
        result("WARN", false, "BME680 ambient", "online but reading failed");
    }
}

static void checkMLX90614() {
    float obj = mlxReadTemp(0x07);
    float amb = mlxReadTemp(0x06);
    if (isnan(obj)) {
        result("FAIL", false, "MLX90614 IR", "no ACK / error on bus A @0x5A");
        return;
    }
    char d[64];
    snprintf(d, sizeof d, "object %.1f C  ambient %.1f C", obj, amb);
    bool sane = obj > -50 && obj < 380;
    result(sane ? "PASS" : "WARN", sane, "MLX90614 IR", d);
}

static void checkINA228() {
    uint16_t mfg = 0, dev = 0;
    bool okm = readReg16(Wire1, ADDR_INA228, 0x3E, mfg);  // MANUFACTURER_ID -> 0x5449 "TI"
    bool okd = readReg16(Wire1, ADDR_INA228, 0x3F, dev);  // DEVICE_ID -> 0x228x
    if (okm || okd) {
        char d[64];
        snprintf(d, sizeof d, "mfg=0x%04X dev=0x%04X (expect TI/0x228x)", mfg, dev);
        bool ok = (mfg == 0x5449) || ((dev & 0xFFF0) == 0x2280);
        result(ok ? "PASS" : "WARN", ok, "INA228", d);
    } else {
        result("FAIL", false, "INA228", "no ACK on bus B @0x40");
    }
}

static void checkHX711(HX711 &hx, const char* name) {
    if (hx.wait_ready_timeout(1000)) {
        long raw = hx.read();
        char d[48];
        snprintf(d, sizeof d, "raw=%ld (responding)", raw);
        // Stuck at the rail (all-1s / all-0s) usually means DT or SCK miswired.
        bool stuck = (raw == 0) || (raw == -1) || (raw == 0x7FFFFF) || (raw == -0x800000);
        result(stuck ? "WARN" : "PASS", !stuck, name, d);
    } else {
        result("FAIL", false, name, "timeout — check DT/SCK wiring");
    }
}

static void checkPitot() {
    int raw = analogRead(PIN_PITOT);
    int mv  = analogReadMilliVolts(PIN_PITOT);
    char d[48];
    snprintf(d, sizeof d, "raw=%d  %d mV", raw, mv);
    result("PASS", true, "Pitot (analog)", d);
}

// Pump 1.5 s of UART into the NMEA parser, then report what we got.
static void checkGPS() {
    Serial1.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    unsigned long start = millis();
    int bytes = 0;
    while (millis() - start < 1500) {
        while (Serial1.available()) {
            char c = Serial1.read();
            gps.encode(c);
            bytes++;
        }
    }
    if (bytes == 0) {
        result("FAIL", false, "GPS UART", "no UART data — check RX/TX & GPS power");
        return;
    }
    char d[80];
    if (gps.location.isValid()) {
        snprintf(d, sizeof d, "FIX: %u sats, course %.0f deg", gps.satellites.value(),
                 gps.course.isValid() ? gps.course.deg() : 0.0);
    } else {
        snprintf(d, sizeof d, "talking (%d bytes) but NO FIX yet — needs open sky", bytes);
    }
    result("PASS", true, "GPS UART", d);
}

void setup() {
    Serial.begin(921600);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000);
    delay(300);

    Serial.println("\n\n========== UAV THRUST STAND — PCB SELF-TEST ==========");
    Serial.println("Pinout from KiCad netlist. Motor will NOT spin (ESC idle only).");
    line();

    // --- I2C buses ---
    Wire.begin(PIN_SDA_A, PIN_SCL_A);
    Wire1.begin(PIN_SDA_B, PIN_SCL_B);
    Serial.println("I2C bus scan:");
    scanBus(Wire,  "  bus A (SDA5/SCL6) ");
    scanBus(Wire1, "  bus B (SDA3/SCL10)");
    line();

    Serial.println("Sensors:");
    checkBME680();
    checkMLX90614();
    checkINA228();

    // --- Load cells ---
    hxThrust.begin(PIN_HX_DT1, PIN_HX_SCK);
    hxPush.begin(PIN_HX_DT2, PIN_HX_SCK);
    hxPull.begin(PIN_HX_DT3, PIN_HX_SCK);
    checkHX711(hxThrust, "HX711 thrust");
    checkHX711(hxPush,   "HX711 push");
    checkHX711(hxPull,   "HX711 pull");

    // --- Analog / UART ---
    checkPitot();
    checkGPS();

    // --- RPM (bodge: RPMData -> GPIO7) ---
    pinMode(PIN_RPM, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), onRpmPulse, RISING);
    result("PASS", true, "RPM (GPIO7)", "interrupt armed — spin motor to see pulses below");

    // --- ESC (idle only) ---
    ledcSetup(ESC_CH, ESC_FREQ, ESC_RES);
    ledcAttachPin(PIN_ESC, ESC_CH);
    ledcWrite(ESC_CH, ESC_IDLE);
    result("PASS", true, "ESC (GPIO8)", "driving 1000us idle — listen for arm beeps");
    line();

    // --- Known schematic wiring problems ---
    Serial.println("WARNINGS (from schematic, cannot be tested in firmware):");
    Serial.printf("  [%sWARN%s] %-18s %s\n", C_WARN, C_RST, "Compass",
                  "compass SCL net has no ESP32 pin -> won't ACK on bus A");
    line();
    Serial.println("Live values (1 Hz). Ctrl-C to stop.\n");
}

void loop() {
    static unsigned long lastPrint = 0;

    // Keep feeding the GPS parser every loop so course/fix stay current.
    while (Serial1.available()) gps.encode(Serial1.read());

    if (millis() - lastPrint < 1000) return;   // print once per second
    lastPrint = millis();

    // --- BME680 ambient ---
    float aT = NAN, aP = NAN, aH = NAN;
    if (bmeOnline && bme.performReading()) {
        aT = bme.temperature;
        aP = bme.pressure / 100.0;   // Pa -> hPa
        aH = bme.humidity;
    }

    // --- MLX90614 IR (laser) object temp ---
    float irObj = mlxReadTemp(0x07);

    // --- GPS direction ---
    float course = (gps.course.isValid()) ? gps.course.deg() : NAN;
    const char* card = gps.course.isValid() ? TinyGPSPlus::cardinal(course) : "--";
    bool fix = gps.location.isValid();

    // --- Load cells / pitot / INA raw ---
    // All three HX711s share one SCK, so reading one clocks (and clobbers) the
    // others. Read them sequentially with a short wait so each gets a fresh
    // conversion; hold the last good count if a chip doesn't respond in time.
    static long t = 0, p = 0, l = 0;
    if (hxThrust.wait_ready_timeout(150)) t = hxThrust.read();
    if (hxPush.wait_ready_timeout(150))   p = hxPush.read();
    if (hxPull.wait_ready_timeout(150))   l = hxPull.read();
    int pitotMv = analogReadMilliVolts(PIN_PITOT);
    uint16_t vbus = 0; readReg16(Wire1, ADDR_INA228, 0x05, vbus);

    // --- RPM ---
    noInterrupts();
    unsigned long interval = rpmIntervalUs;
    unsigned long lastPulse = rpmLastPulseUs;
    unsigned long pulses = rpmPulseCount;
    interrupts();
    float rpm = 0;
    if (interval > 0) rpm = 60000000.0f / (interval * MOTOR_POLE_PAIRS);
    // If no pulse for >200 ms the motor has stopped — don't latch the old value.
    if (micros() - lastPulse > 200000UL) rpm = 0;

    Serial.printf(
        "AMB %5.1fC %6.1fhPa %3.0f%%RH | IR %6.1fC | GPS %s %5.1fdeg(%s) | "
        "HX[%8ld %8ld %8ld] Pitot %4dmV INA 0x%04X | RPM %6.0f (%lu pulses)\n",
        aT, aP, aH,
        irObj,
        fix ? "FIX" : "nofix",
        isnan(course) ? 0.0f : course, card,
        t, p, l, pitotMv, vbus, rpm, pulses);
}
