/*
 * IoT-enabled MEMS wireless inclinometer  -  ESP8266 (NodeMCU) + MPU-6050 + ThingSpeak
 * ---------------------------------------------------------------------------------------
 * Corrected firmware that implements the methodology described in the manuscript
 * "Monitoring of inclinations of shoring walls for safety of substructure excavation
 *  work using MEMS based wireless inclinometers".
 *
 * What this fixes relative to the original MPU6050_testing_new.ino:
 *   - Raw counts are converted to physical units with the MPU-6050 sensitivity
 *     constants (16384 LSB/g, 131 LSB/(deg/s)) instead of the 10-bit analog-sensor
 *     map(265,402,...) that did not apply to a 16-bit I2C IMU.
 *   - The gyroscope is actually read and integrated  ....................  Eq. (1b)
 *   - The accelerometer tilt and the integrated gyro angle are fused with a
 *     complementary filter  ...............................................  Eq. (2a), (2b)
 *   - dt is measured every loop, so the integration and filter coefficient are exact.
 *   - I2C SCL/SDA are on D6/D7 as stated in the paper (Sec. 3.1.1).
 *   - Four ThingSpeak fields are populated (Fig. 3): fused roll, fused pitch,
 *     accelerometer-only roll (drift reference) and die temperature.
 *   - The filter runs fast; ThingSpeak is updated every 15 s (Sec. 3.1.3 cadence).
 *   - WiFi SSID / password / API key live in config.h (gitignored), not in source.
 *
 * Wiring (NodeMCU ESP8266):
 *   MPU-6050 VCC -> 3V3,  GND -> GND,  SCL -> D6 (GPIO12),  SDA -> D7 (GPIO13)
 */

#include <Wire.h>
#include <ESP8266WiFi.h>
#include "config.h"   // WIFI_SSID, WIFI_PASSWORD, THINGSPEAK_API_KEY

// ---------------------------------------------------------------------------
// MPU-6050 register map
// ---------------------------------------------------------------------------
static const uint8_t MPU_ADDR      = 0x68;
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG     = 0x1A; // DLPF
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B; // 14 sequential data bytes start here

// Full-scale sensitivities for the ranges we configure below.
static const float ACCEL_LSB_PER_G  = 16384.0f; // +/-2 g
static const float GYRO_LSB_PER_DPS = 131.0f;   // +/-250 deg/s

// I2C pins (paper Sec. 3.1.1 -> SCL=D6, SDA=D7).
static const uint8_t PIN_SCL = D6;
static const uint8_t PIN_SDA = D7;

// ---------------------------------------------------------------------------
// Complementary-filter time constant (seconds).  The filter coefficient is
// recomputed every loop as  alpha = beta / (beta + dt)   [Eq. (2a)].
// beta ~ 1 s gives alpha ~ 0.99 at a few-ms loop period: the gyro dominates the
// short-term response while the accelerometer removes long-term drift.
// ---------------------------------------------------------------------------
static const float BETA = 1.0f;

// ThingSpeak
static const char*  TS_HOST = "api.thingspeak.com";
static const int    TS_PORT = 80;
static const unsigned long UPLOAD_INTERVAL_MS = 15000UL; // 15 s -> 5760 obs/day

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
float rollAngle  = 0.0f; // fused inclination about X  (theta, Eq. 2b)
float pitchAngle = 0.0f; // fused inclination about Y
uint32_t lastMicros = 0;
unsigned long lastUpload = 0;

// ---------------------------------------------------------------------------
static void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static void mpuInit() {
  mpuWrite(REG_PWR_MGMT_1, 0x00);   // wake up
  mpuWrite(REG_CONFIG,     0x03);   // DLPF ~44 Hz: rejects jackhammer HF energy (Sec. 4 limitation)
  mpuWrite(REG_SMPLRT_DIV, 0x04);   // 1 kHz / (1+4) = 200 Hz internal rate
  mpuWrite(REG_GYRO_CONFIG,  0x00); // +/-250 deg/s
  mpuWrite(REG_ACCEL_CONFIG, 0x00); // +/-2 g
}

// Read all 14 data bytes: ax, ay, az, temp, gx, gy, gz (raw counts).
static bool mpuRead(int16_t& ax, int16_t& ay, int16_t& az,
                    int16_t& temp,
                    int16_t& gx, int16_t& gy, int16_t& gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14, (int)true) != 14) return false;
  ax   = (Wire.read() << 8) | Wire.read();
  ay   = (Wire.read() << 8) | Wire.read();
  az   = (Wire.read() << 8) | Wire.read();
  temp = (Wire.read() << 8) | Wire.read();
  gx   = (Wire.read() << 8) | Wire.read();
  gy   = (Wire.read() << 8) | Wire.read();
  gz   = (Wire.read() << 8) | Wire.read();
  return true;
}

static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.print("\nConnected, IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin(PIN_SDA, PIN_SCL);     // Wire.begin(sda, scl)
  Wire.setClock(400000);
  mpuInit();
  connectWiFi();

  // Seed the fused angle with the accelerometer estimate so it starts level-correct.
  int16_t ax, ay, az, t, gx, gy, gz;
  if (mpuRead(ax, ay, az, t, gx, gy, gz)) {
    float axg = ax / ACCEL_LSB_PER_G;
    float ayg = ay / ACCEL_LSB_PER_G;
    float azg = az / ACCEL_LSB_PER_G;
    rollAngle  = atan2f(ayg, azg) * RAD_TO_DEG;
    pitchAngle = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * RAD_TO_DEG;
  }
  lastMicros = micros();
  lastUpload = millis();
}

// Send four fields to ThingSpeak. Non-fatal on failure; the filter keeps running.
static void uploadThingSpeak(float rollFused, float pitchFused,
                             float rollAccel, float tempC) {
  WiFiClient client;
  if (!client.connect(TS_HOST, TS_PORT)) {
    Serial.println("ThingSpeak connect failed");
    return;
  }
  String url = String("GET /update?api_key=") + THINGSPEAK_API_KEY +
               "&field1=" + String(rollFused, 3) +
               "&field2=" + String(pitchFused, 3) +
               "&field3=" + String(rollAccel, 3) +
               "&field4=" + String(tempC, 2) +
               " HTTP/1.1\r\nHost: " + TS_HOST +
               "\r\nConnection: close\r\n\r\n";
  client.print(url);

  unsigned long t0 = millis();
  while (client.connected() && !client.available()) {
    if (millis() - t0 > 5000) { Serial.println("ThingSpeak timeout"); client.stop(); return; }
    delay(5);
  }
  while (client.available()) client.read(); // drain response
  client.stop();
}

void loop() {
  int16_t ax, ay, az, tRaw, gx, gy, gz;
  if (!mpuRead(ax, ay, az, tRaw, gx, gy, gz)) {
    delay(5);
    return;
  }

  // --- elapsed time since previous sample (seconds) ---
  uint32_t now = micros();
  float dt = (uint32_t)(now - lastMicros) * 1e-6f; // handles micros() overflow
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;         // guard against upload stalls

  // --- physical units ---
  float axg = ax / ACCEL_LSB_PER_G;
  float ayg = ay / ACCEL_LSB_PER_G;
  float azg = az / ACCEL_LSB_PER_G;
  float gxr = gx / GYRO_LSB_PER_DPS; // deg/s
  float gyr = gy / GYRO_LSB_PER_DPS;

  // --- accelerometer tilt, theta_a ---
  float rollAccel  = atan2f(ayg, azg) * RAD_TO_DEG;
  float pitchAccel = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * RAD_TO_DEG;

  // --- gyro integration, theta_g(k) = theta(k-1) + omega*dt   [Eq. 1b] ---
  float rollGyro  = rollAngle  + gxr * dt;
  float pitchGyro = pitchAngle + gyr * dt;

  // --- complementary filter   alpha = beta/(beta+dt)          [Eq. 2a] ---
  //     theta(k) = alpha*theta_g + (1-alpha)*theta_a           [Eq. 2b]
  float alpha = BETA / (BETA + dt);
  rollAngle  = alpha * rollGyro  + (1.0f - alpha) * rollAccel;
  pitchAngle = alpha * pitchGyro + (1.0f - alpha) * pitchAccel;

  // --- publish every 15 s ---
  if (millis() - lastUpload >= UPLOAD_INTERVAL_MS) {
    lastUpload = millis();
    float tempC = tRaw / 340.0f + 36.53f; // MPU-6050 datasheet
    Serial.printf("roll=%.3f  pitch=%.3f  (accel roll=%.3f)  T=%.2fC\n",
                  rollAngle, pitchAngle, rollAccel, tempC);
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    uploadThingSpeak(rollAngle, pitchAngle, rollAccel, tempC);
  }
}
