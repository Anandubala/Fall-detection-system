#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "protocol.h"

// ---------- Pins ----------
#define SDA_PIN 21
#define SCL_PIN 22
#define OLED_ADDR 0x3C
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 4
#define BTN_OK 25
#define BTN_HELP 26
#define BTN_SIM 27   // DEVELOPMENT SIMULATION: starts the fall countdown

// ---------- Config ----------
#define DEVICE_ID 1
#define NODE2_ID 2
#define LORA_FREQ 433E6
#define LORA_TX_DBM 10
#define HEARTBEAT_MS 5000UL
#define LINK_TIMEOUT_MS 15000UL
#define COUNTDOWN_S 10
#define SOS_HOLD_MS 1500UL
#define RETRY_MS 1500UL
#define MAX_TRIES 15
#define DEBOUNCE_MS 30UL

// ---------- MPU6050 (calibration from Phase 3) ----------
#define MPU_ADDR 0x68
const float ACC_LSB_PER_G = 4096.0;   // +-8 g
const float GYRO_LSB_PER_DPS = 65.5;  // +-500 deg/s
const float GYRO_OFF[3]  = {-2.147, 7.062, -1.190};
const float ACC_OFF[3]   = {0.0172, 0.0087, 0.0497};
const float ACC_SCALE[3] = {1.0029, 1.0016, 0.9897};

Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ---------- Globals ----------
bool mpuOk = false;
float amag = 0, gmag = 0;
uint32_t imuErrors = 0;

WearState state = WS_SAFE;
uint16_t seq = 0;
uint16_t pendSeq = 0;
uint8_t pendType = 0;
uint8_t tries = 0;
uint32_t firstTxMs = 0, lastTxMs = 0, countdownStart = 0, lastLatency = 0;
uint32_t lastAckMs = 0, lastHbMs = 0, lastImuMs = 0, lastOledMs = 0, lastDbgMs = 0;
bool caregiverAcked = false;
int lastRssi = 0;
float lastSnr = 0;

// ---------- Buttons ----------
struct Button {
  uint8_t pin;
  bool raw, stable, pressEvt, longFired;
  uint32_t lastChange, pressStart;
};
Button bOk = {BTN_OK}, bHelp = {BTN_HELP}, bSim = {BTN_SIM};

void btnBegin(Button &b) { pinMode(b.pin, INPUT_PULLUP); }

void btnUpdate(Button &b) {
  bool r = (digitalRead(b.pin) == LOW);   // LOW = pressed (INPUT_PULLUP)
  uint32_t now = millis();
  if (r != b.raw) { b.raw = r; b.lastChange = now; }
  if ((now - b.lastChange) >= DEBOUNCE_MS && b.stable != b.raw) {
    b.stable = b.raw;
    if (b.stable) { b.pressStart = now; b.pressEvt = true; b.longFired = false; }
  }
}
bool btnPressed(Button &b) { bool e = b.pressEvt; b.pressEvt = false; return e; }
bool btnLong(Button &b, uint32_t ms) {
  if (b.stable && !b.longFired && (millis() - b.pressStart) >= ms) { b.longFired = true; return true; }
  return false;
}

// ---------- MPU6050 ----------
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

void readImu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) {
    imuErrors++;
    mpuOk = false;
    return;
  }
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  float ax = ((int16_t)((b[0] << 8) | b[1])  / ACC_LSB_PER_G - ACC_OFF[0]) * ACC_SCALE[0];
  float ay = ((int16_t)((b[2] << 8) | b[3])  / ACC_LSB_PER_G - ACC_OFF[1]) * ACC_SCALE[1];
  float az = ((int16_t)((b[4] << 8) | b[5])  / ACC_LSB_PER_G - ACC_OFF[2]) * ACC_SCALE[2];
  float gx = (int16_t)((b[8] << 8) | b[9])   / GYRO_LSB_PER_DPS - GYRO_OFF[0];
  float gy = (int16_t)((b[10] << 8) | b[11]) / GYRO_LSB_PER_DPS - GYRO_OFF[1];
  float gz = (int16_t)((b[12] << 8) | b[13]) / GYRO_LSB_PER_DPS - GYRO_OFF[2];
  amag = sqrtf(ax * ax + ay * ay + az * az);
  gmag = sqrtf(gx * gx + gy * gy + gz * gz);
  mpuOk = true;   // a successful read is the real proof the sensor works
}

// ---------- LoRa ----------
void sendRaw(const Packet &p) {
  LoRa.beginPacket();
  LoRa.write((const uint8_t *)&p, sizeof(p));
  LoRa.endPacket();      // blocks until transmitted
  LoRa.receive();        // go straight back to listening for ACKs
}

void sendOnce(uint8_t type, uint8_t arg) {
  Packet p;
  fillPacket(p, DEVICE_ID, type, ++seq, arg, (int16_t)(amag * 1000));
  sendRaw(p);
}

void sendAlertAttempt() {
  Packet p;
  fillPacket(p, DEVICE_ID, pendType, pendSeq, 0, (int16_t)(amag * 1000));
  sendRaw(p);
  lastTxMs = millis();
  tries++;
  Serial.printf("TX type=%u seq=%u try=%u\n", pendType, pendSeq, tries);
}

void startAlert(uint8_t type) {
  pendType = type;
  pendSeq = ++seq;
  tries = 0;
  caregiverAcked = false;
  firstTxMs = millis();
  state = WS_ALERT_PENDING;
  sendAlertAttempt();
}

void sendCancel() {
  // Sent 3 times with the SAME seq (receiver ignores duplicates)
  Packet p;
  fillPacket(p, DEVICE_ID, PKT_CANCEL, ++seq, 0, 0);
  for (int i = 0; i < 3; i++) { sendRaw(p); delay(150); }
  Serial.println("CANCEL sent x3");
}

void handleRx() {
  int n = LoRa.parsePacket();
  if (n == 0) return;
  if (n != (int)sizeof(Packet)) { while (LoRa.available()) LoRa.read(); return; }
  Packet p;
  LoRa.readBytes((uint8_t *)&p, sizeof(p));
  if (!packetValid(p) || p.devId != NODE2_ID) return;
  lastRssi = LoRa.packetRssi();
  lastSnr = LoRa.packetSnr();

  if (p.type == PKT_ACK) {
    lastAckMs = millis();
    if (state == WS_ALERT_PENDING && p.arg == pendType && p.seq == pendSeq) {
      lastLatency = millis() - firstTxMs;   // measured, includes retries
      state = WS_ALERT_DELIVERED;
      Serial.printf("ACK for type=%u seq=%u after %lu ms\n", pendType, pendSeq, (unsigned long)lastLatency);
    }
  } else if (p.type == PKT_CAREGIVER_ACK) {
    caregiverAcked = true;
    Serial.println("Caregiver acknowledged");
  }
}

// ---------- Display ----------
void drawOled(uint32_t now) {
  bool link = lastAckMs && (now - lastAckMs < LINK_TIMEOUT_MS);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  switch (state) {
    case WS_SAFE:
      display.println("ELDERLY SAFETY");
      display.println();
      display.println("STATUS: SAFE");
      display.printf("|a|: %.2f g\n", amag);
      display.println();
      display.printf("LORA: %s\n", link ? "OK" : "NO LINK");
      display.printf("RSSI:%d SNR:%.1f\n", lastRssi, lastSnr);
      if (!mpuOk) display.println("IMU ERROR");
      break;
    case WS_COUNTDOWN: {
      int rem = COUNTDOWN_S - (int)((now - countdownStart) / 1000);
      if (rem < 0) rem = 0;
      display.println("POSSIBLE FALL (SIM)");
      display.println("ARE YOU OK?");
      display.setTextSize(3);
      display.setCursor(52, 22);
      display.print(rem);
      display.setTextSize(1);
      display.setCursor(0, 56);
      display.print("I'M OK = cancel");
      break;
    }
    case WS_ALERT_PENDING:
      display.println("!!! EMERGENCY !!!");
      display.println(pendType == PKT_MANUAL_SOS ? "MANUAL SOS" : "NO RESPONSE");
      display.println();
      display.println("SENDING HELP...");
      display.printf("TRY %d/%d\n", tries, MAX_TRIES);
      display.println("NOT YET ACKNOWLEDGED");
      break;
    case WS_ALERT_DELIVERED:
      display.println("!!! EMERGENCY !!!");
      display.println(pendType == PKT_MANUAL_SOS ? "MANUAL SOS" : "NO RESPONSE");
      display.println();
      display.println("HELP SENT");
      display.printf("LORA ACK: %lu ms\n", (unsigned long)lastLatency);
      display.println(caregiverAcked ? "CAREGIVER ACKED" : "WAITING CAREGIVER");
      display.println("I'M OK = cancel");
      break;
    case WS_COMM_LOST:
      display.println("!!! EMERGENCY !!!");
      display.println();
      display.println("NO ACK RECEIVED");
      display.println("HELP NOT CONFIRMED");
      display.println();
      display.println("I'M OK = reset");
      break;
  }
  display.display();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  btnBegin(bOk); btnBegin(bHelp); btnBegin(bSim);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(200);

  // MPU first, with retries and a report of which step fails
  bool initOk = false;
  for (int attempt = 1; attempt <= 5 && !initOk; attempt++) {
    bool a = mpuWrite(0x6B, 0x00);   // wake
    delay(100);
    bool b = mpuWrite(0x1A, 0x03);   // low-pass ~44 Hz
    bool c = mpuWrite(0x1B, 0x08);   // +-500 dps
    bool d = mpuWrite(0x1C, 0x10);   // +-8 g
    Serial.printf("MPU init attempt %d: wake=%d dlpf=%d gyro=%d accel=%d\n", attempt, a, b, c, d);
    initOk = a && b && c && d;
    delay(100);
  }
  Serial.println(initOk ? "MPU init OK" : "MPU init reported FAILED (will still try reading)");

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init FAILED");
    while (true) delay(1000);
  }
  display.setTextColor(SSD1306_WHITE);
  Wire.setClock(400000);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init FAILED - check wiring/power/antenna");
    display.setCursor(0, 0);
    display.println("LORA INIT FAILED");
    display.display();
    while (true) delay(1000);
  }
  LoRa.setSyncWord(0xF3);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TX_DBM);
  LoRa.receive();
  seq = (uint16_t)esp_random();   // random start so a reboot never repeats old seq numbers
  Serial.println("Node 1 ready. Serial: s=sim fall, h=SOS, k=I'M OK");
}

// ---------- Loop ----------
void loop() {
  uint32_t now = millis();
  handleRx();

  btnUpdate(bOk); btnUpdate(bHelp); btnUpdate(bSim);
  char c = Serial.available() ? Serial.read() : 0;
  bool evOk  = btnPressed(bOk) || c == 'k';
  bool evSim = btnPressed(bSim) || c == 's';
  bool evSos = btnLong(bHelp, SOS_HOLD_MS) || c == 'h';
  btnPressed(bHelp);   // clear the short-press flag (only long press does anything)

  if (evOk && state != WS_SAFE) {
    sendCancel();
    state = WS_SAFE;
    Serial.println("-> SAFE (cancelled by user)");
  } else if (evSos && (state == WS_SAFE || state == WS_COUNTDOWN)) {
    Serial.println("-> MANUAL SOS");
    startAlert(PKT_MANUAL_SOS);
  } else if (evSim && state == WS_SAFE) {
    Serial.println("-> COUNTDOWN (simulated fall)");
    state = WS_COUNTDOWN;
    countdownStart = now;
    sendOnce(PKT_FALL_WARNING, 0);
  }

  if (state == WS_COUNTDOWN && (now - countdownStart) >= COUNTDOWN_S * 1000UL) {
    Serial.println("-> no response: EMERGENCY");
    startAlert(PKT_EMERGENCY);
  }
  if (state == WS_ALERT_PENDING && (now - lastTxMs) >= RETRY_MS) {
    if (tries >= MAX_TRIES) { state = WS_COMM_LOST; Serial.println("-> COMM LOST (no ACK)"); }
    else sendAlertAttempt();
  }

  if ((now - lastImuMs) >= 20) { lastImuMs = now; readImu(); }   // 50 Hz, always attempted

  if (state == WS_SAFE && (now - lastHbMs) >= HEARTBEAT_MS) {
    lastHbMs = now;
    sendOnce(PKT_HEARTBEAT, (uint8_t)state);
  }

  if ((now - lastOledMs) >= 200) { lastOledMs = now; drawOled(now); }

  if ((now - lastDbgMs) >= 1000) {
    lastDbgMs = now;
    Serial.printf("IMU mpuOk=%d |a|=%.3f g  |gyro|=%.2f dps  imuErr=%lu  state=%d\n",
                  mpuOk, amag, gmag, (unsigned long)imuErrors, (int)state);
  }
}