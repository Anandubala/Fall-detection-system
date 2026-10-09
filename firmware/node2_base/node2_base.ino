#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "protocol.h"

// ---------- Pins ----------
// I2C LCD uses the ESP32 default I2C pins: SDA = GPIO21, SCL = GPIO22
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 4
#define BTN_ACK 25
#define BUZZER_PIN 32          // -> 220R -> AO3400 gate

// ---------- Config ----------
#define BUZZER_ACTIVE 1        // 1 = active buzzer, 0 = passive buzzer (PWM tone)
#define NODE2_ID 2
#define LORA_FREQ 433E6
#define LORA_TX_DBM 10
#define OFFLINE_MS 15000UL     // 3 missed heartbeats (heartbeat = 5 s)
#define DEBOUNCE_MS 30UL

hd44780_I2Cexp lcd;            // auto-detects the I2C address and backpack wiring
bool lcdOk = false;
char lcdCache[2][17] = {"", ""};

enum AlarmLevel : uint8_t { AL_NONE = 0, AL_WARNING, AL_EMERGENCY };

AlarmLevel alarmLvl = AL_NONE;
bool silenced = false;
uint8_t alarmType = 0;
uint8_t wearState = 0;
int16_t lastAmg = 0;

uint8_t lastDev = 0, lastType = 0;
uint16_t lastSeq = 0, cgSeq = 0;
bool haveLast = false;

uint32_t lastSeenMs = 0, cancelInfoUntil = 0, ackInfoUntil = 0, lastLcdMs = 0;
uint32_t rxCount = 0, dupCount = 0, badCount = 0;
int lastRssi = 0;
float lastSnr = 0;

// ---------- LCD helper ----------
// Writes one 16-char line, padded, and only touches the LCD if the text changed
// (no clear() = no flicker, and less I2C traffic).
void lcdLine(uint8_t row, const char *text) {
  if (!lcdOk) return;
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16.16s", text);
  if (strcmp(buf, lcdCache[row]) != 0) {
    lcd.setCursor(0, row);
    lcd.print(buf);
    strcpy(lcdCache[row], buf);
  }
}

// ---------- Button ----------
bool bRaw = false, bStable = false, bEvt = false;
uint32_t bLast = 0;
void btnUpdate() {
  bool r = (digitalRead(BTN_ACK) == LOW);
  uint32_t now = millis();
  if (r != bRaw) { bRaw = r; bLast = now; }
  if ((now - bLast) >= DEBOUNCE_MS && bStable != bRaw) {
    bStable = bRaw;
    if (bStable) bEvt = true;
  }
}

// ---------- Buzzer ----------
void buzzer(bool on) {
#if BUZZER_ACTIVE
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
#else
  ledcWriteTone(BUZZER_PIN, on ? 2700 : 0);
#endif
}

void buzzerTick(uint32_t now) {
  if (alarmLvl == AL_NONE || silenced) { buzzer(false); return; }
  if (alarmLvl == AL_WARNING)  buzzer((now % 1000) < 150);   // short chirp each second
  else                         buzzer((now % 450) < 300);    // fast, urgent
}

// ---------- LoRa ----------
void sendRaw(const Packet &p) {
  LoRa.beginPacket();
  LoRa.write((const uint8_t *)&p, sizeof(p));
  LoRa.endPacket();
  LoRa.receive();
}

void sendAck(const Packet &rx) {
  Packet a;
  fillPacket(a, NODE2_ID, PKT_ACK, rx.seq, rx.type, 0);
  delay(10);   // let the sender switch back to receive
  sendRaw(a);
}

void sendCaregiverAck() {
  Packet a;
  fillPacket(a, NODE2_ID, PKT_CAREGIVER_ACK, ++cgSeq, 0, 0);
  for (int i = 0; i < 3; i++) { sendRaw(a); delay(150); }
}

void handleRx(uint32_t now) {
  int n = LoRa.parsePacket();
  if (n == 0) return;
  if (n != (int)sizeof(Packet)) {
    while (LoRa.available()) LoRa.read();
    badCount++;
    return;
  }
  Packet p;
  LoRa.readBytes((uint8_t *)&p, sizeof(p));
  if (!packetValid(p)) { badCount++; return; }
  if (p.type == PKT_ACK || p.type == PKT_CAREGIVER_ACK) return;   // not for us

  lastRssi = LoRa.packetRssi();
  lastSnr = LoRa.packetSnr();
  lastSeenMs = now;
  rxCount++;

  sendAck(p);   // always ACK, even duplicates (the first ACK may have been lost)

  bool dup = haveLast && p.devId == lastDev && p.type == lastType && p.seq == lastSeq;
  if (dup) { dupCount++; return; }
  haveLast = true; lastDev = p.devId; lastType = p.type; lastSeq = p.seq;

  Serial.printf("RX type=%u seq=%u rssi=%d snr=%.1f\n", p.type, p.seq, lastRssi, lastSnr);

  switch (p.type) {
    case PKT_HEARTBEAT:
      wearState = p.arg;
      lastAmg = p.value;
      break;
    case PKT_FALL_WARNING:
      if (alarmLvl != AL_EMERGENCY) { alarmLvl = AL_WARNING; silenced = false; alarmType = p.type; }
      break;
    case PKT_EMERGENCY:
    case PKT_MANUAL_SOS:
      alarmLvl = AL_EMERGENCY; silenced = false; alarmType = p.type;
      break;
    case PKT_CANCEL:
      if (alarmLvl != AL_NONE) { alarmLvl = AL_NONE; silenced = false; cancelInfoUntil = now + 8000; }
      break;
  }
}

// ---------- Display (16x2 LCD) ----------
void drawLcd(uint32_t now) {
  if (!lcdOk) return;
  bool online = lastSeenMs && (now - lastSeenMs < OFFLINE_MS);
  char b[24];

  if (alarmLvl == AL_EMERGENCY) {
    lcdLine(0, "!! EMERGENCY !!");
    if ((now / 2000) % 2 == 0)
      lcdLine(1, alarmType == PKT_MANUAL_SOS ? "MANUAL SOS" : "FALL-NO RESPONSE");
    else
      lcdLine(1, silenced ? "ACK>PRESS=CLEAR" : "PRESS ACK BTN");
  } else if (alarmLvl == AL_WARNING) {
    lcdLine(0, "POSSIBLE FALL");
    lcdLine(1, "USER CAN CANCEL");
  } else {
    lcdLine(0, online ? "WEARABLE ONLINE" : "WEARABLE OFFLINE");
    if (now < cancelInfoUntil) {
      lcdLine(1, "USER CANCELLED");
    } else if (now < ackInfoUntil) {
      lcdLine(1, "ACK SENT TO USER");
    } else if (!lastSeenMs) {
      lcdLine(1, "WAITING...");
    } else if (!online) {
      snprintf(b, sizeof(b), "SEEN %lus AGO", (unsigned long)((now - lastSeenMs) / 1000));
      lcdLine(1, b);
    } else {
      snprintf(b, sizeof(b), "RSSI%d SNR%.0f", lastRssi, lastSnr);
      lcdLine(1, b);
    }
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  pinMode(BTN_ACK, INPUT_PULLUP);
#if BUZZER_ACTIVE
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
#else
  ledcAttach(BUZZER_PIN, 2700, 8);
  ledcWriteTone(BUZZER_PIN, 0);
#endif
  delay(500);

  int st = lcd.begin(16, 2);   // finds the LCD address itself; also starts Wire on SDA21/SCL22
  lcdOk = (st == 0);
  Serial.printf("LCD init: %s (status %d)\n", lcdOk ? "OK" : "FAILED", st);
  Wire.setClock(100000);
  lcdLine(0, "NODE 2 STARTING");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init FAILED - check wiring/power/antenna");
    lcdLine(0, "LORA INIT FAILED");
    lcdLine(1, "CHECK WIRING");
    while (true) delay(1000);
  }
  LoRa.setSyncWord(0xF3);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TX_DBM);
  LoRa.receive();
  lcdLine(0, "NODE 2 READY");
  Serial.println("Node 2 ready");
}

void loop() {
  uint32_t now = millis();
  handleRx(now);

  btnUpdate();
  if (bEvt) {
    bEvt = false;
    if (alarmLvl != AL_NONE && !silenced) {
      silenced = true;
      buzzer(false);
      sendCaregiverAck();
      ackInfoUntil = now + 5000;
      Serial.println("Caregiver ACK sent");
    } else if (alarmLvl != AL_NONE && silenced) {
      alarmLvl = AL_NONE; silenced = false;   // second press clears the alarm screen
    }
  }

  buzzerTick(now);
  if ((now - lastLcdMs) >= 200) { lastLcdMs = now; drawLcd(now); }
}