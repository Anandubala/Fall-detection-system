#pragma once
#include <stdint.h>
#include <stddef.h>

#define PKT_MAGIC 0xA5
#define PKT_VER   1

enum PktType : uint8_t {
  PKT_HEARTBEAT     = 1,
  PKT_FALL_WARNING  = 2,
  PKT_EMERGENCY     = 3,
  PKT_MANUAL_SOS    = 4,
  PKT_CANCEL        = 5,
  PKT_ACK           = 6,   // Node 2 -> wearable: "I received seq X of type Y"
  PKT_CAREGIVER_ACK = 7    // Node 2 -> wearable: caregiver pressed the button
};

enum WearState : uint8_t {
  WS_SAFE = 0, WS_COUNTDOWN, WS_ALERT_PENDING, WS_ALERT_DELIVERED, WS_COMM_LOST
};

// 10 bytes on air
struct __attribute__((packed)) Packet {
  uint8_t  magic;
  uint8_t  ver;
  uint8_t  devId;
  uint8_t  type;
  uint16_t seq;
  uint8_t  arg;     // ACK: type being acknowledged. Heartbeat: wearable state.
  int16_t  value;   // Heartbeat: |a| in milli-g
  uint8_t  crc;     // CRC-8 of the 9 bytes above (detects corruption, NOT security)
};

static inline uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  while (n--) {
    c ^= *d++;
    for (uint8_t i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

static inline void fillPacket(Packet &p, uint8_t devId, uint8_t type, uint16_t seq, uint8_t arg, int16_t value) {
  p.magic = PKT_MAGIC; p.ver = PKT_VER; p.devId = devId; p.type = type;
  p.seq = seq; p.arg = arg; p.value = value;
  p.crc = crc8((const uint8_t *)&p, sizeof(Packet) - 1);
}

static inline bool packetValid(const Packet &p) {
  return p.magic == PKT_MAGIC && p.ver == PKT_VER &&
         p.crc == crc8((const uint8_t *)&p, sizeof(Packet) - 1);
}