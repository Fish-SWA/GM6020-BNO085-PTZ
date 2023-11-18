#ifndef PKG
#define PKG
#include <stdbool.h>
#include <stdint.h>

typedef struct SendPacket {
  uint8_t header;
  uint8_t detect_color : 1; // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;  // TODO: no checksum for now
} __attribute__((packed)) SendPacket;

typedef struct ReceivePacket {
  uint8_t header;
  bool tracking : 1;
  uint8_t id : 3;         // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum;
} __attribute__((packed)) ReceivePacket;

#define SEND_PKG_SIZE sizeof(SendPacket)

inline void fromVector(uint8_t *data, ReceivePacket *pkg);

#endif
