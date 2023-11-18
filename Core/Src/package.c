#include "package.h"

void fromVector(uint8_t *data, ReceivePacket *pkg) {
  for (int i = 0; i < sizeof(ReceivePacket); ++i) {
    ((uint8_t *)pkg)[i] = data[i];
  }
}
