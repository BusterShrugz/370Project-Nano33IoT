/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "crc16.h"

namespace arduino { namespace crc16 {

uint16_t reflect(uint16_t num) {
  uint16_t reflected = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (num & (1 << i)) {
      reflected |= 1 << (15 - i);
    }
  }
  return reflected;
}

crc16_t calculate(const void *data, size_t data_len, uint16_t poly, uint16_t initialVal, uint16_t finalXor, bool reverseIn, bool reverseOut) {
  const uint8_t *d = (const uint8_t *)data;
  uint16_t crc = initialVal;
  uint16_t polynomial = poly;

  if (reverseIn) {
    crc = reflect(crc);
    polynomial = reflect(poly);

    for (size_t i = 0; i < data_len; i++) {
      crc ^= d[i];

      for (uint8_t b = 0; b < 8; b++) {
        if (crc & 0x0001) {
          crc = (crc >> 1) ^ polynomial;
        } else {
          crc = (crc >> 1);
        }
      }
    }

    if (!reverseOut) {
      crc = reflect(crc);
    }
  } else {

    for (size_t i = 0; i < data_len; i++) {
      crc ^= (uint16_t)d[i] << 8;

      for (uint8_t b = 0; b < 8; b++) {
        if (crc & 0x8000) {
          crc = (crc << 1) ^ polynomial;
        } else {
          crc = (crc << 1);
        }
      }
    }

    if (reverseOut) {
      crc = reflect(crc);
    }
  }

  crc ^= finalXor;

  return crc;
}
}}
