/*
  Copyright (c) 2025 Arduino SA

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#if defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) \
  || defined(ARDUINO_OPTA) || defined(ARDUINO_GIGA)
#include "H7FlashFormatter.h"
#include "certificates.h"

bool MBEDH7FlashFormatter::checkWiFiData()
{
  bool ret = false;
  ret = checkFile("/wlan", "cacert.pem");
  if (!ret) {
    return false;
  }
  ret = checkFile("/wlan", "4343WA1.BIN");
  return ret;
}

bool MBEDH7FlashFormatter::restoreWifiData()
{
  bool ret = false;
  extern const unsigned char wifi_firmware_image_data[];
  ret = writeFile("/wlan", "4343WA1.BIN", wifi_firmware_image_data, 421098, 1024);
  if (!ret) {
    return false;
  }

  ret = writeFile("/wlan", "cacert.pem", cacert_pem, cacert_pem_len, 128);
  if (!ret) {
    return false;
  }

  const uint32_t offset = 15 * 1024 * 1024 + 1024 * 512;
  ret = writeFlash(wifi_firmware_image_data, 421098, offset, 1024);
  return ret;
}

#endif
