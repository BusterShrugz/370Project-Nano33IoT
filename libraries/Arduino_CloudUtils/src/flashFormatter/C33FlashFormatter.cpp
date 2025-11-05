/*
  Copyright (c) 2025 Arduino SA

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#if defined(ARDUINO_PORTENTA_C33)
#include "C33FlashFormatter.h"
#include "certificates.h"

bool C33FlashFormatter::checkWiFiData()
{
  return checkFile("/wlan", "cacert.pem");
}

bool C33FlashFormatter::restoreWifiData()
{
  return writeFile("/wlan", "cacert.pem", cacert_pem, cacert_pem_len, 128);
}

#endif
