/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Arduino_TimedAttempt.h>

#if !defined(LED_BUILTIN) && !defined(ARDUINO_NANO_ESP32)
static int const LED_BUILTIN = 2;
#endif

TimedAttempt blink(500, 1000);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() { 
  if(blink.isExpired()) {
    blink.retry();
    digitalWrite(LED_BUILTIN, blink.getRetryCount() % 2);
  }

}
