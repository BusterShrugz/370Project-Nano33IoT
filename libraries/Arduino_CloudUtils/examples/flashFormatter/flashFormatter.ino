/*
  This file is part of the Arduino_CloudUtils library.

  Copyright (c) 2024 Arduino SA

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Arduino_FlashFormatter.h>

FlashFormatter flashFormatter;

void setup() {
  Serial.begin(9600);
  while(!Serial);

  /* WARNING! Running this sketch all the content of the QSPI flash may be erased.
   * The sketch will check if the QSPI flash is formatted with an ArduinoCloud
   * compatible partitioning. Otherwise, it will format the QSPI flash with the
   * default scheme.
   *
   * If you want to keep the content of the QSPI flash, do not run this sketch.
   *
   * ArduinoCloud compatible partitioning consist of:
   * - 1st partition: WiFi firmware and certificates
   * - 2nd partition: OTA data. Minimum size 5MB.
   * - 3rd partition: Key Value Store data. Minimum size 1MB.
   */

  Serial.println("\nWARNING! Running this sketch all the content of the QSPI flash may be erased.");
  Serial.println("Do you want to proceed? Y/[n]");

  if (true == waitResponse()) {
    if(!flashFormatter.checkAndFormatPartition()){
      Serial.println("Failed to check / format flash");
    } else {
      Serial.println("Flash checked / formatted successfully");
    }
  }
  else {
    Serial.println("Operation canceled");
  }

  Serial.println("It's now safe to reboot or disconnect your board.");
}

void loop() { }

bool waitResponse() {
  bool confirmation = false;
  bool proceed = false;
  while (confirmation == false) {
    if (Serial.available()) {
      char choice = Serial.read();
      switch (choice) {
        case 'y':
        case 'Y':
          confirmation = true;
          proceed = true;
          break;
        case 'n':
        case 'N':
          confirmation = true;
          proceed = false;
          break;
        default:
          continue;
      }
    }
  }
  return proceed;
}
