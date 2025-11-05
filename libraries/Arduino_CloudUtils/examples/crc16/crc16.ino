/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#include <Arduino_CRC16.h>
#include "buffer.h"

void setup() {
    Serial.begin(9600);
    while(!Serial);

    uint16_t crc = arduino::crc16::calculate(buffer, sizeof(buffer), 0x1021, 0xFFFF, 0xFFFF, true, true);

    if(crc == 0xFA3A) {
        Serial.println("CRC is valid");
    } else {
        Serial.println("CRC is invalid");
    }

}

void loop() { }
