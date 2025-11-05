/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#include <Arduino_CRC32.h>
#include "buffer.h"

void setup() {
    Serial.begin(9600);
    while(!Serial);

    uint32_t crc32 = arduino::crc32::begin();

    crc32 = arduino::crc32::update(crc32, buffer, sizeof(buffer));

    crc32 = arduino::crc32::finalize(crc32);

    if(crc32 == 0xD4C2968B) {
        Serial.println("CRC is valid");
    } else {
        Serial.println("CRC is invalid");
    }

}

void loop() { }
