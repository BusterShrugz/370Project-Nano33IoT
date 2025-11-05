/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#include <Arduino_Lzss.h>
#include "lzssBuffer.h"

void setup() {
    Serial.begin(9600);
    while(!Serial);

    uint32_t value = 0;
    uint64_t position = 0;
    bool error = false;

    // the lzss buffer provided as test, when decompressed, are made by incrementing a 4 byte integer
    // starting from 0, up to (file size-1)/4, thus it is easy to validate the correctness of the result
    auto decoder = arduino::lzss::Decoder([&value, &position, &error](const uint8_t b) {
        error = ((value >> ((3-(position%4))<<3)) & 0xFF) != b;

        if(position % 4 == 0 && position != 0) {
            value++;
        }
        position++;
    });

    for(size_t i = 0; i < sizeof(lzssBuffer); i++) {
        decoder.decompress(lzssBuffer + i, 1);

        if(error) {
            Serial.println("Error during decompression of buffer");

            return;
        }
    }

    Serial.println("Decompression completed with success");
}

void loop() { }
