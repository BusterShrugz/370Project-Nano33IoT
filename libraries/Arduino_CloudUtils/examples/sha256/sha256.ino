/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Arduino_SHA256.h>
#include <Arduino_HEX.h>
#include "buffer.h"

void setup() {
    Serial.begin(9600);
    while(!Serial);

    uint8_t sha[SHA256_DIGEST_SIZE];

    SHA256 sha256;
    sha256.begin();
    sha256.update(buffer, sizeof(buffer));
    sha256.finalize(sha);

    Serial.println(THEXT::encode(sha, sizeof(sha)));

    /* One-shot */
    SHA256::sha256(buffer, sizeof(buffer), sha);
    Serial.println(THEXT::encode(sha, sizeof(sha)));
}

void loop() { }
