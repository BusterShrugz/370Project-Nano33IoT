/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2025 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Arduino_CBOR.h>
#include <cbor/standards/StandardMessages.h>

void setup() {
    Serial.begin(9600);
    while(!Serial);

    VersionMessage command;
    command.c.id = StandardMessageId::WiFiFWVersionMessageId;
    command.params.version = "1.6.0";
    uint8_t buffer[512];
    size_t buf_len = sizeof(buffer);

    CBORMessageEncoder encoder;
    MessageEncoder::Status err = encoder.encode((Message*)&command, buffer, buf_len);

    uint8_t expected_result[] = {
    0xda, 0x00, 0x01, 0x20, 0x14, 0x81, 0x65, 0x31, 0x2E, 0x36, 0x2E, 0x30
    };
    size_t res_len=buf_len;
    MessageEncoder::Status res = encoder.encode((Message*)&command, buffer, res_len);

    if(res == MessageEncoder::Status::Complete &&
        memcmp(buffer, expected_result, res_len) == 0) {

        Serial.println("Encode operation completed with success");
    } else {
        Serial.println("Encode operation failed");
    }
}

void loop() {}
