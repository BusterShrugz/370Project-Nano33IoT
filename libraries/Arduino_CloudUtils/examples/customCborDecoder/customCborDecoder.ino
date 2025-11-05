/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Arduino_CBOR.h>

enum : MessageId {
    CBORTestMessageId = 0x0123,
};

enum : CBORTag {
    CBORTestMessageTag = 0x0321,
};


struct CBORTestMessage {
    Message m;
    char parameter[20];
};

class CustomMessageDecoder: public CBORMessageDecoderInterface {
public:
    CustomMessageDecoder()
    : CBORMessageDecoderInterface(CBORTestMessageTag, CBORTestMessageId) {}

protected:
    MessageDecoder::Status decode(CborValue* iter, Message *msg) override {
        CBORTestMessage* test = (CBORTestMessage*) msg;
        size_t dest_size = 20;

        if(!cbor_value_is_text_string(iter)) {
            return MessageDecoder::Status::Error;
        }

        // NOTE: keep in mind that _cbor_value_copy_string tries to put a \0 at the end of the string
        if(_cbor_value_copy_string(iter, test->parameter, &dest_size, NULL) != CborNoError) {
            return MessageDecoder::Status::Error;
        }

        return MessageDecoder::Status::Complete;
    }
} customMessageDecoder;

void setup() {
    Serial.begin(9600);
    while(!Serial);

    CBORMessageDecoder decoder;

    CBORTestMessage expected_result {
        CBORTestMessageId,
        "abcdef",
    };

    uint8_t buffer[] {
        0xD9, 0x03, 0x21, 0x81, 0x66, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66,
    };
    size_t buffer_len = sizeof(buffer);

    CBORTestMessage cmd_res;
    MessageDecoder::Status res = decoder.decode((Message*)&cmd_res, buffer, buffer_len);

    if(res == MessageDecoder::Status::Complete &&
        cmd_res.m.id == expected_result.m.id &&
        strcmp(cmd_res.parameter, expected_result.parameter) == 0) {

        Serial.println("Decode operation completed with success");
    } else {
        Serial.println("Decode operation failed");
    }
}

void loop() {}
