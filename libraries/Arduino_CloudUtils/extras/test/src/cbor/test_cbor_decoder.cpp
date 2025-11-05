/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cbor/MessageDecoder.h>

#include <stdio.h>

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

SCENARIO( "A custom decoder is defined", "[cbor][decode]" ) {
    CBORMessageDecoder decoder;
    GIVEN( "A buffer containing a cbor encoded message" ) {
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

        THEN( "Message decode result is Complete" ) {
            REQUIRE(res == MessageDecoder::Status::Complete);
        }

        THEN( "the decode result matches the expectations" ) {
            REQUIRE(buffer_len == sizeof(buffer));

            REQUIRE(expected_result.m.id == cmd_res.m.id);

            std::string parameter_expected(expected_result.parameter);
            std::string parameter_result(cmd_res.parameter);

            REQUIRE_THAT(parameter_result, Catch::Matchers::Equals(parameter_expected));
        }
    }
}
