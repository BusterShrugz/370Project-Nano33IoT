/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_vector.hpp>
#include <cbor/standards/StandardEncoders.h>

/******************************************************************************
    TEST CODE
 ******************************************************************************/

SCENARIO("Test the encoding of command messages") {

  WHEN("Encode a message with provisioning wifi fw version ")
  {
    VersionMessage command;
    command.c.id = StandardMessageId::WiFiFWVersionMessageId;
    command.params.version = "1.6.0";
    uint8_t buffer[512];
    size_t bytes_encoded = sizeof(buffer);

    CBORMessageEncoder encoder;
    MessageEncoder::Status err = encoder.encode((Message*)&command, buffer, bytes_encoded);

    uint8_t expected_result[] = {
    0xda, 0x00, 0x01, 0x20, 0x14, 0x81, 0x65, 0x31, 0x2E, 0x36, 0x2E, 0x30
    };

    // Test the encoding is
    //DA 00012014     # tag(73748)
    // 81             # array(1)
    //   65           # text(5)
    //     312E362E30 # "1.6.0"
    THEN("The encoding is successful") {
        REQUIRE(err == MessageEncoder::Status::Complete);
        REQUIRE(bytes_encoded == sizeof(expected_result));
        REQUIRE(memcmp(buffer, expected_result, sizeof(expected_result)) == 0);
    }
  }

  WHEN("Error encoding a message with provisioning wifi fw version not enough space for array")
  {
    VersionMessage command;
    command.c.id = StandardMessageId::WiFiFWVersionMessageId;
    command.params.version = "1.6.0";
    uint8_t buffer[5]; // Not enough space
    size_t bytes_encoded = sizeof(buffer);

    CBORMessageEncoder encoder;
    MessageEncoder::Status err = encoder.encode((Message*)&command, buffer, bytes_encoded);

    // Test the encoding fails due to insufficient space
    THEN("The encoding fails with an error") {
        REQUIRE(err == MessageEncoder::Status::Error);
    }
  }

  WHEN("Error encoding a message with provisioning wifi fw version not enough space for version string")
  {
    VersionMessage command;
    command.c.id = StandardMessageId::WiFiFWVersionMessageId;
    command.params.version = "1.6.0";
    uint8_t buffer[7]; // Not enough space
    size_t bytes_encoded = sizeof(buffer);

    CBORMessageEncoder encoder;
    MessageEncoder::Status err = encoder.encode((Message*)&command, buffer, bytes_encoded);

    // Test the encoding fails due to insufficient space
    THEN("The encoding fails with an error") {
        REQUIRE(err == MessageEncoder::Status::Error);
    }
  }

}
