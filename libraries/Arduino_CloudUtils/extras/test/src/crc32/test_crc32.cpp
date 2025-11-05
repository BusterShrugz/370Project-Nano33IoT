/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <catch2/catch_test_macros.hpp>

#include <crc/crc32.h>
#include <stdio.h>

SCENARIO( "Calculating a CRC32 checksum on a binary stream" ) {
    GIVEN( "a binary stream" ) {
        FILE *f = fopen(SOURCE_DIR "/test_files/test-64k", "r");
        REQUIRE( f != nullptr );

        uint32_t crc32 = arduino::crc32::begin();

        THEN( "CRC32 is calculated on the binary stream" ) {
            uint8_t buf[64];
            size_t read_bytes;
            do {
                read_bytes = fread(buf, 1, sizeof(buf), f);

                if(read_bytes > 0) {
                    crc32 = arduino::crc32::update(crc32, buf, sizeof(buf));
                }
            } while(read_bytes > 0);

            crc32 = arduino::crc32::finalize(crc32);

            REQUIRE( crc32 == 0x9512B7A7 );
        }
    }
}
