/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <catch2/catch_test_macros.hpp>

#include <lzss/decoder.h>
#include <stdio.h>

using namespace arduino::lzss;

SCENARIO( "Decoding an LZSS stream of data", "[lzss::Decoder]" ) {
    GIVEN( "A LZSS compressed file and an LZSS Decoder" ) {
        // the lzss file provided as test when decompressed are made by incrementing a 4 byte integer
        // starting from 0, up to (file size-1)/4, thus it is easy to validate the correctness of the result

        FILE *f = fopen(SOURCE_DIR "/test_files/test-64k.lzss", "r");

        REQUIRE( f != nullptr );

        uint32_t value = 0;
        uint64_t position = 0;

        auto decoder = arduino::lzss::Decoder([&value, &position](const uint8_t b) {
            REQUIRE(((value >> ((3-(position%4))<<3)) & 0xFF) == b); // make this an assertion, not a require

            if(position % 4 == 0 && position != 0) {
                value++;
            }
            position++;
        });

        WHEN( "Decompress is called on the decoder" ) {
            uint8_t buf[64];
            size_t read_bytes;
            do {
                read_bytes = fread(buf, 1, sizeof(buf), f);

                if(read_bytes > 0) {
                    decoder.decompress(buf, read_bytes);
                }
            } while(read_bytes > 0);

            REQUIRE(value == (1<<14) -1);
        }
        THEN( "There is no error on the file" ) {
            REQUIRE(feof(f) == 0);
            REQUIRE(ferror(f) == 0);
            REQUIRE(fclose(f) == 0);
        }
    }
}
