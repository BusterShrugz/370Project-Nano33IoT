/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <catch2/catch_test_macros.hpp>

#include <hex/chex.h>

SCENARIO( "HEX encoding test" ) {
    /* check all possible 16-bit values in network byte order */
    unsigned i;
    for(i = 0; i <= 0xffff; ++i){
        /* encode to hex string using our implementation */
        const unsigned char org[2] = {(unsigned char)(i>>8), (unsigned char)i};
        char hex[5] = {0,0,0,0,0};
        chex_encode(hex, sizeof(hex), org, sizeof(org));

        /* decode from hex string using a function from the standard library */
        unsigned char bin[2] = {0,0};
        int n = sscanf(hex, "%02hhx%02hhx", bin, bin+1);
        REQUIRE(n == sizeof(bin));

        /* decoded matches the original */
        REQUIRE(memcmp(org, bin, sizeof(bin)) == 0);
    }
    REQUIRE(i == 0x10000);
}

SCENARIO( "HEX decoding test" ) {
    /* check all possible 16-bit values in network byte order */
    unsigned i;
    for(i = 0; i <= 0xffff; ++i){
        /* encode to hex string using a function from the standard library */
        char hex[5] = {0,0,0,0,0};
        snprintf(hex, sizeof(hex), "%02hhX%02hhx", (unsigned char)(i>>8), (unsigned char)i);

        /* decode from hex string using our implementation */
        unsigned char bin[2];
        unsigned n = chex_decode(bin, sizeof(bin), hex, sizeof(hex));
        REQUIRE(n == sizeof(bin));

        /* decoded matches the original */
        const unsigned char org[2] = {(unsigned char)(i>>8), (unsigned char)i};
        REQUIRE(memcmp(org, bin, sizeof(bin)) == 0);
    }
    REQUIRE(i == 0x10000);
}
