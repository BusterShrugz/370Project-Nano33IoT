/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "hex.h"
#include "chex.h"

namespace arduino { namespace hex {

    String encode(const uint8_t* in, uint32_t size) {
        char out[(size * 2) + 1];
        unsigned int byteNumber;
        byteNumber = chex_encode(out, sizeof(out), in, size);
        out[byteNumber] = 0;
        return String(out);
    }

    String encodeUpper(const uint8_t* in, uint32_t size) {
        String out = encode(in, size);
        out.toUpperCase();
        return out;
    }

    bool decode(const String in, uint8_t* out, uint32_t size) {
        return decode(in.c_str(), out, size);
    }

    bool decode(const char *in, uint8_t* out, uint32_t size) {
        unsigned int byteNumber;
        size_t len = strlen(in);
        byteNumber = chex_decode(out, size, in, len);
        return byteNumber * 2  == len;
    }

}} // arduino::hex
