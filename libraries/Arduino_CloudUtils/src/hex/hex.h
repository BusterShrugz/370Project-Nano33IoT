/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <Arduino.h>

namespace arduino { namespace hex {
    /*
     * This library contains the methods to hex encode a buffer into a String
     * and vice-versa
     */

    String encode(const uint8_t* in, uint32_t size);
    String encodeUpper(const uint8_t* in, uint32_t size);
    bool decode(const char *in, uint8_t* out, uint32_t size);
    bool decode(const String in, uint8_t* out, uint32_t size);
}} // arduino::hex

class THEXT {
public:
    static inline String encode(const uint8_t* in, uint32_t size) {
        return arduino::hex::encode(in, size);
    }
    static inline String encodeUpper(const uint8_t* in, uint32_t size) {
        return arduino::hex::encodeUpper(in, size);
    }

    static inline bool decode(const String in, uint8_t* out, uint32_t size) {
        return arduino::hex::decode(in, out, size);
    }

    static inline bool decode(const char *in, uint8_t* out, uint32_t size) {
        return arduino::hex::decode(in, out, size);
    }
};
