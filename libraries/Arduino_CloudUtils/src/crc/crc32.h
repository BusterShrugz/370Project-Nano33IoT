/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace arduino { namespace crc32 {
    /*
     * This library contains the methods to calculate CRC32 checksums
     * in order to calculate crc of a binary blob you need to:
     * 1- call begin() and get an initialized crc32 value
     * 2- call update() on the bynary chunks
     * 3- finalize the value with finalize()
     * 4- store the value or use it for comparison
     */

    typedef uint32_t crc32_t;

    inline crc32_t begin() { return 0xFFFFFFFF; }
    crc32_t update(crc32_t crc, const void * data, size_t data_len);
    inline crc32_t finalize(crc32_t crc) { return crc ^ 0xFFFFFFFF;}

}} // arduino::crc32
