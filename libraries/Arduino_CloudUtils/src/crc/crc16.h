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

namespace arduino { namespace crc16 {

typedef uint16_t crc16_t;


crc16_t calculate(const void* data, size_t data_len, uint16_t poly, uint16_t initialVal, uint16_t finalXor, bool reverseIn, bool reverseOut);

}}  // arduino::crc16
