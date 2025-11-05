/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#ifndef min
  #define min(a,b) ((a)<(b)?(a):(b))
#endif

void          set_millis(unsigned long const millis);
unsigned long millis();
