/*
  Copyright (c) 2025 Arduino SA

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#pragma once
#if defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) \
  || defined(ARDUINO_OPTA) || defined(ARDUINO_GIGA)
#include "H7FlashFormatter.h"
using FlashFormatter = MBEDH7FlashFormatter;
#elif defined(ARDUINO_PORTENTA_C33)
#include "C33FlashFormatter.h"
using FlashFormatter = C33FlashFormatter;
#else
#include "FlashFormatterBase.h"
using FlashFormatter = FlashFormatterBase;
#endif
