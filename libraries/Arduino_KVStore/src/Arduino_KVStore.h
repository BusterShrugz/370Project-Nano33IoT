/*
 * This file is part of Arduino_KVStore.
 *
 * Copyright (c) 2024 Arduino SA
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#pragma once
#include "kvstore/kvstore.h"

#if defined(ARDUINO_UNOR4_WIFI)
#include "kvstore/implementation/UnoR4.h"

using KVStore = Unor4KVStore;

#elif defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) \
    || defined(ARDUINO_OPTA) || defined(ARDUINO_GIGA)
#include "kvstore/implementation/stm32h7.h"

using KVStore = STM32H7KVStore;

#elif defined(ARDUINO_PORTENTA_C33)

#include "kvstore/implementation/portentac33.h"

using KVStore = PortentaC33KVStore;

#elif defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_NANO_RP2040_CONNECT)

#include "kvstore/implementation/Nina.h"

using KVStore = NinaKVStore;

#elif defined(ARDUINO_ARCH_ESP32)

#include "kvstore/implementation/ESP32.h"

using KVStore = ESP32KVStore;

#else
#error "Arduino KVStore is not supported on current platform"
#endif
