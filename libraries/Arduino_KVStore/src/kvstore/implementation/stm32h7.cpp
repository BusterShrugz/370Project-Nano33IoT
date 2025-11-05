/*
 * This file is part of Arduino_KVStore.
 *
 * Copyright (c) 2024 Arduino SA
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#if defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) \
    || defined(ARDUINO_OPTA) || defined(ARDUINO_GIGA)
#include "stm32h7.h"


STM32H7KVStore::STM32H7KVStore(): kvstore(nullptr), bd(nullptr) {}

bool STM32H7KVStore::begin() {
    return begin(false);
}

bool STM32H7KVStore::begin(bool reformat, mbed::KVStore* store) {
    // bd gets allocated if a kvstore is not provided as parameter here
    // if either one of bd or kvstore is different from NULL it means that the kvstore
    // had already been called begin on
    if(bd != nullptr || kvstore != nullptr) {
        return false;
    }

    if(store != nullptr) {
        kvstore = store;
    } else {
        auto root = mbed::BlockDevice::get_default_instance();

        if (root->init() != QSPIF_BD_ERROR_OK) {
            Serial.println(F("Error: QSPI init failure."));
            return false;
        }

        bd = new mbed::MBRBlockDevice(root, 3);
        int res = bd->init();
        if (res != QSPIF_BD_ERROR_OK && !reformat) {
            Serial.println(F("Error: QSPI is not properly formatted, "
                "run QSPIformat.ino or set reformat to true"));
            return false;
        } else if (res != QSPIF_BD_ERROR_OK && reformat) {
            Serial.println(F("Error: QSPI is not properly formatted, "
                "reformatting it according to the following scheme:"));
            Serial.println(F("Partition 1: WiFi firmware and certificates 1MB"));
            Serial.println(F("Partition 2: OTA and user data 12MB"));
            Serial.println(F("Partition 3: Provisioning KVStore 1MB"));

            // clearing MBR Table
            root->erase(0x0, root->get_erase_size());

            mbed::MBRBlockDevice::partition(root, 1, 0x0B, 0, 1024 * 1024);
            mbed::MBRBlockDevice::partition(root, 2, 0x0B, 1024 * 1024, 13 * 1024 * 1024);
            mbed::MBRBlockDevice::partition(root, 3, 0x0B, 13 * 1024 * 1024, 14 * 1024 * 1024);
        }

        kvstore = new mbed::TDBStore(bd);
    }

    return kvstore->init() == MBED_SUCCESS;
}

bool STM32H7KVStore::end() {
    bool res = false;

    if(kvstore != nullptr && bd == nullptr) {
        res = kvstore->deinit() == MBED_SUCCESS;
        kvstore = nullptr;
    } else if(kvstore != nullptr && bd != nullptr) {
        res = kvstore->deinit() == MBED_SUCCESS;

        delete kvstore;
        kvstore = nullptr;

        delete bd;
        bd = nullptr;
    }

    return res;
}

template<typename T=int>
static inline typename KVStoreInterface::res_t fromMbedErrors(int error, T res=1) {
    return error == MBED_SUCCESS ? res : -error;
}

bool STM32H7KVStore::clear() {
    return kvstore != nullptr ? kvstore->reset() == MBED_SUCCESS : false;
}

typename KVStoreInterface::res_t STM32H7KVStore::remove(const key_t& key) {
    return kvstore != nullptr ? fromMbedErrors(kvstore->remove(key)) : -1;
}

typename KVStoreInterface::res_t STM32H7KVStore::putBytes(const key_t& key, const uint8_t buf[], size_t len) {
    return kvstore != nullptr ? fromMbedErrors(kvstore->set(key, buf, len, 0), len) : -1; // TODO flags
}

typename KVStoreInterface::res_t STM32H7KVStore::getBytes(const key_t& key, uint8_t buf[], size_t maxLen) const {
    if(kvstore == nullptr) {
        return -1;
    }

    size_t actual_size = maxLen;
    auto res = kvstore->get(key, buf, maxLen, &actual_size);

    return fromMbedErrors(res, actual_size);
}

size_t STM32H7KVStore::getBytesLength(const key_t& key) const {
    if(kvstore == nullptr) {
        return 0;
    }

    mbed::KVStore::info_t info;
    auto res = kvstore->get_info(key, &info);

    return res == MBED_SUCCESS ? info.size : 0;
}

bool STM32H7KVStore::exists(const key_t& key) const {
    return getBytesLength(key) > 0;
}

#endif // defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) || defined(ARDUINO_OPTA) || defined(ARDUINO_GIGA)
