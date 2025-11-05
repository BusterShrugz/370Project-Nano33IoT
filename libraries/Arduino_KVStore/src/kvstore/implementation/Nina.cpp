/*
 * This file is part of Arduino_KVStore.
 *
 * Copyright (c) 2024 Arduino SA
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_NANO_RP2040_CONNECT)

#include "Nina.h"

using namespace std;

bool NinaKVStore::begin(const char* name, bool readOnly, const char* partitionLabel) {
    WiFiDrv::wifiDriverInit();

    String fv = WiFi.firmwareVersion();
    if (fv < "3.0.0") {
        Serial.println("KVStore is not supported on Nina chip for versions older than 3.0.0");

        return false;
    }

    return WiFiDrv::prefBegin(name, readOnly, partitionLabel);
}

bool NinaKVStore::begin() {
    return begin(DEFAULT_KVSTORE_NAME);
}

bool NinaKVStore::end() {
    WiFiDrv::prefEnd();
    return true;
}

bool NinaKVStore::clear() {
    return WiFiDrv::prefClear();
}

typename KVStoreInterface::res_t NinaKVStore::remove(const key_t& key) {
    return WiFiDrv::prefRemove(key);
}

typename KVStoreInterface::res_t NinaKVStore::putBytes(const key_t& key, const uint8_t value[], size_t len) {
    return WiFiDrv::prefPut(key, static_cast<PreferenceType>(PT_BLOB), value, len);
}

typename KVStoreInterface::res_t NinaKVStore::getBytes(const key_t& key, uint8_t buf[], size_t maxLen) const {
    return WiFiDrv::prefGet(key, static_cast<PreferenceType>(PT_BLOB), buf, maxLen);
}

size_t NinaKVStore::getBytesLength(const key_t& key) const {
    auto len = WiFiDrv::prefLen(key);
    return WiFiDrv::prefGetType(key) == PT_STR ? len-1 : len;
}

bool NinaKVStore::exists(const key_t& key) const {
    return WiFiDrv::prefGetType(key) != PT_INVALID;
}

typename KVStoreInterface::res_t NinaKVStore::_put(
    const key_t& key, const uint8_t value[], size_t len, KVStoreInterface::Type t) {
    if(t == PT_DOUBLE || t == PT_FLOAT) {
        t = PT_BLOB;
    }

    if(t == PT_STR) {
        len++; // For strings we also send the \0
    }
    return WiFiDrv::prefPut(key, static_cast<PreferenceType>(t), value, len);
}

typename KVStoreInterface::res_t NinaKVStore::_get(const key_t& key, uint8_t value[], size_t len, Type t) {
    if(t == PT_DOUBLE || t == PT_FLOAT) {
        t = PT_BLOB;
    }

    size_t res = WiFiDrv::prefGet(key, static_cast<PreferenceType>(t), value, len);

    if(t == PT_STR) {
        value[res] = '\0';
    }

    return res;
}

#endif // defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_NANO_RP2040_CONNECT)
