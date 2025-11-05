/*
 * This file is part of Arduino_KVStore.
 *
 * Copyright (c) 2024 Arduino SA
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#if defined(ARDUINO_PORTENTA_C33)
#include "portentac33.h"

PortentaC33KVStore::PortentaC33KVStore(): kvstore(nullptr), bd(nullptr) {}

bool PortentaC33KVStore::begin() {
    return begin(false);
}

bool PortentaC33KVStore::begin(bool reformat, mbed::KVStore* store) {
    // bd gets allocated if a kvstore is not provided as parameter here
    // if either one of bd or kvstore is different from NULL it means that the kvstore
    // had already been called begin on
    if(bd != nullptr || kvstore != nullptr) {
        return false;
    }

    if(store != nullptr) {
        kvstore = store;
    } else {
        auto bd = new MBRBlockDevice(BlockDevice::get_default_instance(), 3);

        kvstore = new TDBStore(bd);
    }

    return kvstore->init() == KVSTORE_SUCCESS;
}

bool PortentaC33KVStore::end() {
    bool res = false;

    if(kvstore != nullptr && bd == nullptr) {
        res = kvstore->deinit() == KVSTORE_SUCCESS;
        kvstore = nullptr;
    } else if(kvstore != nullptr && bd != nullptr) {
        res = kvstore->deinit() == KVSTORE_SUCCESS;

        delete kvstore;
        kvstore = nullptr;

        delete bd;
        bd = nullptr;
    }

    return res;
}

template<typename T=int>
static inline typename KVStoreInterface::res_t fromMbedErrors(int error, T res=1) {
    return error == KVSTORE_SUCCESS ? res : -error;
}

bool PortentaC33KVStore::clear() {
    return kvstore != nullptr ? kvstore->reset() : false;
}

typename KVStoreInterface::res_t PortentaC33KVStore::remove(const key_t& key) {
    return kvstore != nullptr ? fromMbedErrors(kvstore->remove(key)) : -1;
}

typename KVStoreInterface::res_t PortentaC33KVStore::putBytes(const key_t& key, const uint8_t buf[], size_t len) {
    return kvstore != nullptr ? fromMbedErrors(kvstore->set(key, buf, len, 0), len) : -1; // TODO flags
}

typename KVStoreInterface::res_t PortentaC33KVStore::getBytes(const key_t& key, uint8_t buf[], size_t maxLen) const {
    if(kvstore == nullptr) {
        return -1;
    }

    size_t actual_size = maxLen;
    auto res = kvstore->get(key, buf, maxLen, &actual_size);

    return fromMbedErrors(res, actual_size);
}

size_t PortentaC33KVStore::getBytesLength(const key_t& key) const {
    if(kvstore == nullptr) {
        return 0;
    }

    mbed::KVStore::info_t info;
    auto res = kvstore->get_info(key, &info);

    return res == KVSTORE_SUCCESS ? info.size : 0;
}

bool PortentaC33KVStore::exists(const key_t& key) const {
    return getBytesLength(key) > 0;
}
#endif // defined(ARDUINO_PORTENTA_C33)
