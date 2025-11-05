/*
 * This file is part of Arduino_KVStore.
 *
 * Copyright (c) 2024 Arduino SA
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#if defined(ARDUINO_ARCH_ESP32)
#include "ESP32.h"
#include "nvs.h"
#include "nvs_flash.h"

using namespace std;

bool ESP32KVStore::begin(const char* name, bool readOnly, const char* partition_label) {
    if(_started){
        return false;
    }
    _readOnly = readOnly;
    esp_err_t err = ESP_OK;
    if (partition_label != NULL) {
        err = nvs_flash_init_partition(partition_label);
        if (err) {
            log_e("nvs_flash_init_partition failed: %s", nvs_error(err));
            return false;
        }
        err = nvs_open_from_partition(partition_label, name, readOnly ? NVS_READONLY : NVS_READWRITE, &_handle);
    } else {
        err = nvs_open(name, readOnly ? NVS_READONLY : NVS_READWRITE, &_handle);
    }
    if(err){
        log_e("nvs_open failed: %s", nvs_error(err));
        return false;
    }
    _started = true;
    return true;
}

bool ESP32KVStore::begin() {
    return begin(DEFAULT_KVSTORE_NAME);
}

bool ESP32KVStore::end() {
    if(!_started){
        return false;
    }
    nvs_close(_handle);
    _started = false;

    return true;
}

bool ESP32KVStore::clear() {
    if(!_started || _readOnly){
        return false;
    }
    esp_err_t err = nvs_erase_all(_handle);
    if(err){
        log_e("nvs_erase_all fail: %s", nvs_error(err));
        return false;
    }
    err = nvs_commit(_handle);
    if(err){
        log_e("nvs_commit fail: %s", nvs_error(err));
        return false;
    }
    return true;
}

typename KVStoreInterface::res_t ESP32KVStore::remove(const key_t& key) {
    if(!_started || !key || _readOnly){
        return false;
    }
    esp_err_t err = nvs_erase_key(_handle, key);
    if(err){
        log_e("nvs_erase_key fail: %s %s", key, nvs_error(err));
        return false;
    }
    err = nvs_commit(_handle);
    if(err){
        log_e("nvs_commit fail: %s %s", key, nvs_error(err));
        return false;
    }
    return true;
}

typename KVStoreInterface::res_t ESP32KVStore::putBytes(const key_t& key, const uint8_t value[], size_t len) {
    if(!_started || !key || !value || !len || _readOnly){
        return 0;
    }
    esp_err_t err = nvs_set_blob(_handle, key, value, len);
    if(err){
        log_e("nvs_set_blob fail: %s %s", key, nvs_error(err));
        return 0;
    }
    err = nvs_commit(_handle);
    if(err){
        log_e("nvs_commit fail: %s %s", key, nvs_error(err));
        return 0;
    }
    return len;
}

typename KVStoreInterface::res_t ESP32KVStore::getBytes(const key_t& key, uint8_t buf[], size_t maxLen) const {
    size_t len = getBytesLength(key);
    if(!len || !buf || !maxLen){
        return len;
    }
    if(len > maxLen){
        log_e("not enough space in buffer: %u < %u", maxLen, len);
        return 0;
    }
    esp_err_t err = nvs_get_blob(_handle, key, buf, &len);
    if(err){
        log_e("nvs_get_blob fail: %s %s", key, nvs_error(err));
        return 0;
    }
    return len;
}

size_t ESP32KVStore::getBytesLength(const key_t& key) const {
    if(!_started || !key){
        return 0;
    }
    esp_err_t err = ESP_OK;

    int8_t mt1; uint8_t mt2; int16_t mt3; uint16_t mt4;
    int32_t mt5; uint32_t mt6; int64_t mt7; uint64_t mt8;
    size_t len = 0;
    if       ((err = nvs_get_i8(_handle, key, &mt1)) == ESP_OK)  { len = sizeof(mt1);
    } else if((err = nvs_get_u8(_handle, key, &mt2)) == ESP_OK)  { len = sizeof(mt2);
    } else if((err = nvs_get_i16(_handle, key, &mt3)) == ESP_OK) { len = sizeof(mt3);
    } else if((err = nvs_get_u16(_handle, key, &mt4)) == ESP_OK) { len = sizeof(mt4);
    } else if((err = nvs_get_i32(_handle, key, &mt5)) == ESP_OK) { len = sizeof(mt5);
    } else if((err = nvs_get_u32(_handle, key, &mt6)) == ESP_OK) { len = sizeof(mt6);
    } else if((err = nvs_get_i64(_handle, key, &mt7)) == ESP_OK) { len = sizeof(mt7);
    } else if((err = nvs_get_u64(_handle, key, &mt8)) == ESP_OK) { len = sizeof(mt8);
    } else if((err = nvs_get_str(_handle, key, NULL, &len)) == ESP_OK) {
    } else if((err = nvs_get_blob(_handle, key, NULL, &len)) == ESP_OK) {}

    if(err){
        log_e("nvs_get_blob len fail: %s %s", key, nvs_error(err));
        return 0;
    }
    return len;
}


bool ESP32KVStore::exists(const key_t& key) const {
    return getType(key) != PT_INVALID;
}

ESP32KVStore::Type ESP32KVStore::getType(const key_t& key) const {
    if(!_started || !key || strlen(key)>15){
        return PT_INVALID;
    }
    int8_t mt1; uint8_t mt2; int16_t mt3; uint16_t mt4;
    int32_t mt5; uint32_t mt6; int64_t mt7; uint64_t mt8;
    size_t len = 0;
    if(nvs_get_i8(_handle, key, &mt1) == ESP_OK) return PT_I8;
    if(nvs_get_u8(_handle, key, &mt2) == ESP_OK) return PT_U8;
    if(nvs_get_i16(_handle, key, &mt3) == ESP_OK) return PT_I16;
    if(nvs_get_u16(_handle, key, &mt4) == ESP_OK) return PT_U16;
    if(nvs_get_i32(_handle, key, &mt5) == ESP_OK) return PT_I32;
    if(nvs_get_u32(_handle, key, &mt6) == ESP_OK) return PT_U32;
    if(nvs_get_i64(_handle, key, &mt7) == ESP_OK) return PT_I64;
    if(nvs_get_u64(_handle, key, &mt8) == ESP_OK) return PT_U64;
    if(nvs_get_str(_handle, key, NULL, &len) == ESP_OK) return PT_STR;
    if(nvs_get_blob(_handle, key, NULL, &len) == ESP_OK) return PT_BLOB;
    return PT_INVALID;
}



typename KVStoreInterface::res_t ESP32KVStore::_put(
    const key_t& key, const uint8_t value[], size_t len, KVStoreInterface::Type t) {
    if(!_started || !key || _readOnly){
        return 0;
    }

    esp_err_t err;
    switch(t) {
    case PT_I8:
        err = nvs_set_i8(_handle, key, *((int8_t*) value));
        break;
    case PT_U8:
        err = nvs_set_u8(_handle, key, *((uint8_t*) value));
        break;
    case PT_I16:
        err = nvs_set_i16(_handle, key, *((int16_t*) value));
        break;
    case PT_U16:
        err = nvs_set_u16(_handle, key, *((uint16_t*) value));
        break;
    case PT_I32:
        err = nvs_set_i32(_handle, key, *((int32_t*) value));
        break;
    case PT_U32:
        err = nvs_set_u32(_handle, key, *((uint32_t*) value));
        break;
    case PT_I64:
        err = nvs_set_i64(_handle, key, *((int64_t*) value));
        break;
    case PT_U64:
        err = nvs_set_u64(_handle, key, *((uint64_t*) value));
        break;
    case PT_STR:
        err = nvs_set_str(_handle, key, (char*)value);
        break;
    case PT_BLOB:
        err = nvs_set_blob(_handle, key, value, len);
        break;
    case PT_FLOAT:
        err = nvs_set_u32(_handle, key, *((uint32_t*) value));
        break;
    case PT_DOUBLE:
        err = nvs_set_u64(_handle, key, *((uint64_t*) value));
        break;
    case PT_INVALID:
    default:
        log_e("nvs_set fail: invalid type");
        return 0;
    }

    if(err){
        log_e("nvs_set_ fail: %s %s", key, nvs_error(err)); // TODO put type
        return 0;
    }

    err = nvs_commit(_handle);
    if(err){
        log_e("nvs_commit fail: %s %s", key, nvs_error(err));
        return 0;
    }

    return len;
}

typename KVStoreInterface::res_t ESP32KVStore::_get(const key_t& key, uint8_t value[], size_t len, Type t) {
    if(!_started || !key){
        return 0;
    }

    esp_err_t err;
    switch(t) {
    case PT_I8:
        err = nvs_get_i8(_handle, key, (int8_t*) value);
        break;
    case PT_U8:
        err = nvs_get_u8(_handle, key, (uint8_t*) value);
        break;
    case PT_I16:
        err = nvs_get_i16(_handle, key, (int16_t*) value);
        break;
    case PT_U16:
        err = nvs_get_u16(_handle, key, (uint16_t*) value);
        break;
    case PT_I32:
        err = nvs_get_i32(_handle, key, (int32_t*) value);
        break;
    case PT_U32:
        err = nvs_get_u32(_handle, key, (uint32_t*) value);
        break;
    case PT_I64:
        err = nvs_get_i64(_handle, key, (int64_t*) value);
        break;
    case PT_U64:
        err = nvs_get_u64(_handle, key, (uint64_t*) value);
        break;
    case PT_STR:
        err = nvs_get_str(_handle, key, (char*)value, &len);
        break;
    case PT_BLOB:
        err = nvs_get_blob(_handle, key, value, &len);
        break;
    case PT_FLOAT:
        err = nvs_get_u32(_handle, key, (uint32_t*) value);
        break;
    case PT_DOUBLE:
        err = nvs_get_u64(_handle, key, (uint64_t*) value);
        break;
    case PT_INVALID:
    default:
        log_e("nvs_get fail: invalid type");
        break;
    }

    if(err){
        log_e("nvs_get_ fail: %s %s", key, nvs_error(err)); // TODO put type
        return 0;
    }

    return len;
}

#endif // defined(ARDUINO_ARCH_ESP32)
