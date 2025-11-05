/*
 * This file is part of Arduino_KVStore.
 *
 * Copyright (c) 2024 Arduino SA
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "kvstore.h"

template<typename T>
typename KVStoreInterface::res_t KVStoreInterface::put(const key_t& key, T value) {
    return _put(key, (uint8_t*)&value, sizeof(value), getType(value));
}

template<>
typename KVStoreInterface::res_t KVStoreInterface::put<const char*>(const key_t& key, const char* value) {
    return _put(key, (uint8_t*)value, strlen(value), PT_STR);
}

#ifdef ARDUINO
template<>
typename KVStoreInterface::res_t KVStoreInterface::put<String>(const key_t& key, String value) {
    return _put(key, (uint8_t*)value.c_str(), value.length(), PT_STR);
}
#endif // ARDUINO

template<typename T> // TODO this could be called when class is const
KVStoreInterface::reference<T> KVStoreInterface::get(const key_t& key, const T def) {
    if(exists(key)) {
        T t;
        auto res = _get(key, (uint8_t*)&t, sizeof(t), getType(t));

        if(res > 0) {
            return KVStoreInterface::reference<T>(key, t, *this);
        }
    }

    return KVStoreInterface::reference<T>(key, def, *this);
}

size_t   KVStoreInterface::putChar(const key_t& key, const int8_t value)             { return put(key, value); }
size_t   KVStoreInterface::putUChar(const key_t& key, const uint8_t value)           { return put(key, value); }
size_t   KVStoreInterface::putShort(const key_t& key, const int16_t value)           { return put(key, value); }
size_t   KVStoreInterface::putUShort(const key_t& key, const uint16_t value)         { return put(key, value); }
size_t   KVStoreInterface::putInt(const key_t& key, const int32_t value)             { return put(key, value); }
size_t   KVStoreInterface::putUInt(const key_t& key, const uint32_t value)           { return put(key, value); }
size_t   KVStoreInterface::putLong(const key_t& key, const int32_t value)            { return put(key, value); }
size_t   KVStoreInterface::putULong(const key_t& key, const uint32_t value)          { return put(key, value); }
size_t   KVStoreInterface::putLong64(const key_t& key, const int64_t value)          { return put(key, value); }
size_t   KVStoreInterface::putULong64(const key_t& key, const uint64_t value)        { return put(key, value); }
size_t   KVStoreInterface::putFloat(const key_t& key, const float value)             { return put(key, value); }
size_t   KVStoreInterface::putDouble(const key_t& key, const double value)           { return put(key, value); }
size_t   KVStoreInterface::putBool(const key_t& key, const bool value)               { return put(key, value); }
size_t   KVStoreInterface::putString(const key_t& key, const char * const value)     { return put(key, value); }

#ifdef ARDUINO
size_t   KVStoreInterface::putString(const key_t& key, const String value)           { return put(key, value); }
#endif // ARDUINO


int8_t   KVStoreInterface::getChar(const key_t& key, const int8_t defaultValue)      { return get(key, defaultValue); }
uint8_t  KVStoreInterface::getUChar(const key_t& key, const uint8_t defaultValue)    { return get(key, defaultValue); }
int16_t  KVStoreInterface::getShort(const key_t& key, const int16_t defaultValue)    { return get(key, defaultValue); }
uint16_t KVStoreInterface::getUShort(const key_t& key, const uint16_t defaultValue)  { return get(key, defaultValue); }
int32_t  KVStoreInterface::getInt(const key_t& key, const int32_t defaultValue)      { return get(key, defaultValue); }
uint32_t KVStoreInterface::getUInt(const key_t& key, const uint32_t defaultValue)    { return get(key, defaultValue); }
int32_t  KVStoreInterface::getLong(const key_t& key, const int32_t defaultValue)     { return get(key, defaultValue); }
uint32_t KVStoreInterface::getULong(const key_t& key, const uint32_t defaultValue)   { return get(key, defaultValue); }
int64_t  KVStoreInterface::getLong64(const key_t& key, const int64_t defaultValue)   { return get(key, defaultValue); }
uint64_t KVStoreInterface::getULong64(const key_t& key, const uint64_t defaultValue) { return get(key, defaultValue); }
float    KVStoreInterface::getFloat(const key_t& key, const float defaultValue)      { return get(key, defaultValue); }
double   KVStoreInterface::getDouble(const key_t& key, const double defaultValue)    { return get(key, defaultValue); }
bool     KVStoreInterface::getBool(const key_t& key, const bool defaultValue)        { return get(key, defaultValue); }
size_t   KVStoreInterface::getString(const key_t& key, char* value, size_t maxLen)   { return _get(key, (uint8_t*)value, maxLen, PT_STR); }

#ifdef ARDUINO
String KVStoreInterface::getString(const key_t& key, const String defaultValue) {
    size_t len = getBytesLength(key);
    char *str = new char[len+1];

    getString(key, str, len+1);
    str[len] = '\0';

    String res(str);
    delete str;
    str = nullptr;

    return res;
}
#endif // ARDUINO

typename KVStoreInterface::res_t KVStoreInterface::_put(const key_t& key, const uint8_t value[], size_t len, Type t) {
    (void) t;

    return putBytes(key, value, len);
}

typename KVStoreInterface::res_t KVStoreInterface::_get(const key_t& key, uint8_t value[], size_t len, Type t) {
    (void) t;

    if(exists(key)) {
        res_t res=0;
        if(t == PT_STR) {
            res = getBytes(key, value, len-1);
            value[res] = '\0';
        } else {
            res = getBytes(key, value, len);
        }

        return res;
    } else {
        return 0;
    }
}
