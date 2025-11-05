/*
 * This file is part of Arduino_KVStore.
 *
 * Copyright (c) 2024 Arduino SA
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#if defined(ARDUINO_UNOR4_WIFI)
#include "UnoR4.h"

using namespace std;

bool Unor4KVStore::begin(const char* name, bool readOnly, const char* partitionLabel) {
    this->name = name;
    string res = "";

    modem.begin();
    if (this->name != nullptr && strlen(this->name) > 0) {
        if (modem.write(string(PROMPT(_PREF_BEGIN)), res, "%s%s,%d,%s\r\n", CMD_WRITE(_PREF_BEGIN), name, readOnly, partitionLabel != NULL ? partitionLabel : "")) {
            return (atoi(res.c_str()) != 0) ? true : false;
        }
    }
    return false;
}

bool Unor4KVStore::begin() {
    return begin(DEFAULT_KVSTORE_NAME);
}

bool Unor4KVStore::end() {
    string res = "";
    modem.write(string(PROMPT(_PREF_END)), res, "%s", CMD(_PREF_END));
}

bool Unor4KVStore::clear() {
    string res = "";
    if (modem.write(string(PROMPT(_PREF_CLEAR)), res, "%s", CMD(_PREF_CLEAR))) {
        return (atoi(res.c_str()) != 0) ? true : false;
    }
    return false;
}

typename KVStoreInterface::res_t Unor4KVStore::remove(const key_t& key) {
    string res = "";
    if (key != nullptr && strlen(key) > 0) {
        if (modem.write(string(PROMPT(_PREF_REMOVE)), res, "%s%s\r\n", CMD_WRITE(_PREF_REMOVE), key)) {
            return (atoi(res.c_str()) != 0) ? true : false;
        }
    }
    return false;
}

typename KVStoreInterface::res_t Unor4KVStore::putBytes(const key_t& key, const uint8_t value[], size_t len) {
    string res = "";
    if ( key != nullptr && strlen(key) > 0 && value != nullptr && len > 0) {
        modem.write_nowait(string(PROMPT(_PREF_PUT)), res, "%s%s,%d,%d\r\n", CMD_WRITE(_PREF_PUT), key, PT_BLOB, len);
        if(modem.passthrough((uint8_t *)value, len)) {
            return len;
        }
    }
    return 0;
}

typename KVStoreInterface::res_t Unor4KVStore::getBytes(const key_t& key, uint8_t buf[], size_t maxLen) const {
    size_t len = getBytesLength(key);
    string res = "";
    if (key != nullptr && strlen(key) > 0 && buf != nullptr && len > 0) {
        modem.read_using_size();
        if (modem.write(string(PROMPT(_PREF_GET)), res, "%s%s,%d\r\n", CMD_WRITE(_PREF_GET), key, PT_BLOB)) {
            if (res.size() >= len && len <= maxLen) {
                memcpy(buf, (uint8_t*)&res[0], len);
                return len;
            }
        }
    }
    return 0;
}

size_t Unor4KVStore::getBytesLength(const key_t& key) const {
    string res = "";
    if (key != nullptr && strlen(key) > 0) {
        if (modem.write(string(PROMPT(_PREF_LEN)), res, "%s%s\r\n", CMD_WRITE(_PREF_LEN), key)) {
            return atoi(res.c_str());
        }
    }
    return 0;
}


bool Unor4KVStore::exists(const key_t& key) const {
    string res = "";
    if (key != nullptr && strlen(key) > 0) {
        if (modem.write(string(PROMPT(_PREF_TYPE)), res, "%s%s\r\n", CMD_WRITE(_PREF_TYPE), key)) {
            return static_cast<Type>(atoi(res.c_str())) != PT_INVALID;
        }
    }
    return false;
}

typename KVStoreInterface::res_t Unor4KVStore::_put(
    const key_t& key, const uint8_t value[], size_t len, KVStoreInterface::Type t) {

    if (key == nullptr || strlen(key) == 0) {
        return 0;
    }
    string res = "";
    string format = "%s%s,%d,%hu\r\n";

    switch(t) {
    case PT_I8:     format = "%s%s,%d,%hd\r\n"; break;
    case PT_U8:     format = "%s%s,%d,%hu\r\n"; break;
    case PT_I16:    format = "%s%s,%d,%hd\r\n"; break;
    case PT_U16:    format = "%s%s,%d,%hu\r\n"; break;
    case PT_I32:    format = "%s%s,%d,%d\r\n";  break;
    case PT_U32:    format = "%s%s,%d,%u\r\n";  break;
    }

    uint32_t tmp = 0;
    if( t == PT_I8 || t == PT_U8 || t == PT_I16 || t == PT_U16 ||
        t == PT_I32 || t == PT_U32) {
        memcpy(&tmp, value, len);
    }

    switch(t) {
    case PT_I8:
    case PT_U8:
    case PT_I16:
    case PT_U16:
    case PT_I32:
    case PT_U32:
        // sprintf doesn't support 64 bits on unor4
        if (modem.write(string(PROMPT(_PREF_PUT)), res, format.c_str(), CMD_WRITE(_PREF_PUT), key, t, tmp)) {
            return atoi(res.c_str());
        }
        break;
    case PT_I64:
    case PT_U64:
    case PT_FLOAT:
    case PT_DOUBLE:
        return putBytes(key, value, len);
    case PT_STR:
        modem.write_nowait(string(PROMPT(_PREF_PUT)), res, "%s%s,%d,%d\r\n", CMD_WRITE(_PREF_PUT), key, t, len);
        if(modem.passthrough(value, len)) {
            return len;
        }
        break;
    case PT_BLOB:
        return putBytes(key, value, len);
        break;
    case PT_INVALID:
    default:
        break;
    }

    return 0;
}

typename KVStoreInterface::res_t Unor4KVStore::_get(const key_t& key, uint8_t value[], size_t len, Type t) {

    if (key == nullptr || strlen(key) == 0) {
        return 0;
    }
    string res = "";
    string format = "%s%s,%d,%u\r\n";

    switch(t) {
    case PT_I8:     format = "%hd"; break;
    case PT_U8:     format = "%hu"; break;
    case PT_I16:    format = "%hd"; break;
    case PT_U16:    format = "%hu"; break;
    case PT_I32:    format = "%d";  break;
    case PT_U32:    format = "%u";  break;
    }

    switch(t) {
    case PT_I8:
    case PT_U8:
    case PT_I16:
    case PT_U16:
    case PT_I32:
    case PT_U32:
        if (modem.write(string(PROMPT(_PREF_GET)), res, "%s%s,%d,%u\r\n", CMD_WRITE(_PREF_GET), key, t)) {
            sscanf(res.c_str(), format.c_str(), value);

            return len;
        }
        break;
    case PT_I64:
    case PT_U64:
    case PT_FLOAT:
    case PT_DOUBLE:
        return getBytes(key, value, len);
    case PT_STR:
        return getString(key, (char*)value, len);
        break;
    case PT_BLOB:
        return getBytes(key, value, len);
        break;
    case PT_INVALID:
    default:
        break;
    }
    return 0;
}

size_t Unor4KVStore::getString(const key_t& key, char value[], size_t maxLen) {
    string res;
    if (key != nullptr && strlen(key) > 0) {
        modem.read_using_size();
        if (modem.write(string(PROMPT(_PREF_GET)), res, "%s%s,%d,%s\r\n", CMD_WRITE(_PREF_GET), key, PT_STR, res)) {
            res.push_back('\0');

            if(res.length() > maxLen-1) {
                return 0;
            }

            strcpy(value, res.c_str());
            return res.length();
        }
    }
    return 0;
}

String Unor4KVStore::getString(const key_t& key, const String defaultValue) {
    string res = defaultValue.c_str();;
    if (key != nullptr && strlen(key) > 0) {
        modem.read_using_size();
        if (modem.write(string(PROMPT(_PREF_GET)), res, "%s%s,%d,%s\r\n", CMD_WRITE(_PREF_GET), key, PT_STR, defaultValue.c_str())) {

            return String(res.c_str());
        }
    }
    return String(res.c_str());
}

#endif // defined(ARDUINO_UNOR4_WIFI)
