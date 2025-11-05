/*
 * This file is part of Arduino_KVStore.
 *
 * Copyright (c) 2024 Arduino SA
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <kvstore/kvstore.h>
#include <map>
#include <cstdint>
#include <cstring>
#include <stdexcept>

class KVStore: public KVStoreInterface {
public:
    bool begin() { return true; }
    bool end()   { return true; }
    bool clear();

    typename KVStoreInterface::res_t remove(const Key& key) override;
    bool exists(const Key& key) const override;
    typename KVStoreInterface::res_t putBytes(const Key& key, const uint8_t b[], size_t s) override;
    typename KVStoreInterface::res_t getBytes(const Key& key, uint8_t b[], size_t s) const override;
    size_t getBytesLength(const Key& key) const override;

private:
    std::map<Key, std::pair<uint8_t*, size_t>> kvmap;
};

bool KVStore::clear() {
    kvmap.clear();

    return true;
}

typename KVStoreInterface::res_t KVStore::remove(const Key& key) {
    auto el = kvmap.at(key);
    kvmap.erase(key);

    delete [] el.first;

    return 1;
}

bool KVStore::exists(const Key& key) const {
    try {
        kvmap.at(key);
        return true;
    } catch(const std::out_of_range&) {
        return false;
    }
}

typename KVStoreInterface::res_t KVStore::putBytes(const Key& key, const uint8_t b[], size_t s) {
    if(exists(key)) {
        remove(key);
    }

    uint8_t* buf = new uint8_t[s];
    std::memset(buf, 0, s);
    std::memcpy(buf, b, s);

    kvmap[key] = {buf, s};

    return s;
}

typename KVStoreInterface::res_t KVStore::getBytes(const Key& key, uint8_t b[], size_t s) const {
    auto el = kvmap.at(key);

    std::memcpy(b, el.first, s <= el.second? s : el.second);

    return el.second;
}

size_t KVStore::getBytesLength(const Key& key) const {
    auto el = kvmap.at(key);

    return el.second;
}

TEST_CASE( "KVStore can store values of different types, get them and remove them", "[kvstore][putgetremove]" ) {
    KVStore store;
    store.begin();

    SECTION( "adding a char and getting it back" ) {
        char value = 'A';

        REQUIRE( store.putChar("0", value) == sizeof(value));
        REQUIRE( store.getChar("0") == value );
        REQUIRE( store.remove("0") == 1 );
    }

    SECTION( "adding a uchar and getting it back" ) {
        unsigned char value = 0x55;

        REQUIRE( store.putUChar("0", value) == sizeof(value));
        REQUIRE( store.getUChar("0") == value );
        REQUIRE( store.remove("0") == 1 );
    }

    SECTION( "adding a short and getting it back" ) {
        short value = 0x5555;

        REQUIRE( store.putShort("0", value) == sizeof(value));
        REQUIRE( store.getShort("0") == value );
        REQUIRE( store.remove("0") == 1 );
    }

    SECTION( "adding an unsigned short and getting it back" ) {
        unsigned short value = 0x5555;

        REQUIRE( store.putUShort("0", value) == sizeof(value));
        REQUIRE( store.getUShort("0") == value );
        REQUIRE( store.remove("0") == 1 );
    }

    SECTION( "adding an uint32_t and getting it back" ) {
        uint32_t value = 0x01020304;

        REQUIRE( store.putUInt("0", value) == sizeof(value));
        REQUIRE( store.getUInt("0") == value );
        REQUIRE( store.remove("0") == 1 );
    }

    SECTION( "adding a string and getting it back" ) {
        char value[] = "pippo";
        char res[6];

        REQUIRE( store.putString("0", value) == strlen(value));

        store.getString("0", res, 6);
        REQUIRE( strcmp(res, value) == 0 );
        REQUIRE( store.remove("0") == 1 );
    }
}



TEST_CASE( "KVStore references are a useful tool to indirectly access kvstore", "[kvstore][references]" ) {
    KVStore store;
    store.begin();

    REQUIRE( store.put("0", (uint8_t) 0x55) == 1);
    REQUIRE( store.put("1", (uint16_t) 0x5555) == 2);
    REQUIRE( store.put("2", (uint32_t) 0x55555555) == 4);
    REQUIRE( store.put("3", (uint32_t) 0x55555555) == 4);

    SECTION( "I can get an uint8_t reference and update its value indirectly" ) {
        auto ref = store.get<uint8_t>("0");

        REQUIRE( ref == 0x55 );
        ref = 0x56;

        REQUIRE( store.getUChar("0") == 0x56 );
    }

    SECTION( "I can get an uint16_t reference and update its value indirectly" ) {
        auto ref = store.get<uint16_t>("1");

        REQUIRE( ref == 0x5555 );
        ref = 0x5656;

        REQUIRE( store.getUShort("1") == 0x5656 );
    }

    SECTION( "I can get an uint32_t reference and update its value indirectly" ) {
        auto ref = store.get<uint32_t>("2");

        REQUIRE( ref == 0x55555555 );
        ref = 0x56565656;

        REQUIRE( store.getUInt("2") == 0x56565656 );
    }

    SECTION( "If I update the value from a reference I am able to load it from another one" ) {
        auto ref1 = store.get<uint32_t>("3");
        auto ref2 = store.get<uint32_t>("3");

        REQUIRE(ref1 == 0x55555555);
        REQUIRE(ref2 == 0x55555555);

        ref1 = 0x56565656;
        ref2.load();

        REQUIRE(ref2 == 0x56565656);
    }

    REQUIRE( store.remove("0") == 1 );
    REQUIRE( store.remove("1") == 1 );
    REQUIRE( store.remove("2") == 1 );
    REQUIRE( store.remove("3") == 1 );
}
