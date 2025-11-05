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

#include <kvstore/kvstore.h>

TEST_CASE( "Testing KVStore getType utility function", "[kvstore][utility]" ) {

    REQUIRE(KVStoreInterface::getType((int8_t) 0x55)        == KVStoreInterface::PT_I8);
    REQUIRE(KVStoreInterface::getType((uint8_t) 0x55)       == KVStoreInterface::PT_U8);
    REQUIRE(KVStoreInterface::getType((int16_t) 0x55)       == KVStoreInterface::PT_I16);
    REQUIRE(KVStoreInterface::getType((uint16_t) 0x55)      == KVStoreInterface::PT_U16);
    REQUIRE(KVStoreInterface::getType((int32_t) 0x55)       == KVStoreInterface::PT_I32);
    REQUIRE(KVStoreInterface::getType((uint32_t) 0x55)      == KVStoreInterface::PT_U32);
    REQUIRE(KVStoreInterface::getType((int64_t) 0x55)       == KVStoreInterface::PT_I64);
    REQUIRE(KVStoreInterface::getType((uint64_t) 0x55)      == KVStoreInterface::PT_U64);
    REQUIRE(KVStoreInterface::getType((bool) true)          == KVStoreInterface::PT_I8);


    char string[] = "my test string";
    REQUIRE(KVStoreInterface::getType(string)               == KVStoreInterface::PT_STR);

    char* cstr = nullptr;
    REQUIRE(KVStoreInterface::getType(cstr)                 == KVStoreInterface::PT_STR);
    REQUIRE(KVStoreInterface::getType("my test string")     == KVStoreInterface::PT_STR);

    uint8_t buf[] = { 0x55, 0x55 };
    REQUIRE(KVStoreInterface::getType(buf)                  == KVStoreInterface::PT_BLOB);
    // REQUIRE(KVStoreInterface::getType()                     == KVStoreInterface::PT_INVALID);
}
