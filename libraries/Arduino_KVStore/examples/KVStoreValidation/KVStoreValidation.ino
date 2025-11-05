/*
 * This file is part of Arduino_KVStore.
 *
 * The purpose of this example is to test all the apis available on KVStore
 * on every supported platform and check their conformity to the standard.
 * It is not intended to showcase the usage of the apis
 *
 * Copyright (c) 2024 Arduino SA
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test.h"

KVStore kvstore;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println();
  delay(3000);

  // Open kvstore
  if (!kvstore.begin()) {
    Serial.println("Cannot initialize kvstore");
    Serial.println("Make sure your WiFi firmware version is greater than 0.3.0");
    while(1) {};
  }

  // Remove all kvstore under the opened namespace
  Serial.print("Clearing content of kvstore: ");
  Serial.println(kvstore.clear());

  Serial.println("Testing Char operations");
  if(!test_kvstore<char>(0x50,
    std::bind(&KVStore::putChar, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getChar(key);
    }, &kvstore)) {

    Serial.println("Char test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing UChar operations");
  if(!test_kvstore<char>(0x50,
    std::bind(&KVStore::putUChar, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getUChar(key);
    }, &kvstore)) {

    Serial.println("UChar test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing Short operations");
  if(!test_kvstore<int16_t>(0x5050,
    std::bind(&KVStore::putShort, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getShort(key);
    }, &kvstore)) {

    Serial.println("Short test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing UShort operations");
  if(!test_kvstore<uint16_t>(0x5050,
    std::bind(&KVStore::putUShort, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getUShort(key);
    }, &kvstore)) {

    Serial.println("UShort test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing Int operations");
  if(!test_kvstore<int32_t>(0x50505050,
    std::bind(&KVStore::putInt, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getInt(key);
    }, &kvstore)) {

    Serial.println("Int test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing Int operations with negative value");
  if(!test_kvstore<int32_t>(-12345678,
    std::bind(&KVStore::putInt, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getInt(key);
    }, &kvstore)) {

    Serial.println("Int test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing UInt operations");
  if(!test_kvstore<uint32_t>(0x50505050,
    std::bind(&KVStore::putUInt, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getUInt(key);
    }, &kvstore)) {

    Serial.println("UInt test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing Long operations");
  if(!test_kvstore<int32_t>(0x50505050,
    std::bind(&KVStore::putLong, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getLong(key);
    }, &kvstore)) {

    Serial.println("Long test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing ULong operations");
  if(!test_kvstore<uint32_t>(0x50505050,
    std::bind(&KVStore::putULong, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getULong(key);
    }, &kvstore)) {

    Serial.println("ULong test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing Long64 operations");
  if(!test_kvstore<int64_t>(0x5050505050505050,
    std::bind(&KVStore::putLong64, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getLong64(key);
    }, &kvstore)) {

    Serial.println("Long64 test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing ULong64 operations");
  if(!test_kvstore<uint64_t>(0x5050505050505050,
    std::bind(&KVStore::putULong64, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getULong64(key);
    }, &kvstore)) {

    Serial.println("ULong64 test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing Float operations");
  if(!test_kvstore<float>(0x50505050,
    std::bind(&KVStore::putFloat, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getFloat(key);
    }, &kvstore)) {

    Serial.println("Float test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing Double operations");
  if(!test_kvstore<double>(0x5050505050505050,
    std::bind(&KVStore::putDouble, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getDouble(key);
    }, &kvstore)) {

    Serial.println("Double test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing Bool operations");
  if(!test_kvstore<bool>(true,
    std::bind(&KVStore::putBool, &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getBool(key);
    }, &kvstore)) {

    Serial.println("Bool test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing C - String operations");
  if(!test_kvstore("la mia stringsa asdasdafasdjdsnajdnaskjlasda\n\n\r\nsdafasdjdsnajdnaskjlasdasdafasdjdsnajdnaskjlasdasdafasdjdsnajdnaskjlasdasdafasdjdsnajdnaskjlasdasdafasdjdsnajdnaskjl\r\nOK\r\nERROR",
      &kvstore)) {
    Serial.println("C - String test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing Arduino - String operations");
  if(!test_kvstore<String>("la mia stringsa asdasdafasdjdsnajdnaskjlasda\n\n\r\nsdafasdjdsnajdnaskjlasdasdafasdjdsnajdnaskjlasdasdafasdjdsnajdnaskjlasdasdafasdjdsnajdnaskjlasdasdafasdjdsnajdnaskjl\r\nOK\r\nERROR",
    std::bind(static_cast<size_t(KVStore::*)(const char* const&, String)>(&KVStore::putString), &kvstore, std::placeholders::_1, std::placeholders::_2),
    [&kvstore](const char* key) { // we need a lambda, because it requires a default value from apis
      return kvstore.getString(key);
    }, &kvstore)) {

    Serial.println("Arduino - String test failed");
    kvstore.remove(KEY);
  }

  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Testing byte buffer operations");
  uint8_t buffer[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF, 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF, 0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B};
  if(!test_kvstore(buffer, sizeof(buffer), &kvstore)) {
    Serial.println("byte buffer test failed");
    kvstore.remove(KEY);
  }

  kvstore.end();
  Serial.println("\n\nTesting finished");
}

void loop() {}
