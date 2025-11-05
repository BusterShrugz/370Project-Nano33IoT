/*
 * This file is part of Arduino_KVStore.
 *
 * The purpose of this file is to define templated functions to test KVStore
 *
 * Copyright (c) 2024 Arduino SA
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <Arduino_KVStore.h>
#include <functional>


const char KEY[] = "arduino";

template<class T>
bool test_kvstore(T value,
    std::function<size_t(const char* const, T)> putf,
    std::function<T(const char* const)> getf,
    KVStore *kvstore) {
  // we are going now to test all the apis of kvstore:
  // .1 We check that the key do not exist
  // .2 we put a value inside the KVStore
  // .3 we check that the size returned is correct
  // .4 we check again the key exists
  // .5 we get the value contained
  // .6 we compare the value with what we inserted at the beginning
  // .7 we remove the value

  Serial.println("Testing exists()");
  if(kvstore->exists(KEY)) {
    Serial.println("[Error] kvstore already contains a key");
    return false;
  }

  Serial.println("Testing put()");
  size_t s;
  if((s=putf(KEY, value)) != sizeof(value)) {
    Serial.println("[Error] kvstore put returned a size that is different from the expected one: ");
    Serial.print(s, HEX);
    Serial.print(" != ");
    Serial.println(sizeof(value), HEX);
    return false;
  }

  Serial.println("Testing exists()");
  if(!kvstore->exists(KEY)) {
    Serial.println("[Error] The inserted key is not present in the KVStore");
    return false;
  }

  Serial.println("Testing getBytesLength()");
  if((s=kvstore->getBytesLength(KEY)) != sizeof(value)) {
    Serial.println("[Error] The length of the value do not match with the expected one");
    Serial.print(s);
    Serial.print(" != ");
    Serial.println(sizeof(value));
    return false;
  }

  Serial.println("Testing getf()");
  T val;
  if((val = getf(KEY)) != value) {
    Serial.print("[Error] get of the previously inserted value returned a wrong value: ");
    Serial.print(val, HEX);
    Serial.print(" != ");
    Serial.println(value, HEX);
    return false;
  }

  Serial.println("Testing remove()");
  if(!kvstore->remove(KEY)) {
    Serial.println("[Error] Failed removing the inserted key");
    return false;
  }

  return true;
}

template<>
bool test_kvstore(String value,
  std::function<size_t(const char* const, String)> putf,
  std::function<String(const char* const)> getf,
  KVStore *kvstore) {
// we are going now to test all the apis of kvstore:
// .1 We check that the key do not exist
// .2 we put a value inside the KVStore
// .3 we check that the size returned is correct
// .4 we check again the key exists
// .5 we get the value contained
// .6 we compare the value with what we inserted at the beginning
// .7 we remove the value

Serial.println("Testing exists()");
if(kvstore->exists(KEY)) {
  Serial.println("[Error] kvstore already contains a key");
  return false;
}

Serial.println("Testing put()");
size_t s;
if((s=putf(KEY, value)) != value.length()) {
  Serial.println("[Error] kvstore put returned a size that is different from the expected one: ");
  Serial.print(s);
  Serial.print(" != ");
  Serial.println(value.length());
  return false;
}

Serial.println("Testing exists()");
if(!kvstore->exists(KEY)) {
  Serial.println("[Error] The inserted key is not present in the KVStore");
  return false;
}

Serial.println("Testing getBytesLength()");
if((s=kvstore->getBytesLength(KEY)) != value.length()) {
  Serial.println("[Error] The length of the value do not match with the expected one");
  Serial.print(s);
  Serial.print(" != ");
  Serial.println(value.length());
  return false;
}

Serial.println("Testing getf()");
String val;
if((val = getf(KEY)) != value) {
  Serial.print("[Error] get of the previously inserted value returned a wrong value: ");
  Serial.print(val);
  Serial.print(" != ");
  Serial.println(value);
  return false;
}

Serial.println("Testing remove()");
if(!kvstore->remove(KEY)) {
  Serial.println("[Error] Failed removing the inserted key");
  return false;
}

return true;
}

bool test_kvstore(const char* value, KVStore *kvstore) {
  // we are going now to test all the apis of kvstore:
  // .1 We check that the key do not exist
  // .2 we put a value inside the KVStore
  // .3 we check that the size returned is correct
  // .4 we check again the key exists
  // .5 we get the value contained
  // .6 we compare the value with what we inserted at the beginning
  // .7 we remove the value

  Serial.println("Testing exists()");
  if(kvstore->exists(KEY)) {
    Serial.println("[Error] kvstore already contains a key");
    return false;
  }

  Serial.println("Testing put()");
  size_t s;
  if((s=kvstore->putString(KEY, value)) != strlen(value)) {
    Serial.println("[Error] kvstore put returned a size that is different from the expected one: ");
    Serial.print(s);
    Serial.print(" != ");
    Serial.println(strlen(value));
    return false;
  }

  Serial.println("Testing exists()");
  if(!kvstore->exists(KEY)) {
    Serial.println("[Error] The inserted key is not present in the KVStore");
    return false;
  }

  Serial.println("Testing getf()");
  char val[500];
  kvstore->getString(KEY, val, sizeof(val));
  if(strcmp(val, value) != 0) {
    Serial.print("[Error] get of the previously inserted value returned a wrong value: \"");
    Serial.print(val);
    Serial.print("\" != \"");
    Serial.print(value);
    Serial.println("\"");

    for(int i=0; i<strlen(val); i++) {
      Serial.print(i);
      Serial.print(": ");
      Serial.print((uint8_t)val[i]);
      Serial.print(val[i]!=value[i]? "\" != \"" : "\" == \"");
      Serial.print((uint8_t)value[i]);
      Serial.println("\"");
    }

    return false;
  }

  Serial.println("Testing remove()");
  if(!kvstore->remove(KEY)) {
    Serial.println("[Error] Failed removing the inserted key");
    return false;
  }

  return true;
}

bool test_kvstore(uint8_t* value, size_t size, KVStore *kvstore) {
  // we are going now to test all the apis of kvstore:
  // .1 We check that the key do not exist
  // .2 we put a value inside the KVStore
  // .3 we check that the size returned is correct
  // .4 we check again the key exists
  // .5 we get the value contained
  // .6 we compare the value with what we inserted at the beginning
  // .7 we remove the value

  Serial.println("Testing exists()");
  if(kvstore->exists(KEY)) {
    Serial.println("[Error] kvstore already contains a key");
    return false;
  }

  Serial.println("Testing put()");
  size_t s;
  if((s=kvstore->putBytes(KEY, value, size)) != size) {
    Serial.println("[Error] kvstore put returned a size that is different from the expected one: ");
    Serial.print(s);
    Serial.print(" != ");
    Serial.println(size);
    return false;
  }

  Serial.println("Testing exists()");
  if(!kvstore->exists(KEY)) {
    Serial.println("[Error] The inserted key is not present in the KVStore");
    return false;
  }

  Serial.println("Testing getBytesLength()");
  if((s=kvstore->getBytesLength(KEY)) != size) {
    Serial.println("[Error] The length of the value do not match with the expected one");
    Serial.print(s);
    Serial.print(" != ");
    Serial.println(size+1);
    return false;
  }

  Serial.println("Testing getf()");
  uint8_t val[500];
  s = kvstore->getBytes(KEY, val, sizeof(val));

  if(s != size ) {
    Serial.print("[Error] get of the previously inserted value returned a wrong size: ");
    Serial.print(s);
    Serial.print(" != ");
    Serial.println(size );
    return false;
  }

  if(memcmp(val, value, s) != 0) {
    Serial.println("[Error] get of the previously inserted value returned a wrong value");
    return false;
  }

  Serial.println("Testing remove()");
  if(!kvstore->remove(KEY)) {
    Serial.println("[Error] Failed removing the inserted key");
    return false;
  }

  return true;
}
