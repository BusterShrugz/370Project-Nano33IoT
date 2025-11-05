/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#pragma once
#include "../tinycbor/cbor-lib.h"

namespace cbor { namespace utils {

  inline MessageDecoder::Status copyCBORStringToArray(CborValue * param, char * dest, size_t& dest_size) {
    if (!cbor_value_is_text_string(param)) {
      return MessageDecoder::Status::Error;
    }

    // NOTE: keep in mind that _cbor_value_copy_string tries to put a \0 at the end of the string
    if(_cbor_value_copy_string(param, dest, &dest_size, NULL) != CborNoError) {
      return MessageDecoder::Status::Error;
    }

    return MessageDecoder::Status::InProgress;
  }

  inline MessageDecoder::Status copyCBORByteToArray(CborValue * param, uint8_t * dest, size_t& dest_size) {
    if (!cbor_value_is_byte_string(param)) {
      return MessageDecoder::Status::Error;
    }

    // NOTE: keep in mind that _cbor_value_copy_string tries to put a \0 at the end of the string
    if(_cbor_value_copy_string(param, dest, &dest_size, NULL) != CborNoError) {
      return MessageDecoder::Status::Error;
    }

    return MessageDecoder::Status::InProgress;
  }
}}
