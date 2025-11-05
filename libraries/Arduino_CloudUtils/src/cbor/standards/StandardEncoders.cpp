/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2025 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "StandardEncoders.h"

MessageEncoder::Status VersionMessageEncoder::encode(CborEncoder* encoder, Message *msg) {
  VersionMessage * versionMsg = (VersionMessage*) msg;
  CborEncoder array_encoder;

  if(cbor_encoder_create_array(encoder, &array_encoder, 1) != CborNoError) {
      return MessageEncoder::Status::Error;
  }

  if(cbor_encode_text_stringz(&array_encoder, versionMsg->params.version) != CborNoError) {
      return MessageEncoder::Status::Error;
  }

  if(cbor_encoder_close_container(encoder, &array_encoder) != CborNoError) {
      return MessageEncoder::Status::Error;
  }

  return MessageEncoder::Status::Complete;
}

static VersionMessageEncoder wifiFWVersionMessageEncoderCbor(CBORWiFiFWVersionMessage, WiFiFWVersionMessageId);
