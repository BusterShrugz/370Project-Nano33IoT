/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#include "MessageEncoder.h"
#include <algorithm>

MessageEncoder::Status CBORMessageEncoderSingleton::encode(Message* message, uint8_t * data, size_t& len) {
  // prepare cbor structure
  CborEncoder encoder;

  cbor_encoder_init(&encoder, data, len, 0);

  auto encoder_it = encoders.begin();

  for(; encoder_it != encoders.end(); encoder_it++) {
    if(encoder_it->first == message->id) {
      break;
    }
  }

  // check if message.id exists on the encoders list or return error
  if(encoder_it == encoders.end()) {
    return MessageEncoder::Status::Error;
  }

  // encode the message
  if(encoder_it->second->_encode(&encoder, message) == MessageEncoder::Status::Error) {
    return MessageEncoder::Status::Error;
  }

  len = cbor_encoder_get_buffer_size(&encoder, data);

  return MessageEncoder::Status::Complete;
}

CBORMessageEncoderSingleton& CBORMessageEncoderSingleton::getInstance() {
  static CBORMessageEncoderSingleton singleton;

  return singleton;
}

void CBORMessageEncoderSingleton::append(MessageId id, CBORMessageEncoderInterface* encoder) {
  auto encoder_it = encoders.begin();

  for(; encoder_it != encoders.end(); encoder_it++) {
    if(encoder_it->first == id) {
      return;
    }
  }

  encoders.push_back(
    std::make_pair(id, encoder)
  );
}

CBORMessageEncoderInterface::CBORMessageEncoderInterface(const CBORTag tag, const MessageId id)
: tag(tag), id(id) {
  // call singleton/global variable and insert this encoder
  CBORMessageEncoderSingleton::getInstance().append(id, this);
}

MessageEncoder::Status CBORMessageEncoderInterface::_encode(CborEncoder* encoder, Message *msg) {
  // this must always be true, it could mean that there are issues in the map of encoders
  assert(msg->id == id);

  if (tag == cbor::tag::CBORUnknownCmdTag16b ||
      tag == cbor::tag::CBORUnknownCmdTag32b ||
      tag == cbor::tag::CBORUnknownCmdTag64b ||
      cbor_encode_tag(encoder, tag) != CborNoError) {
    return MessageEncoder::Status::Error;
  }

  return this->encode(encoder, msg);
}
