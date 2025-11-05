/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#pragma once

#include <vector>
#include "../interfaces/MessageEncoder.h"
#include "CBOR.h"
#include "../interfaces/message.h"
#include "./tinycbor/cbor-lib.h"

// forward declaration
class CBORMessageEncoderSingleton;

/*
 * This library collects the interfaces used to encode Messages. Messages are C structs
 * that are identified by a uint32 id after which they can contain anything.
 * The objective of this library is to be modular and extensible in other libraries.
 *
 * In order to do so one have to extend the class `CBORMessageEncoderInterface`,
 * provide the associated CBORTag and MessageId and provide a way to encode the message
 * specific data. MessageId must be univocal across all EncoderInterfaces instantiated.
 * The class implemented must be instantiated somewhere (using extern may be helpful)
 * and that is enough to have it included in the encode process.
 *
 * In order to encode a message one can instantiate `CBORMessageEncoder` and
 * call the encode function.
 */

/**
 * CBORMessageEncoderInterface class is an abstract class one has to implement
 * in order to provide the instructions needed to encode a buffer contasining
 * a message specific data.
 */
class CBORMessageEncoderInterface {
public:

  /**
   * Constructor that initializes the CBORMessageEncoderInterface by providing the associated
   * CBORTag and MessageId. The constructor also appends the object into CBORMessageEncoderSingleton
   * allowing it to be used in CBORMessageEncoderSingleton::encode
   *
   * @param tag the cbor tag the message is associated with
   * @param id  the message id the message is associated with
   */
  CBORMessageEncoderInterface(const CBORTag tag, const MessageId id);
  virtual ~CBORMessageEncoderInterface() {}

protected:

  /**
   * Abstract encode function one must implement to encode a Message into a provided buffer
   *
   * @param encoder tinycbor encoder struct
   * @param msg     The message from which data must be extracted
   */
  virtual MessageEncoder::Status encode(CborEncoder* encoder, Message *msg) = 0;

private:
  const CBORTag tag;
  const MessageId id;

  friend CBORMessageEncoderSingleton;

  /**
   * Encode wrapper function that encodes the cbor tag and then calls the encode virtual function
   *
   * @param encoder tinycbor encoder struct
   * @param msg     The message from which data must be extracted
   */
  MessageEncoder::Status _encode(CborEncoder* encoder, Message *msg);
};

/**
 * This class is a singleton. It collects CBORMessageEncoderInterfaces implementations to encode
 * all possible CBOR encoded messages
 */
class CBORMessageEncoderSingleton: public MessageEncoder {
public:
  /**
   * Get the singleton instance
   */
  static CBORMessageEncoderSingleton& getInstance();

  /**
   * Add a new encoder to the singleton instance associating it to the provided cbor tag
   *
   * @param tag     the tag to which we associate the encoder
   * @param encoder the instance of encoder to append
   */
  void append(MessageId id, CBORMessageEncoderInterface* encoder);

  /**
   * Encode a message and put the contents into a byte buffer. The message should be big enough
   * to accommodate the content of the buffer.
   *
   * @param[in] msg     A struct that embeds struct Message
   * @param[in out] buf The output buffer onto which the message will be encoded
   * @param[in out] len The length of the buffer
   */
  MessageEncoder::Status encode(Message* message, uint8_t * data, size_t& len);
private:
  CBORMessageEncoderSingleton() {}

  std::vector<std::pair<MessageId, CBORMessageEncoderInterface*>> encoders;
};

/**
 * In order to encode a message one can instantiate `CBORMessageEncoder` and
 * call the encode function. In the future this class will contain encode session related
 * information, in order to allow streaming decoding
 */
class CBORMessageEncoder: public MessageEncoder {
public:
  /**
   * Encode a message and put the contents into a byte buffer. The message should be big enough
   * to accommodate the content of the buffer.
   *
   * @param[in] msg     A struct that embeds struct Message
   * @param[in out] buf The output buffer onto which the message will be encoded
   * @param[in out] len The length of the buffer
   */
  inline MessageEncoder::Status encode(Message* msg, uint8_t* buf, size_t &len) {
    return CBORMessageEncoderSingleton::getInstance().encode(msg, buf, len);
  }
};
