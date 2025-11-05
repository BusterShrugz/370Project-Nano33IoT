/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#pragma once

#include <vector>
#include "../interfaces/MessageDecoder.h"
#include "CBOR.h"
#include "../interfaces/message.h"
#include "./tinycbor/cbor-lib.h"

// forward declaration
class CBORMessageDecoderSingleton;

/*
 * This library collects the interfaces used to decode Messages. Messages are C structs
 * that are identified by a uint32 id after which they can contain anything.
 * The objective of this library is to be modular and extensible in other libraries.
 *
 * In order to do so one have to extend the class `CBORMessageDecoderInterface`,
 * provide the associated CBORTag and MessageId and provide a way to decode the message
 * specific data. MessageId must be univocal across all DecoderInterfaces instantiated.
 * The class implemented must be instantiated somewhere (using extern may be helpful)
 * and that is enough to have it included in the decode process.
 *
 * In order to decode a message one can instantiate `CBORMessageDecoder` and
 * call the decode function.
 */

/**
 * CBORMessageDecoderInterface class is an abstract class one has to implement
 * in order to provide the instructions needed to decode a buffer contasining
 * a message specific data.
 */
class CBORMessageDecoderInterface {
public:

  /**
   * Constructor that initializes the CBORMessageDecoderInterface by providing the associated
   * CBORTag and MessageId. The constructor also appends the object into CBORMessageDecoderSingleton
   * allowing it to be used in CBORMessageDecoderSingleton::decode
   *
   * @param tag the cbor tag the message is associated with
   * @param id  the message id the message is associated with
   */
  CBORMessageDecoderInterface(const CBORTag tag, const MessageId id);
  virtual ~CBORMessageDecoderInterface() {}

protected:

  /**
   * Abstract decode function one must implement to decode the Message meaningful
   * information present into the provided buffer
   *
   * @param iter tinycbor iterator to the buffer provided to CBORMessageDecoderSingleton::decode
   * @param msg  The message to which data must be applied to. Casting may be needed
   */
  virtual MessageDecoder::Status decode(CborValue* iter, Message *msg) = 0;

private:
  const CBORTag tag;
  const MessageId id;

  friend CBORMessageDecoderSingleton;

  /**
   * Decode wrapper function that opens the cbor array and calls the abstract decode function
   *
   * @param iter tinycbor iterator to the buffer provided to CBORMessageDecoderSingleton::decode
   * @param msg  The message to which data must be applied to. Casting may be needed
   */
  MessageDecoder::Status _decode(CborValue* iter, Message *msg);
};

/**
 * This class is a singleton. It collects CBORMessageDecoderInterfaces implementations to decode
 * all possible CBOR encoded messages
 */
class CBORMessageDecoderSingleton: public MessageDecoder {
public:
  /**
   * Get the singleton instance
   */
  static CBORMessageDecoderSingleton& getInstance();

  /**
   * Add a new decoder to the singleton instance associating it to the provided cbor tag
   *
   * @param tag     the tag to which we associate the decoder
   * @param decoder the instance of decoder to append
   */
  void append(CBORTag tag, CBORMessageDecoderInterface* decoder);

  /**
   * Decode a buffer and put the contents into a message. The message should be big enough
   * to accommodate the content of the buffer.
   *
   * @param[out] msg    A struct that embeds struct Message onto which the content of buf is copied
   * @param[in] buf     The input buffer containing a cbor encoded message
   * @param[in out] len The length of the buffer
   */
  MessageDecoder::Status decode(Message* msg, const uint8_t* const buf, size_t &len);
private:
  CBORMessageDecoderSingleton() {}

  std::vector<std::pair<CBORTag, CBORMessageDecoderInterface*>> decoders;
};

/**
 * In order to decode a message one can instantiate `CBORMessageDecoder` and
 * call the decode function. In the future this class will contain decode session related
 * information, in order to allow streaming decoding
 */
class CBORMessageDecoder: public MessageDecoder {
public:
  /**
   * Decode a buffer and put the contents into a message. The message should be big enough
   * to accommodate the content of the buffer.
   *
   * @param[out] msg    A struct that embeds struct Message onto which the content of buf is copied
   * @param[in] buf     The input buffer containing a cbor encoded message
   * @param[in out] len The length of the buffer
   */
  inline MessageDecoder::Status decode(Message* msg, const uint8_t* const buf, size_t &len) {
    return CBORMessageDecoderSingleton::getInstance().decode(msg, buf, len);
  }
};
