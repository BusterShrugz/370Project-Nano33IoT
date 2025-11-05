/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

typedef uint32_t MessageId;

/**
 * The following struct can be used as a template to create ArduinoCloud compatible
 * Messages that can be handled by encoders/decoders and other Cloud related message exchange
 * These kind of messages are required to be identifiable by a starting uint32_t uinique identification number.
 *
 * In order to use this structure you need to embed this into another structure and
 * add additional parameters
 */
struct Message {
  MessageId id;
};

/**
 * The following enum is defined to assign Arduino MessageIds starting values
 * and boundaries and avoid value clashing
 */
enum : MessageId {
  ArduinoMessageStartId             = 0x000,
  ArduinoIOTCloudStartMessageId     = 0x100,
  ArduinoProvisioningStartMessageId = 0x200,
  ArduinoVersionsStartId            = 0x300,
};
