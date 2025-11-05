/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2025 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "StandardMessages.h"

class VersionMessageEncoder: public CBORMessageEncoderInterface {
public:
  VersionMessageEncoder(CBORStandardMessageTag tag, StandardMessageId id)
  : CBORMessageEncoderInterface(tag, id) {}
protected:
  MessageEncoder::Status encode(CborEncoder* encoder, Message *msg) override;
};
