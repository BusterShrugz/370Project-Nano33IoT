/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2025 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include "../MessageEncoder.h"

enum CBORStandardMessageTag: CBORTag {

  CBORWiFiFWVersionMessage      = 0x012014 //Next tag starts at 0x013000
};

enum StandardMessageId: MessageId {
  /* Standard commands*/
  WiFiFWVersionMessageId = ArduinoVersionsStartId,
};

struct VersionMessage {
  Message c;
  struct {
    const char *version; //The payload is a string.
  } params;
};
