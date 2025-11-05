/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#pragma once

typedef uint64_t CBORTag;

namespace cbor {
  namespace tag {
    enum : CBORTag {
      // Unknown Command Tag https://www.iana.org/assignments/cbor-tags/cbor-tags.xhtml
      CBORUnknownCmdTag16b    = 0xffff,              // invalid tag
      CBORUnknownCmdTag32b    = 0xffffffff,          // invalid tag
      CBORUnknownCmdTag64b    = 0xffffffffffffffff,  // invalid tag
      CBORUnknownCmdTag       = CBORUnknownCmdTag32b
    };
  }
}
