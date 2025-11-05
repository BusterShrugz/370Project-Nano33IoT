/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "sha2.h"

namespace arduino { namespace sha256 {

    inline void begin(acu_sha256_ctx * ctx) {
        return acu_sha256_init(ctx);
    }
    inline void update(acu_sha256_ctx *ctx, const uint8_t *input, uint32_t length) {
        return acu_sha256_update(ctx, input, length);
    }
    inline void finalize(acu_sha256_ctx * ctx, uint8_t digest[SHA256_DIGEST_SIZE]) {
        return acu_sha256_final(ctx, digest);
    }
    inline void sha256(const unsigned char *input, unsigned int ilen, unsigned char *output) {
        ::acu_sha256(input, ilen, output);
    }

}} // arduino::sha256

class SHA256 {
public:

    static constexpr uint32_t HASH_SIZE = SHA256_DIGEST_SIZE;

    inline void begin() {
        return arduino::sha256::begin(&_ctx);
    }
    inline void update(uint8_t const * data, uint32_t const len) {
        return arduino::sha256::update(&_ctx, data, len);
    }
    inline void finalize(uint8_t * hash) {
        return arduino::sha256::finalize(&_ctx, hash);
    }
    static inline void sha256(uint8_t const * data, uint32_t const len, uint8_t * hash) {
        return arduino::sha256::sha256(data, len, hash);
    }

private:

    acu_sha256_ctx _ctx;
};
