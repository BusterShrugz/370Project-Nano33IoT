/*
 * FIPS 180-2 256 implementation based on code by Oliver Gay
 * under a BSD-style license. http://ouah.org/ogay/sha2/
 *
 * Copyright (C) 2005-2023 Olivier Gay <olivier.gay@a3.epfl.ch>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE PROJECT AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef SHA256_H
#define SHA256_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifndef SHA256_DIGEST_SIZE
    #define SHA256_DIGEST_SIZE ( 256 / 8)
#endif // SHA256_DIGEST_SIZE

#ifndef SHA256_BLOCK_SIZE
    #define SHA256_BLOCK_SIZE  ( 512 / 8)
#endif // SHA256_BLOCK_SIZE

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t tot_len;
    uint64_t len;
    uint8_t block[2 * SHA256_BLOCK_SIZE];
    uint32_t h[8];
} acu_sha256_ctx;

void acu_sha256_init(acu_sha256_ctx * ctx);
void acu_sha256_update(acu_sha256_ctx *ctx, const uint8_t *message, uint64_t len);
void acu_sha256_final(acu_sha256_ctx *ctx, uint8_t *digest);
void acu_sha256(const uint8_t *message, uint64_t len, uint8_t *digest);

#ifdef __cplusplus
}
#endif

#endif /* !SHA256_H */
