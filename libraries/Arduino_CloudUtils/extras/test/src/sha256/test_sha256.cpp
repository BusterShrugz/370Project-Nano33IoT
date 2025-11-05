/*
    This file is part of the Arduino_CloudUtils library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <catch2/catch_test_macros.hpp>

#include <sha256/SHA256.h>
#include <stdio.h>

#define SHA256_DIGEST_SIZE ( 256 / 8)
#define SHA512_DIGEST_SIZE ( 512 / 8)

#ifdef TEST_VECTORS_LONG
static void test_sha256_long_message(uint8_t *digest)
{
    acu_sha256_ctx ctx;
    uint8_t message[1000];
    int i;

    memset(message, 'a', sizeof (message));

    arduino::sha256::begin(&ctx);
    for (i = 0; i < 10000000; i++) {
        arduino::sha256::update(&ctx, message, sizeof (message));
    }
    arduino::sha256::finalize(&ctx, digest);
}
#endif

static void test_sha256_message4(uint8_t *digest)
{
    /* Message of 929271 bytes */
    acu_sha256_ctx ctx;
    uint8_t message[1000];
    int i;

    memset(message, 'a', sizeof (message));
    arduino::sha256::begin(&ctx);
    for (i = 0; i < 929; i++) {
        arduino::sha256::update(&ctx, message, sizeof (message));
    }
    arduino::sha256::update(&ctx, message, 271);
    arduino::sha256::finalize(&ctx, digest);
}

static int test(const char *vector, uint8_t *digest, uint32_t digest_size)
{
    char output[2 * SHA512_DIGEST_SIZE + 1];
    int i;

    output[2 * digest_size] = '\0';
    for (i = 0; i < (int) digest_size ; i++) {
       sprintf(output + 2 * i, "%02x", digest[i]);
    }

    //printf("H: %s\n", output);
    return strcmp(vector, output);
}

SCENARIO( "FIPS 180-2 Validation tests" ) {
    static const char *vectors[1][5] =
    {   /* SHA-256 */
        {
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
        "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1",
        "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0",
        "8c14f43ad81026351e9b60025b5420e6072ff617f5c72145b179599211514947",
        "53748286337dbe36f5df22e7ef1af3ad71530398cf569adc7eb5fefa7af7003c",
        }
    };

    static const char message1[] = "abc";
    static const char message2[] = "abcdbcdecdefdefgefghfghighijhi"
                                    "jkijkljklmklmnlmnomnopnopq";
    uint8_t *message3;
    uint32_t message3_len = 1000000;
    uint8_t digest[SHA512_DIGEST_SIZE];

    message3 = (uint8_t *)malloc(message3_len);
    REQUIRE(message3 != nullptr);
    memset(message3, 'a', message3_len);

    arduino::sha256::sha256((const uint8_t *) message1, strlen(message1), digest);
    REQUIRE(test(vectors[0][0], digest, SHA256_DIGEST_SIZE) == 0);
    arduino::sha256::sha256((const uint8_t *) message2, strlen(message2), digest);
    REQUIRE(test(vectors[0][1], digest, SHA256_DIGEST_SIZE) == 0);
    arduino::sha256::sha256(message3, message3_len, digest);
    REQUIRE(test(vectors[0][2], digest, SHA256_DIGEST_SIZE) == 0);
    test_sha256_message4(digest);
    REQUIRE(test(vectors[0][3], digest, SHA256_DIGEST_SIZE) == 0);
#ifdef TEST_VECTORS_LONG
    test_sha256_long_message(digest);
    REQUIRE(test(vectors[0][4], digest, SHA256_DIGEST_SIZE) == 0);
#endif
   free(message3);
}
