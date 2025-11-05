cmake_minimum_required(VERSION 3.5)
project(testArduinoCloudUtils)

Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.4.0
)

FetchContent_MakeAvailable(Catch2)

set(TEST_TARGET ${CMAKE_PROJECT_NAME})

##########################################################################

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(../../src)
include_directories(src/arduino)

set(TEST_SRCS
  src/lzss/test_decoder.cpp
  src/crc32/test_crc32.cpp
  src/crc16/test_crc16.cpp
  src/sha256/test_sha256.cpp
  src/hex/test_hex.cpp
  src/cbor/test_cbor_encoder.cpp
  src/cbor/test_cbor_decoder.cpp
  src/cbor/test_cbor_standard_enc.cpp
  src/time/test_TimedAttempt.cpp
)

set(TEST_DUT_SRCS
  ../../src/crc/crc32.cpp
  ../../src/crc/crc16.cpp
  ../../src/sha256/sha2.c
  ../../src/hex/chex.h
  ../../src/cbor/MessageDecoder.cpp
  ../../src/cbor/MessageEncoder.cpp
  ../../src/cbor/standards/StandardEncoders.cpp
  ../../src/cbor/tinycbor
  ../../src/cbor/tinycbor/src/cborencoder.c
  ../../src/cbor/tinycbor/src/cborencoder_close_container_checked.c
  ../../src/cbor/tinycbor/src/cborerrorstrings.c
  ../../src/cbor/tinycbor/src/cborparser.c
  ../../src/cbor/tinycbor/src/cborparser_dup_string.c
  ../../src/cbor/tinycbor/src/cborpretty.c
  ../../src/cbor/tinycbor/src/cborpretty_stdio.c
  ../../src/cbor/tinycbor/src/cbortojson.c
  ../../src/cbor/tinycbor/src/cborvalidation.c
  ../../src/cbor/tinycbor/src/open_memstream.c
  ../../src/time/TimedAttempt.cpp
)

set(TEST_STUB_SRCS
  src/arduino/Arduino.cpp
)

##########################################################################

add_compile_definitions(HOST)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
add_compile_options(-Wno-cast-function-type)

set(CMAKE_C_FLAGS   ${CMAKE_C_FLAGS}   "--coverage")
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "--coverage -Wno-deprecated-copy")

add_executable( ${TEST_TARGET} ${TEST_SRCS} ${TEST_DUT_SRCS} ${TEST_STUB_SRCS})
target_compile_definitions( ${TEST_TARGET} PUBLIC SOURCE_DIR="${CMAKE_SOURCE_DIR}" )

target_link_libraries( ${TEST_TARGET} Catch2WithMain )
