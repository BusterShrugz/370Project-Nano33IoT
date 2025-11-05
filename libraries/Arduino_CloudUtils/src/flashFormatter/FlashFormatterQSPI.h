/*
  Copyright (c) 2025 Arduino SA

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#pragma once
#include "FlashFormatterBase.h"
#include <BlockDevice.h>
#include <MBRBlockDevice.h>
#include <FATFileSystem.h>

#if !defined(ARDUINO_PORTENTA_C33)
using namespace mbed;
#endif

class FlashFormatterQSPI : public FlashFormatterBase {
public:
  FlashFormatterQSPI();
  bool checkPartition() override;
  bool formatPartition() override;

protected:
  bool checkFile(const char* partition, const char* filename);
  bool writeFile(const char* partition, const char* filename, const uint8_t* data, size_t size, size_t maxChunkSize);
  bool writeFlash(const uint8_t* data, size_t size, size_t offset, size_t maxChunkSize);

private:
  virtual bool checkWiFiData() { return true; };
  virtual bool restoreWifiData() { return true; };
  BlockDevice* _root;
  MBRBlockDevice _wifiData;
  FATFileSystem _wifiFS;
  MBRBlockDevice _otaData;
  FATFileSystem _otaFS;
  MBRBlockDevice _kvstoreData;
  MBRBlockDevice _runtimeData;
  FATFileSystem _runtimeFS;
  bool _runtimeFormat;
};

