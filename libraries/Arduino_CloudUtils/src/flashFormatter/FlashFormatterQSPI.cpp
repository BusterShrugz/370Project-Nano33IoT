/*
  Copyright (c) 2025 Arduino SA

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#if defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) \
  || defined(ARDUINO_OPTA) || defined(ARDUINO_GIGA) || defined(ARDUINO_PORTENTA_C33)
#include "FlashFormatterQSPI.h"

FlashFormatterQSPI::FlashFormatterQSPI():
_root(BlockDevice::get_default_instance()),
_wifiData(_root, 1),
_wifiFS("wlan"),
_otaData(_root, 2),
_otaFS("fs"),
_kvstoreData(_root, 3),
_runtimeData(_root, 4),
_runtimeFS("opt"),
_runtimeFormat(false)
{
}

bool FlashFormatterQSPI::checkPartition()
{
  if (_root->init() != BD_ERROR_OK) {
    _runtimeFormat = true;
    return false;
  }

  /* check PLC runtime partition. This should be performed as first check to
   * correctly set the _runtimeFormat flag.
   */
  _runtimeFormat = false;
  if (_runtimeData.init() !=  BD_ERROR_OK) {
    _runtimeFormat = true;
    return false;
  }
  _runtimeData.deinit();

  /* check WiFi partition */
  if (_wifiData.init() !=  BD_ERROR_OK || _wifiFS.mount(&_wifiData) != 0) {
    return false;
  }

  if (!checkWiFiData()) {
    return false;
  }

  _wifiFS.unmount();
  _wifiData.deinit();

  /* check OTA partition */
  if (_otaData.init() !=  BD_ERROR_OK || _otaFS.mount(&_otaData) != 0) {
    return false;
  }

  if (_otaData.size() < 5 * 1024 * 1024) {
    return false;
  }

  _otaFS.unmount();
  _otaData.deinit();

  /* check KVStore partition */
  if (_kvstoreData.init() !=  BD_ERROR_OK) {
    return false;
  }

  if (_kvstoreData.size() < 1 * 1024 * 1024) {
    return false;
  }
  _kvstoreData.deinit();

  _root->deinit();
  return true;
}

bool FlashFormatterQSPI::formatPartition() {

  if (_root->init() != BD_ERROR_OK) {
    return false;
  }

  if (_root->erase(0x0, _root->get_erase_size()) != BD_ERROR_OK) {
    return false;
  }

  MBRBlockDevice::partition(_root, 1, 0x0B, 0, 1 * 1024 * 1024);
  if (_wifiFS.reformat(&_wifiData) != 0) {
    return false;
  }

  if (!restoreWifiData()) {
    return false;
  }
  _wifiFS.unmount();

  MBRBlockDevice::partition(_root, 2, 0x0B, 1 * 1024 * 1024, 6 * 1024 * 1024);
  if (_otaFS.reformat(&_otaData) != 0) {
    return false;
  }
  _otaFS.unmount();

  MBRBlockDevice::partition(_root, 3, 0x0B, 6 * 1024 * 1024, 7 * 1024 * 1024);

  if (_runtimeFormat) {
    MBRBlockDevice::partition(_root, 4, 0x0B, 7 * 1024 * 1024, 14 * 1024 * 1024);
    if (_runtimeFS.reformat(&_runtimeData) != 0) {
      return false;
    }
    _runtimeFS.unmount();
  }

  _root->deinit();

  return true;
}

bool FlashFormatterQSPI::checkFile(const char* partition, const char* filename)
{
  DIR *dir;
  struct dirent *ent;

  if ((dir = opendir(partition)) == NULL) {
    return false;
  }

  bool foundFile = false;
  while ((ent = readdir (dir)) != NULL) {
    String fullname = String(partition) + "/" + String(ent->d_name);
    if (fullname == String(partition) + "/" + String(filename)) {
      foundFile = true;
      break;
    }
  }
  closedir (dir);
  return foundFile;
}

bool FlashFormatterQSPI::writeFile(const char* partition, const char* filename, const uint8_t* data, size_t size, size_t maxChunkSize)
{
  String fullname = String(partition) + "/" + String(filename);
  FILE* fp = fopen(fullname.c_str(), "wb");
  if (!fp) {
    return false;
  }

  size_t chunkSize = maxChunkSize;
  size_t byteCount = 0;
  while (byteCount < size) {
    if (byteCount + chunkSize > size)
      chunkSize = size - byteCount;
    int ret = fwrite(&data[byteCount], chunkSize, 1, fp);
    if (ret != 1) {
      fclose(fp);
      return false;
    }
    byteCount += chunkSize;
  }
  size_t written = ftell(fp);
  fclose(fp);

  return written == size;
}

bool FlashFormatterQSPI::writeFlash(const uint8_t* data, size_t size, size_t offset, size_t maxChunkSize)
{
  size_t chunkSize = maxChunkSize;
  size_t byteCount = 0;
  while (byteCount < size) {
    if(byteCount + chunkSize > size)
      chunkSize = size - byteCount;
    int ret = _root->program(data, offset + byteCount, chunkSize);
    if (ret != 0) {
      return false;
    }
    byteCount += chunkSize;
  }
  return true;
}

#endif
