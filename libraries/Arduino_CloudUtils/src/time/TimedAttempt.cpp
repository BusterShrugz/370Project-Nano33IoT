/*
  This file is part of the Arduino_CloudUtils library.

  Copyright (c) 2024 Arduino SA

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#include <Arduino.h>
#include "TimedAttempt.h"

TimedAttempt::TimedAttempt(unsigned long minDelay, unsigned long maxDelay)
: _minDelay(minDelay)
, _maxDelay(maxDelay) {
}

void TimedAttempt::begin(unsigned long delay) {
    _retryCount = 0;
    _retryDelay = 0;
    _retryTick = 0;
    _minDelay = delay;
    _maxDelay = delay;
}

void TimedAttempt::begin(unsigned long minDelay, unsigned long maxDelay) {
    _retryCount = 0;
    _retryDelay = 0;
    _retryTick = 0;
    _minDelay = minDelay;
    _maxDelay = maxDelay;
}

unsigned long TimedAttempt::reconfigure(unsigned long minDelay, unsigned long maxDelay) {
    _minDelay = minDelay;
    _maxDelay = maxDelay;
    return reload();
}

unsigned long TimedAttempt::retry() {
    _retryCount++;
    return reload();
}

unsigned long TimedAttempt::reload() {
    unsigned long shift = _retryCount > 31 ? 31 : _retryCount;
    unsigned long delay = (1UL << shift) * _minDelay;
    /* delay should always increase */
    _retryDelay =  (delay > _retryDelay) ? min(delay, _maxDelay): _maxDelay;
    _retryTick = millis();
    return _retryDelay;
}

void TimedAttempt::reset() {
    _retryCount = 0;
}

bool TimedAttempt::isRetry() {
    return _retryCount > 0;
}

bool TimedAttempt::isExpired() {
    return millis() - _retryTick > _retryDelay;
}

unsigned int TimedAttempt::getRetryCount() {
    return _retryCount;
}

unsigned int TimedAttempt::getWaitTime() {
    return _retryDelay;
}
