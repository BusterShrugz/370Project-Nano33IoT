/*
  This file is part of the Arduino_CloudUtils library.

  Copyright (c) 2024 Arduino SA

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

/**
    * The TimedAttempt class manages retry attempts with configurable delays.
    * It allows setting minimum and maximum delays, and provides methods to
    * begin, retry, reset, and check the status of attempts.
    */
class TimedAttempt {

public:
    /**
    * Constructor to initialize TimedAttempt with minimum and maximum delay
    * @param minDelay: minimum delay between attempts
    * @param maxDelay: maximum delay between attempts
    */
    TimedAttempt(unsigned long minDelay, unsigned long maxDelay);

    /**
    * Begin the attempt with a fixed delay
    * @param delay: fixed delay between attempts
    */
    void begin(unsigned long delay);

    /**
    * Begin the attempt with a range of delays
    * @param minDelay: minimum delay between attempts
    * @param maxDelay: maximum delay between attempts
    */
    void begin(unsigned long minDelay, unsigned long maxDelay);

    /**
    * Reconfigure the attempt with new minimum and maximum delays and reload
    * @param minDelay: new minimum delay between attempts
    * @param maxDelay: new maximum delay between attempts
    * @return the new delay after reconfiguration
    */
    unsigned long reconfigure(unsigned long minDelay, unsigned long maxDelay);

    /**
    * Retry the attempt, incrementing the retry count and reloading the delay
    * @return the new delay after retry
    */
    unsigned long retry();

    /**
    * Reload the delay based on the retry count and return the new delay
    * @return the new delay after reload
    */
    unsigned long reload();

    /**
    * Reset the retry count to zero
    */
    void reset();

    /**
    * Check if a retry has been attempted
    * @return true if a retry has been attempted, false otherwise
    */
    bool isRetry();

    /**
    * Check if the current attempt has expired based on the delay
    * @return true if the current attempt has expired, false otherwise
    */
    bool isExpired();

    /**
    * Get the current retry count
    * @return the current retry count
    */
    unsigned int getRetryCount();

    /**
    * Get the current wait time for the next retry
    * @return the current wait time for the next retry
    */
    unsigned int getWaitTime();

private:
    unsigned long _minDelay;
    unsigned long _maxDelay;
    unsigned long _retryTick;
    unsigned long _retryDelay;
    unsigned int _retryCount;
};
