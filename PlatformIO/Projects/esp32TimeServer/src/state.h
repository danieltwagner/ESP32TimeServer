#ifndef __STATE_H
#define __STATE_H

#include <Arduino.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>

enum class TimeServerState {
    INITIALIZING,
    WAITING_FOR_DHCP,
    WAITING_FOR_INITIAL_FIX,
    MEASURING_DRIFT_INITIAL, // we have a fix but no idea about drift yet
    MEASURING_DRIFT_VARIATION, // we have measured initial drift, let's see how stable it is
    SERVING_NTP,
};

class TimeServer {
public:
    volatile TimeServerState state = TimeServerState::WAITING_FOR_DHCP;
    volatile int64_t ppsRiseMicros; // The micros() of the last PPS rising edge
    volatile int8_t precision;

    volatile int64_t maxAdjustmentGapMicros = 0;
    volatile int64_t lastAdjustmentMicros = 0;
    volatile int64_t lastErrorMicros;
    // Last clock drift we observed when syncing against GPS
    volatile double lastDrift = 0;
    // the previous value of lastDrift, only used for debugging
    volatile double previousDrift = 0;
    volatile uint32_t maxObservedDrift = 0; // pre-computed field used as part of the NTP message

    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();         // used to ensure an NTP request results are not impacted by the process that refreshes the time

    bool debugIsOn = true;
  
    int64_t microsSinceLastAdjustment(int64_t microsNow);
    int64_t getDriftAdjustmentMicros();
    void setDateAndTimeFromGPS(ESP32Time *rtc, TinyGPSPlus *gps);
};

#endif // __STATE_H