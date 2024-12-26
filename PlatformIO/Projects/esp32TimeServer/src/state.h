#ifndef __STATE_H
#define __STATE_H

#include <Arduino.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>

#include "clockdrift.h"

enum class TimeServerState {
    INITIALIZING,
    WAITING_FOR_DHCP,
    WAITING_FOR_INITIAL_FIX,
    MEASURING_DRIFT, // we have a fix but haven't characterized drift yet
    SERVING_NTP,
};

// What is the periodic task doing?
enum class StateDetail {
    IDLE,
    AWAIT_PPS,
    AWAIT_FIX,
    SETTING_TIME,
};

class TimeServer {
private:
    DriftCalculator driftCalc;

public:
    volatile TimeServerState state = TimeServerState::WAITING_FOR_DHCP;
    volatile StateDetail stateDetail = StateDetail::IDLE;
    volatile int64_t ppsRiseMicros; // The micros() of the last PPS rising edge
    volatile int8_t precision;

    volatile int64_t maxAdjustmentGapMicros = 0;
    volatile int64_t lastAdjustmentMicros = 0;
    volatile int64_t lastErrorMicros;
    // Last clock drift we observed when syncing against GPS
    volatile double lastClockDrift = 0;
    volatile double driftEstimate = 0;
    volatile uint32_t maxObservedDrift = 0; // pre-computed field used as part of the NTP message

    // used to ensure an NTP request results and time updates are mutually exclusive
    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

    bool debugIsOn = true;

    int64_t getDriftAdjustmentMicros();
    void setDateAndTimeFromGPS(ESP32Time *rtc, TinyGPSPlus *gps);
};

#endif // __STATE_H