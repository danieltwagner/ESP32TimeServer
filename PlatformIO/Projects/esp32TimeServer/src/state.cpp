#include <Arduino.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>

#include "state.h"

const int adjustmentTaskSleepSec = 10;

int64_t TimeServer::getDriftAdjustmentMicros() {
  if (driftEstimate == 0) return 0;

  return -driftEstimate * (esp_timer_get_time() - lastAdjustmentMicros);
}

void TimeServer::setDateAndTimeFromGPS(ESP32Time *rtc, TinyGPSPlus *gps) {
  while (true) {

    stateDetail = StateDetail::AWAIT_PPS;

    int64_t lastPpsRise = ppsRiseMicros;
    // wait for a new pps rising flag. This makes life a little easier later because we
    // know we're at the start of a second rather than *just* before a new pulse arrives
    while (ppsRiseMicros == lastPpsRise) {
      delay(10);
    }
    int64_t thisPpsRise = ppsRiseMicros;

    stateDetail = StateDetail::AWAIT_FIX;

    // Now wait until both date and time are more recent than the corresponding pulse.
    // Note that date is only updated as part of the RMC sentence, along with time and
    // location (location only if we have a valid fix), so we can just check for date.
    uint32_t ppsAgeMillis = (esp_timer_get_time() - thisPpsRise)/1000;
    while (gps->date.age() > ppsAgeMillis) {
      delay(10);
      ppsAgeMillis = (esp_timer_get_time() - thisPpsRise)/1000;
    }
    if (gps->location.age() > ppsAgeMillis) {
      // Location age before the PPS pulse implies we don't have a fix.
      // try again in a little while.
      if (debugIsOn) {
        Serial.println("Got a time fix but no location. GPS location mode A/D would indicate a fix, we got: " + String(gps->location.FixMode()));
      }
      continue;
    }

    if (thisPpsRise != ppsRiseMicros) {
      // we've had a new PPS pulse since we started waiting for a fix
      continue;
    }

    // GPS time now matches the PPS pulse

    struct tm wt;
    wt.tm_year = gps->date.year() - 1900; // 1900 is year 0
    wt.tm_mon = gps->date.month() - 1;    // January is month 0
    wt.tm_mday = gps->date.day();
    wt.tm_hour = gps->time.hour();
    wt.tm_min = gps->time.minute();
    wt.tm_sec = gps->time.second();
    time_t gpsDateAndTime = mktime(&wt);

    struct timeval rtc_now;
    gettimeofday(&rtc_now, NULL);
    int64_t microsAfterRTC = esp_timer_get_time();

    stateDetail = StateDetail::SETTING_TIME;

    // At this point we have the pps rise time in micros as well as the corresponding GPS time.
    // We also know our RTC time and when we took it, in micros.
    time_t updateDeltaSecs = gpsDateAndTime - rtc_now.tv_sec;

    // avoid changing date/time while an NTP request is being answered
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {

      // try again if a new PPS pulse has arrived since, as we'd be a second out from when we started
      if (ppsRiseMicros != thisPpsRise) {
        continue;
      }

      // set the real time clock, accounting for the extra micros that passed since we saw the PPS pulse
      rtc->setTime(gpsDateAndTime, esp_timer_get_time() - thisPpsRise);

      // release the hold
      xSemaphoreGive(mutex);

      if (debugIsOn) {
        Serial.print("Date and time set to ");
        String dateTimeNow = rtc->getDateTime(true);
        dateTimeNow.trim();
        Serial.println(dateTimeNow + " (UTC). We used " + String(gps->satellites.value()) + " satellites.");
      }

      if (
        state == TimeServerState::MEASURING_DRIFT ||
        state == TimeServerState::SERVING_NTP
      ) {
        // calculate drift
        int64_t microsBetweenPulseAndRTCMeasurement = microsAfterRTC - thisPpsRise;
        int64_t microsBetweenAdjustments = microsAfterRTC - lastAdjustmentMicros;
        if (microsBetweenAdjustments > maxAdjustmentGapMicros && state == TimeServerState::SERVING_NTP) {
          maxAdjustmentGapMicros = microsBetweenAdjustments;
        }

        // clock error in microseconds, accounting for time taken to measure the RTC time
        time_t errorMicros = (updateDeltaSecs * 1e6) - (rtc_now.tv_usec - microsBetweenPulseAndRTCMeasurement);
        lastClockDrift = (double)errorMicros / (double)microsBetweenAdjustments;

        // how close was the previous drift adjustment?
        lastErrorMicros = errorMicros - microsBetweenAdjustments * driftEstimate;

        // Pre-compute fixed-point format for use in the NTP message
        if (state == TimeServerState::SERVING_NTP || driftCalc.hasMaxSamples()) {

          // convert to seconds, left shift for fixed point representation, ceil so the error we report is >= observed
          uint32_t fixedPointValue = static_cast<uint32_t>(std::ceil(abs(lastErrorMicros) * 1e-6 * (1 << 16)));
          if (fixedPointValue > maxObservedDrift) {
            maxObservedDrift = fixedPointValue;
          }
        }

        // Update our drift measurement. As we only have microsecond resolution timers,
        // we want to do some kind of regression as a single microsecond is 0.1ppm drift
        // if we measure every 10s. The alternative would be to wait longer between
        // measurements but then we'd accumulate more error before we could correct it.
        driftCalc.addSample(microsBetweenAdjustments, errorMicros);
        driftEstimate = driftCalc.getDrift();
        
        if (debugIsOn) {
          Serial.println("Adjusted the clock by adding " + String(updateDeltaSecs) + "s (" + String(errorMicros) + "us)");
          Serial.print(microsBetweenAdjustments);
          Serial.print("us passed between adjustments. Clock drift is ");
          Serial.print(lastClockDrift * 1e6);
          Serial.print("ppm. Drift regression suggests " + String(driftEstimate * 1e6) + "ppm.");
          Serial.println("ppm.");

          Serial.println("When adjusting for drift we had a cumulative error of " + String(lastErrorMicros) + "us.");
        }

      } else {
        // we did get a fix, now we want to characterize drift
        state = TimeServerState::MEASURING_DRIFT;
      }

      lastAdjustmentMicros = microsAfterRTC;
      stateDetail = StateDetail::IDLE;

      vTaskDelay(adjustmentTaskSleepSec * 1000 / portTICK_PERIOD_MS);
    }
  }
}
