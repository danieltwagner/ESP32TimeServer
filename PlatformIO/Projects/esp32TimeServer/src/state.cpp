#include <Arduino.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>

#include "state.h"

const int adjustmentTaskSleepSec = 10;

int64_t TimeServer::getDriftAdjustmentMicros() {
  if (driftEstimate == 0) return 0;

  return -driftEstimate * (esp_timer_get_time() - lastAdjustmentPpsRiseMicros);
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
    int64_t thisPpsRiseMicros = ppsRiseMicros;

    stateDetail = StateDetail::AWAIT_FIX;

    // Now wait until both date and time are more recent than the corresponding pulse.
    // Note that date is only updated as part of the RMC sentence, along with time and
    // location (location only if we have a valid fix), so we can just check for date.
    uint32_t ppsAgeMillis = (esp_timer_get_time() - thisPpsRiseMicros)/1000;
    while (gps->date.age() > ppsAgeMillis) {
      delay(10);
      ppsAgeMillis = (esp_timer_get_time() - thisPpsRiseMicros)/1000;
    }
    if (gps->location.age() > ppsAgeMillis) {
      // Location age before the PPS pulse implies we don't have a fix.
      // try again in a little while.
      if (debugIsOn) {
        Serial.println("Got a time fix but no location. GPS location mode A/D would indicate a fix, we got: " + String(gps->location.FixMode()));
      }
      continue;
    }

    if (thisPpsRiseMicros != ppsRiseMicros) {
      // we've had a new PPS pulse since we started waiting for a fix
      continue;
    }

    // GPS time now matches the PPS pulse. What is the local time?
    struct timeval rtc_now;
    gettimeofday(&rtc_now, NULL);

    // We may have spent some time waiting for gps data to be read.
    // microsAfterRTC will allow us to account for this.
    int64_t microsAfterRTC = esp_timer_get_time();

    stateDetail = StateDetail::SETTING_TIME;

    struct tm wt;
    wt.tm_year = gps->date.year() - 1900; // 1900 is year 0
    wt.tm_mon = gps->date.month() - 1;    // January is month 0
    wt.tm_mday = gps->date.day();
    wt.tm_hour = gps->time.hour();
    wt.tm_min = gps->time.minute();
    wt.tm_sec = gps->time.second();
    time_t gpsDateAndTime = mktime(&wt);

    // Compute the RTC error. If local time is ahead of GPS time, this will be >= 0.
    // Note that 0 can also be the case if we're behind, as we waited for NMEA data.
    time_t rtcErrorSecs = rtc_now.tv_sec - gpsDateAndTime;

    // avoid changing date/time while an NTP request is being answered
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {

      // try again if a new PPS pulse has arrived since, as we'd be a second out from when we started
      if (ppsRiseMicros != thisPpsRiseMicros) {
        continue;
      }

      // set the real time clock, accounting for the extra time that passed since we saw the PPS pulse
      rtc->setTime(gpsDateAndTime, esp_timer_get_time() - thisPpsRiseMicros);

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
        int64_t microsBetweenAdjustments = thisPpsRiseMicros - lastAdjustmentPpsRiseMicros;
        // we'll keep track of the max adjustment gap for debugging purposes
        if ((microsBetweenAdjustments > maxAdjustmentGapMicros) && (state == TimeServerState::SERVING_NTP)) {
          maxAdjustmentGapMicros = microsBetweenAdjustments;
        }

        // clock error in microseconds, accounting for time taken to measure the RTC time
        int64_t microsBetweenPulseAndRTCMeasurement = microsAfterRTC - thisPpsRiseMicros;
        time_t errorMicros = (rtcErrorSecs * 1e6) + rtc_now.tv_usec - microsBetweenPulseAndRTCMeasurement;

        // how close was the previous drift adjustment?
        lastAdjustedErrorMicros = errorMicros - microsBetweenAdjustments * driftEstimate;

        // Update our drift measurement. As we only have microsecond resolution timers,
        // we want to do some kind of regression as a single microsecond is 0.1ppm drift
        // if we measure every 10s. The alternative would be to wait longer between
        // measurements but then we'd accumulate more error before we could correct it.
        driftCalc.addSample(microsBetweenAdjustments, errorMicros);
        driftEstimate = driftCalc.getDrift();

        // Pre-compute fixed-point format for use in the NTP message
        if (state == TimeServerState::SERVING_NTP || driftCalc.hasMaxSamples()) {

          if (abs(lastAdjustedErrorMicros) > maxObservedErrorMicros) {
            maxObservedErrorMicros = abs(lastAdjustedErrorMicros);

            // convert to seconds, left shift for fixed point representation, ceil so the error we report is >= observed
            rootDispersion = static_cast<uint32_t>(std::ceil(abs(lastAdjustedErrorMicros) * 1e-6 * (1 << 16)));
          }
        }

        // we keep this for debugging purposes
        lastClockDrift = (double)errorMicros / (double)microsBetweenAdjustments;
        
        if (debugIsOn) {
          Serial.println("Adjusted the clock by adding " + String(rtcErrorSecs) + "s (" + String(errorMicros) + "us)");
          Serial.print(microsBetweenAdjustments);
          Serial.print("us passed between adjustments. Clock drift is ");
          Serial.print(lastClockDrift * 1e6);
          Serial.print("ppm. Drift regression suggests " + String(driftEstimate * 1e6) + "ppm.");
          Serial.println("ppm.");

          Serial.println("When adjusting for drift we had a cumulative error of " + String(lastAdjustedErrorMicros) + "us.");
        }

        if (driftCalc.hasMaxSamples()) {
          // we have enough samples to be confident in our drift estimate
          state = TimeServerState::SERVING_NTP;
        }

      } else {
        // we did get a fix, now we want to characterize drift
        state = TimeServerState::MEASURING_DRIFT;
      }

      lastAdjustmentPpsRiseMicros = thisPpsRiseMicros;
      stateDetail = StateDetail::IDLE;

      vTaskDelay(adjustmentTaskSleepSec * 1000 / portTICK_PERIOD_MS);
    }
  }
}
