#include <Arduino.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>

#include "state.h"

const int adjustmentTaskSleepSec = 10;

int64_t TimeServer::microsSinceLastAdjustment(int64_t microsNow)
{
    // corner case: deal with micros wrapping
    if (microsNow < lastAdjustmentMicros)
    {
        return microsNow + (ULONG_MAX - lastAdjustmentMicros);
    }
    return microsNow - lastAdjustmentMicros;
}

int64_t TimeServer::getDriftAdjustmentMicros() {
  // exit if we don't have an adjustment or computed drift
  if (lastAdjustmentMicros == 0 || lastDrift == 0) {
    return 0;
  }

  int64_t deltaMicros = microsSinceLastAdjustment(esp_timer_get_time());
  return -lastDrift * (double)deltaMicros;
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

    stateDetail = StateDetail::AWAIT_FIX;

    // Now wait until both date and time are more recent than the corresponding pulse.
    // Note that date is only updated as part of the RMC sentence, along with time and
    // location (location only if we have a valid fix), so we can just check for date.
    uint32_t ppsAgeMillis = (esp_timer_get_time() - ppsRiseMicros)/1000;
    while (gps->date.age() > ppsAgeMillis) {
      delay(10);
    }
    if (gps->location.age() > ppsAgeMillis) {
      // Location age before the PPS pulse implies we don't have a fix.
      // try again in a little while.
      if (debugIsOn) {
        Serial.println("Got a time fix but no location. GPS location mode A/D would indicate a fix, we got: " + String(gps->location.FixMode()));
      }
      continue;
    }
    int64_t thisPpsRise = ppsRiseMicros;

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
        state == TimeServerState::MEASURING_DRIFT_INITIAL || 
        state == TimeServerState::MEASURING_DRIFT_VARIATION ||
        state == TimeServerState::SERVING_NTP
      ) {
        // calculate drift
        int64_t microsBetweenPulseAndRTCMeasurement = microsAfterRTC - thisPpsRise;
        int64_t microsBetweenAdjustments = microsSinceLastAdjustment(microsAfterRTC);

        // adjustment in microseconds, accounting for time taken to measure the RTC time
        time_t deltaMicros = (updateDeltaSecs * 1000000) - (rtc_now.tv_usec - microsBetweenPulseAndRTCMeasurement);
        double clockDrift = (double)deltaMicros / (double)microsBetweenAdjustments;
        if (microsBetweenAdjustments > maxAdjustmentGapMicros) {
          maxAdjustmentGapMicros = microsBetweenAdjustments;
        }

        // how close was the previous drift adjustment?
        lastErrorMicros = deltaMicros - microsBetweenAdjustments*lastDrift;
        
        // Pre-compute fixed-point format for use in the NTP message
        if (state == TimeServerState::MEASURING_DRIFT_VARIATION || state == TimeServerState::SERVING_NTP) {

          // convert to seconds, left shift for fixed point representation, ceil so the error we report is >= observed
          uint32_t fixedPointValue = static_cast<uint32_t>(std::ceil(abs(lastErrorMicros) * 1e-6 * (1 << 16)));
          if (fixedPointValue > maxObservedDrift) {
            maxObservedDrift = fixedPointValue;
          }

          // we're ready to serve NTP now
          state = TimeServerState::SERVING_NTP;
        } else {
          // we now have an initial drift measurement. See how it changes over time before we start serving.
          state = TimeServerState::MEASURING_DRIFT_VARIATION;
        }
        
        if (debugIsOn) {
          Serial.println("Adjusted the clock by adding " + String(updateDeltaSecs) + "s (" + String(deltaMicros) + "us)");
          Serial.print(microsBetweenAdjustments);
          Serial.print("us passed between adjustments. Clock drift is ");
          Serial.print(clockDrift * 1000000.0);
          Serial.print("ppm. Last run was ");
          Serial.print(lastDrift * 1000000.0);
          Serial.println("ppm.");

          Serial.println("When adjusting for drift we had a cumulative error of " + String(lastErrorMicros) + "us.");
        }

        previousDrift = lastDrift;
        lastDrift = clockDrift;
      } else {
        // we did get a fix, now we want to characterize drift
        state = TimeServerState::MEASURING_DRIFT_INITIAL;
      }

      lastAdjustmentMicros = microsAfterRTC;

      stateDetail = StateDetail::IDLE;
      vTaskDelay(adjustmentTaskSleepSec * 1000 / portTICK_PERIOD_MS);
    }
  }
}
