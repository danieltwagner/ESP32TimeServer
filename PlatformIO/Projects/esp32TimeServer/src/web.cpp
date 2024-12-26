#include <Arduino.h>
#include "soc/rtc.h"

#include "display.h"
#include "web.h"
#include "state.h"

String statusString(TimeServerState state) {
  if (state == TimeServerState::WAITING_FOR_INITIAL_FIX) {
    return "Waiting for initial GPS fix";
  } else if (state == TimeServerState::MEASURING_DRIFT) {
    return "Measuring clock drift";
  } else if (state == TimeServerState::SERVING_NTP) {
    return "Serving NTP";
  }

  return "Unknown";
}

String detailedStatusString(StateDetail stateDetail) {
  if (stateDetail == StateDetail::IDLE) {
    return "Idle";
  } else if (stateDetail == StateDetail::AWAIT_PPS) {
    return "Awaiting PPS";
  } else if (stateDetail == StateDetail::AWAIT_FIX) {
    return "Awaiting GPS fix";
  } else if (stateDetail == StateDetail::SETTING_TIME) {
    return "Setting time";
  }

  return "Unknown";
}

String satelliteTypeString(TinyGPSSystem system, uint8_t satId) {
    if (system == TinyGPSSystem::GPS) {
      if (satId >= 193) {
        return " (QZSS)";
      } else if (satId >= 33) {
        return " (SBAS)";
      } else {
        return " (GPS)";
      }
    } else if (system == TinyGPSSystem::GLONASS) {
      return " (GLONASS)";
    } else if (system == TinyGPSSystem::GALILEO) {
      return " (GALILEO)";
    } else if (system == TinyGPSSystem::BEIDOU) {
      return " (BEIDOU)";
    }
    return (" (Unknown)");
}

void renderWeb(WebServer &server, TimeServer &timeServer, ESP32Time &rtc, TinyGPSPlus &gps, std::vector<String> &lastNmeaSentences) {
  // This allows us to keep outputting NMEA sentences without 
  // constructing the string or computing the length first.
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);

  server.send(200, "text/html", "Uptime: " + String(getUptime()) + "</br></br>");
  server.sendContent("Status: " + statusString(timeServer.state) + " (Adjustment task: " + detailedStatusString(timeServer.stateDetail) + ")" + "</br></br>");

  if(timeServer.state != TimeServerState::WAITING_FOR_INITIAL_FIX) {
    server.sendContent("Clock drift was last " + String(timeServer.lastClockDrift * 1e6) + "ppm, regression suggests: " + String(timeServer.driftEstimate * 1e6) + "ppm.</br>");
    server.sendContent("Last cumulative clock drift when adjusting: " + String(timeServer.lastErrorMicros) + "us, max observed error: " + String(timeServer.maxObservedErrorMicros) + "us -> root dispersion (fixed point, min is 15.26us): " + String((static_cast<float>(timeServer.rootDispersion) / (1 << 16)) * 1e6) + "us<br/>");
    server.sendContent("Time since last adjustment: " + String((esp_timer_get_time() - timeServer.lastAdjustmentPpsRiseMicros)/1e6) + "s</br>");
    server.sendContent("Max time between adjustments: " + String(timeServer.maxAdjustmentGapMicros/1e6) + "s</br></br>");
  }

  server.sendContent("Time: " + rtc.getDateTime(true) + " UTC<br/>");
  
  if (gps.location.age() < ULONG_MAX) {
    server.sendContent("Fix satellites: " + String(gps.satellites.value()) + "</br>");
    server.sendContent("Last fix: " + String(gps.location.age()/1000.0) + "s ago, last PPS pulse: " + String((esp_timer_get_time() - timeServer.ppsRiseMicros)/1e6) + "s ago.</br>");
  }

  server.sendContent("</br>Satellites visible: " + String(gps.satellitesStats.nrSatsVisible()) + " Satellites tracked: " + String(gps.satellitesStats.nrSatsTracked()));
  for (TinyGPSSystem system : {TinyGPSSystem::GPS, TinyGPSSystem::GLONASS, TinyGPSSystem::GALILEO, TinyGPSSystem::BEIDOU}) {  
    int systemOffset = static_cast<uint8_t>(system) * _GPS_MAX_NR_ACTIVE_SATELLITES;

    for (int i = 0; i < _GPS_MAX_NR_ACTIVE_SATELLITES; i++) {
      uint8_t snr = gps.satellitesStats.snr[i + systemOffset];
      if (snr == 0) {
        continue;
      }
      uint8_t satId = gps.satellitesStats.id[i + systemOffset];
      server.sendContent("</br>&nbsp;&nbsp;Satellite " + String(satId) + satelliteTypeString(system, satId) + " SNR: " + String(snr));
    }
  }
  server.sendContent("</br></br>Last NMEA sentences:</br></br>");
  std::vector<String>::iterator it = lastNmeaSentences.begin(), end = lastNmeaSentences.end();
  for (; it != end; ++it) {
    server.sendContent(*it + "</br>");
  }
  server.sendContent("</br></br><a href='/update'>Firmware update</a>");
  
  // This tells the client to disconnect
  server.sendContent("");
}
