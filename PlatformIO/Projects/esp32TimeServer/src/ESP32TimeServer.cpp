// Rob Latour, 2023 with modifications by Daniel Wagner
// License: MIT
// https://github.com/roblatour
//
// This program's key setting can be viewed/updated in the file:
//      ESP32TimeServerKeySettings.h
//
// https://time.is may be used a reference point to confirm your computer's date/time accuracy
//
#include <Timezone.h>
#include <ElegantOTA.h>
#include <ESP32Time.h>

#include "display.h"
#include "gps.h"
#include "network.h"
#include "ntp.h"
#include "state.h"
#include "web.h"

TimeServer timeServer;

// Handles OTA and web requests
WebServer server(80);

// ESP32Time real time clock
ESP32Time rtc(0);

TaskHandle_t taskHandle0 = NULL; // task handle for updating the display
TaskHandle_t taskHandle1 = NULL; // task handle for setting/refreshing the time

void updateDisplay(void *param) {
  while (true) {
    displayInfo(timeServer, rtc, gps);

    struct timeval rtc_now;
    gettimeofday(&rtc_now, NULL);
    unsigned long nextFullSec = (1000000 - rtc_now.tv_usec) / 1000;
    vTaskDelay(nextFullSec / portTICK_PERIOD_MS);
  }
}

void handleWebRequest() {
  renderWeb(server, timeServer, rtc, gps, lastNmeaSentences);
}

void ppsHandlerRising() {
  timeServer.ppsRiseMicros = esp_timer_get_time();
}

void setDateAndTimeFromGPS(void *parameter) {
  timeServer.setDateAndTimeFromGPS(&rtc, &gps);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Time Server starting");

  // Regular internal 150 kHz RC oscillator results in around 9ppm drift, and is constant to about 0.3ppm minute-to-minute.
  // Supposedly the 8MHz RC oscillator is more accurate.
  // See also https://esp32.com/viewtopic.php?t=31371 and https://github.com/espressif/arduino-esp32/issues/7669
  // Ah, it turns out the RTC slow clock is not used for the main clock, it just drives "RTC domain" analog functions.
  // See https://esp32.com/viewtopic.php?t=2742
  // rtc_clk_slow_freq_set(RTC_SLOW_FREQ_8MD256);

  timeServer.state = TimeServerState::INITIALIZING;

  displayInit();
  
  // wifi and bluetooth aren't needed so turn them off
  Serial.println("Disabling wifi and bluetooth");
  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.print("Measuring time to read the clock (\"precision\")... ");
  int64_t clockReadMicros = measureClockReadDurationMicros(timeServer);
  double seconds = static_cast<double>(clockReadMicros) / 1e6;
  timeServer.precision = static_cast<int8_t>(std::ceil(std::log2(seconds)));
  Serial.println(String(clockReadMicros) + "us, resulting precision: " + String(timeServer.precision));

  // interrupt handler for the pulse-per-second pin
  pinMode(pinGPSpps, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinGPSpps), ppsHandlerRising, RISING);

  timeServer.state = TimeServerState::WAITING_FOR_DHCP;

  // TODO: Would be nice not to block on DHCP in case we need to move the device
  // to a window to get a GPS fix.
  Serial.println("Setting up networking");
  drawCentered("Waiting for DHCP...");
  setupEthernet();

  // Web server and OTA
  server.on("/", handleWebRequest);
  ElegantOTA.begin(&server); // serves /update
  server.begin();

  timeServer.state = TimeServerState::WAITING_FOR_INITIAL_FIX;

  // Configure GPS to 115200 baud
  Serial.print("Opening GPS with 9600 baud... ");
  GPSDevice.begin(9600, SERIAL_8N1, pinGPSrx, pinGPStx);

  // // wait for at least one valid sentence
  // if (!checkForValidGPSSentence()) {
  //   Serial.print("fail. Trying 115200 baud... ");
  // } else {
  //   // send the command to 
  //   Serial.print("success! Configuring 115200 baud...");
  //   GPSDevice.write(switchTo115200, sizeof(switchTo115200));
  // }
  // delay(100);
  // GPSDevice.end();

  // GPSDevice.begin(115200, SERIAL_8N1, pinGPSrx, pinGPStx);
  if (!checkForValidGPSSentence()) {
    Serial.println("fail. Check GPS connection.");
    drawCentered("Check GPS");

    // loop for 60s so we don't get locked out of the firmware updater
    unsigned long start = millis();
    while (millis() - start < 60000) {
      server.handleClient();
    }
    ESP.restart();

  } else {
    Serial.println("success!");
  }

  // set up task for gps time
  xTaskCreate(
    setDateAndTimeFromGPS,
    "Set Date and Time from GPS",
    3000,
    NULL,
    20, // reasonably high priority, we do want to keep time after all
    &taskHandle0
  );

  // wait until the time is actually set
  Serial.println("Waiting for GPS time...");

  while (timeServer.state == TimeServerState::WAITING_FOR_INITIAL_FIX) {
    while (GPSDevice.available()) {
      if(readGPS() && timeServer.debugIsOn) {
        // output the sentence that just finished
        Serial.print(lastNmeaSentences.back());
      }
      if(gps.satellitesStats.isUpdated()) {
        displayAcquiring(gps.satellitesStats);
      }
    }
    delay(10); // keep the watchdog happy
    server.handleClient(); // ensure we can reach the http interface
  }

  // set up task for updating the display
  xTaskCreate(
    updateDisplay,
    "updateDisplay",
    3000,
    NULL,
    5, // not as important
    &taskHandle1
  );

  ntpStartListening();
  Serial.println("ESP32 Time Server setup complete - listening for NTP requests now");
}

void loop()
{
  while (GPSDevice.available()) {
    readGPS();
  }
  processNTPRequests(timeServer);
  server.handleClient();
}