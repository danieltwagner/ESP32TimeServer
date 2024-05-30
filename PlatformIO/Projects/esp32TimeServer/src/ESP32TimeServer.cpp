// Rob Latour, 2023
// License: MIT
// https://github.com/roblatour
//
// This program's key setting can be viewed/updated in the file:
//      ESP32TimeServerKeySettings.h
//
// https://time.is may be used a reference point to confirm your computer's date/time accuracy
//

// board: WT32-ETH01
// Neo6M GPS or similar
const int pinGPSrx = 5;  // GPIO 5 is used to receive from GPS (connect to GPS TX pin)
const int pinGPStx = 17; // GPIO 17 is used to transmit to GPS (connect to GPS RX pin)
const int pinGPSpps = 33; // GPIO 33 reads the PPS pulse  

// SSD1306 I2C 128x64 OLED display
const int pinDisplaySda = 15; // Display SDA connected to GPIO15
const int pinDisplayScl = 14; // Display SCL connected to GPIO14

#include <ETH.h>
#include <Timezone.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>
#include <ElegantOTA.h>
#include <U8g2lib.h>

bool debugIsOn = true;

// ESP32Time real time clock
ESP32Time rtc(0);

// Display setup. We can afford to spend 1K on a full frame buffer.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, pinDisplayScl, pinDisplaySda);

// Ethernet
bool eth_connected = false;
bool eth_got_IP = false;
String ip = "";

// Web server for OTA
WebServer server(80);

// GPS
TinyGPSPlus gps;
#define GPSDevice Serial2
volatile unsigned long ppsRiseMicros; // The micros() of the last rising flag

// NTP port and packet buffer
#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];
WiFiUDP Udp;

const byte switchTo115200[] = {
  0xB5, 0x62, // Sync chars
  0x06, 0x00, // Message class and ID
  0x14, 0x00, // Payload length (20 bytes)
  0x01, 0x00, // portID = 1, reserved0 = 0
  0x00, 0x00, // txReady
  0xD0, 0x08, 0x00, 0x00, // mode
  0x00, 0xC2, 0x01, 0x00, // baudRate (115200 in little-endian)
  0x07, 0x00, // inProtoMask
  0x07, 0x00, // outProtoMask
  0x00, 0x00, // flags
  0x00, 0x00, // reserved5
  0xC4, 0x96  // checksum (CK_A, CK_B)
};


volatile bool didSetGPSTime;  // have we ever set time based on GPS?
volatile int8_t precision;

volatile unsigned long lastAdjustmentMicros = 0;
volatile double lastErrorMicros;
volatile double lastDrift = 0;
volatile uint32_t maxObservedDrift = 0; // pre-computed field used as part of the NTP message

SemaphoreHandle_t mutex;         // used to ensure an NTP request results are not impacted by the process that refreshes the time
                                 //
TaskHandle_t taskHandle0 = NULL; // task handle for updating the display
TaskHandle_t taskHandle1 = NULL; // task handle for setting/refreshing the time

unsigned long microsSinceLastAdjustment(unsigned long microsNow) {
  // corner case: deal with micros wrapping
  if (microsNow < lastAdjustmentMicros) {
    return microsNow + (ULONG_MAX - lastAdjustmentMicros);
  }
  return microsNow - lastAdjustmentMicros;
}

long getDriftAdjustmentMicros() {
  // exit if we don't have an adjustment or computed drift
  if (lastAdjustmentMicros == 0 || lastDrift == 0) {
    return 0;
  }

  unsigned long deltaMicros = microsSinceLastAdjustment(micros());
  return - lastDrift * (double)deltaMicros;
}

String getUptime() {
  unsigned long ms = millis();

  const int oneSecond = 1000;
  const int oneMinute = oneSecond * 60;
  const int oneHour = oneMinute * 60;
  const int oneDay = oneHour * 24;

  int numberOfDays = ms / oneDay;
  ms = ms - numberOfDays * oneDay;

  int numberOfHours = ms / oneHour;
  ms = ms - numberOfHours * oneHour;

  int numberOfMinutes = ms / oneMinute;
  ms = ms - numberOfMinutes * oneMinute;

  int numberOfSeconds = ms / oneSecond;

  String returnValue = "";

  char buffer[21];

  sprintf(buffer, "%d days %02d:%02d:%02d", numberOfDays, numberOfHours, numberOfMinutes, numberOfSeconds);

  returnValue = String(buffer);
  return returnValue;
}

void setDateAndTimeFromGPS(void *parameter) {
  time_t gpsDateAndTime;
  long numSyncs = 0;

  while (true) {

    unsigned long lastPpsRise = ppsRiseMicros;
    // wait for a new pps rising flag. This makes life a little easier later because we
    // know we're at the start of a second rather than *just* before a new pulse arrives
    while (ppsRiseMicros == lastPpsRise) {
      delay(10);
    }

    // now wait until both date and time are more recent than the corresponding pulse
    unsigned long ppmAge = (micros() - ppsRiseMicros)/1000;
    bool updated = gps.date.age() < ppmAge && gps.time.age() < ppmAge;
    while (!updated) {
      delay(10);
      ppmAge = (micros() - ppsRiseMicros)/1000;
      updated = gps.date.age() < ppmAge && gps.time.age() < ppmAge;
    }
    unsigned long thisPpsRise = ppsRiseMicros;

    struct tm wt;
    wt.tm_year = gps.date.year() - 1900; // 1900 is year 0
    wt.tm_mon = gps.date.month() - 1;    // January is month 0
    wt.tm_mday = gps.date.day();
    wt.tm_hour = gps.time.hour();
    wt.tm_min = gps.time.minute();
    wt.tm_sec = gps.time.second();
    gpsDateAndTime = mktime(&wt);

    struct timeval rtc_now;
    gettimeofday(&rtc_now, NULL);
    unsigned long microsAfterRTC = micros();

    // At this point we have the pps rise time in micros as well as the corresponding GPS time.
    // We also know our RTC time and when we took it, in micros.
    time_t updateDelta = gpsDateAndTime - rtc_now.tv_sec;

    // avoid changing date/time while an NTP request is being answered
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {

      // try again if a new PPS pulse has arrived since, as we'd be a second out from when we started
      if (ppsRiseMicros != thisPpsRise) {
        continue;
      }

      // set the real time clock, accounting for the extra micros that passed since we saw the PPS pulse
      rtc.setTime(gpsDateAndTime, micros() - thisPpsRise);

      // release the hold
      xSemaphoreGive(mutex);

      if (debugIsOn) {
        Serial.print("Date and time set to ");
        String ws = rtc.getDateTime(true);
        ws.trim();
        Serial.println(ws + " (UTC). We used " + String(gps.satellites.value()) + " satellites.");
      }

      if (didSetGPSTime) {
        // calculate drift
        unsigned long microsBetweenPulseAndRTCMeasurement = microsAfterRTC - thisPpsRise;
        unsigned long microsBetweenAdjustments = microsSinceLastAdjustment(microsAfterRTC);
        time_t deltaMicros = (updateDelta * 1000000) - (rtc_now.tv_usec - microsBetweenPulseAndRTCMeasurement);
        double clockDrift = (double)deltaMicros / (double)microsBetweenAdjustments;

        // how close was the previous drift adjustment?
        lastErrorMicros = deltaMicros - microsBetweenAdjustments*lastDrift;
        
        // Pre-compute fixed-point format for use in the NTP message
        if (numSyncs > 1) { // skip the first two runs so we only count drift-adjusted numbers

          // convert to seconds, left shift for fixed point representation, ceil so the error we report is >= observed
          uint32_t fixedPointValue = static_cast<uint32_t>(std::ceil(abs(lastErrorMicros) * 1e-6 * (1 << 16)));
          if (fixedPointValue > maxObservedDrift) {
            maxObservedDrift = fixedPointValue;
          }
        }
        
        if (debugIsOn) {
          Serial.println("Adjusted the clock by adding " + String(updateDelta) + "s (" + String(deltaMicros) + "us)");
          Serial.print(microsBetweenAdjustments);
          Serial.print("us passed between adjustments. Clock drift is ");
          Serial.print(clockDrift * 1000000.0);
          Serial.print("ppm. Last run was ");
          Serial.print(lastDrift * 1000000.0);
          Serial.println("ppm.");

          Serial.println("When adjusting for drift we had a cumulative error of " + String(lastErrorMicros) + "us.");
        }

        lastDrift = clockDrift;
      }

      lastAdjustmentMicros = microsAfterRTC;
      didSetGPSTime = true;
      numSyncs++;

      // whew that was hard work but fun, lets take a break and then do it all again
      vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
  }
}

void EthEvent(WiFiEvent_t event)
{

  const int rowOfDisplayToShowStatus = 1;
  const int rowOfDisplayToShowIP = 3;

  switch (event)
  {
  case ARDUINO_EVENT_ETH_START:
    ETH.setHostname("MasterClock");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    eth_connected = true;
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    ip = ETH.localIP().toString();
    eth_got_IP = true;
    eth_connected = true;
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    eth_connected = false;
    break;
  case ARDUINO_EVENT_ETH_STOP:
    eth_connected = false;
    break;
  default:
    break;
  }
}

void setupEthernet() {
  WiFi.onEvent(EthEvent);
  ETH.begin();

  while (!eth_got_IP)
    delay(1);
}

uint64_t getCurrentTimeInNTP64BitFormat() {
  const uint64_t secsBetween1900and1970 = 2208988800;

  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);

  // use our last observed clock drift to improve accuracy
  suseconds_t adjusted_usec = tv_now.tv_usec + getDriftAdjustmentMicros();
  time_t adjusted_sec = tv_now.tv_sec;
  if (adjusted_usec < 0) {
    adjusted_usec += 1000000;
    adjusted_sec -= 1;
  }
  
  // See https://tickelton.gitlab.io/articles/ntp-timestamps/
  uint64_t seconds = (uint64_t)adjusted_sec + secsBetween1900and1970;
  uint64_t fraction = (uint64_t)((double)(adjusted_usec + 1) * (double)(1LL << 32) * 1.0e-6);
  return (seconds << 32) | fraction;
}

// send NTP reply
void sendNTPpacket(IPAddress remoteIP, int remotePort) {

  // set the receive time to the current time
  uint64_t receiveTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  // Initialize values needed to form NTP request

  // LI: 0, Version: 4, Mode: 4 (server)
  // packetBuffer[0] = 0b00100100;
  // LI: 0, Version: 3, Mode: 4 (server)
  packetBuffer[0] = 0b00011100;

  // Stratum, or type of clock
  packetBuffer[1] = 0b00000001;

  // Polling Interval (2^x seconds)
  packetBuffer[2] = 6;

  // Peer Clock Precision: log2(sec) 
  packetBuffer[3] = precision;

  // bytes 4..7 = root delay
  // The value is a 32-bit signed fixed-point number in units of seconds,
  // with the fraction point between bits 15 and 16.
  // ES32 interrupts seem to take about 2us, which rounds to 0


  // bytes 8..11 = root dispersion
  // The value is a 32-bit signed fixed-point number in units of seconds,
  // with the fraction point between bits 15 and 16.
  // We use the maximum error observed once drift adjustment happens.
  packetBuffer[8] = (int)((maxObservedDrift >> 24) & 0xFF);
  packetBuffer[9] = (int)((maxObservedDrift >> 16) & 0xFF);
  packetBuffer[10] = (int)((maxObservedDrift >> 8) & 0xFF);
  packetBuffer[11] = (int)(maxObservedDrift & 0xFF);

  // time source (namestring)
  packetBuffer[12] = 71; // G
  packetBuffer[13] = 80; // P
  packetBuffer[14] = 83; // S
  packetBuffer[15] = 0;

   // get the current time and write it out as the reference time to bytes 16 to 23 of the response packet
  uint64_t referenceTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  packetBuffer[16] = (int)((referenceTime_uint64_t >> 56) & 0xFF);
  packetBuffer[17] = (int)((referenceTime_uint64_t >> 48) & 0xFF);
  packetBuffer[18] = (int)((referenceTime_uint64_t >> 40) & 0xFF);
  packetBuffer[19] = (int)((referenceTime_uint64_t >> 32) & 0xFF);
  packetBuffer[20] = (int)((referenceTime_uint64_t >> 24) & 0xFF);
  packetBuffer[21] = (int)((referenceTime_uint64_t >> 16) & 0xFF);
  packetBuffer[22] = (int)((referenceTime_uint64_t >> 8) & 0xFF);
  packetBuffer[23] = (int)(referenceTime_uint64_t & 0xFF);

  // copy transmit time from the NTP original request to bytes 24 to 31 of the response packet
  packetBuffer[24] = packetBuffer[40];
  packetBuffer[25] = packetBuffer[41];
  packetBuffer[26] = packetBuffer[42];
  packetBuffer[27] = packetBuffer[43];
  packetBuffer[28] = packetBuffer[44];
  packetBuffer[29] = packetBuffer[45];
  packetBuffer[30] = packetBuffer[46];
  packetBuffer[31] = packetBuffer[47];

  // write out the receive time (it was set above) to bytes 32 to 39 of the response packet
  packetBuffer[32] = (int)((receiveTime_uint64_t >> 56) & 0xFF);
  packetBuffer[33] = (int)((receiveTime_uint64_t >> 48) & 0xFF);
  packetBuffer[34] = (int)((receiveTime_uint64_t >> 40) & 0xFF);
  packetBuffer[35] = (int)((receiveTime_uint64_t >> 32) & 0xFF);
  packetBuffer[36] = (int)((receiveTime_uint64_t >> 24) & 0xFF);
  packetBuffer[37] = (int)((receiveTime_uint64_t >> 16) & 0xFF);
  packetBuffer[38] = (int)((receiveTime_uint64_t >> 8) & 0xFF);
  packetBuffer[39] = (int)(receiveTime_uint64_t & 0xFF);

  // get the current time and write it out as the transmit time to bytes 40 to 47 of the response packet
  uint64_t transmitTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  packetBuffer[40] = (int)((transmitTime_uint64_t >> 56) & 0xFF);
  packetBuffer[41] = (int)((transmitTime_uint64_t >> 48) & 0xFF);
  packetBuffer[42] = (int)((transmitTime_uint64_t >> 40) & 0xFF);
  packetBuffer[43] = (int)((transmitTime_uint64_t >> 32) & 0xFF);
  packetBuffer[44] = (int)((transmitTime_uint64_t >> 24) & 0xFF);
  packetBuffer[45] = (int)((transmitTime_uint64_t >> 16) & 0xFF);
  packetBuffer[46] = (int)((transmitTime_uint64_t >> 8) & 0xFF);
  packetBuffer[47] = (int)(transmitTime_uint64_t & 0xFF);

  /*
  uint64_t *pb64 = (uint64_t *)packetBuffer;
  // get the current time and write it out as the reference time to bytes 16 to 23 of the response packet
  uint64_t referenceTime_uint64_t = getCurrentTimeInNTP64BitFormat();
  pb64[2] = referenceTime_uint64_t;

  // Copy transmit time from the NTP original request (bytes 40 to 47) to response (bytes 24 to 31)
  pb64[3] = pb64[5];

  // Write the receive time (bytes 32 to 39)
  pb64[4] = receiveTime_uint64_t;

  // Write the transmit time (bytes 40 to 47)
  uint64_t transmitTime_uint64_t = getCurrentTimeInNTP64BitFormat();
  pb64[5] = transmitTime_uint64_t;
  */

  // send the reply
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void processNTPRequests() {

  unsigned long replyStartTime = micros();

  int packetSize = Udp.parsePacket();
  if (packetSize == NTP_PACKET_SIZE) // an NTP request has arrived
  {

    // store sender ip for later use
    IPAddress remoteIP = Udp.remoteIP();

    // read the data from the packet into the buffer for later use
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    // hold here if and while the date and time are being refreshed
    // when ok to proceed place a hold on using the mutex to prevent the date and time from being refreshed while the reply packet is being built
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
      // send NTP reply
      sendNTPpacket(remoteIP, Udp.remotePort());
      xSemaphoreGive(mutex);
    }

    // report query in serial monitor
    // note: unlike other serial monitor writes in this sketch, this particular write is on the critical path for processing NTP requests.
    // while it does not delay the response to an initial NTP request, if subsequent NTP requests are queued up to run directly afterward
    // this serial monitor write will delay responding to the queued request by approximately 1 milli second.
    if (debugIsOn) {
      Serial.println("Query from " + remoteIP.toString());
    }
  }
  else
  {
    if (packetSize > 0)
    {
      Udp.flush(); // not sure what this incoming packet is, but it is not an ntp request so get rid of it
      if (debugIsOn)
        Serial.println("Invalid request received on port " + String(NTP_PORT) + ", length =" + String(packetSize));
    }
  }
}

void ppsHandlerRising() {
  ppsRiseMicros = micros();
}

bool checkForValidGPSSentence() {
  gps.encode('$');
  unsigned long start = millis();
  while (millis() < start + 2000) {
    while (GPSDevice.available()) {
      if(gps.encode(GPSDevice.read())) {
        return true;
      }
    }
    delay(10);
  }
  return false;
}

void drawLineCentered(const char *line, uint8_t yOffset) {
  u8g2_uint_t width = u8g2.getUTF8Width(line);
  u8g2.drawStr((u8g2.getDisplayWidth()-width)/2, u8g2.getDisplayHeight()/2 + yOffset, line);
}

void displayCentered(const char *line1, const char *line2 = NULL) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvR10_tr);
  u8g2.setFontPosCenter();
  if (line2 == NULL) {
    drawLineCentered(line1, 0);
  } else {
    uint8_t offset = u8g2.getMaxCharHeight()/2;
    drawLineCentered(line1, -offset);
    drawLineCentered(line2, offset);
  }
  u8g2.sendBuffer();
}

void displayInfo(void *param) {
  // Thursday, May 30 2024
  // <big> 09:50:26 UTC </big>
  // Drift: 123us, max: 1234us
  // Sats: 12, Last fix: 1234.56s
  // uptime: 123 days 12:34:56

  while (true) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_originalsans_tr);
    uint8_t lineHeight = u8g2.getMaxCharHeight();
    u8g2.setFontPosTop();
    u8g2.drawStr(0, 0, rtc.getDate(true).c_str());
    u8g2.setFont(u8g2_font_helvB14_tr);
    u8g2.drawStr(0, lineHeight+1, (rtc.getTime() + " UTC").c_str());

    u8g2.setFont(u8g2_font_originalsans_tr);
    float maxDriftMicros = (static_cast<float>(maxObservedDrift) / (1 << 16)) * 1e6;
    u8g2.drawStr(0, 3*lineHeight-2,("Drift: " + String(int(ceil(lastErrorMicros))) + "us, max: " + String(int(ceil(maxDriftMicros))) + "us").c_str());
    u8g2.drawStr(0, 4*lineHeight-2, ("Sats: " + String(gps.satellites.value()) + ", Last fix: " + String(gps.time.age()/1000.0) + "s").c_str());
    u8g2.drawStr(0, 5*lineHeight-2, ("Uptime: " + getUptime()).c_str());
    u8g2.sendBuffer();

    struct timeval rtc_now;
    gettimeofday(&rtc_now, NULL);
    unsigned long nextFullSec = (1000000 - rtc_now.tv_usec) / 1000;
    vTaskDelay(nextFullSec / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Time Server starting");

  u8g2.begin();
  displayCentered("ESP32 Time Server");  
  
  // wifi and bluetooth aren't needed so turn them off
  Serial.println("Disabling wifi and bluetooth");
  WiFi.mode(WIFI_OFF);
  btStop();

  // interrupt handler for the pulse-per-second pin
  pinMode(pinGPSpps, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinGPSpps), ppsHandlerRising, RISING);

  Serial.print("Measuring time to read the clock (\"precision\")... ");
  unsigned long clockReadTime = 1000000;
  unsigned long before, after;
  for (int i = 0; i < 10; i++) {
    before = micros();
    getCurrentTimeInNTP64BitFormat();
    after = micros();
    if (clockReadTime > (after - before)) {
      clockReadTime = after - before;
    }
  }
  double seconds = static_cast<double>(clockReadTime) / 1e6;
  precision = static_cast<int8_t>(std::ceil(std::log2(seconds)));
  Serial.println(String(clockReadTime) + "us, precision = " + String(precision));

  Serial.println("Setting up networking");
  setupEthernet();

  // Web server and OTA
  server.on("/", []() {
    server.send(200, "text/html", "Time: " + rtc.getDateTime(true) + " UTC<br/>Last drift: " + String(lastErrorMicros) + "us, max observed drift (root dispersion): " + String((static_cast<float>(maxObservedDrift) / (1 << 16)) * 1e6) + "us<br/>Uptime: " + String(getUptime()) + "</br>Satellites: " + gps.satellites.value());
  });
  ElegantOTA.begin(&server); // serves /update
  server.begin();

  // create a mutex to be used to ensure an NTP request results are not impacted by the process that refreshes the time
  mutex = xSemaphoreCreateMutex();

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
    displayCentered("GPS failed");

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
  displayCentered("Acquiring GPS");
  while (!didSetGPSTime) {
    while (GPSDevice.available()) {
      gps.encode(GPSDevice.read());
      if(gps.satellites.isUpdated()) {
        displayCentered("Acquiring GPS", (String(gps.satellites.value()) + " sats").c_str());
      }
    }
    delay(10);
  }

  // set up task for updating the display
  xTaskCreate(
    displayInfo,
    "displayInfo",
    3000,
    NULL,
    5, // not as important
    &taskHandle1
  );

  Udp.begin(NTP_PORT);
  Serial.println("ESP32 Time Server setup complete - listening for NTP requests now");
}

void loop()
{
  while (GPSDevice.available()) {
    gps.encode(GPSDevice.read());
  }
  processNTPRequests();
  server.handleClient();
}