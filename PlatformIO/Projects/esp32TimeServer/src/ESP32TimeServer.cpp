// Rob Latour, 2023
// License: MIT
// https://github.com/roblatour
//
// This program's key setting can be viewed/updated in the file:
//      ESP32TimeServerKeySettings.h
//
// https://time.is may be used a reference point to confirm your computer's date/time accuracy
//

// board: WT32-ETH01 with Neo6M GPS
// GPS RX pin connected to GPIO17 on the ESP32
// GPS TX pin connected to GPIO5 on the ESP32
// GPS PPS pin connected to GPIO33 on the ESP32

#include <ETH.h>
#include <Timezone.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>
#include "ESP32TimeServerKeySettings.h"

// ESP32Time real time clock
ESP32Time rtc(0);

// Ethernet
bool eth_connected = false;
bool eth_got_IP = false;
String ip = "";

// GPS
TinyGPSPlus gps;
#define GPSDevice Serial2
volatile bool ppsFlag; // GPS one-pulse-per-second flag

// TimeZone
TimeChangeRule *tcr;

// NTP port and packet buffer
#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];
WiFiUDP Udp;

// Constants and global variables
const unsigned long oneSecond_inMilliseconds = 1000;                              // one second in milliseconds
const unsigned long oneMinute_inMilliseconds = 60 * oneSecond_inMilliseconds;     // one minute in milliseconds
const long oneSecond_inMicroseconds_L = 1000000;                                  // one second in microseconds (signed long)
const double oneSecond_inMicroseconds_D = 1000000.0;                              // one second in microseconds (double)
                                                                                  //
const unsigned long periodicTimeRefreshPeriod = oneMinute_inMilliseconds;         // how often the system's real time clock is refreshed with GPS data
const time_t safeguardThresholdInSeconds = 1;                                     // used to ensure a GPS time refresh is only performed if the difference between the old and new times is this many seconds or less

volatile bool didSetGPSTime;  // have we ever set time based on GPS?
volatile int8_t precision;

//
SemaphoreHandle_t mutex;         // used to ensure an NTP request results are not impacted by the process that refreshes the time
                                 //
TaskHandle_t taskHandle0 = NULL; // task handle for updating the display
TaskHandle_t taskHandle1 = NULL; // task handle for setting/refreshing the time

//**************************************************************************************************************************

void display(uint8_t row, String msg, bool writeToSerialMonitor = true)
{

  // String displayLine = msg + fullyBlankLine; // adding the fully blank line here clears the remnants of previously displayed information
  // displayLine = displayLine.substring(0, 20);

  // lcd.setCursor(0, row);
  // lcd.print(displayLine);

  // if (debugIsOn && writeToSerialMonitor)
  //   Serial.println(displayLine);
  Serial.println(msg);
}

void GetAdjustedDateAndTimeStrings(time_t UTC_Time, String &dateString, String &timeString)
{

  // adjust utc time to local time
  time_t now_Local_Time = myTZ.toLocal(UTC_Time, &tcr);

  // format dateLine

  dateString = String(year(now_Local_Time));

  dateString.concat("-");

  if (month(now_Local_Time) < 10)
    dateString.concat("0");

  dateString.concat(String(month(now_Local_Time)));

  dateString.concat("-");

  if (day(now_Local_Time) < 10)
    dateString.concat("0");

  dateString.concat(String(day(now_Local_Time)));

  // format timeLine

  timeString = String(hourFormat12(now_Local_Time));

  timeString.concat(":");

  if (minute(now_Local_Time) < 10)
    timeString.concat("0");

  timeString.concat(String(minute(now_Local_Time)));

  timeString.concat(":");

  if (second(now_Local_Time) < 10)
    timeString.concat("0");

  timeString.concat(String(second(now_Local_Time)));

  if (isAM(now_Local_Time))
    timeString.concat(" AM ");
  else
    timeString.concat(" PM ");
  
  if (displayTimeZone)
    timeString.concat(tcr -> abbrev);
};

String GetUpTime()
{

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

  sprintf(buffer, "%d %02d:%02d:%02d", numberOfDays, numberOfHours, numberOfMinutes, numberOfSeconds);

  returnValue = String(buffer);
  return returnValue;
}

void setDateAndTimeFromGPS(void *parameter) {
  static unsigned long lastAdjustmentMicros = 0;
  static double lastPpmDrift = 0;

  /// used below to ensure a GPS time refresh if is only performed if the difference between the old and new times is reasonable for the periodicTimeRefreshPeriod
  const time_t safeguardThresholdHigh = safeguardThresholdInSeconds;
  const time_t safeguardThresholdLow = -1 * safeguardThresholdInSeconds;

  time_t candidateDateAndTime;

  if (debugIsOn)
    Serial.println("Start setDateAndTimeFromGPS task");

  while (true) {
    // wait for the ppsFlag to be raised at the start of the 1st second
    ppsFlag = false;
    while (!ppsFlag) {
      delay(10);
    }

    if (gps.date.isValid() && gps.time.isValid()) {
      struct tm wt;
      wt.tm_year = gps.date.year();
      wt.tm_mon = gps.date.month();
      wt.tm_mday = gps.date.day();
      wt.tm_hour = gps.time.hour();
      wt.tm_min = gps.time.minute();
      wt.tm_sec = gps.time.second();

      wt.tm_year -= 1900;  // adjust year (see you again in 2036)
      wt.tm_mon -= 1;      // adjust month (January is month 0)

      // by the time the next pps pulse comes around it will be one second later
      candidateDateAndTime = mktime(&wt) + 1;

      if (debugIsOn) {
        Serial.println("Received date and time " + String(wt.tm_year) + " " + String(wt.tm_mon) + " " + String(wt.tm_mday) + " " + String(wt.tm_hour) + " " + String(wt.tm_min) + " " + String(wt.tm_sec));
      }

      // give some time to ensure the PPS pin is reset
      vTaskDelay(200 / portTICK_PERIOD_MS);

      // wait for the PPS flag to be raised (signifying the true start of the candidate time)
      ppsFlag = false;
      while (!ppsFlag) {
        vTaskDelay(portTICK_PERIOD_MS/10);
      }

      unsigned long pegProcessingAdjustmentStartTime = micros();
      struct timeval tv_now;
      gettimeofday(&tv_now, NULL);
      suseconds_t rtcMicrosAfterPPSFlag = tv_now.tv_usec;

      // at this point:
      // apply a sanity check; the current rtc time and the candidate time just taken from the gps readings which will be used to refresh the current rtc should be within a second of each other (safeguardThresholdInSeconds)
      // if the sanity check fails, do not set the time and raise a Safeguard flag which be used to update the display to show the user the latest time refresh failed
      // if the sanity check passes, proceed with refreshing the time and if the Safeguard flag been previously been raised then lower it

      bool SanityCheckPassed;
      time_t updateDelta;

      if (!didSetGPSTime) {
        SanityCheckPassed = true;
      } else {
        time_t currentRTC = tv_now.tv_sec;
        updateDelta = candidateDateAndTime - currentRTC;
        SanityCheckPassed = (((updateDelta >= safeguardThresholdLow) && (updateDelta <= safeguardThresholdHigh)));
      }

      if (SanityCheckPassed)
      {

        // place a hold on (the date and time) so if an NTP request is underway in the fraction of a second this code will take, the time and date values don't change mid way through that request.
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
        {

          // set the date and time
          unsigned long pegProcessingAdjustmentEndTime = micros();
          unsigned long ProcessingAdjustment = pegProcessingAdjustmentEndTime - pegProcessingAdjustmentStartTime;

          // set the real time clock
          rtc.setTime((unsigned long)candidateDateAndTime, (int)ProcessingAdjustment);

          // release the hold
          xSemaphoreGive(mutex);

          if (debugIsOn) {
            Serial.print("Date and time set to ");
            String ws = rtc.getDateTime(true);
            ws.trim();
            Serial.println(ws + " (UTC)");
          }

          if (didSetGPSTime) {
            // calculate drift
            unsigned long microsBetweenAdjustments = pegProcessingAdjustmentStartTime - lastAdjustmentMicros;
            time_t deltaMicros = (updateDelta * 1000000) - rtcMicrosAfterPPSFlag;
            double ppmDrift = ((double)deltaMicros / (double)microsBetweenAdjustments) * 1000000.0;

            if (debugIsOn) {
              Serial.println("rtcMicrosAfterPPSFlag = " + String(rtcMicrosAfterPPSFlag));
              Serial.println("We adjusted the clock by " + String(updateDelta) + "s (" + String(deltaMicros) + "us)");
              Serial.print(microsBetweenAdjustments);
              Serial.print("us passed between adjustments. Clock drift is ");
              Serial.print(ppmDrift);
              Serial.print("ppm. Last run was ");
              Serial.println(lastPpmDrift);
            }

            lastPpmDrift = ppmDrift;
          }

          lastAdjustmentMicros = pegProcessingAdjustmentStartTime;
          didSetGPSTime = true;

          // whew that was hard work but fun, lets take a break and then do it all again
          vTaskDelay(periodicTimeRefreshPeriod / portTICK_PERIOD_MS);
        }
        else
        {
          if (debugIsOn)
          {
            Serial.println("Could not refresh the time as a NTP request was underway");
            Serial.println("Will try again");
          }
        }
      }
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
    display(rowOfDisplayToShowStatus, "Ethernet started");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    display(rowOfDisplayToShowStatus, "Ethernet connected");
    eth_connected = true;
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    ip = ETH.localIP().toString();
    display(rowOfDisplayToShowIP, ip);
    eth_got_IP = true;
    eth_connected = true;
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    display(rowOfDisplayToShowStatus, "Ethernet disconnect");
    eth_connected = false;
    break;
  case ARDUINO_EVENT_ETH_STOP:
    display(rowOfDisplayToShowStatus, "Ethernet stopped");
    eth_connected = false;
    break;
  default:
    break;
  }
}

void setupEthernet()
{

  WiFi.onEvent(EthEvent);
  ETH.begin();

  while (!eth_got_IP)
    delay(1);
}

void startUDPSever()
{

  Udp.begin(NTP_PORT);
}

uint64_t getCurrentTimeInNTP64BitFormat()
{
  const uint64_t secsBetween1900and1970 = 2208988800;

  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  
  uint64_t seconds = (uint64_t)tv_now.tv_sec + secsBetween1900and1970;
  uint64_t fraction = (uint64_t)((double)(tv_now.tv_usec + 1) * (double)(1LL << 32) * 1.0e-6);
  return (seconds << 32) | fraction;
}

// send NTP reply
void sendNTPpacket(IPAddress remoteIP, int remotePort)
{

  // set the receive time to the current time
  uint64_t receiveTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  // Initialize values needed to form NTP request

  // LI: 0, Version: 4, Mode: 4 (server)
  // packetBuffer[0] = 0b00100100;
  // LI: 0, Version: 3, Mode: 4 (server)
  packetBuffer[0] = 0b00011100;

  // Stratum, or type of clock
  packetBuffer[1] = 0b00000001;

  // Polling Interval
  packetBuffer[2] = 4;

  // Peer Clock Precision
  // log2(sec)
  // 0xF6 <--> -10 <--> 0.0009765625 s
  // 0xF7 <--> -9 <--> 0.001953125 s
  // 0xF8 <--> -8 <--> 0.00390625 s
  // 0xF9 <--> -7 <--> 0.0078125 s
  // 0xFA <--> -6 <--> 0.0156250 s
  // 0xFB <--> -5 <--> 0.0312500 s
  packetBuffer[3] = precision;

  // 8 bytes for Root Delay & Root Dispersion
  packetBuffer[11] = 0x50;

  // time source (namestring)
  packetBuffer[12] = 71; // G
  packetBuffer[13] = 80; // P
  packetBuffer[14] = 83; // S

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

void processNTPRequests()
{

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
    };

    // report query in serial monitor
    // note: unlike other serial monitor writes in this sketch, this particular write is on the critical path for processing NTP requests.
    // while it does not delay the response to an initial NTP request, if subsequent NTP requests are queued up to run directly afterward
    // this serial monitor write will delay responding to the queued request by approximately 1 milli second.
    if (debugIsOn)
    {

      String dateLine = "";
      String timeLine = "";
      GetAdjustedDateAndTimeStrings(rtc.getEpoch(), dateLine, timeLine);
      String updatemessage = "Query from " + remoteIP.toString() + " on " + dateLine + " at " + timeLine;
      Serial.println(updatemessage);
    };
  }
  else
  {
    if (packetSize > 0)
    {
      Udp.flush(); // not sure what this incoming packet is, but it is not an ntp request so get rid of it
      if (debugIsOn)
        Serial.println("Invalid request received on port " + String(NTP_PORT) + ", length =" + String(packetSize));
    };
  };
}

void ppsHandlerRising()
{                 // PPS interrupt handler
  ppsFlag = true; // raise the flag that signals the start of the next second
}

void setup() {
  Serial.begin(SerialMonitorSpeed);
  Serial.println("ESP32 Time Server starting");

  // wifi and bluetooth aren't needed so turn them off
  Serial.println("Disabling wifi and bluetooth");
  WiFi.mode(WIFI_OFF);
  btStop();

  // interrupt handler for the pulse-per-second pin
  pinMode(GPSPinPPS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPSPinPPS), ppsHandlerRising, RISING);

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

  // create a mutex to be used to ensure an NTP request results are not impacted by the process that refreshes the time
  mutex = xSemaphoreCreateMutex();

  Serial.println("Opening GPS with 9600 baud");
  GPSDevice.begin(9600, SERIAL_8N1, GPSPinRX, GPSPinTX);

  // TODO: Switch to 115k2 baud?

  // set up periodic date/time task
  xTaskCreatePinnedToCore(
    setDateAndTimeFromGPS,
    "Set Date and Time from GPS",
    3000,
    NULL,
    20, // task priority must be reasonably high or the queues from which the gps data is drawn will not be adequately replenished
    &taskHandle1,
    0 // core 1 handles setup() and loop()
  );

  // wait until the time is actually set
  Serial.println("Waiting for GPS time...");
  while (!didSetGPSTime) {
    while (GPSDevice.available()) {
      gps.encode(GPSDevice.read());
    }
    delay(10);
  }

  Serial.println("Setting up networking");
  setupEthernet();
  startUDPSever();

  Serial.println("ESP32 Time Server setup complete - listening for NTP requests now");
}

void loop()
{
  while (GPSDevice.available()) {
    if(gps.encode(GPSDevice.read())) {
    }
  }
  processNTPRequests();
}