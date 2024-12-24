#include <ETH.h>

#include "ntp.h"
#include "state.h"

// NTP port and packet buffer
#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];
WiFiUDP Udp;

uint64_t getCurrentTimeInNTP64BitFormat(TimeServer timeServer) {
  const uint64_t secsBetween1900and1970 = 2208988800;

  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);

  // use our last observed clock drift to improve accuracy
  suseconds_t adjusted_usec = tv_now.tv_usec + timeServer.getDriftAdjustmentMicros();
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
void sendNTPpacket(IPAddress remoteIP, int remotePort, TimeServer timeServer) {

  // set the receive time to the current time
  uint64_t receiveTime_uint64_t = getCurrentTimeInNTP64BitFormat(timeServer);

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
  packetBuffer[3] = timeServer.precision;

  // bytes 4..7 = root delay
  // The value is a 32-bit signed fixed-point number in units of seconds,
  // with the fraction point between bits 15 and 16.
  // ES32 interrupts seem to take about 2us, which rounds to 0


  // bytes 8..11 = root dispersion
  // The value is a 32-bit signed fixed-point number in units of seconds,
  // with the fraction point between bits 15 and 16.
  // We use the maximum error observed once drift adjustment happens.
  packetBuffer[8] = (int)((timeServer.maxObservedDrift >> 24) & 0xFF);
  packetBuffer[9] = (int)((timeServer.maxObservedDrift >> 16) & 0xFF);
  packetBuffer[10] = (int)((timeServer.maxObservedDrift >> 8) & 0xFF);
  packetBuffer[11] = (int)(timeServer.maxObservedDrift & 0xFF);

  // time source (namestring)
  packetBuffer[12] = 71; // G
  packetBuffer[13] = 80; // P
  packetBuffer[14] = 83; // S
  packetBuffer[15] = 0;

   // get the current time and write it out as the reference time to bytes 16 to 23 of the response packet
  uint64_t referenceTime_uint64_t = getCurrentTimeInNTP64BitFormat(timeServer);

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
  uint64_t transmitTime_uint64_t = getCurrentTimeInNTP64BitFormat(timeServer);

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

void ntpStartListening() {
  Udp.begin(NTP_PORT);
}

void processNTPRequests(TimeServer timeServer) {

  int packetSize = Udp.parsePacket();
  if (packetSize == NTP_PACKET_SIZE) // an NTP request has arrived
  {

    // store sender ip for later use
    IPAddress remoteIP = Udp.remoteIP();

    // read the data from the packet into the buffer for later use
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    // hold here if and while the date and time are being refreshed
    // when ok to proceed place a hold on using the mutex to prevent the date and time from being refreshed while the reply packet is being built
    if (xSemaphoreTake(timeServer.mutex, portMAX_DELAY) == pdTRUE)
    {
      // send NTP reply
      sendNTPpacket(remoteIP, Udp.remotePort(), timeServer);
      xSemaphoreGive(timeServer.mutex);
    }

    // report query in serial monitor
    // note: unlike other serial monitor writes in this sketch, this particular write is on the critical path for processing NTP requests.
    // while it does not delay the response to an initial NTP request, if subsequent NTP requests are queued up to run directly afterward
    // this serial monitor write will delay responding to the queued request by approximately 1 milli second.
    if (timeServer.debugIsOn) {
      Serial.println("Query from " + remoteIP.toString());
    }
  }
  else
  {
    if (packetSize > 0)
    {
      Udp.flush(); // not sure what this incoming packet is, but it is not an ntp request so get rid of it
      if (timeServer.debugIsOn)
        Serial.println("Invalid request received on port " + String(NTP_PORT) + ", length =" + String(packetSize));
    }
  }
}

int64_t measureClockReadDurationMicros(TimeServer timeServer) {
  int64_t clockReadTime = 1000000;
  int64_t before, delta;
  for (int i = 0; i < 10; i++) {
    before = esp_timer_get_time();
    getCurrentTimeInNTP64BitFormat(timeServer);
    delta = esp_timer_get_time() - before;
    if (clockReadTime > delta) {
      clockReadTime = delta;
    }
  }
  return clockReadTime;
}
