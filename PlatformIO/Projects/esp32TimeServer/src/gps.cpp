#include "gps.h"

TinyGPSPlus gps;
String nmeaCurrSentence;
std::vector<String> lastNmeaSentences;

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

// Read from GPS and accumulate sentences in a vector buffer
bool readGPS() {
  char c = GPSDevice.read();
  nmeaCurrSentence += c;
  if (gps.encode(c)) {
    if (nmeaCurrSentence.indexOf("GLL") >= 0) {
      // GLL seems to be the last of a set of sentences.
      // find any previous GLL sentence and remove it and anything before.
      std::vector<String>::iterator it = lastNmeaSentences.begin(), end = lastNmeaSentences.end();
      for (; it != end; ++it) {
        if ((*it).indexOf("GLL") > 0) {
          lastNmeaSentences.erase(lastNmeaSentences.begin(), it+1);
          break;
        }
      }
    }
    lastNmeaSentences.push_back(nmeaCurrSentence);
    nmeaCurrSentence = "";
    return true;
  }
  return false;
}
