#ifndef __GPS_H
#define __GPS_H

#include <Arduino.h>
#include <vector>
#include <TinyGPSPlus.h>

// board: WT32-ETH01
// Neo6M GPS or similar
const int pinGPSrx = 5;  // GPIO 5 is used to receive from GPS (connect to GPS TX pin)
const int pinGPStx = 17; // GPIO 17 is used to transmit to GPS (connect to GPS RX pin)
const int pinGPSpps = 33; // GPIO 33 reads the PPS pulse  

#define GPSDevice Serial2
extern TinyGPSPlus gps;

extern std::vector<String> lastNmeaSentences;

bool checkForValidGPSSentence();
bool readGPS();

#endif // __GPS_H
