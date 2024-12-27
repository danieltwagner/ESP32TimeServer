#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <Arduino.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>

#include "state.h"

// SSD1306 I2C 128x64 OLED display
const int pinDisplaySda = 15; // Display SDA connected to GPIO15
const int pinDisplayScl = 14; // Display SCL connected to GPIO14

void drawCentered(const char *line);

void displayInit();
void displayAcquiring(TimeServerState state, TinyGPSSatellites &satellitesStats);
void displayInfo(TimeServer &timeServer, ESP32Time &rtc, TinyGPSPlus &gps);
String getUptime();

#endif // __DISPLAY_H
