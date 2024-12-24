#ifndef WEB_H
#define WEB_H

#include <vector>
#include <ElegantOTA.h>

void renderWeb(WebServer &server, TimeServer &timeServer, ESP32Time &rtc, TinyGPSPlus &gps, std::vector<String> &lastNmeaSentences);

#endif // WEB_H
