#include <U8g2lib.h>

#include "display.h"

// Display setup. We can afford to spend 1K on a full frame buffer.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, pinDisplayScl, pinDisplaySda);

void drawLineCentered(const char *line, int8_t yOffset) {
  u8g2.setFont(u8g2_font_helvR10_tr);
  u8g2.setFontPosCenter();
  u8g2_uint_t width = u8g2.getUTF8Width(line);
  u8g2.drawStr((u8g2.getDisplayWidth()-width)/2, u8g2.getDisplayHeight()/2 + yOffset, line);
}

void drawCentered(const char *line) {
  u8g2.clearBuffer();
  drawLineCentered(line, 0);
  u8g2.sendBuffer();
}

void displayAcquiring(TimeServerState state, TinyGPSSatellites &satellitesStats) {
    u8g2.clearBuffer();
    drawLineCentered(state == TimeServerState::WAITING_FOR_INITIAL_FIX ? "Acquiring GPS" : "Measuring Drift", -16);
    drawLineCentered((String(satellitesStats.nrSatsVisible()) + " visible, " + String(satellitesStats.nrSatsTracked()) + " tracked").c_str(), 0);
    drawLineCentered(getUptime().c_str(), 16);
    u8g2.sendBuffer();
}

void displayInit() {
    u8g2.begin();
    drawCentered("ESP32 Time Server");
}

void displayInfo(TimeServer &timeServer, ESP32Time &rtc, TinyGPSPlus &gps) {
    // Thursday, May 30 2024
    // <big> 09:50:26 UTC </big>
    // Drift: 123us, max: 1234us
    // Sats: 6/6/12, Fix: 1234s
    // uptime: 123 days 12:34:56
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_originalsans_tr);
    uint8_t lineHeight = u8g2.getMaxCharHeight();
    u8g2.setFontPosTop();
    u8g2.drawStr(0, 0, rtc.getDate(true).c_str());
    u8g2.setFont(u8g2_font_helvB14_tr);
    u8g2.drawStr(0, lineHeight+1, (rtc.getTime() + " UTC").c_str());

    u8g2.setFont(u8g2_font_originalsans_tr);
    float maxDriftMicros = (static_cast<float>(timeServer.rootDispersion) / (1 << 16)) * 1e6;
    u8g2.drawStr(0, 3*lineHeight-2,("Drift: " + String(int(ceil(timeServer.lastAdjustedErrorMicros))) + "us, max: " + String(int(ceil(maxDriftMicros))) + "us").c_str());
    u8g2.drawStr(0, 4*lineHeight-2, ("Sats: " + String(gps.satellites.value()) + "/" + String(gps.satellitesStats.nrSatsTracked()) + "/" + String(gps.satellitesStats.nrSatsVisible()) + ", Fix: " + String(gps.location.age()/1000) + "s").c_str());
    u8g2.drawStr(0, 5*lineHeight-2, ("Uptime: " + getUptime()).c_str());
    u8g2.sendBuffer();
}

String getUptime() {
    unsigned long ms = millis();

    unsigned long seconds = ms / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    unsigned long days = hours / 24;

    seconds %= 60;
    minutes %= 60;
    hours %= 24;

    char buffer[21];
    sprintf(buffer, "%lu days %02lu:%02lu:%02lu", days, hours, minutes, seconds);

    return String(buffer);
}
