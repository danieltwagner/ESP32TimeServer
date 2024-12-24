#include <ETH.h>

#include "network.h"

bool eth_connected = false;
bool eth_got_IP = false;
String ip = "";

void EthEvent(WiFiEvent_t event) {

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
    delay(10);

  Serial.println("Connected to Ethernet, IP: " + ip);
}
