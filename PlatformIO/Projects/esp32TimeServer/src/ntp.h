#ifndef NTP_H
#define NTP_H

#include "state.h"

void ntpStartListening();
void processNTPRequests(TimeServer timeServer);
int64_t measureClockReadDurationMicros(TimeServer timeServer);

#endif // NTP_H
