#pragma once

#include <Arduino.h>

enum class StatusCode : uint8_t
{
    Startup,
    SensorsInit,
    Idle,
    DrivingForward,
    TurningLeft,
    TurningRight,
    CrashRecovery,
    ForwardStuck,
    ManualPause,
    Error
};

struct StatusPattern
{
    StatusCode code;
    const char *name;
    uint8_t flashes;
    uint16_t onMs;
    uint16_t offMs;
    bool repeat;
};

extern const StatusPattern STATUS_PATTERNS[];
extern const size_t STATUS_PATTERNS_COUNT;

void initStatusLED(uint8_t pin = LED_BUILTIN);
void setStatus(StatusCode code, bool retriggerPattern = true);
StatusCode getStatus();

void triggerStatusPattern(StatusCode code);
void flashStatusLED(uint8_t flashes, uint16_t onMs, uint16_t offMs);
bool isStatusLEDFlashing();
void updateStatusLED();
void delayWithStatusLEDUpdate(unsigned long durationMs);
void printStatusList(Stream &out);
