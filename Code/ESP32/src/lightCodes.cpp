#include "lightCodes.h"

namespace
{
    struct LedFlashState
    {
        bool active = false;
        bool repeat = false;
        uint8_t totalFlashes = 0;
        uint8_t flashesCompleted = 0;
        bool ledOn = false;
        unsigned long lastChangeMs = 0;
        uint16_t onDurationMs = 0;
        uint16_t offDurationMs = 0;
    };

    LedFlashState flashState;
    uint8_t ledPin = LED_BUILTIN;
    StatusCode currentStatus = StatusCode::Startup;

    const StatusPattern *findPattern(StatusCode code)
    {
        for (size_t i = 0; i < STATUS_PATTERNS_COUNT; ++i)
        {
            if (STATUS_PATTERNS[i].code == code)
            {
                return &STATUS_PATTERNS[i];
            }
        }
        return nullptr;
    }
}

// LIST OF THE STATUS PATTERNS
const StatusPattern STATUS_PATTERNS[] = {
    {StatusCode::Startup, "Startup", 5, 120, 120, false},
    {StatusCode::SensorsInit, "SensorsInit", 3, 200, 200, false},
    {StatusCode::Idle, "Idle", 1, 200, 600, true},
    {StatusCode::DrivingForward, "DrivingForward", 2, 90, 160, false},
    {StatusCode::TurningLeft, "TurningLeft", 3, 90, 90, false},
    {StatusCode::TurningRight, "TurningRight", 3, 90, 250, false},
    {StatusCode::CrashRecovery, "CrashRecovery", 5, 70, 70, false},
    {StatusCode::ForwardStuck, "ForwardStuck", 4, 100, 100, false},
    {StatusCode::ManualPause, "ManualPause", 2, 200, 600, true},
    {StatusCode::Error, "Error", 1, 1000, 1000, true},
};

const size_t STATUS_PATTERNS_COUNT = sizeof(STATUS_PATTERNS) / sizeof(STATUS_PATTERNS[0]);

void initStatusLED(uint8_t pin)
{
    ledPin = pin;
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    flashState = LedFlashState{};
}

void flashStatusLED(uint8_t flashes, uint16_t onMs, uint16_t offMs)
{
    if (flashes == 0)
    {
        return;
    }

    flashState.totalFlashes = flashes;
    flashState.onDurationMs = onMs;
    flashState.offDurationMs = offMs;
    flashState.flashesCompleted = 1;
    flashState.ledOn = true;
    flashState.active = true;
    flashState.repeat = false;
    flashState.lastChangeMs = millis();

    digitalWrite(ledPin, HIGH);
}

bool isStatusLEDFlashing()
{
    return flashState.active;
}

void triggerStatusPattern(StatusCode code)
{
    const StatusPattern *pattern = findPattern(code);
    if (pattern != nullptr && pattern->flashes > 0)
    {
        flashStatusLED(pattern->flashes, pattern->onMs, pattern->offMs);
        flashState.repeat = pattern->repeat;
    }
    else
    {
        flashState.active = false;
        flashState.repeat = false;
        digitalWrite(ledPin, LOW);
    }
}

void setStatus(StatusCode code, bool retriggerPattern)
{
    if (code == currentStatus && !retriggerPattern)
    {
        return;
    }

    currentStatus = code;

    if (retriggerPattern)
    {
        triggerStatusPattern(code);
    }
}

StatusCode getStatus()
{
    return currentStatus;
}

void updateStatusLED()
{
    if (!flashState.active)
    {
        return;
    }

    unsigned long now = millis();

    if (flashState.ledOn)
    {
        if (now - flashState.lastChangeMs >= flashState.onDurationMs)
        {
            digitalWrite(ledPin, LOW);
            flashState.ledOn = false;
            flashState.lastChangeMs = now;

            if (flashState.flashesCompleted >= flashState.totalFlashes)
            {
                if (flashState.repeat)
                {
                    flashState.flashesCompleted = 0;
                }
                else
                {
                    flashState.active = false;
                }
            }
        }
    }
    else
    {
        if (flashState.flashesCompleted >= flashState.totalFlashes && !flashState.repeat)
        {
            flashState.active = false;
            return;
        }

        if (now - flashState.lastChangeMs >= flashState.offDurationMs)
        {
            digitalWrite(ledPin, HIGH);
            flashState.ledOn = true;
            flashState.lastChangeMs = now;
            flashState.flashesCompleted++;
        }
    }
}

void delayWithStatusLEDUpdate(unsigned long durationMs)
{
    unsigned long start = millis();
    while (millis() - start < durationMs)
    {
        updateStatusLED();
        delay(1);
    }
}

void printStatusList(Stream &out)
{
    out.println(F("Status Codes:"));
    for (size_t i = 0; i < STATUS_PATTERNS_COUNT; ++i)
    {
        out.print(F("  ["));
        out.print(static_cast<int>(STATUS_PATTERNS[i].code));
        out.print(F("] "));
        out.print(STATUS_PATTERNS[i].name);
        out.print(F(" -> flashes:"));
        out.print(STATUS_PATTERNS[i].flashes);
        out.print(F(" on:"));
        out.print(STATUS_PATTERNS[i].onMs);
        out.print(F("ms off:"));
        out.print(STATUS_PATTERNS[i].offMs);
        out.print(F("ms repeat:"));
        out.println(STATUS_PATTERNS[i].repeat ? F("yes") : F("no"));
    }
}
