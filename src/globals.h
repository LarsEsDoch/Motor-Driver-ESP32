#pragma once

#include <Arduino.h>
#include <FastLED.h>
#include <Preferences.h>
#include <ESPAsyncWebServer.h>

enum class DebugLevel { NONE = 0, INFO = 1, DEBUG = 2, VERBOSE = 3 };

extern Preferences preferences;
extern CRGB leds[1];
extern AsyncWebServer server;
extern AsyncWebSocket ws;

extern uint16_t motorSpeed;
extern uint16_t minStartDuty;
extern uint16_t currentSpeed;
extern float accelInertia;
extern float decelInertia;

extern volatile uint32_t lastPulseMicros;
extern volatile uint32_t latestDuration;
extern volatile bool newPulseReceived;
extern volatile float currentRPM;
extern float smoothedRPM;
extern float displayRPM;

extern uint32_t tuneTimer;
extern float rpmAt50;
extern float rpmAt80;
extern float systemGain;
extern float timeConstant;

extern float targetRPM;
extern float Kp;
extern float Ki;
extern float integrator;
extern float maxRPM;
extern time_t lastCalibrationTime;

extern float lastRPM;
extern uint32_t lastCheckTime;
extern float acceleration;

extern volatile uint32_t debugTickCount;
extern float smoothedPot;
extern float lastTriggeredPot;

extern bool emergencyStop;
extern bool calibrating;
extern int calibrateStep;
extern bool testing;

extern uint8_t ledBrightness;
extern volatile bool triggerFlash;
extern uint32_t flashDurationUS;
extern int controlMode;
extern DebugLevel currentDebugLevel;
extern volatile bool firstIntervalSeeded;

extern bool webUIControl;
extern float potAtWebUITakeover;