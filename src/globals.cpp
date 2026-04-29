#include "globals.h"

Preferences preferences;
CRGB leds[1];
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

uint16_t motorSpeed = 0;
uint16_t minStartDuty = 0;
uint16_t currentSpeed = 0;
float accelInertia = 0.5f;
float decelInertia = 0.25f;

volatile uint32_t lastPulseMicros = 0;
volatile uint32_t latestDuration = 0;
volatile bool newPulseReceived = false;
volatile float currentRPM = 0;
float smoothedRPM = 0;
float displayRPM = 0;

uint32_t tuneTimer = 0;
float rpmAt50 = 0;
float systemGain = 0;
float timeConstant = 0;

float targetRPM = 0;
float Kp = 0.8f;
float Ki = 0.1f;
float integrator = 0;
float maxRPM = 2000.0f;
time_t lastCalibrationTime = 0;

float lastRPM = 0;
uint32_t lastCheckTime = 0;
float acceleration = 0;

volatile uint32_t debugTickCount = 0;
float smoothedPot = 0;
float lastTriggeredPot = 0;

bool emergencyStop = false;
bool calibrating = false;
int calibrateStep = 0;
bool testing = false;

uint8_t ledBrightness = 0;
volatile bool triggerFlash = false;
uint32_t flashDurationUS = 100;
int controlMode = 0;
DebugLevel currentDebugLevel = DebugLevel::INFO;
volatile bool firstIntervalSeeded = false;
