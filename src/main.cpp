#include <Arduino.h>
#include <FastLED.h>
#include <Preferences.h>

Preferences preferences;

#define STATUS_LED_PIN 48

CRGB leds[1];

#define POT_PIN 15
#define CALIBRATE_BUTTON_PIN 16
#define MODE_BUTTON_PIN 17
#define EMERGENCY_STOP_BUTTON_PIN 18

#define MOTOR_PIN 14

#define SPEAKER_PIN 4

#define SENSOR_PIN 12

const int speakerChannel = 0;
const int freqSpeaker = 2000;
const int resolutionSpeaker = 8;

uint16_t motorSpeed = 0;

uint16_t minStartDuty = 0;

const int motorChannel = 2;
const int freqMotor = 8000;
const int resolutionMotor = 12;

uint16_t currentSpeed = 0;
float accelInertia = 0.5f;
float decelInertia = 0.25f;

volatile uint32_t lastPulseMicros = 0;
volatile uint32_t latestDuration = 0;
volatile bool newPulseReceived = false;

volatile float currentRPM = 0;
float smoothedRPM = 0;

uint32_t tuneTimer = 0;
float rpmAt50 = 0;
float systemGain = 0;
float timeConstant = 0;

float targetRPM = 0;
float Kp = 0.8f;
float Ki = 0.1f;
float integrator = 0;
float maxRPM = 2000.0f;

volatile uint32_t debugTickCount = 0;

const float ADC_MIN = 50.0f;
const float ADC_MAX = 4046.0f;
const float ADC_TOLERANCE = 40.0f;

static float smoothedPot = 0;

static float lastTriggeredPot = 0;

bool emergencyStop = false;
bool calibrating = false;
int calibrateStep = 0;
bool testing = false;

uint8_t ledBrightness = 0;


int controlMode = 0;

enum class DebugLevel {NONE = 0, INFO = 1, DEBUG = 2, VERBOSE = 3};
DebugLevel currentDebugLevel = DebugLevel::INFO;

volatile bool firstIntervalSeeded = false;

void IRAM_ATTR pulseISR() {
    uint32_t now = micros();

    if (lastPulseMicros == 0) {
        lastPulseMicros = now;
        return;
    }

    uint32_t duration = now - lastPulseMicros;
    uint32_t minDuration = (latestDuration > 0) ? (latestDuration / 2) : 5000;
    if (minDuration < 5000) minDuration = 5000;

    if (duration > minDuration) {
        lastPulseMicros = now;

        if (!firstIntervalSeeded) {
            latestDuration = duration;
            firstIntervalSeeded = true;
            return;
        }

        latestDuration = duration;
        newPulseReceived = true;
        debugTickCount++;
    }
}

void setup() {
    Serial.begin(921600);
    delay(2000);

    pinMode(SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), pulseISR, RISING);

    preferences.begin("motor-settings", false);

    minStartDuty = preferences.getUShort("minDuty", 0);
    if (minStartDuty != 0) Serial.printf("\nReused min start duty of %hu out of preferences.\n", minStartDuty);
    maxRPM = preferences.getFloat("maxRPM", 2000);
    if (maxRPM != 2000) Serial.printf("\nReused Max RPM of %f out of preferences.\n", maxRPM);
    preferences.end();

    pinMode(STATUS_LED_PIN, OUTPUT);

    FastLED.addLeds<WS2812B, STATUS_LED_PIN, GRB>(leds, 1);
    FastLED.setBrightness(127);

    pinMode(EMERGENCY_STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);

    ledcSetup(speakerChannel, freqSpeaker, resolutionSpeaker);
    ledcAttachPin(SPEAKER_PIN, speakerChannel);

    ledcSetup(motorChannel, freqMotor, resolutionMotor);
    ledcAttachPin(MOTOR_PIN, motorChannel);

    Serial.println("System ready!\n");
}

void playClick(int freq, int duration) {
    ledcWriteTone(speakerChannel, freq);
    delay(duration);
    ledcWriteTone(speakerChannel, 0);
}

void test() {
    Serial.println("Starting Frequency Sweep (200Hz - 10000Hz)...");

    for (int freq = 100; freq <= 20000; freq += 100) {
        ledcWriteTone(speakerChannel, freq);
        Serial.printf("Frequency: %i\n", freq);
        delay(40);
    }

    ledcWriteTone(speakerChannel, 0);
    Serial.println("Sweep finished.");

    Serial.println("Motor starts up...");
    for (int dutyCycle = 0; dutyCycle <= 4095; dutyCycle += 8) {
        ledcWrite(motorChannel, dutyCycle);
        Serial.printf("Speed: %i\n", dutyCycle);
        delay(20);
    }

    Serial.println("Maximum speed reached.");
    delay(1000);
    Serial.println("3");
    delay(1000);
    Serial.println("2");
    delay(1000);
    Serial.println("1");
    delay(1000);
    Serial.println("0");
    delay(1000);

    Serial.println("Motor slows down...");
    for (int dutyCycle = 4095; dutyCycle >= 0; dutyCycle -= 8) {
        ledcWrite(motorChannel, dutyCycle);
        Serial.printf("Speed: %i\n", dutyCycle);
        delay(20);
    }

    Serial.println("Start LED rainbow test.");
    FastLED.setBrightness(255);
    for (int hue = 0; hue < 256; hue++) {
        leds[0] = CHSV(hue, 255, 255);
        FastLED.show();
        Serial.printf("Hue: %i\n", hue);

        delay(15);
    }
    leds[0] = CRGB::White;
    FastLED.setBrightness(ledBrightness);
    FastLED.show();

    Serial.println("Finished test.");
}

void calibrate() {
    if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) {
        switch (calibrateStep) {
            case 0:
                minStartDuty = motorSpeed;
                calibrateStep = 1;

                Serial.printf("Speed calibration finished. Set min duty cycle to %hhu\n", minStartDuty);
                Serial.println("Continuing with motor max speed");
                playClick(1500, 100);
                break;
        }
        delay(300);
    }

    switch (calibrateStep) {
        case 1: {
            ledcWrite(motorChannel, 4095);
                Serial.println("calibrate case 2 ledc 4095");

            uint32_t now = millis();
            if (now - lastCheckTime >= 100) {
                float deltaTime = (now - lastCheckTime) / 1000.0f;

                acceleration = (smoothedRPM - lastRPM) / deltaTime;

                if (abs(acceleration) < 5.0f && smoothedRPM > 500) {
                    maxRPM = smoothedRPM;
                    Serial.printf("Set maximum RPM to: %f", maxRPM);
                    playClick(1800, 200);
                    calibrateStep = 2;
                }

                lastRPM = smoothedRPM;
                lastCheckTime = now;
            }
            break;
        }
        case 2:
        case 3:
            Serial.println("Calibration finished.");

            preferences.begin("motor-settings", false);
            preferences.putUShort("minDuty", minStartDuty);
            preferences.putFloat("maxRPM", maxRPM);
            preferences.end();

            Serial.println("Saved settings to flash storage.");
            playClick(1000, 100);
            calibrating = false;
            calibrateStep = 0;
            break;
    }
}

void readPot() {
    int raw = analogRead(POT_PIN);
    smoothedPot = (smoothedPot * 0.9f) + (raw * 0.1f);
}

void adjustSpeed() {
    if (abs(smoothedPot - lastTriggeredPot) > ADC_TOLERANCE) {
        lastTriggeredPot = smoothedPot;
        playClick(150, 10);

        if (smoothedPot < (ADC_MIN + ADC_TOLERANCE)) {
            motorSpeed = 0;
            ledBrightness = 0;
        } else {
            float constrainedPot = constrain(smoothedPot, ADC_MIN, ADC_MAX);

            float percent = (constrainedPot - ADC_MIN) / (ADC_MAX - ADC_MIN);

            motorSpeed = static_cast<uint16_t>((percent * (4095.0f - minStartDuty)) + minStartDuty);

            ledBrightness = static_cast<uint8_t>(percent * 255.0f);
        }

        ledcWrite(motorChannel, motorSpeed);
        if (DebugLevel::VERBOSE <= currentDebugLevel) {
            Serial.printf("adjust speed %hu\n", motorSpeed);
        }
    }
}

void controlRPM() {
    float constrainedPot = constrain(smoothedPot, ADC_MIN, ADC_MAX);
    float percent = (constrainedPot - ADC_MIN) / (ADC_MAX - ADC_MIN);
    targetRPM = percent * maxRPM;

    if (targetRPM < 100) {
        integrator = 0;
        currentSpeed = 0;
        ledcWrite(motorChannel, 0);
        return;
    }

    if (smoothedRPM < 50 && targetRPM > 100) {
        currentSpeed = minStartDuty;
        ledcWrite(motorChannel, (uint16_t)currentSpeed);
        if (DebugLevel::VERBOSE <= currentDebugLevel) {
            Serial.printf("control RPM if %hu\n", currentSpeed);
        }
        return;
    }

    float error = targetRPM - smoothedRPM;

    integrator += error;
    integrator = constrain(integrator, -INTEGRATOR_CLAMP, INTEGRATOR_CLAMP);

    float output = (Kp * error) + (Ki * integrator);
    currentSpeed += output;
    currentSpeed = constrain(currentSpeed, minStartDuty, 4095);

    ledcWrite(motorChannel, (uint16_t)currentSpeed);
    if (DebugLevel::VERBOSE <= currentDebugLevel) {
        Serial.printf("control RPM %hu\n", currentSpeed);
    }
}

void loop() {
    if (newPulseReceived) {
        currentRPM = 60000000.0f / latestDuration;
        newPulseReceived = false;
        smoothedRPM = (smoothedRPM * 0.8f) + (currentRPM * 0.2f);
    }

    if (micros() - lastPulseMicros > 1000000) {
        currentRPM = 0;
    }

    static uint32_t lastSeenTick = 0;

    if (debugTickCount != lastSeenTick && currentDebugLevel <= DebugLevel::INFO) {
        Serial.printf("Rotation: %u | RPM: %.2f | Smoothed RPM: %.2f | Target RPM: %.2f | Motor speed: %hu | Current speed: %.2hu | ADC Pot: %.2f \n", debugTickCount, currentRPM, smoothedRPM, targetRPM, motorSpeed, currentSpeed, smoothedPot);
        lastSeenTick = debugTickCount;
    }

    if (digitalRead(EMERGENCY_STOP_BUTTON_PIN) == LOW) {
        emergencyStop = !emergencyStop;
        testing = false;
        calibrating = false;
        calibrateStep = 0;
        ledcWriteTone(speakerChannel, 0);
        delay(500);
    }

    static uint32_t modeButtonPressStartTime = 0;
    static bool modeButtonWasPressed = false;
    bool modeActionExecuted = false;

    if (digitalRead(MODE_BUTTON_PIN) == LOW) {
        if (!modeButtonWasPressed) {
            modeButtonPressStartTime = millis();
            modeButtonWasPressed = true;
            modeActionExecuted = false;
        } else {
            if (!modeActionExecuted && millis() - modeButtonPressStartTime >= 3000) {
            }
        }
    } else {
        if (modeButtonWasPressed) {
            if (!modeActionExecuted && millis() - modeButtonPressStartTime < 3000) {
                controlMode = (controlMode == 0) ? 1 : 0;
                Serial.printf("Control mode got set to %s\n", controlMode == 1 ? "rpm" : "voltage (duty cycle)");
                playClick(2000, 100);
            }
            modeButtonWasPressed = false;
            modeActionExecuted = false;
        }
    }


    if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW && !calibrating) {
        if (!buttonWasPressed) {
            buttonPressStartTime = millis();
            buttonWasPressed = true;
        } else {
            if (millis() - buttonPressStartTime >= 3000) {
                testing = !testing;

                playClick(2000, 100);

                buttonWasPressed = false;
                Serial.printf("Test mode %s\n", testing ? "activated" : "deactivated");
            }
        }
    } else {
        if (buttonWasPressed) {
            if (millis() - buttonPressStartTime < 3000) {
                calibrating = true;
                minStartDuty = 0;

                playClick(2000, 100);

                Serial.println("Calibration started.");
            }
            buttonWasPressed = false;
        }
    }

    if (emergencyStop) {
        motorSpeed = 0;
        ledcWrite(motorChannel, 0);

        uint8_t pulse = beatsin8(40, 50, 255);
        leds[0] = CRGB::Red;
        FastLED.setBrightness(pulse);
        FastLED.show();

        uint16_t sirenFreq = beatsin16(40, 600, 1200);
        ledcWriteTone(speakerChannel, sirenFreq);

        delay(10);
        return;
    }

    if (calibrating) {
        calibrate();
    }

    if (testing && !calibrating) {
        test();
        return;
    }

    readPot();
    if (!calibrating || calibrateStep == 1) {
        if (controlMode == 0) {
            adjustSpeed();
        } else if (controlMode == 1) {
            controlRPM();
        }
    }

    FastLED.setBrightness(ledBrightness);
    uint8_t currentHue = map(motorSpeed, minStartDuty, 4095, 160, 0);
    leds[0] = CHSV(currentHue, 255, 255);
    FastLED.show();

    delay(10);
}