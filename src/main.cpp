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
const float INTEGRATOR_CLAMP = 500.0f;

static float lastRPM = 0;
static uint32_t lastCheckTime = 0;
float acceleration = 0;

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

    if (duration > 1000000) {
        lastPulseMicros = now;
        latestDuration = 0;
        firstIntervalSeeded = false;
        return;
    }

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
    maxRPM = preferences.getFloat("maxRPM", 2000.0f);
    Kp = preferences.getFloat("Kp", 0.8f);
    Ki = preferences.getFloat("Ki", 0.1f);

    if (minStartDuty != 0) {
        Serial.println("\nLoaded settings from flash:");
        Serial.printf("Min Duty: %hu | Max RPM: %.2f | Kp: %.4f | Ki: %.4f\n\n", minStartDuty, maxRPM, Kp, Ki);
    }

    pinMode(STATUS_LED_PIN, OUTPUT);

    CFastLED::addLeds<WS2812B, STATUS_LED_PIN, GRB>(leds, 1);
    FastLED.setBrightness(127);

    pinMode(EMERGENCY_STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
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
            case 1:
                minStartDuty = motorSpeed;
                calibrateStep = 2;

                Serial.printf("Speed calibration finished. Set min duty cycle to %hu\n", minStartDuty);
                Serial.println("Continuing with motor max speed");
                playClick(1500, 100);
                break;
            case 5:
                Serial.println("Calibration finished.");

                preferences.begin("motor-settings", false);
                preferences.putUShort("minDuty", minStartDuty);
                preferences.putFloat("maxRPM", maxRPM);
                preferences.putFloat("Kp", Kp);
                preferences.putFloat("Ki", Ki);
                preferences.end();

                Serial.println("Saved settings to flash storage.");
                playClick(1000, 100);
                calibrating = false;
                calibrateStep = 0;
                break;
            default:
                break;
        }
        delay(300);
    }

    switch (calibrateStep) {
        case 0: {
            calibrateStep = 1;
            controlMode = 0;
            break;
        }
        case 2: {
            ledcWrite(motorChannel, 4095);
            if (DebugLevel::VERBOSE <= currentDebugLevel) {
                Serial.println("calibrate case 2 ledc 4095");
            }

            uint32_t now = millis();
            if (now - lastCheckTime >= 200) {
                float deltaTime = (now - lastCheckTime) / 1000.0f;
                acceleration = (smoothedRPM - lastRPM) / deltaTime;

                static uint8_t stableCount = 0;

                if (abs(acceleration) < 100.0f && smoothedRPM > 500) {
                    stableCount++;
                    if (DebugLevel::DEBUG <= currentDebugLevel) Serial.printf("Stable check %u/8 | RPM: %.2f | Accel: %.2f\n", stableCount, smoothedRPM, acceleration);

                    if (stableCount >= 8) {
                        maxRPM = smoothedRPM;
                        stableCount = 0;
                        Serial.printf("Max RPM confirmed: %.2f\n", maxRPM);
                        playClick(1800, 200);
                        tuneTimer = millis();
                        calibrateStep = 3;
                    }
                } else {
                    if (stableCount > 0) {
                        if (DebugLevel::DEBUG <= currentDebugLevel) Serial.printf("Stability broken at count %u | Accel: %.2f — resetting\n", stableCount, acceleration);
                    }
                    stableCount = 0;
                }

                lastRPM = smoothedRPM;
                lastCheckTime = now;
            }
            break;
        }
        case 3: {
            ledcWrite(motorChannel, 2048);
            if (DebugLevel::VERBOSE <= currentDebugLevel) {
                Serial.println("calibrate case e ledc 2048");
            }

            uint32_t now3 = millis();
            if (now3 - lastCheckTime >= 200) {
                float deltaTime = (now3 - lastCheckTime) / 1000.0f;
                acceleration = (smoothedRPM - lastRPM) / deltaTime;

                static uint8_t stableCount50 = 0;

                if (abs(acceleration) < 50.0f && (now3 - tuneTimer > 5000)) {
                    stableCount50++;
                    if (DebugLevel::DEBUG <= currentDebugLevel) Serial.printf("Stable check %u/8 | RPM: %.2f | Accel: %.2f\n", stableCount50, smoothedRPM, acceleration);

                    if (stableCount50 >= 10) {
                        rpmAt50 = smoothedRPM;
                        Serial.printf("Base RPM at 50%% PWM stabilized: %.2f\n", rpmAt50);
                        Serial.println("Jumping to 80%% PWM...");

                        stableCount50 = 0;
                        ledcWrite(motorChannel, 3276);
                        if (DebugLevel::VERBOSE <= currentDebugLevel) {
                            Serial.println("calibrate case 3 ledc 3276");
                        }
                        tuneTimer = millis();
                        lastCheckTime = millis();
                        calibrateStep = 4;
                    }
                } else {
                    if (stableCount50 > 0) {
                        if (DebugLevel::DEBUG <= currentDebugLevel) Serial.printf("Stability broken at count %u | Accel: %.2f — resetting\n", stableCount50, acceleration);
                    }
                    stableCount50 = 0;
                }

                lastRPM = smoothedRPM;
                lastCheckTime = now3;
            }
            break;
        }
        case 4: {
            ledcWrite(motorChannel, 3276);
            if (DebugLevel::VERBOSE <= currentDebugLevel) {
                Serial.println("calibrate case 4 ledc 3276");
            }

            uint32_t now4 = millis();
            if (now4 - lastCheckTime >= 200) {
                float deltaTime = (now4 - lastCheckTime) / 1000.0f;
                acceleration = (smoothedRPM - lastRPM) / deltaTime;

                if (abs(acceleration) < 5.0f && (now4 - tuneTimer > 1000)) {
                    float rpmAt80 = smoothedRPM;
                    float deltaRPM = rpmAt80 - rpmAt50;
                    float deltaPWM = 3276.0f - 2048.0f;

                    systemGain = deltaRPM / deltaPWM;

                    float timeToStabilize = (now4 - tuneTimer) / 1000.0f;
                    timeConstant = timeToStabilize / 3.0f;

                    Serial.printf("RPM at 80%%: %.2f\n", rpmAt80);
                    Serial.printf("System Gain (K): %.4f\n", systemGain);
                    Serial.printf("Time Constant (Tau): %.2f sec\n", timeConstant);

                    Kp = 0.5f / systemGain;

                    Ki = (Kp / timeConstant) * 0.5f;

                    Serial.printf("\n--- TUNING SUCCESSFUL ---\n");
                    Serial.printf("Calculated Kp: %.4f | Ki: %.4f\n", Kp, Ki);
                    Serial.println("Press Calibrate Button to save settings.");

                    playClick(2000, 300);

                    ledcWrite(motorChannel, 0);
                    calibrateStep = 5;
                }

                lastRPM = smoothedRPM;
                lastCheckTime = now4;
            }
            break;
        }
        default:
            break;
    }
}

void readPot() {
    int raw = analogRead(POT_PIN);
    smoothedPot = (smoothedPot * 0.9f) + (static_cast<float>(raw) * 0.1f);
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
    if (abs(smoothedPot - lastTriggeredPot) > ADC_TOLERANCE) {
        lastTriggeredPot = smoothedPot;

        float constrainedPot = constrain(smoothedPot, ADC_MIN, ADC_MAX);
        float percent = (constrainedPot - ADC_MIN) / (ADC_MAX - ADC_MIN);
        targetRPM = percent * maxRPM;

        playClick(150, 10);
    }

    if (targetRPM < 100) {
        integrator = 0;
        currentSpeed = 0;
        ledcWrite(motorChannel, 0);
        return;
    }

    if (smoothedRPM < 50 && targetRPM > 100) {
        currentSpeed = minStartDuty;
        ledcWrite(motorChannel, currentSpeed);
        if (DebugLevel::VERBOSE <= currentDebugLevel) {
            Serial.printf("Start-Kick: %hu\n", currentSpeed);
        }
        return;
    }

    float error = targetRPM - smoothedRPM;
    integrator += error;
    integrator = constrain(integrator, -INTEGRATOR_CLAMP, INTEGRATOR_CLAMP);

    float output = (Kp * error) + (Ki * integrator);
    currentSpeed += output;

    if (currentSpeed < minStartDuty) {
        currentSpeed = minStartDuty;
    }

    currentSpeed = constrain(currentSpeed, 0, 4095);
    ledcWrite(motorChannel, (uint16_t)currentSpeed);

    if (DebugLevel::VERBOSE <= currentDebugLevel) {
        Serial.printf("PID Control: Target %.2f | Ist %.2f | PWM %hu\n", targetRPM, smoothedRPM, (uint16_t)currentSpeed);
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
        delay(300);
    }

    static uint32_t modeButtonPressStartTime = 0;
    static uint32_t totalPressStartTime = 0;
    static bool modeButtonWasPressed = false;
    bool modeActionExecuted = false;

    if (digitalRead(MODE_BUTTON_PIN) == LOW) {
        if (!modeButtonWasPressed) {
            modeButtonPressStartTime = millis();
            totalPressStartTime = millis();
            modeButtonWasPressed = true;
            modeActionExecuted = false;
        } else {
            if (millis() - modeButtonPressStartTime >= 3000) {
                int next = static_cast<int>(currentDebugLevel) + 1;
                if (next > static_cast<int>(DebugLevel::VERBOSE)) next = 0;
                currentDebugLevel = static_cast<DebugLevel>(next);

                Serial.print("New Debug Level: ");
                playClick(1000, 100);

                switch (currentDebugLevel) {
                    case DebugLevel::NONE: Serial.println("NONE"); break;
                    case DebugLevel::INFO: Serial.println("INFO"); break;
                    case DebugLevel::DEBUG: Serial.println("DEBUG"); break;
                    case DebugLevel::VERBOSE: Serial.println("VERBOSE"); break;
                }

                modeButtonPressStartTime = millis();
                modeActionExecuted = true;
            }
        }
    } else {
        if (modeButtonWasPressed) {
            unsigned long totalDuration = millis() - totalPressStartTime;

            if (!modeActionExecuted && totalDuration < 3000 && totalDuration > 50) {
                controlMode = (controlMode == 0) ? 1 : 0;
                Serial.printf("Control mode set to %s\n", controlMode == 1 ? "rpm" : "voltage");
                playClick(2000, 100);
            }

            modeButtonWasPressed = false;
            modeActionExecuted = false;
        }
    }

    static uint32_t CalibrateButtonPressStartTime = 0;
    static bool CalibrateButtonWasPressed = false;

    if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW && !calibrating) {
        if (!CalibrateButtonWasPressed) {
            CalibrateButtonPressStartTime = millis();
            CalibrateButtonWasPressed = true;
        } else {
            if (millis() - CalibrateButtonPressStartTime >= 3000) {
                testing = !testing;

                playClick(2000, 100);

                CalibrateButtonWasPressed = false;
                Serial.printf("Test mode %s\n", testing ? "activated" : "deactivated");
            }
        }
    } else {
        if (CalibrateButtonWasPressed) {
            if (millis() - CalibrateButtonPressStartTime < 3000) {
                calibrating = true;
                minStartDuty = 0;

                playClick(2000, 100);

                Serial.println("Calibration started.");
            }
            CalibrateButtonWasPressed = false;
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