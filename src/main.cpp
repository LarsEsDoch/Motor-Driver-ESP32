#include <Arduino.h>
#include <FastLED.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include "secrets.h"
#include "config.h"
#include "globals.h"
#include "motor.h"
#include "calibrate.h"
#include "test.h"
#include "websocket.h"

void setup() {
    Serial.begin(921600);
    delay(2000);

    pinMode(SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), pulseISR, RISING);

    preferences.begin("motor-settings", false);

    lastCalibrationTime = (time_t)preferences.getULong("last_calibration", 0);
    minStartDuty = preferences.getUShort("minDuty", 0);
    maxRPM = preferences.getFloat("maxRPM", 2000.0f);
    Kp = preferences.getFloat("Kp", 0.8f);
    Ki = preferences.getFloat("Ki", 0.1f);

    if (lastCalibrationTime != 0) {
        struct tm * timeinfo;
        timeinfo = localtime(&lastCalibrationTime);

        char buffer[30];
        strftime(buffer, sizeof(buffer), "%d.%m.%Y %H:%M:%S", timeinfo);

        Serial.println("\nLoaded settings from flash:");
        Serial.printf("Calibration Date: %s | Min Duty: %hu | Max RPM: %.2f | Kp: %.4f | Ki: %.4f\n\n", buffer, minStartDuty, maxRPM, Kp, Ki);
    } else {
        Serial.println("\nUsing default values.");
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

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFiClass::status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println(WiFi.localIP());

    ws.onEvent(onEvent);
    server.addHandler(&ws);

    if (!LittleFS.begin()) {
        Serial.println("An Error has occurred while mounting LittleFS");
        return;
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.begin();

    Serial.println("HTTP server started");
}

void loop() {
    if (triggerFlash) {
        triggerFlash = true;
    }

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
        Serial.printf("Rotation: %u | RPM: %.2f | Smoothed RPM: %.2f | Target RPM: %.2f | Motor speed: %hu | Current speed: %.2hu | ADC Pot: %.2f \n",
            debugTickCount, currentRPM, smoothedRPM, targetRPM, motorSpeed, currentSpeed, smoothedPot);
        lastSeenTick = debugTickCount;
    }

    static uint32_t lastUpload = 0;
    if (millis() - lastUpload > 200) {
        static float lastRPMDisplay = 0;
        const float tolerance = 200.0f;

        if (micros() - lastPulseMicros > 2000000) {
            displayRPM = (displayRPM * 0.5f) + (currentRPM * 0.5f);
        } else {
            displayRPM = (displayRPM * 0.95f) + (currentRPM * 0.05f);
        }

        int movementState = 1;

        if (displayRPM > (lastRPMDisplay + tolerance)) {
            movementState = 2;
        } else if (displayRPM < (lastRPMDisplay - tolerance)) {
            movementState = 0;
        }

        String calibrationStatus;

        if (lastCalibrationTime == 0) {
            calibrationStatus = "No calibration done";
        } else {
            struct tm * timeinfo;
            timeinfo = localtime(&lastCalibrationTime);
            char buffer[30];
            strftime(buffer, sizeof(buffer), "%d.%m.%Y %H:%M:%S", timeinfo);
            calibrationStatus = String(buffer);
        }

        String json = "{";
        json += "\"rpm\":" + String(displayRPM, 2) + ",";
        json += "\"trend\":" + String(movementState) + ",";
        json += "\"pot\":" + String(smoothedPot) + ",";
        json += "\"rotation\":" + String(debugTickCount) + ",";
        json += "\"target_rpm\":" + String(targetRPM) + ",";
        json += "\"target_speed\":" + String(motorSpeed) + ",";
        json += "\"pid_output\":" + String(currentSpeed) + ",";
        json += "\"control_mode\":" + String(controlMode) + ",";
        json += "\"emergency\":" + String(emergencyStop) + ",";
        json += "\"testing\":" + String(testing) + ",";
        json += "\"calibrate_step\":" + String(calibrateStep) + ",";
        json += "\"min_duty\":" + String(minStartDuty) + ",";
        json += "\"max_rpm\":" + String(maxRPM, 2) + ",";
        json += "\"kp\":" + String(Kp, 4) + ",";
        json += "\"ki\":" + String(Ki, 4) + ",";
        json += "\"last_calibration_time\":\"" + calibrationStatus + "\"";
        json += "}";

        ws.textAll(json);
        lastUpload = millis();
        lastRPMDisplay = smoothedRPM;
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
                    case DebugLevel::NONE:    Serial.println("NONE");    break;
                    case DebugLevel::INFO:    Serial.println("INFO");    break;
                    case DebugLevel::DEBUG:   Serial.println("DEBUG");   break;
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
                motorSpeed = 0;
                targetRPM = 0;
                Serial.printf("Control mode set to %s\n", controlMode == 1 ? "rpm" : "voltage");
                playClick(2000, 100);
            }

            modeButtonWasPressed = false;
            modeActionExecuted = false;
        }
    }

    static uint32_t calibrateButtonPressStartTime = 0;
    static bool calibrateButtonWasPressed = false;

    if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW && !calibrating) {
        if (!calibrateButtonWasPressed) {
            calibrateButtonPressStartTime = millis();
            calibrateButtonWasPressed = true;
        } else {
            if (millis() - calibrateButtonPressStartTime >= 3000) {
                testing = !testing;
                playClick(2000, 100);
                calibrateButtonWasPressed = false;
                Serial.printf("Test mode %s\n", testing ? "activated" : "deactivated");
            }
        }
    } else {
        if (calibrateButtonWasPressed) {
            if (millis() - calibrateButtonPressStartTime < 3000) {
                if (!calibrating) {
                    calibrating = true;
                    calibrateStep = 1;
                    FastLED.setBrightness(0);
                    FastLED.show();
                    Serial.println("Calibration started.");
                } else {
                    calibrating = false;
                    calibrateStep = 0;
                    Serial.println("Calibration cancelled. Returned to old values.");
                }
                playClick(2000, 100);
            }
            calibrateButtonWasPressed = false;
        }
    }

    if (emergencyStop) {
        motorSpeed = 0;
        targetRPM = 0;
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
        delay(10);
        return;
    }

    if (testing && !calibrating) {
        test();
        delay(10);
        return;
    }

    readPot();
    if (!calibrating) {
        if (controlMode == 0) {
            adjustSpeed();
        } else if (controlMode == 1) {
            controlRPM();
        }
    }

    uint16_t referenceSpeed = (controlMode == 0) ? motorSpeed : (uint16_t)currentSpeed;

    if (controlMode == 0) {
        if (referenceSpeed < minStartDuty) {
            ledBrightness = 0;
        } else {
            ledBrightness = map(referenceSpeed, minStartDuty, 4095, 0, 255);
        }
    } else {
        ledBrightness = map(referenceSpeed, 0, 4095, 0, 255);

        if (referenceSpeed == 0) ledBrightness = 0;
    }

    ledBrightness = constrain(ledBrightness, 0, 255);
    FastLED.setBrightness(ledBrightness);

    uint8_t currentHue = 0;
    if (controlMode == 0) {
        uint16_t constrainedSpeed = constrain(motorSpeed, minStartDuty, 4095);
        currentHue = map(constrainedSpeed, minStartDuty, 4095, 160, 0);
    } else {
        uint32_t constrainedRPM = constrain(targetRPM, 0, maxRPM);
        currentHue = map(constrainedRPM, 0, maxRPM, 160, 0);
    }

    leds[0] = CHSV(currentHue, 255, 255);
    FastLED.show();

    if (DebugLevel::VERBOSE <= currentDebugLevel) {
        Serial.printf("Brightness: %u\n", ledBrightness);
        Serial.printf("Motor Speed: %hu\n", motorSpeed);
        Serial.printf("Speed: %hu\n", currentSpeed);
        Serial.printf("Hue: %hu\n", currentHue);
    }

    delay(10);
}