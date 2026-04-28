#include <Arduino.h>
#include "calibrate.h"
#include "config.h"
#include "globals.h"
#include "motor.h"

void calibrate() {
    switch (calibrateStep) {
        case 1: {
            static uint32_t lastStepTime = 0;
            static uint16_t testDuty = 500;

            uint32_t now = millis();

            if (now - lastStepTime >= 300) {
                lastStepTime = now;

                if (smoothedRPM < 50.0f) {
                    testDuty += 15;
                    ledcWrite(motorChannel, testDuty);

                    if (DebugLevel::DEBUG <= currentDebugLevel) {
                        Serial.printf("Testing minDuty: %u | Current RPM: %.2f\n", testDuty, smoothedRPM);
                    }
                } else {
                    static uint8_t startCount = 0;
                    startCount++;

                    if (startCount >= 5) {
                        minStartDuty = testDuty;
                        Serial.printf("Auto-minDuty found: %u\n", minStartDuty);
                        playClick(1000, 100);

                        startCount = 0;
                        calibrateStep = 2;
                    }
                }
            }

            if (testDuty > 2000) {
                Serial.println("Calibration Error: Motor not starting!");
                calibrating = false;
                calibrateStep = 0;
                ledcWrite(motorChannel, 0);
            }
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

                if (abs(acceleration) < 50.0f && smoothedRPM > 500) {
                    stableCount++;

                    if (DebugLevel::DEBUG <= currentDebugLevel) {
                        Serial.printf("Stable check %u/10 | RPM: %.2f | Accel: %.2f\n", stableCount, smoothedRPM, acceleration);
                    }

                    if (stableCount >= 10) {
                        maxRPM = smoothedRPM;
                        stableCount = 0;
                        Serial.printf("Max RPM confirmed: %.2f\n", maxRPM);
                        playClick(1250, 200);
                        tuneTimer = millis();
                        calibrateStep = 3;
                    }
                } else {
                    if (stableCount > 0) {
                        if (DebugLevel::DEBUG <= currentDebugLevel) {
                            Serial.printf("Stability broken at count %u | Accel: %.2f — resetting\n", stableCount, acceleration);
                        }
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

                if (abs(acceleration) < 30.0f && (now3 - tuneTimer > 5000)) {
                    stableCount50++;

                    if (DebugLevel::DEBUG <= currentDebugLevel) {
                        Serial.printf("Stable check %u/10 | RPM: %.2f | Accel: %.2f\n", stableCount50, smoothedRPM, acceleration);
                    }

                    if (stableCount50 >= 10) {
                        rpmAt50 = smoothedRPM;
                        playClick(1500, 300);
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
                        if (DebugLevel::DEBUG <= currentDebugLevel) {
                            Serial.printf("Stability broken at count %u | Accel: %.2f — resetting\n", stableCount50, acceleration);
                        }
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

                static uint8_t stableCount80 = 0;

                if (abs(acceleration) < 40.0f && (now4 - tuneTimer > 5000)) {
                    stableCount80++;

                    if (DebugLevel::DEBUG <= currentDebugLevel) {
                        Serial.printf("Stable check %u/10 | RPM: %.2f | Accel: %.2f\n", stableCount80, smoothedRPM, acceleration);
                    }

                    if (stableCount80 >= 10) {
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

                        playClick(1750, 400);

                        ledcWrite(motorChannel, 0);
                        calibrateStep = 5;
                    }
                } else {
                    if (stableCount80 > 0) {
                        if (DebugLevel::DEBUG <= currentDebugLevel) {
                            Serial.printf("Stability broken at count %u | Accel: %.2f — resetting\n", stableCount80, acceleration);
                        }
                    }
                    stableCount80 = 0;
                }

                lastRPM = smoothedRPM;
                lastCheckTime = now4;
            }
            break;
        }
        default:
            break;
    }

    if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) {
        switch (calibrateStep) {
            case 5:
                Serial.println("Calibration finished.");

                preferences.begin("motor-settings", false);

                configTime(3600, 3600, "pool.ntp.org");
                struct tm timeinfo;
                if (!getLocalTime(&timeinfo)) {
                    Serial.println("Time couldn't get requested.");
                } else {
                    time_t now;
                    time(&now);
                    preferences.putULong("last_calibration", (uint32_t)now);
                }

                preferences.putUShort("minDuty", minStartDuty);
                preferences.putFloat("maxRPM", maxRPM);
                preferences.putFloat("Kp", Kp);
                preferences.putFloat("Ki", Ki);
                preferences.end();

                Serial.println("Saved settings to flash storage.");
                playClick(2000, 500);
                calibrating = false;
                calibrateStep = 0;
                break;
            default:
                break;
        }
        delay(300);
    }
}
