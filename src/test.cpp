#include <Arduino.h>
#include <FastLED.h>
#include "test.h"
#include "config.h"
#include "globals.h"

// testPhase values:
// 0 = Init
// 1 = Frequenz sweep
// 2 = Start up motor
// 3 = Countdown
// 4 = Slow down motor
// 5 = LED rainbow
// 6 = Done

void test() {
    static uint32_t lastStepTime = 0;
    static int sweepFreq = 100;
    static int motorDuty = 0;
    static int countdownStep = 5;
    static int hue = 0;

    const uint32_t now = millis();

    switch (testPhase) {
        case 0: {
            FastLED.setBrightness(0);
            FastLED.show();
            Serial.println("Starting Frequency Sweep (100Hz - 20000Hz)...");
            sweepFreq = 100;
            lastStepTime = now;
            testPhase = 1;
            break;
        }
        case 1: {
            if (now - lastStepTime >= 40) {
                lastStepTime = now;
                ledcWriteTone(speakerChannel, sweepFreq);
                Serial.printf("Frequency: %i\n", sweepFreq);
                sweepFreq += 100;

                if (sweepFreq > 20000) {
                    ledcWriteTone(speakerChannel, 0);
                    Serial.println("Sweep finished.");
                    Serial.println("Motor starts up...");
                    motorDuty = 0;
                    testPhase = 2;
                }
            }
            break;
        }
        case 2: {
            if (now - lastStepTime >= 20) {
                lastStepTime = now;
                ledcWrite(motorChannel, motorDuty);
                Serial.printf("Speed: %i\n", motorDuty);
                motorDuty += 8;

                if (motorDuty >= 4095) {
                    motorDuty = 4095;
                    ledcWrite(motorChannel, motorDuty);
                    Serial.println("Maximum speed reached.");
                    countdownStep = 5;
                    testPhase = 3;
                }
            }
            break;
        }
        case 3: {
            if (now - lastStepTime >= 1000) {
                lastStepTime = now;
                Serial.println(countdownStep);
                countdownStep--;

                if (countdownStep < 0) {
                    Serial.println("Motor slows down...");
                    motorDuty = 4095;
                    testPhase = 4;
                }
            }
            break;
        }
        case 4: {
            if (now - lastStepTime >= 20) {
                lastStepTime = now;
                ledcWrite(motorChannel, motorDuty);
                Serial.printf("Speed: %i\n", motorDuty);
                motorDuty -= 8;

                if (motorDuty <= 0) {
                    motorDuty = 0;
                    ledcWrite(motorChannel, 0);
                    Serial.println("Start LED rainbow test.");
                    FastLED.setBrightness(255);
                    hue = 0;
                    testPhase = 5;
                }
            }
            break;
        }
        case 5: {
            if (now - lastStepTime >= 15) {
                lastStepTime = now;
                leds[0] = CHSV(hue, 255, 255);
                FastLED.show();
                Serial.printf("Hue: %i\n", hue);
                hue++;

                if (hue >= 256) {
                    leds[0] = CRGB::White;
                    FastLED.setBrightness(0);
                    FastLED.show();
                    Serial.println("Finished test.");
                    testing = false;
                    testPhase = 0;
                }
            }
            break;
        }
        default:
            break;
    }
}