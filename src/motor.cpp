#include <Arduino.h>
#include "motor.h"
#include "config.h"
#include "globals.h"

void playClick(const int freq, const int duration) {
    ledcWriteTone(speakerChannel, freq);
    speakerOffTime = millis() + duration;
    speakerActive = true;
}

void updateSpeaker() {
    if (speakerActive && millis() >= speakerOffTime) {
        ledcWriteTone(speakerChannel, 0);
        speakerActive = false;
    }
}

void IRAM_ATTR pulseISR() {
    const uint32_t now = micros();

    if (lastPulseMicros == 0) {
        lastPulseMicros = now;
        return;
    }

    const uint32_t duration = now - lastPulseMicros;

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
        triggerFlash = true;
    }
}

void readPot() {
    const int raw = analogRead(POT_PIN);
    smoothedPot = (smoothedPot * 0.9f) + (static_cast<float>(raw) * 0.1f);
}

void adjustSpeed() {
    if (webUIControl) {
        if (abs(smoothedPot - potAtWebUITakeover) > ADC_TOLERANCE * 3) {
            webUIControl = false;
            lastTriggeredPot = smoothedPot;
        } else {
            return;
        }
    }

    if (abs(smoothedPot - lastTriggeredPot) > ADC_TOLERANCE) {
        lastTriggeredPot = smoothedPot;
        playClick(150, 10);

        if (smoothedPot < (ADC_MIN + ADC_TOLERANCE)) {
            motorSpeed = 0;
        } else {
            float constrainedPot = constrain(smoothedPot, ADC_MIN, ADC_MAX);
            float percent = (constrainedPot - ADC_MIN) / (ADC_MAX - ADC_MIN);
            motorSpeed = static_cast<uint16_t>((percent * (4095.0f - minStartDuty)) + minStartDuty);
        }

        ledcWrite(motorChannel, motorSpeed);

        if (DebugLevel::VERBOSE <= currentDebugLevel) {
            Serial.printf("adjust speed %hu\n", motorSpeed);
        }
    }
}

void controlRPM() {
    if (webUIControl) {
        if (abs(smoothedPot - potAtWebUITakeover) > 150) {
            webUIControl = false;
            lastTriggeredPot = smoothedPot;
        }
    }

    if (!webUIControl && abs(smoothedPot - lastTriggeredPot) > ADC_TOLERANCE) {
        lastTriggeredPot = smoothedPot;

        float constrainedPot = constrain(smoothedPot, ADC_MIN, ADC_MAX);
        float percent = (constrainedPot - ADC_MIN) / (ADC_MAX - ADC_MIN);
        targetRPM = percent * maxRPM;

        playClick(150, 10);
    }

    if (targetRPM < 100) {
        integrator = 0;
        currentSpeed = 0;
        ledBrightness = 0;
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
    ledcWrite(motorChannel, currentSpeed);
    ledBrightness = map(currentSpeed, 0, 4095, 0, 255);

    if (DebugLevel::VERBOSE <= currentDebugLevel) {
        Serial.printf("PID Control: Target %.2f | Ist %.2f | PWM %hu\n", targetRPM, smoothedRPM, static_cast<uint16_t>(currentSpeed));
    }
}