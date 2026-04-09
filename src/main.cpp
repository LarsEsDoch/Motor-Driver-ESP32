#include <Arduino.h>
#include <FastLED.h>
#include <Preferences.h>

Preferences preferences;

#define STATUS_LED_PIN 48

CRGB leds[1];

#define POT_PIN 15
#define CALIBRATE_BUTTON_PIN 16
#define EMERGENCY_STOP_BUTTON_PIN 17

#define MOTOR_PIN 14

#define SPEAKER_PIN 4

#define SENSOR_PIN 12

const int speakerChannel = 0;
const int freqSpeaker = 2000;
const int resolution = 8;

uint8_t motorSpeed = 0;

uint8_t minStartDuty = 0;

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

int workingMode = 1;

bool debug = true;


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

    minStartDuty = preferences.getUChar("minDuty", 0);
    if (minStartDuty != 0) Serial.println("\nReused min start duty out of preferences.\n");

    pinMode(STATUS_LED_PIN, OUTPUT);

    FastLED.addLeds<WS2812B, STATUS_LED_PIN, GRB>(leds, 1);
    FastLED.setBrightness(127);

    pinMode(EMERGENCY_STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);

    ledcSetup(speakerChannel, freqSpeaker, resolution);
    ledcAttachPin(SPEAKER_PIN, speakerChannel);

    pinMode(MOTOR_PIN, OUTPUT);

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
    for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
        analogWrite(MOTOR_PIN, dutyCycle);
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

    Serial.println("Motor slows down...");
    for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
        analogWrite(MOTOR_PIN, dutyCycle);
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
    FastLED.setBrightness(motorSpeed);
    FastLED.show();

    Serial.println("Finished test.");
}

void calibrate() {
    if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) {
        switch (calibrateStep) {
            case 0:
                minStartDuty = motorSpeed;
                calibrateStep = 1;

                preferences.begin("motor-settings", false);
                preferences.putUChar("minDuty", minStartDuty);
                preferences.end();

                Serial.printf("Calibration finished. Set min duty cycle to %hhu\n", minStartDuty);
                playClick(1000, 100);
                delay(300);
                break;
            case 1:
                calibrating = false;
                calibrateStep = 0;
                break;
        }
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

            motorSpeed = static_cast<uint8_t>((percent * (255.0f - minStartDuty)) + minStartDuty);

            ledBrightness = static_cast<uint8_t>(percent * 255.0f);
        }

        analogWrite(MOTOR_PIN, motorSpeed);
    }
}

void loop() {
    if (digitalRead(EMERGENCY_STOP_BUTTON_PIN) == LOW) {
        emergencyStop = !emergencyStop;
        testing = false;
        calibrating = false;
        calibrateStep = 0;
        ledcWriteTone(speakerChannel, 0);
        delay(500);
    }

    static uint32_t buttonPressStartTime = 0;
    static bool buttonWasPressed = false;

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
        analogWrite(MOTOR_PIN, motorSpeed);

        uint8_t pulse = beatsin8(40, 50, 255);
        leds[0] = CRGB::Red;
        FastLED.setBrightness(pulse);
        FastLED.show();

        uint16_t sirenFreq = beatsin16(40, 600, 1200);
        ledcWriteTone(speakerChannel, sirenFreq);

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
    adjustSpeed();

    FastLED.setBrightness(ledBrightness);
    uint8_t currentHue = map(motorSpeed, minStartDuty, 255, 160, 0);
    leds[0] = CHSV(currentHue, 255, 255);
    FastLED.show();

    delay(10);
}