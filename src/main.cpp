#include <Arduino.h>
#include <FastLED.h>

#define STATUS_LED_PIN 48

CRGB leds[1];

#define POT_PIN 15
#define CALIBRATE_BUTTON_PIN 16
#define EMERGENCY_STOP_BUTTON_PIN 17

#define MOTOR_PIN 14

#define SPEAKER_PIN 4

const int speakerChannel = 0;
const int freqSpeaker = 2000;
const int resolution = 8;

uint8_t motorSpeed = 0;
bool emergencyStop = false;
bool calibrating = false;
bool testing = false;

void setup() {
    Serial.begin(921600);
    delay(2000);

    pinMode(EMERGENCY_STOP_BUTTON_PIN, INPUT_PULLUP);

    pinMode(STATUS_LED_PIN, OUTPUT);

    FastLED.addLeds<WS2812B, STATUS_LED_PIN, GRB>(leds, 1);
    FastLED.setBrightness(127);

    ledcSetup(speakerChannel, freqSpeaker, resolution);
    ledcAttachPin(SPEAKER_PIN, speakerChannel);

    pinMode(MOTOR_PIN, OUTPUT);

    Serial.println("System ready!");
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

void readPot() {
    static float smoothedPot = 0;

    static float lastTriggeredPot = 0;

    const float ADC_MIN = 100.0f;
    const float ADC_MAX = 4000.0f;

    const float ADC_TOLERANCE = 25.0f;

    int raw = analogRead(POT_PIN);

    smoothedPot = (smoothedPot * 0.9f) + (raw * 0.1f);

    if (abs(smoothedPot - lastTriggeredPot) > ADC_TOLERANCE) {
        lastTriggeredPot = smoothedPot;

        float constrainedPot = constrain(smoothedPot, ADC_MIN, ADC_MAX);

        uint8_t newSpeed = static_cast<uint8_t>((constrainedPot - ADC_MIN) / (ADC_MAX - ADC_MIN) * 255.0f);

        motorSpeed = newSpeed;
        analogWrite(MOTOR_PIN, motorSpeed);
    }
}

void loop() {
    if (digitalRead(EMERGENCY_STOP_BUTTON_PIN) == LOW) {
        emergencyStop = !emergencyStop;
        ledcWriteTone(speakerChannel, 0);
        delay(500);
    }

    if (emergencyStop == true) {
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

    readPot();

    FastLED.setBrightness(127);
    leds[0] = CRGB::Green;
    FastLED.show();
    
    test();

    delay(10);
}