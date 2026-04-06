#include <Arduino.h>
#include <FastLED.h>

#define STATUS_LED_PIN 48

CRGB leds[1];

#define POT_PIN 4

#define EMERGENCY_STOP_BUTTON_PIN 5

#define SPEAKER_PIN 7
#define MOTOR_PIN 6

const int speakerChannel = 0;
const int freqSpeaker = 2000;
const int resolution = 8;

uint8_t motorSpeed = 0;
bool emergencyStop = false;

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
    Serial.println("Play test tone 3000hz...");
    ledcWriteTone(speakerChannel, 3000);
    delay(100);
    ledcWriteTone(speakerChannel, 0);

    Serial.println("Motor starts up...");
    for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
        analogWrite(MOTOR_PIN, dutyCycle);
        delay(20);
    }

    Serial.println("Maximum speed reached.");
    delay(3000);

    Serial.println("Motor slows down...");
    for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
        analogWrite(MOTOR_PIN, dutyCycle);
        delay(20);
    }
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
    readPot();

    FastLED.setBrightness(127);
    leds[0] = CRGB::Green;
    FastLED.show();
    
    test();

    delay(10);
}