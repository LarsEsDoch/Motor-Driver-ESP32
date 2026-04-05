#include <Arduino.h>
#include <FastLED.h>

void setup() {
    Serial.begin(921600);
    delay(2000);
    Serial.println("System ready!");
}
void loop() {
    delay(10);
}