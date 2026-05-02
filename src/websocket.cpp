#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include "websocket.h"
#include "config.h"
#include "globals.h"
#include "motor.h"

void onEvent(AsyncWebSocket *server, const AsyncWebSocketClient *client, const AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA: {
            const AwsFrameInfo *info = static_cast<AwsFrameInfo *>(arg);
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                data[len] = 0;
                String message = reinterpret_cast<char *>(data);

                JsonDocument doc;
                const DeserializationError error = deserializeJson(doc, message);

                if (!error) {
                    const char* sliderType = doc["type"];
                    const int val = doc["value"];

                    if (strcmp(sliderType, "speed") == 0 || strcmp(sliderType, "speedChange") == 0) {
                        if (controlMode == 0) {
                            motorSpeed = val;
                            ledcWrite(motorChannel, motorSpeed);
                        } else {
                            targetRPM = val;
                        }
                        Serial.printf("Slider Speed changed over web server: %d\n", val);
                    }
                    webUIControl = true;
                    potAtWebUITakeover = smoothedPot;
                    if (strcmp(sliderType, "speedChange") == 0) {
                        playClick(3000, 20);
                    }
                } else {
                    Serial.printf("Message received: %s\n", message.c_str());

                    if (message == "toggleCalibration") {
                        if (!calibrating) {
                            calibrating = true;
                            FastLED.setBrightness(0);
                            FastLED.show();
                            zeroCount = 0;
                            calibrateStep = 1;
                            Serial.println("Calibration started via web socket.");
                        } else {
                            calibrating = false;
                            calibrateStep = 0;
                            Serial.println("Calibration cancelled via web socket. Returned to old values.");
                        }
                        playClick(1000, 500);
                    }

                    if (message == "toggleTest") {
                        testing = !testing;
                        Serial.printf("%s test mode via web server.\n", controlMode == 1 ? "Activated" : "Deactivated");
                        playClick(1000, 200);
                    }

                    if (message == "toggleMode") {
                        controlMode = (controlMode == 0) ? 1 : 0;
                        motorSpeed = 0;
                        targetRPM = 0;
                        Serial.printf("Changed control mode via web server to: %s.\n", controlMode == 1 ? "RPM" : "Voltage");
                        playClick(1000, 100);
                    }

                    if (message == "emergencyStop") {
                        emergencyStop = !emergencyStop;
                        testing = false;
                        calibrating = false;
                        calibrateStep = 0;
                        testPhase = 0;
                        speakerActive = false;
                        ledcWriteTone(speakerChannel, 0);
                        Serial.printf("Emergency stop %s via web server.\n", emergencyStop ? "activated" : "deactivated");
                    }
                }
            }
            break;
        }
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
        default:
            break;
    }
}