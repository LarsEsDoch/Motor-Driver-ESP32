#pragma once

#include <ESPAsyncWebServer.h>

void onEvent(AsyncWebSocket *server, const AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);