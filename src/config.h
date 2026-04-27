#pragma once

#define STATUS_LED_PIN 48

#define POT_PIN 15
#define CALIBRATE_BUTTON_PIN 16
#define MODE_BUTTON_PIN 17
#define EMERGENCY_STOP_BUTTON_PIN 18

#define MOTOR_PIN 14
#define SPEAKER_PIN 4
#define SENSOR_PIN 12

const int speakerChannel = 0;
const int freqSpeaker = 2000;
const int resolutionSpeaker = 8;

const int motorChannel = 2;
const int freqMotor = 8000;
const int resolutionMotor = 12;

const float ADC_MIN = 50.0f;
const float ADC_MAX = 4046.0f;
const float ADC_TOLERANCE = 40.0f;

const float INTEGRATOR_CLAMP = 500.0f;
