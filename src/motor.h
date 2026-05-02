#pragma once

void playClick(int freq, int duration);
void updateSpeaker();

void IRAM_ATTR pulseISR();

void readPot();
void adjustSpeed();
void controlRPM();