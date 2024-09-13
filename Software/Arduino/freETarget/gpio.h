#ifndef FREETARGET_GPIO_H
#define FREETARGET_GPIO_H

#include <Arduino.h>

// Define GPIO pins for ESP32
#define LED_PIN 2 // Example pin for an LED
#define BUTTON_PIN 0 // Example pin for a button

// Function to initialize GPIO pins
void initGPIO() {
    // Set LED_PIN as output
    pinMode(LED_PIN, OUTPUT);
    // Set BUTTON_PIN as input
    pinMode(BUTTON_PIN, INPUT);
}

// Function to turn on the LED
void turnOnLED() {
    digitalWrite(LED_PIN, HIGH);
}

// Function to turn off the LED
void turnOffLED() {
    digitalWrite(LED_PIN, LOW);
}

// Function to read the button state
bool isButtonPressed() {
    return digitalRead(BUTTON_PIN) == LOW; // Assuming active low button
}

#endif // FREETARGET_GPIO_H

