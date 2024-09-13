#include <Arduino.h>

// Define the timer and its parameters
hw_timer_t *timer = NULL;
volatile bool timerFlag = false;

// Timer interrupt service routine
void IRAM_ATTR onTimer() {
    timerFlag = true; // Set a flag to indicate the timer has triggered
}

void setup() {
    Serial.begin(115200);

    // Initialize the timer
    timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up

    // Attach the interrupt service routine
    timerAttachInterrupt(timer, &onTimer, true);

    // Set the timer alarm to trigger every second (1,000,000 microseconds)
    timerAlarmWrite(timer, 1000000, true);

    // Enable the timer alarm
    timerAlarmEnable(timer);
}

void loop() {
    if (timerFlag) {
        timerFlag = false; // Reset the flag
        Serial.println("Timer triggered!"); // Perform the desired action
    }
}

