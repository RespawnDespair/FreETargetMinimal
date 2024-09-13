#include <Arduino.h>

// Define the timer and its interrupt service routine
hw_timer_t *timer = NULL;
volatile bool timerFlag = false;

// Timer interrupt service routine
void IRAM_ATTR onTimer() {
  timerFlag = true;
}

void setup() {
  Serial.begin(115200);

  // Initialize the timer
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up

  // Attach the interrupt service routine to the timer
  timerAttachInterrupt(timer, &onTimer, true);

  // Set the timer to trigger every 1 second (1,000,000 microseconds)
  timerAlarmWrite(timer, 1000000, true);

  // Start the timer
  timerAlarmEnable(timer);
}

void loop() {
  if (timerFlag) {
    timerFlag = false;
    Serial.println("Timer triggered!");
    // Add your timer-based code here
  }
}

