#include "freETarget.h"
#include "Wire.h"

// Define LEDC channel and frequency for PWM
#define LEDC_CHANNEL_0 0
#define LEDC_TIMER_13_BIT 13
#define LEDC_BASE_FREQ 5000

/*----------------------------------------------------------------
 * 
 * function: init_analog_io()
 * 
 * brief: Initialize the analog I/O
 * 
 * return: None
 * 
 *--------------------------------------------------------------*/
void init_analog_io(void)
{
  if (DLT(DLT_CRITICAL)) // and not in trace mode (DIAG jumper installed)
  {
    Serial.print(T("init_analog_io()")); // Blink the LEDs
  }

  // Configure LED PWM functionalitites
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(LED_PWM, LEDC_CHANNEL_0);
  return;
}

/*----------------------------------------------------------------
 * 
 * function: set_LED_PWM()
 * function: set_LED_PWM_now()
 * 
 * brief: Program the PWM value
 * 
 * return: None
 * 
 *----------------------------------------------------------------
 *
 * json_LED_PWM is a number 0-100 %  It must be scaled 0-255
 * 
 * The function ramps the level between the current and desired
 * 
 *--------------------------------------------------------------*/
static unsigned int old_LED_percent = 0;

void set_LED_PWM_now(int new_LED_percent)
{
  if (new_LED_percent == old_LED_percent)
  {
    return;
  }

  if (DLT(DLT_DIAG))
  {
    Serial.print(T("new_LED_percent:"));
    Serial.print(new_LED_percent);
    Serial.print(T("  old_LED_percent:"));
    Serial.print(old_LED_percent);
  }

  old_LED_percent = new_LED_percent;
  ledcWrite(LEDC_CHANNEL_0, old_LED_percent * 255 / 100); // Write the value out

  return;
}

void set_LED_PWM(int new_LED_percent)
{
  if (DLT(DLT_DIAG))
  {
    Serial.print(T("new_LED_percent:"));
    Serial.print(new_LED_percent);
    Serial.print(T("  old_LED_percent:"));
    Serial.print(old_LED_percent);
  }

  /*
   * Special case, toggle the LED state
   */
  if (new_LED_percent == LED_PWM_TOGGLE)
  {
    new_LED_percent = 0;
    if (old_LED_percent == 0)
    {
      new_LED_percent = json_LED_PWM;
    }
  }

  /*
   * Loop and ramp the LED  PWM up or down slowly
   */
  while (new_LED_percent != old_LED_percent) // Change in the brightness level?
  {
    ledcWrite(LEDC_CHANNEL_0, old_LED_percent * 255 / 100); // Write the value out

    if (new_LED_percent < old_LED_percent)
    {
      old_LED_percent--; // Ramp the value down
    }
    else
    {
      old_LED_percent++; // Ramp the value up
    }

    delay(ONE_SECOND / 50); // Worst case, take 2 seconds to get there
  }

  /*
   * All done, begin the program
   */
  if (new_LED_percent == 0)
  {
    ledcWrite(LEDC_CHANNEL_0, 0);
  }
  return;
}

/*----------------------------------------------------------------
 * 
 * function: revision(void)
 * 
 * brief: Return the board revision
 * 
 * return: Board revision level
 * 
 *--------------------------------------------------------------*/

unsigned int revision(void)
{
  return REV_300;
}

/*----------------------------------------------------------------
 * 
 * funciton: temperature_C()
 * 
 * brief: Read the temperature sensor and return temperature in degrees C
 * 
 *----------------------------------------------------------------
 *
 * See TI Documentation for LM75
 * http://www.ti.com/general/docs/suppproductinfo.tsp?distId=10&gotoUrl=http%3A%2F%2Fwww.ti.com%2Flit%2Fgpn%2Flm75b
 *
 * The output of the LM75 is a signed nine bit number 
 * -55C < temp < 125C
 * 
 *--------------------------------------------------------------*/
#define RTD_SCALE (0.5) // 1/2C / LSB

double temperature_C(void)
{
  double return_value;
  int raw; // Allow for negative temperatures
  int i;

  raw = 0xffff;

  /*
   *  Point to the temperature register
   */
  Wire.beginTransmission(TEMP_IC);
  Wire.write(0),
      Wire.endTransmission();

  /*
   * Read in the temperature register
   */
  Wire.requestFrom(TEMP_IC, 2);
  raw = Wire.read();
  raw <<= 8;
  raw += Wire.read();
  raw >>= 7;

  if (raw & 0x0100)
  {
    raw |= 0xFF00; // Sign extend
  }

  /*
   *  Return the temperature in C
   */
  return_value = (double)(raw)*RTD_SCALE;

  if (return_value < 5)
    return_value = 22;

#if (SAMPLE_CALCULATIONS)
  return_value = 23.0;
#endif

  return return_value;
}

