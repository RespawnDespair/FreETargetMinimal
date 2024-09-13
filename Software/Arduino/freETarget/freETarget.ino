#include "freETarget.h"
#include "gpio.h"
#include "compute_hit.h"
#include "analog_io.h"
#include "json.h"
#include "EEPROM.h"
#include "nonvol.h"
#include "mechanical.h"
#include "diag_tools.h"
#include "esp-01.h"
#include "timer.h"
#include "token.h"
#include <WiFi.h>

shot_record_t record[SHOT_STRING];          // Array of shot records
volatile unsigned int this_shot;            // Index into the shot array  
volatile unsigned int last_shot;            // Last shot processed.

double s_of_sound;                          // Speed of sound
unsigned int shot = 0;                      // Shot counter
unsigned int face_strike = 0;               // Miss Face Strike interrupt count
unsigned int is_trace = 0;                  // Turn off tracing
unsigned int rapid_count = 0;               // Number of shots to be expected in Rapid Fire
unsigned int tabata_state;                  // Tabata state
unsigned int shot_number;                   // Shot Identifier
volatile unsigned long keep_alive;          // Keep alive timer
volatile unsigned long tabata_rapid_timer;  // Tabita or Rapid fire timer
volatile unsigned long in_shot_timer;       // Time inside of the shot window
volatile unsigned long power_save;          // Power save timer
volatile unsigned long token_tick;          // Token ring watchdog
volatile unsigned long gpt;                 // General purpose timer

const char* namesensor[] = { "TARGET", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "DOC", "DOPEY", "HAPPY", "GRUMPY", "BASHFUL", "SNEEZEY", "SLEEPY", "RUDOLF", "DONNER", "BLITZEN", "DASHER", "PRANCER", "VIXEN", "COMET", "CUPID", "DUNDER", "ODIN", "WODEN", "THOR", "BALDAR", 0 };

const char nesw[] = "NESW";                  // Cardinal Points
const char to_hex[] = "0123456789ABCDEF";    // Quick Hex to ASCII
static void bye(void);                       // Say good night Gracie
static long tabata(bool reset_time);         // Tabata state machine
static bool discard_tabata(void);            // TRUE if the shot should be discarded 

#define TABATA_OFF          0         // No tabata cycles at all
#define TABATA_REST         1         // Tabata is doing nothing (typically 60 seconds)
#define TABATA_WARNING      2         // Time the warning LED is on (typically 2 seconds)
#define TABATA_DARK         3         // Time the warning LED is off before the shot (typically 2 seconds)
#define TABATA_ON           4         // Time the TABATA lED is on (typically 5 seconds)

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200); // Use Serial2 for ESP32 equivalent of AUX_SERIAL
    Serial1.begin(115200); // Use Serial1 for ESP32 equivalent of DISPLAY_SERIAL

    POST_version();                         // Show the version string on all ports

    init_gpio();  
    init_sensors();
    init_analog_io();
    init_timer();
    set_LED('*', '.', '.');                 // Hello World
    read_nonvol();
  
    is_trace = 0; // DLT_CRITICAL;          // Turn on tracing ToDo change back

    timer_new(&keep_alive, (unsigned long)json_keep_alive * ONE_SECOND); // Keep alive timer
    timer_new(&tabata_rapid_timer, 0);                                    // Free running tabata or rapid fire timer
    timer_new(&in_shot_timer, FULL_SCALE);                                // Time inside of the shot window
    timer_new(&power_save, (unsigned long)(json_power_save) * (long)ONE_SECOND * 60L);// Power save timer
    timer_new(&token_tick, 5 * ONE_SECOND);                              // Token ring watchdog
    timer_new(&gpt, 0);                                                  // General purpose timer
  
    randomSeed(analogRead(V_REFERENCE));   // Seed the random number generator
  
    set_LED('.', '.', '*');                // Hello World
    tabata(true);                          // Reset the Tabata timers
  
    while ((POST_counters() == false) && !DLT(DLT_CRITICAL)) {
        Serial.print(T("POST_2 Failed
"));
        blink_fault(POST_COUNT_FAILED);   // and try again
    }
  
    set_LED('.', '*', '.');                 // Hello World

    if (DLT(DLT_CRITICAL)) {
        Serial.print(T("Starting timers"));
    }

    enable_timer_interrupt();
     
    set_LED('*', '*', '.');                 // Hello World
    esp01_init();                         // Prepare the WiFi channel if installed
   
    multifunction_switch();
    set_LED('*', '*', '*');                 // Hello World
    set_LED_PWM(json_LED_PWM);
    POST_LEDs();                            // Cycle the LEDs
    set_LED(LED_READY);                     // to a client, then the RDY light is steady on
  
    if (DLT(DLT_CRITICAL)) {
        Serial.print(T("Clearing serial buffers"));
    }
    while (available_all()) {
        get_all();                                // Flush any garbage before we start up
    }
  
    DLT(DLT_CRITICAL); 
    Serial.print(T("Finished startup
"));
    show_echo();
}

void loop() {
    multifunction_switch();         // Read the switches
    tabata(false);                  // Update the Tabata state

    switch (json_token) {
        case TOKEN_WIFI:
            esp01_receive();
            break;

        case TOKEN_MASTER:
            if (token_tick == 0) {
                token_init();
                if (my_ring == TOKEN_UNDEF) {
                    token_tick = ONE_SECOND * 5;
                } else {
                    token_tick = ONE_SECOND * 60;
                }
            }

        case TOKEN_SLAVE:
            token_poll();
            break;
    }
    
    if (read_JSON()) {
        power_save = (long)json_power_save * 60L * (long)ONE_SECOND;
    }

    if ((json_keep_alive != 0) && (keep_alive == 0)) {
        send_keep_alive();
        keep_alive = json_keep_alive * ONE_SECOND;
    }

    if ((json_power_save != 0) && (power_save == 0)) {
        bye();
        power_save = (unsigned long)json_power_save * (unsigned long)ONE_SECOND * 60L;
        state = SET_MODE;
    }

    if ((state != old_state) && DLT(DLT_APPLICATION)) {
        Serial.print(T("Loop State: ")); Serial.print(loop_name[state]);
    } 
    old_state = state;
  
    switch (state) {
        default:
        case SET_MODE:
            state = set_mode();
            break;

        case ARM:
            state = arm();
            break;

        case WAIT:
            state = wait();
            break;

        case REDUCE:
            state = reduce();
            break;

        case FINISH:
            state = finish();
            break;
    }
}

