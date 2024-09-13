#include "freETarget.h"
#include "gpio.h"
#include "token.h"
#include "json.h"
#include <driver/gpio.h>
#include <esp_timer.h>

const char* which_one[4] = {"North:", "East:", "South:", "West:"};

#define TICK(x) (((x) / 0.33) * OSCILLATOR_MHZ)
#define RX(Z,X,Y) (16000 - (sqrt(sq(TICK(x)-sensor[(Z)].x_tick) + sq(TICK(y)-sensor[(Z)].y_tick))))
#define GRID_SIDE 25
#define TEST_SAMPLES ((GRID_SIDE)*(GRID_SIDE))

static void show_analog_on_PC(int v);
static void unit_test(unsigned int mode);
static bool sample_calculations(unsigned int mode, unsigned int sample);
static void log_sensor(int sensor);
extern int json_clock[4];

unsigned int tick;
void self_test(uint16_t test) {
    double volts;
    unsigned int i;
    char ch;
    unsigned int sensor_status;
    unsigned long sample;
    unsigned int random_delay;
    bool pass;
    unsigned long start_time;
    shot_record_t shot;
    unsigned char s[128];

    tick++;
    volts = TO_VOLTS(analogRead(V_REFERENCE));

    switch (test) {
        default:
        case T_HELP:
            Serial.print(T("
 1 - Digital inputs"));
            Serial.print(T("
 2 - Counter values (external trigger)"));
            if (revision() >= REV_220) {
                Serial.print(T("
 3 - Counter values (internal trigger)"));
            }
            Serial.print(T("
 4 - Oscilloscope"));
            Serial.print(T("
 5 - Oscilloscope (PC)"));
            Serial.print(T("
 6 - Advance paper backer"));
            Serial.print(T("
 7 - Spiral Unit Test"));
            Serial.print(T("
 8 - Grid calibration pattern"));
            Serial.print(T("
 9 - One time calibration pattern"));
            if (revision() >= REV_220) {
                Serial.print(T("
10 - Aux port passthrough"));
            }
            Serial.print(T("
11 - Calibrate"));
            Serial.print(T("
12 - Transfer loopback"));
            Serial.print(T("
13 - Serial port test"));
            Serial.print(T("
14 - LED brightness test"));
            Serial.print(T("
15 - Face strike test"));
            Serial.print(T("
16 - WiFi test"));
            Serial.print(T("
17 - Dump NonVol"));
            Serial.print(T("
18 - Send sample shot record"));
            Serial.print(T("
19 - Show WiFi status"));
            Serial.print(T("
20 - Send clock out of all serial ports"));
            Serial.print(T("
21 - Log North Sensor"));
            Serial.print(T("
22 - Log East Sensor"));
            Serial.print(T("
23 - Log South Sensor"));
            Serial.print(T("
24 - Log West Sensor"));
            Serial.print(T("
25 - Test Push Buttons"));
            Serial.print(T("
26 - Unit Test speed_of_sound()"));
            Serial.print(T("
27 - Token Ring Test()"));
            Serial.print(T("
28 - Count on the LEDs"));
            Serial.print(T("
29 - Force calculations"));
            Serial.print(T("
"));
            break;

        case T_DIGITAL:
            digital_test();
            break;

        case T_TRIGGER:
            Serial.print(T("
Waiting for Trigger
"));
        case T_CLOCK:
            stop_timers();
            arm_timers();

            set_LED(L('*', '-', '-'));

            if (test == T_CLOCK) {
                if (revision() >= REV_220) {
                    random_delay = random(1, 6000);
                    Serial.print(T("
Random clock test: "));
                    Serial.print(random_delay);
                    Serial.print(T("us. All outputs must be the same. "));
                    trip_timers();
                    delayMicroseconds(random_delay);
                } else {
                    Serial.print(T("
This test not supported on this hardware revision"));
                    break;
                }
            }

            while (is_running() == B00001111) {
                continue;
            }
            sensor_status = is_running();
            stop_timers();
            read_timers(&shot.timer_count[0]);

            if (test == T_CLOCK) {
                sample = shot.timer_count[N];
                pass = true;

                for (i = N; i <= W; i++) {
                    if (shot.timer_count[i] != sample) {
                        pass = false;
                    }
                }

                if (pass == true) {
                    Serial.print(T(" PASS
"));
                } else {
                    Serial.print(T(" FAIL
"));
                }
            }
            send_timer(sensor_status);

            set_LED(L('-', '-', '-'));
            delay(ONE_SECOND);
            break;

        case T_OSCOPE:
            show_analog(0);
            break;

        case T_OSCOPE_PC:
            show_analog_on_PC(0);
            break;

        case T_PAPER:
            Serial.print(T("
Advancing backer paper "));
            Serial.print(((json_paper_time) + (json_step_time)));
            Serial.print(T(" ms  "));
            Serial.print(json_step_count);
            Serial.print(T(" steps"));
            drive_paper();
            Serial.print(T("
Done"));
            break;

        case T_SPIRAL:
            Serial.print(T("
Spiral Calculation
"));
            unit_test(T_SPIRAL);
            break;

        case T_GRID:
            Serial.print(T("
Grid Calculation
"));
            unit_test(T_GRID);
            break;

        case T_ONCE:
            Serial.print(T("
Single Calculation
"));
            unit_test(T_ONCE);
            break;

        case T_PASS_THRU:
            Serial.print(T("
Pass through active.  Cycle power to exit
"));
            while (1) {
                if (Serial.available()) {
                    ch = Serial.read();
                    AUX_SERIAL.print(ch);
                }
                if (aux_spool_available()) {
                    ch = aux_spool_read();
                    Serial.print(ch);
                }
            }
            break;

        case T_SERIAL_PORT:
            Serial.print(T("
Arduino Serial Port: Hello World
"));
            AUX_SERIAL.print(T("
Aux Serial Port: Hello World
"));
            DISPLAY_SERIAL.print(T("
Display Serial Port: Hello World
"));
            break;

        case T_LED:
            Serial.print(T("
Ramping the LED"));
            for (i = 0; i != 256; i++) {
                analogWrite(LED_PWM, i);
                delay(ONE_SECOND / 50);
            }
            for (i = 255; i != -1; i--) {
                analogWrite(LED_PWM, i);
                delay(ONE_SECOND / 50);
            }
            analogWrite(LED_PWM, 0);
            Serial.print(T(" Done
"));
            break;

        case T_FACE:
            Serial.print(T("
Face strike test
"));
            face_strike = 0;
            sample = 0;
            enable_face_interrupt();
            ch = 0;

            while (ch != '!') {
                if (face_strike != 0) {
                    face_strike--;
                    set_LED(L('*', '*', '*'));
                } else {
                    set_LED(L('.', '.', '.'));
                }
                if (sample != face_strike) {
                    if ((face_strike % 20) == 0) {
                        Serial.print(T("
"));
                    }
                    Serial.print(T(" S:"));
                    Serial.print(face_strike);
                    sample = face_strike;
                }
                esp01_receive();
                ch = get_all();
            }
            Serial.print(T("
Done
"));
            break;

        case T_WIFI:
            esp01_test();
            break;

        case T_NONVOL:
            dump_nonvol();
            break;

        case T_SHOT:
            if (json_B == 0) {
                json_B = 1;
            }
            shot.shot_number = shot_number;
            for (i = 0; i != json_B; i++) {
                if (json_A != 0) {
                    shot.xphys_mm = (double)json_A / 2.0d - (double)random(json_A);
                    shot.yphys_mm = (double)json_A / 2.0d - (double)random(json_A);
                } else {
                    shot.xphys_mm = 10;
                    shot.yphys_mm = 20;
                }
                shot.shot_time = (FULL_SCALE - in_shot_timer);
                send_score(&shot);
                shot.shot_number++;
            }
            break;

        case T_WIFI_STATUS:
            esp01_status();
            break;

        case T_WIFI_BROADCAST:
            sprintf(s, "Type ! to exit ");
            output_to_all(s);
            ch = 0;
            while (ch != '!') {
                i = millis() / 1000;
                if ((i % 60) == 0) {
                    sprintf(s, "
%d:%d ", i / 60, i % 60);
                    output_to_all(s);
                } else {
                    sprintf(s, " %d:%d ", i / 60, i % 60);
                    output_to_all(s);
                }
                esp01_receive();
                ch = get_all();
                delay(1000);
            }
            sprintf(s, "
Done");
            break;

        case T_LOG + 0:
        case T_LOG + 1:
        case T_LOG + 2:
        case T_LOG + 3:
            log_sensor(test - T_LOG);
            break;

        case T_SWITCH:
            ch = 0;
            while (ch != '!') {
                set_LED(1, DIP_SW_A, DIP_SW_B);
                esp01_receive();
                ch = get_all();
            }
            Serial.print(T("
Done"));
            break;

        case T_S_OF_SOUND:
            sound_test();
            Serial.print(T("
Done"));
            break;

        case T_TOKEN:
            token_init();
            Serial.print(T("
Done"));
            break;

        case T_LED_CYCLE:
            i = 0;
            while (!json_spool_available()) {
                set_LED((i >> 0) & 1, (i >> 1) & 1, (i >> 2) & 1);
                delay(ONE_SECOND / 2);
                i++;
                token_poll();
            }
            Serial.print(T("
Done"));
            break;

        case T_FORCE_CALC:
            Serial.print(T("
Shot Test using entered counts."));
            is_trace = 255;

            set_mode();
            arm();
            this_shot = 1;
            last_shot = 0;
            record[last_shot].shot_number = 99;
            record[last_shot].xphys_mm = 0;
            record[last_shot].yphys_mm = 0;
            if (json_A != 0) {
                record[last_shot].timer_count[N] = json_A;
                record[last_shot].timer_count[E] = json_B;
                record[last_shot].timer_count[S] = json_C;
                record[last_shot].timer_count[W] = json_D;
            } else {
                record[last_shot].timer_count[N] = 20465;
                record[last_shot].timer_count[E] = 20640;
                record[last_shot].timer_count[S] = 20565;
                record[last_shot].timer_count[W] = 20474;
            }
            record[last_shot].face_strike = 0;
            record[last_shot].sensor_status = 0xf;
            record[last_shot].shot_time = micros();

            reduce();
            finish();
            shot.shot_time = (FULL_SCALE - in_shot_timer);
            shot.shot_number = 99;
            is_trace = 0;
            Serial.print(T("
Done."));
            break;
    }

    return;
}

void POST_version(void) {
    int i;
    char str[64];
    sprintf(str, "
freETarget %s
", SOFTWARE_VERSION);
    output_to_all(str);

    return;
}

void POST_LEDs(void) {
    if (DLT(DLT_CRITICAL)) {
        Serial.print(T("POST LEDs"));
    }

    set_LED(L('*', '.', '.'));
    delay(ONE_SECOND / 4);
    set_LED(L('.', '*', '.'));
    delay(ONE_SECOND / 4);
    set_LED(L('.', '.', '*'));
    delay(ONE_SECOND / 4);
    set_LED(L('.', '.', '.'));

    return;
}

bool POST_counters(void) {
    unsigned int i, j;
    unsigned int random_delay;
    unsigned int sensor_status;
    int x;
    bool test_passed;
    unsigned long now;

    if (revision() < REV_300) {
        return true;
    }

    if (DLT(DLT_CRITICAL)) {
        Serial.print(T("POST_counters()"));
    }

    test_passed = true;

    stop_timers();
    arm_timers();
    delay(1);
    sensor_status = is_running();
    if ((sensor_status != 0) && DLT(DLT_CRITICAL)) {
        Serial.print(T("
Failed Clock Test. Spurious trigger:"));
        show_sensor_status(sensor_status, 0);
        return false;
    }

    for (i = 0; i != POST_counteres_cycles; i++) {
        stop_timers();
        arm_timers();
        delay(1);

        for (j = N; j <= W; j++) {
            if ((read_counter(j) != 0) && DLT(DLT_CRITICAL)) {
                Serial.print(T("Failed Clock Test. Counter free running:"));
                Serial.print(nesw[j]);
                test_passed = false;
            }
        }

        stop_timers();
        arm_timers();
        delay(1);
        random_delay = random(1, 6000);
        now = micros();
        trip_timers();
        sensor_status = is_running();

        while (micros() < (now + random_delay)) {
            continue;
        }

        stop_timers();
        if ((sensor_status != 0x0F) && DLT(DLT_CRITICAL)) {
            Serial.print(T("Failed Clock Test. sensor_status:"));
            show_sensor_status(sensor_status, 0);
            test_passed = false;
        }

        random_delay *= 8;
        for (j = N; j <= W; j++) {
            x = read_counter(j);
            if ((read_counter(j) != x) && DLT(DLT_CRITICAL)) {
                Serial.print(T("Failed Clock Test. Counter did not stop:"));
                Serial.print(nesw[j]);
                show_sensor_status(sensor_status, 0);
                test_passed = false;
            }

            x = x - random_delay;
            if (x < 0) {
                x = -x;
            }

            if ((x > CLOCK_TEST_LIMIT) && DLT(DLT_CRITICAL)) {
                Serial.print(T("Failed Clock Test. Counter:"));
                Serial.print(nesw[j]);
                Serial.print(T(" Is:"));
                Serial.print(read_counter(j));
                Serial.print(T(" Should be:"));
                Serial.print(random_delay);
                Serial.print(T(" Delta:"));
                Serial.print(x);
                test_passed = false;
            }
        }
    }

    set_LED(L('.', '.', '.'));
    return test_passed;
}

void show_analog(int v) {
    unsigned int i, sample;
    char o_scope[FULL_SCALE];
    unsigned long now;

    set_LED((1 << cycle) & 1, (1 << cycle) & 2, (1 << cycle) & 4);
    cycle = (cycle + 1) % 4;

    for (i = 0; i != FULL_SCALE; i++) {
        o_scope[i] = ' ';
    }
    o_scope[FULL_SCALE - 1] = 0;

    i = analogRead(V_REFERENCE) * SCALE;
    o_scope[i] = '|';

    max_input[N] = 0;
    max_input[E] = 0;
    max_input[S] = 0;
    max_input[W] = 0;
    now = micros();
    while ((micros() - now) <= SAMPLE_TIME) {
        for (i = N; i <= W; i++) {
            sample = analogRead(channel[i]) * SCALE;
            if (sample >= FULL_SCALE - 1) {
                sample = FULL_SCALE - 2;
            }
            if (sample > max_input[i]) {
                max_input[i] = sample;
            }
        }
    }

    for (i = N; i <= W; i++) {
        o_scope[max_input[i]] = nesw[i];
    }

    Serial.print(T("{\"OSCOPE\": "));
    Serial.print(o_scope);
    Serial.print(T("\"}
"));

    return;
}

static void show_analog_on_PC(int v) {
    unsigned int i, j, k;
    char o_scope[FULL_SCALE];

    Serial.print(T("
{Ref:"));
    Serial.print(TO_VOLTS(analogRead(V_REFERENCE)));
    Serial.print(T("  "));

    for (i = N; i != W + 1; i++) {
        Serial.print(which_one[i]);
        if (max_input[i] != 0) {
            max_input[i]--;
        }

        j = analogRead(channel[i]) * SCALE;
        if ((j * DECAY_RATE) > max_input[i]) {
            max_input[i] = j * DECAY_RATE;
        }

        if (j > FULL_SCALE - 1) {
            j = FULL_SCALE - 1;
        }

        for (k = 0; k != FULL_SCALE; k++) {
            o_scope[k] = ' ';
        }
        o_scope[j] = '*';
        o_scope[(max_input[i]) / DECAY_RATE] = '#';
        o_scope[FULL_SCALE - 1] = 0;

        Serial.print(o_scope);
    }
    Serial.print(T("}"));

    return;
}

static void unit_test(unsigned int mode) {
    unsigned int i;
    unsigned int location;
    unsigned int shot_number;

    init_sensors();
    shot_number = 1;
    for (i = 0; i != TEST_SAMPLES; i++) {
        if (sample_calculations(mode, i)) {
            location = compute_hit(&record[0]);
            sensor_status = 0xF;
            record[0].shot_number = shot_number++;
            send_score(&record[0]);
            delay(ONE_SECOND / 2);
        }
        if (mode == T_ONCE) {
            break;
        }
    }

    return;
}

static bool sample_calculations(unsigned int mode, unsigned int sample) {
    double x, y;
    double angle;
    double radius;
    double polar;
    int ix, iy;
    double step_size;
    double grid_step;
    shot_record_t shot;

    switch (mode) {
        case T_ONCE:
            angle = 0;
            radius = json_sensor_dia / sqrt(2.0d) / 2.0d;

            x = radius * cos(angle);
            y = radius * sin(angle);
            shot.timer_count[N] = RX(N, x, y);
            shot.timer_count[E] = RX(E, x, y);
            shot.timer_count[S] = RX(S, x, y);
            shot.timer_count[W] = RX(W, x, y);
            shot.timer_count[W] -= 200;

            Serial.print(T("
Result should be: "));
            Serial.print(T("x:"));
            Serial.print(x);
            Serial.print(T(" y:"));
            Serial.print(y);
            Serial.print(T(" radius:"));
            Serial.print(radius);
            Serial.print(T(" angle:"));
            Serial.print(angle * 180.0d / PI);
            break;

        default:
        case T_SPIRAL:
            angle = (PI_ON_4) / 5.0 * ((double)sample);
            radius = 0.99d * (json_sensor_dia / 2.0) / sqrt(2.0d) * (double)sample / TEST_SAMPLES;

            x = radius * cos(angle);
            y = radius * sin(angle);
            shot.timer_count[N] = RX(N, x, y);
            shot.timer_count[E] = RX(E, x, y);
            shot.timer_count[S] = RX(S, x, y);
            shot.timer_count[W] = RX(W, x, y);
            break;

        case T_GRID:
            radius = 0.99d * (json_sensor_dia / 2.0d / sqrt(2.0d));
            grid_step = radius * 2.0d / (double)GRID_SIDE;

            ix = -GRID_SIDE / 2 + (sample % GRID_SIDE);
            iy = GRID_SIDE / 2 - (sample / GRID_SIDE);

            x = (double)ix * grid_step;
            y = (double)iy * grid_step;
            polar = sqrt(sq(x) + sq(y));
            angle = atan2(y, x) - (PI * json_sensor_angle / 180.0d);
            x = polar * cos(angle);
            y = polar * sin(angle);

            if (sqrt(sq(x) + sq(y)) > radius) {
                return false;
            }

            shot.timer_count[N] = RX(N, x, y);
            shot.timer_count[E] = RX(E, x, y);
            shot.timer_count[S] = RX(S, x, y);
            shot.timer_count[W] = RX(W, x, y);
            break;
    }

    return true;
}

void show_sensor_status(unsigned int sensor_status, shot_record_t* shot) {
    unsigned int i;

    Serial.print(T(" Latch:"));

    for (i = N; i <= W; i++) {
        if (sensor_status & (1 << i)) Serial.print(nesw[i]);
        else Serial.print(T("."));
    }

    if (shot != 0) {
        Serial.print(" Timers:");

        for (i = N; i <= W; i++) {
            Serial.print(T(" "));
            Serial.print(nesw[i]);
            Serial.print(T(":"));
            Serial.print(shot->timer_count[i]);
        }
    }

    Serial.print(T("  Face Strike:"));
    Serial.print(face_strike);

    Serial.print(T("  V_Ref:"));
    Serial.print(TO_VOLTS(analogRead(V_REFERENCE)));

    Serial.print(T("  Temperature:"));
    Serial.print(temperature_C());

    Serial.print(T("  WiFi:"));
    Serial.print(esp01_is_present());

    Serial.print(T("  Switch:"));

    if (DIP_SW_A == 0) {
        Serial.print(T("--"));
    } else {
        Serial.print(T("A1"));
    }
    Serial.print(T(" "));
    if (DIP_SW_B == 0) {
        Serial.print(T("--"));
    } else {
        Serial.print(T("B2"));
    }

    if (((sensor_status & 0x0f) == 0x0f) && (face_strike != 0)) {
        Serial.print(T(" PASS"));
        delay(ONE_SECOND);
    }

    return;
}

void log_sensor(int sensor_n) {
    unsigned int i;
    unsigned long start;
    unsigned int max_cycle;
    unsigned int max_all;
    unsigned int sample;
    unsigned int sensor_status;
    char s[128];
    char ch;
    bool is_new;

    sprintf(s, "
Logging %s Use X to reset,  ! to exit
", which_one[sensor_n]);
    output_to_all(s);
    output_to_all(0);
    max_all = 0;
    arm_timers();

    while (1) {
        start = LOG_TIME;
        max_cycle = analogRead(channel[sensor_n]);
        is_new = false;
        while ((--start)) {
            sample = analogRead(channel[sensor_n]);
            if (sample > max_cycle) {
                max_cycle = sample;
                is_new = true;
            }
            if (sample > max_all) {
                max_all = sample;
                is_new = true;
            }
        }

        sensor_status = is_running();
        if (is_new || (sensor_status != 0)) {
            sprintf(s, "
%s cycle:%d  max:%d is_running:", which_one[sensor_n], max_cycle, max_all);
            output_to_all(s);

            s[1] = 0;
            for (i = N; i <= W; i++) {
                if (sensor_status & (1 << i)) s[0] = nesw[i];
                else s[0] = '.';
                output_to_all(s);
            }
            arm_timers();
        }

        while (available_all()) {
            ch = get_all();
            switch (ch) {
                case '!':
                    sprintf(s, "
Done");
                    output_to_all(s);
                    return;

                case 'x':
                case 'X':
                    max_all = 0;
                    max_cycle = 0;
                    break;
            }
        }
    }

    return;
}

bool do_dlt(unsigned int level) {
    char s[20], str[20];

    if ((level & (is_trace | DLT_CRITICAL)) == 0) {
        return false;
    }

    dtostrf(micros() / 1000000.0, 7, 6, str);
    sprintf(s, "
%s: ", str);
    Serial.print(s);

    return true;
}

