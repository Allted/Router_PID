/*
 * PWM PID control using an optical sensor, a PWM enable AC triac power control, LCD panel....more info soon.
 *
 * Details for now, https://www.v1engineering.com/forum/topic/hardware-needed-for-a-software-fix/
 *
 * V1 Engineering
 */

#include <PID_v1.h>                           // Arduino library PID Library by Brett Beauregard

const int PHOTO_PIN = 3;                      // Spindle IR Sensor input pin
volatile unsigned long current_rpm_time = 0;  // Spindle RPM PWM input calc
volatile unsigned long prev_rpm_time = 0;     // Spindle RPM PWM input calc
volatile unsigned long rpm_value = 60000000;  // Spindle RPM input in microseconds

const int PWM_PIN = 2;                        // Marlin PWMms Input pin
volatile unsigned long pwm_value = 0;         // Marlin PWMms value in microseconds
volatile unsigned long prev_time = 0;         // Marlin PWMms Math

const int SPINDLE_ENABLE_PIN = 5;             // Marlin Spindle enabled pin

const int ROUTER_PWM_OUT_PIN = 11;            // Triac output to router

// PID variables
double set_point = 0.0;
double input = 0.0;
double output = 0.0;

// PID Gains
double k_p = 1.8;
double k_i = 7.4;
double k_d = .215;

PID my_pid(&input, &output, &set_point, k_p, k_i, k_d, DIRECT); // PID library

const int MAX_TOOL_RPM = 30000;                         // SETTINGS per "spindle" ?1k headroom needed?

void setup()
{
    pinMode(PHOTO_PIN, INPUT_PULLUP);                                        // Spindle RPM input pin
    attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), spindleRPM, FALLING);  // Spindle RPM interrupt

    pinMode(PWM_PIN, INPUT);                                             // Marlin PWM Input pin
    attachInterrupt(digitalPinToInterrupt(PWM_PIN), rising, RISING);     // Marlin PWM interrupt

    pinMode(SPINDLE_ENABLE_PIN, INPUT);                                  // Marlin Spindle enabled pin

    analogWrite(ROUTER_PWM_OUT_PIN,0);                                   // Triac initial state = off safety?

    my_pid.SetMode(AUTOMATIC);                                           // PID auto/manual *TODO change with lcd button
    my_pid.SetSampleTime(16.6);                                          // PID Loop time this is faster than 60Hz Double check

    Serial.begin(9600);
}

void loop() {

    int spindle_enabled = digitalRead(SPINDLE_ENABLE_PIN);  // Marlin spindle power control

    unsigned long rpm_math = (60000000 / rpm_value) - 1;    // Spindle, interrupt microseconds to RPM
    int optical_pwm = (rpm_math / MAX_TOOL_RPM) * 255;      // Spindle, RPM to PWM
    Serial.print("RPM = ");                                 // Spindle, Display RPM ---LCD
    Serial.println(rpm_math);                               // Spindle, Display RPM ---LCD

    input = optical_pwm;                                    // PID Input from router
    set_point = map(pwm_value, 0, 2024, 0, 255);            // PID SetPoint from Marlin
    my_pid.Compute();                                       // PID Run the Loop

    if (spindle_enabled == 0) {                             // Marlin is spindle off?
        pwm_value = 0;                                      // Reset PID
        analogWrite(ROUTER_PWM_OUT_PIN, 0);                 // Turn off Spindle AC
    }
    else {                                                  // If spindle is enabled write PID value to triac
        analogWrite(ROUTER_PWM_OUT_PIN, output);            // Out to AC control Triac
    }
}

// spindleRPM gets called on a falling edge of the PHOTO_PIN, and records the amount of time
// between edges in the rpm_value field (in microseconds). ISR.
void spindleRPM() {
    current_rpm_time = micros();
    rpm_value = current_rpm_time - prev_rpm_time;
    prev_rpm_time = current_rpm_time;
    attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), spindleRPM, FALLING);
}

// rising() is called on the rising edge of PWM_PIN. Basically starts the timer for measuring the
// PWM duty cycle. ISR.
void rising() {                                                      // Marlin check incoming PWM
    attachInterrupt(digitalPinToInterrupt(PWM_PIN), falling, FALLING);
    prev_time = micros();
}

// falling() is called on the falling edge of PWM_PIN. Records the amount of time since the
// rising(). ISR.
void falling() {
    attachInterrupt(digitalPinToInterrupt(PWM_PIN), rising, RISING);
    pwm_value = micros() - prev_time;
    //Serial.println(pwm_value);                                      // For debugging the PWM MAP
}
