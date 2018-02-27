/*
 * PWM PID control using an optical sensor, a PWM enable AC triac power control, LCD panel....more info soon.
 *
 * Details for now, https://www.v1engineering.com/forum/topic/hardware-needed-for-a-software-fix/
 *
 * V1 Engineering
 */

#include <PID_v1.h>                                 // Arduino library PID Library by Brett Beauregard

const int US_PERIOD_TO_RPM = 60 * 1000 * 1000;      // Convert from us of one rotation to RPM.

const int PHOTO_PIN = 3;                            // Spindle IR Sensor input pin
volatile unsigned long current_rpm_time = 0;        // Spindle RPM PWM input calc
volatile unsigned long prev_rpm_time = 0;           // Spindle RPM PWM input calc
volatile unsigned long rpm_value = US_PERIOD_TO_RPM;// Spindle RPM input in microseconds

const int PWM_PIN = 2;                              // Marlin PWMms Input pin
volatile unsigned long pwm_value = 0;               // Marlin PWMms value in microseconds
volatile unsigned long prev_time = 0;               // Marlin PWMms Math

const int SPINDLE_ENABLE_PIN = 5;                   // Marlin Spindle enabled pin

const int ROUTER_PWM_OUT_PIN = 11;                  // Triac output to router

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
const int MAX_PWM_INPUT_US = 2024;                      // SETTINGS the microseconds of the max PWM from Marlin.

void setup()
{
    pinMode(PHOTO_PIN, INPUT_PULLUP);                                        // Spindle RPM input pin
    attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), spindleRPM, FALLING);  // Spindle RPM interrupt

    pinMode(PWM_PIN, INPUT);                                             // Marlin PWM Input pin
    attachInterrupt(digitalPinToInterrupt(PWM_PIN), rising, RISING);     // Marlin PWM interrupt

    pinMode(SPINDLE_ENABLE_PIN, INPUT);                                  // Marlin Spindle enabled pin

    analogWrite(ROUTER_PWM_OUT_PIN,0);                                   // Triac initial state = off safety?

    my_pid.SetMode(MANUAL);                                              // PID auto/manual *TODO change with lcd button
    const double SAMPLE_TIME_MS = 16.6;                                  // PID Loop Period, in ms. 16.6 is just faster than 60Hz.
    my_pid.SetSampleTime(SAMPLE_TIME_MS);                                // PID Loop time

    Serial.begin(9600);
}

void loop() {

    int spindle_enabled = digitalRead(SPINDLE_ENABLE_PIN);          // Marlin spindle power control

    // Compute the spindle's RPM value.
    unsigned long rpm_math = (US_PERIOD_TO_RPM / rpm_value) - 1;    // Spindle, interrupt microseconds to RPM
    int optical_pwm = rpm_math * 255 / MAX_TOOL_RPM;                // Spindle, RPM to PWM

    Serial.print("RPM = ");                                         // Spindle, Display RPM ---LCD
    Serial.println(rpm_math);                                       // Spindle, Display RPM ---LCD

    // Set the output to the speed controller.
    if (spindle_enabled == 0) {                                     // Marlin is spindle off?
        pwm_value = 0;                                              // Reset set point
        analogWrite(ROUTER_PWM_OUT_PIN, 0);                         // Turn off Spindle AC
        my_pid.SetMode(MANUAL);                                     // Reset PID
    }
    else {                                                          // If spindle is enabled write PID value to triac

        if (my_pid.GetMode() == MANUAL) {
            // TODO, if you want actual manual control, then we will need to not do this step.
            // Depending on the LCD stuff.
            my_pid.SetMode(AUTOMATIC);                              // Turn the PID back on
        }

        // One iteration of the PID.
        input = optical_pwm;                                        // PID Input from router
        set_point = map(pwm_value, 0, MAX_PWM_INPUT_US, 0, 255);    // PID SetPoint from Marlin
        my_pid.Compute();                                           // PID Run the Loop

        analogWrite(ROUTER_PWM_OUT_PIN, output);                    // Out to AC control Triac
    }
}

// spindleRPM gets called on a falling edge of the PHOTO_PIN, and records the amount of time
// between edges in the rpm_value field (in microseconds). ISR.
void spindleRPM() {
    // Capture the time at "now"
    current_rpm_time = micros();

    // Store the microseconds since the last sample.
    rpm_value = current_rpm_time - prev_rpm_time;

    // Start a new "timer" by setting the previous to "now"
    prev_rpm_time = current_rpm_time;

    // Call this function on the next falling edge.
    attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), spindleRPM, FALLING);
}

// rising() is called on the rising edge of PWM_PIN. Basically starts the timer for measuring the
// PWM duty cycle. ISR.
void rising() {                                                      // Marlin check incoming PWM
    // Capture when this is rising.
    prev_time = micros();

    // Set the next interrupt.
    attachInterrupt(digitalPinToInterrupt(PWM_PIN), falling, FALLING);
}

// falling() is called on the falling edge of PWM_PIN. Records the amount of time since the
// rising(). ISR.
void falling() {
    // Measure the time since the last rising edge, in microseconds.
    pwm_value = micros() - prev_time;

    // Set the next interrupt.
    attachInterrupt(digitalPinToInterrupt(PWM_PIN), rising, RISING);

    //Serial.println(pwm_value);                                      // For debugging the PWM MAP
}
