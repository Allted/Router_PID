/*
 * PWM PID control using an optical sensor, a PWM enable AC triac power control, LCD panel....more info soon.
 * 
 * Details for now, https://www.v1engineering.com/forum/topic/hardware-needed-for-a-software-fix/
 * 
 * V1 Engineering
 */

#include <PID_v1.h>                                  // Arduino library PID Library by Brett Beauregard

const long US_PERIOD_TO_RPM = 60000000;              // Convert from us of one rotation to RPM.

const int PHOTO_PIN = 3;                             // Spindle IR Sensor input pin
volatile unsigned long current_rpm_time = 0;         // Spindle RPM PWM input calc
volatile unsigned long prev_rpm_time = 0;            // Spindle RPM PWM input calc
volatile unsigned long rpm_value = US_PERIOD_TO_RPM; // Spindle RPM input in microseconds

const int PWM_PIN = 2;                               // Marlin PWMms Input pin
volatile unsigned long pwm_value = 0;                // Marlin PWMms value in microseconds
volatile unsigned long prev_time = 0;                // Marlin PWMms Math

const int SPINDLE_ENABLE_PIN = 5;                    // Marlin Spindle enabled pin 

const int ROUTER_PWM_OUTPIN = 11;                    // Triac output to router

// PID variables
double Setpoint = 0.0;                           
double Input = 0.0;
double Output = 0.0;

//PID Gains
double Kp=1.8;
double Ki=12;
double Kd=0.2;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);                   // PID library

const int MAX_TOOL_RPM = 30000;                                              // SETTINGS per "spindle" ?1k headroom needed?
const int MAX_PWM_INPUT_US = 2024;                                           // Settings the microseconds of teh max PWM from Marlin.

void setup()
{
    pinMode(PHOTO_PIN, INPUT_PULLUP);                                        // Spindle RPM input pin
    attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), spindleRPM, FALLING);  // Spindle RPM interrupt
    
    pinMode(PWM_PIN, INPUT);                                                 // Marlin PWM Input pin
    attachInterrupt(digitalPinToInterrupt(PWM_PIN), rising, RISING);         // Marlin PWM interrupt
    
    pinMode(SPINDLE_ENABLE_PIN, INPUT);                                      // Marlin Spindle enabled pin
       
    analogWrite(ROUTER_PWM_OUTPIN,0);                                        // Triac initial state = off safety? 
    
    myPID.SetMode(AUTOMATIC);                                                // PID auto/manual *TODO change with lcd button
    myPID.SetSampleTime(8.2);                                                // PID Loop time this is faster than 60Hz Double check
    
    Serial.begin(9600);
}

void loop() {

   int spindle_enable = digitalRead(SPINDLE_ENABLE_PIN);     // Marlin spindle power control
    
   // Compute the spindle's RPM value.
   unsigned long rpm_math = (US_PERIOD_TO_RPM / rpm_value)-1;// Spindle, interrupt microseconds to RPM
   unsigned int optical_pwm = rpm_math * 255 / MAX_TOOL_RPM; // Spindle, RPM to PWM
             
   // One iteration of the PID.
   Input = optical_pwm;                                      // PID Input from router
   Setpoint = map(pwm_value, 0, MAX_PWM_INPUT_US, 0, 255);   // PID setpoit from Marlin
   myPID.Compute();                                          // PID Run the Loop

   // Set the output of the speed controller.
   if (spindle_enable == 0){                                 // Marlin is spindle off?
       pwm_value = 0;                                        // Reset PID
       analogWrite(ROUTER_PWM_OUTPIN, 0);                    // Turn off Spindle AC
       rpm_value = US_PERIOD_TO_RPM;                         // Disregards the spindown but clears the PID
       }
    else {                                                   // If spindle is enabled write PID value to triac
       analogWrite(ROUTER_PWM_OUTPIN, Output);               // Out to AC control Triac
       }
   
      Serial.print("RPM = ");                                // Spindle, Display RPM
      Serial.println(rpm_math);                              // Spindle, Display RPM
}

// spindleRPM gets called on a falling edge of the PHOTO_PIN, and records the amount of time
// between edges in the rpm_value field (in microsecinds). ISr.
void spindleRPM() {
   // Capture the time at "now"
   current_rpm_time = micros();

   // Store the microseconds since the last sample.
   rpm_value = current_rpm_time - prev_rpm_time;

   // Start a new "timer" by setting the previous to "now".
   prev_rpm_time = current_rpm_time;

   // Call this function on the next falling edge.
   attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), spindleRPM, FALLING);
   }

// rising() is called on the rising edge of the PHOTO_PIN. Basically starts the timer for measuring the
// PWM duty cycle. ISR.
void rising() {
   // Capture when this is rising.
   prev_time = micros();

   // Set teh next interrupt.
   attachInterrupt(digitalPinToInterrupt(PWM_PIN), falling, FALLING);
   }

// falling() is called on the falling edgeof the PWM_PIN. Records the amount of time since the
// rising(). ISR.  
 void falling() {
   // Measure the time since teh last rising edge, in microseconds.
   pwm_value = micros()-prev_time;

   // Set the next interrupt.
   attachInterrupt(digitalPinToInterrupt(PWM_PIN), rising, RISING);
   //Serial.println(pwm_value);                                      // For debugging the PWM MAP
  }
