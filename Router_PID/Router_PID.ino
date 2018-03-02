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
int spindleEnable = 0;                        // Marlin Spindle enabled value

const int ROUTER_PWM_OUTPIN = 11;             // Triac output to router

// PID variables
double Setpoint = 0.0;                           
double Input = 0.0;
double Output = 0.0;

//PID Gains
double Kp=1.8;
double Ki=7.4;
double Kd=.215;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);                   // PID library

float MAX_TOOL_RPM = 30000;                                                  // SETTINGS per "spindle" ?1k headroom needed?

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

   int spindle_enable = digitalRead(SPINDLE_ENABLE_PIN); // Marlin spindle power control
    
   unsigned long rpm_math = (60000000 / rpm_value) - 1;  // Spindle, interrupt microseconds to RPM
   int optical_pwm = rpm_math * 255 / MAX_TOOL_RPM;      // Spindle, RPM to PWM
             
   Input = optical_pwm;                                  // PID Input from router
   Setpoint = map(pwm_value, 0, 2024, 0, 255);           // PID setpoit from Marlin
   myPID.Compute();                                      // PID Run the Loop

   if (spindle_enable == 0){                             // Marlin is spindle off?
       pwm_value = 0;                                    // Reset PID
       analogWrite(ROUTER_PWM_OUTPIN, 0);                // Turn off Spindle AC
       }
    else {                                               // If spindle is enabled write PID value to triac
       analogWrite(ROUTER_PWM_OUTPIN, Output);           // Out to AC control Triac
       }
   
      Serial.print("RPM = ");                            // Spindle, Display RPM
      Serial.println(rpm_math);                          // Spindle, Display RPM
}

// spindleRPM gets called on a falling edge of the PHOTO_PIN, and records the amount of time
// between edges in the rpm_value field (in microsecinds). ISr.
void spindleRPM() {
   current_rpm_time = micros();
   rpm_value = current_rpm_time - prev_rpm_time;
   prev_rpm_time = current_rpm_time;
   attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), spindleRPM, FALLING);
   }

// rising() is called on the rising edge of the PHOTO_PIN. Basically starts teh timer for measuring the
// PWM duty cycle. ISR.
void rising() {
   attachInterrupt(digitalPinToInterrupt(PWM_PIN), falling, FALLING);
   prev_time = micros();
   }

// falling() is called on the falling edgeof the PWM_PIN. Records teh amount of time since the
// rising(). ISR.  
 void falling() {
   attachInterrupt(digitalPinToInterrupt(PWM_PIN), rising, RISING);
   pwm_value = micros()-prev_time;
   //Serial.println(pwm_value);                                      // For debugging the PWM MAP
  }
