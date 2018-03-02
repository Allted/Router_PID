/*
 * PWM PID control using an optical sensor, a PWM enable AC triac power control, LCD panel....more info soon.
 * 
 * Details for now, https://www.v1engineering.com/forum/topic/hardware-needed-for-a-software-fix/
 * 
 * V1 Engineering
 */

#include <PID_v1.h>                           // Arduino library PID Library by Brett Beauregard

const int Photo = 3;                          // Spindle IR Sensor input pin
volatile unsigned long current_rpm_time = 0;  // Spindle RPM PWM input calc
volatile unsigned long prev_rpmTime = 0;      // Spindle RPM PWM input calc
volatile unsigned long rpm_value = 60000000;  // Spindle RPM input in microseconds
unsigned long RPMmath;                        // Spindle interrupt to RPM
int opticalPWM;                               // Spindle RPM to PWM

const int PWM_PIN = 2;                        // Marlin PWMms Input pin
volatile unsigned long pwm_value;             // Marlin PWMms value in microseconds
volatile unsigned long prev_time = 0;         // Marlin PWMms Math

const int SpindleEnablePIN = 5;               // Marlin Spindle enabled pin 
int spindleEnable = 0;                        // Marlin Spindle enabled value
int MarlinConverted;                          // Marlin PWM value

const int routerPWMout = 11;                  // Triac output to router

double Setpoint, Input, Output;                            // PID variables
double Kp=1.8, Ki=7.4, Kd=.215;                            // PID P.I.D.
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // PID library

float ToolRPM = 30000;                         // SETTINGS per "spindle" ?1k headroom needed?

void setup()
{
    pinMode(Photo, INPUT_PULLUP);                                        // Spindle RPM input pin
    attachInterrupt(digitalPinToInterrupt(Photo), spindleRPM, FALLING);  // Spindle RPM interrupt
    
    pinMode(PWM_PIN, INPUT);                                             // Marlin PWM Input pin
    attachInterrupt(digitalPinToInterrupt(PWM_PIN), rising, RISING);     // Marlin PWM interrupt
    
    pinMode(SpindleEnablePIN, INPUT);                                    // Marlin Spindle enabled pin
       
    analogWrite(routerPWMout,0);                                         // Triac initial state = off safety? 
    
    myPID.SetMode(AUTOMATIC);                                            // PID auto/manual *TODO change with lcd button
    myPID.SetSampleTime(16.6);                                           // PID Loop time this is faster than 60Hz Double check
    
    Serial.begin(9600);
}

void loop() {

   spindleEnable = digitalRead(SpindleEnablePIN);      // Marlin spindle power control
    
   RPMmath = (60000000 / rpm_value) - 1;               // Spindle, interrupt microseconds to RPM
   opticalPWM = (RPMmath / ToolRPM) * 255;             // Spindle, RPM to PWM
             
   MarlinConverted = map(pwm_value, 0, 2024, 0, 255);  // Marlin, scale PWM_ms to pwm value
   
   Input = opticalPWM;                                 // PID Input from router
   Setpoint = MarlinConverted;                         // PID SetPoint from Marlin
   myPID.Compute();                                    // PID Run the Loop

   if (spindleEnable == 0){                            // Marlin is spindle off?
       pwm_value = 0;                                  // Reset PID
       analogWrite(routerPWMout, 0);                   // Turn off Spindle AC
       }
    else {                                             // If spindle is enabled write PID value to triac
       analogWrite(routerPWMout, Output);              // Out to AC control Triac
       }
   
      Serial.print("RPM = ");                          // Spindle, Display RPM
      Serial.println(RPMmath);                         // Spindle, Display RPM
}

void spindleRPM() {                                                  // Optical RPM sensor in microseconds
   current_rpm_time = micros();
   rpm_value = current_rpm_time - prev_rpmTime;
   prev_rpmTime = current_rpm_time;
   attachInterrupt(digitalPinToInterrupt(Photo), spindleRPM, FALLING);
   }

void rising() {                                                      // Marlin check incoming PWM
   attachInterrupt(digitalPinToInterrupt(PWM_PIN), falling, FALLING);
   prev_time = micros();
   }
 void falling() {
   attachInterrupt(digitalPinToInterrupt(PWM_PIN), rising, RISING);
   pwm_value = micros()-prev_time;
   //Serial.println(pwm_value);                                      // For debugging the PWM MAP
  }
