// Gains obtained:
// kp = 1.5;
// kd = 0.8;
// t1=1345;
// t2=1450;

// Range pd 0 < pd < 100 
// .............. Ultrasonic settings ............................ //
#include <NewPing.h>
#define trigger_pin  A3   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define echo_pin     A4    // Arduino pin tied to echo pin on the ultrasonic sensor.
#define dist_max 30      // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define dist_min 3        // Minimun distance we want to ping.
#define ultrasonic_offset 3
NewPing sonar(trigger_pin, echo_pin, dist_max); // NewPing setup of pins and maximum distance.
// .................... Motor settings ........................... //
#include <Servo.h>
Servo mymotor1;
Servo mymotor2;

// .................... Global variables ......................... //
// ------------------- Ultrasonic ----------------- //
 int high_0 = dist_min;
 unsigned int uS = 0;
// ------------------- References ----------------- // 
 int reference = 5;       // High reference in centimeters.
// ------------------- Controller ----------------- //
 int err = 0;            // Error variable 
 int previous_err = 0;     // Error from previous state.                        
 float kp1 = 0, kp2 = 0;   // Proportional gain controller motor 1 and 2. 
 float kd1 = 0, kd2 = 0;   // Derivative gain controller motor 1 and 2.  
 float pd1 = 0, pd2 = 0;   // Controller value motor 1 and 2.  
 float ts = 20;            // Sample time in miliseconds 
 int pd_max = 50;         // Maximum value.
 //int pd_min = -50;         // Minimum value.
 int pd_min = 0;         // Minimum value.
// ---------------------- Motors ------------------- //
 int motor1_pin = 5; 
 int motor2_pin = 6;  
 long pwm_motor_esc1 = 0;
 long pwm_motor_esc2 = 0;
 long pwm_min = 1000;
 long pwm_max = 1700;
 
 //long t1 = 1230;
 //long t2 = 1300;
 long t1 = 1345;
 long t2 = 1450;
 
 
 // ------------------- Serial Port --------------- //
 String reference_command = "";      // a String to hold incoming data
 bool data_sent = false;             // whether the string is complete
 bool cond_start = false;            // flag to indicate that start command was introduced.
 bool cond_stop = true;            // flag to indicate that stop command was introduced.
 int location = 0;
    
void setup() {
  // 1. Configure Serial Port.
  Serial.begin(115200);                    // Set up serial port.
  // 2. Configure Motors.
  mymotor1.attach(motor1_pin);             // Configure pwm motor 1 to  pin 9
  mymotor2.attach(motor2_pin);             // Configure pwm motor 1 to  pin 10
  // 3. Calibration if it is necessary.
  // calibration();
  // 4. Init brushless
   mov_mot_time (pwm_min,15000);
}

void loop() {
  serial_references(); 
  //condition = cond_start;
  while (cond_start)
    { 
      serial_references(); 
      if (cond_stop==true)
      {
            reference = 0;
            cond_start=false;
      }
      
      // 1. Read data from ultrasonic sensor.
      uS = sonar.ping();                         // Send ping, get ping time in microseconds (uS).
      int high=uS/US_ROUNDTRIP_CM;               // Convert ping time to distance in cm and print result (0 = outside set distance range)  
      if(high <= 0)
         high=high_0;                          // Repeat previous value in case of getting the value of 0
      high = high - ultrasonic_offset;           // Correct offset.
      
      high = saturation(high,0,30); 
      Serial.print("Reference: ");
      Serial.println(reference);
      
      Serial.print("Distance: ");
      Serial.println(high);
      // 2. Calculate errors.
      err = reference - high;                    // Update error. 
      Serial.print("Error: ");
      Serial.println(err);
      // 3. Controller.
      float ts1 = ts/1000;                             // Sample time in milliseconds 
      //pd1 = kp1*err + (kd1*(err-previous_err))/ts1;   // PD controller motor 1.
      //pd2 = kp2*err + (kd2*(err-previous_err))/ts1;   // PD controller motor 2.
      pd1 = kp1*err - (kd1*(high-high_0))/ts;   // PD controller motor 1.
      pd2 = kp2*err - (kd2*(high-high_0))/ts;   // PD controller motor 1.
      previous_err = err;                            // Update error.
      high_0 = high;                             // Update data for next iteration.
      pd1 = saturation(pd1,pd_min,pd_max);           // Saturation 
      pd2 = saturation(pd2,pd_min,pd_max);           // Saturation 
      Serial.print("kp1: ");
      Serial.print(kp1);
      Serial.print(", kd1: "); 
      Serial.println(kd1);
      Serial.print("kp2: ");
      Serial.print(kp2); 
      Serial.print(", kd2: "); 
      Serial.println(kd2);
      Serial.print("PD: ");
      Serial.print(pd1); 
      Serial.print(", "); 
      Serial.println(pd2); 
      // 4. Map the signal.
      
      pwm_motor_esc1 = t1 + map (pd1,pd_min,pd_max,0,300);  
      pwm_motor_esc2 = t2 + map (pd2,pd_min,pd_max,0,300);  

      Serial.print("t1: ");
      Serial.print(t1);
      Serial.print(", t2: "); 
      Serial.println(t2);
      
      // 5. Introduce PWM to motors.
      Serial.print("PWM esc: ");
      Serial.print(pwm_motor_esc1); 
      Serial.print(", "); 
      Serial.println(pwm_motor_esc2); 
      mymotor1.writeMicroseconds(pwm_motor_esc1);
      mymotor2.writeMicroseconds(pwm_motor_esc2);
      
      Serial.print('\n');
      delay(ts);
    }   
  // 6. delay signal.
  motor_stop();
  
}

long saturation (long data_converted, int min_value, int max_value)
{
  long sat_data = data_converted;      // Select output variable.
  if (data_converted >= max_value)
      sat_data = max_value;
  else 
  {
    if (data_converted <= min_value)
        sat_data = min_value;
  }      
  return sat_data;    
}

void calibration() {
  mov_mot_time (pwm_min,5000);   // Sent pwm max during 10 seconds
  mov_mot_time (pwm_max,10000);   // Sent pwm max during 10 seconds
  mov_mot_time (pwm_min,10000);   // Sent pwm min during 10 seconds
}
void mov_mot_time (int pwm_value, int time_delay)
{
    unsigned long now = millis();
    while(millis() - now < time_delay)
    {
        mymotor1.writeMicroseconds(pwm_value);
        mymotor2.writeMicroseconds(pwm_value);
    }
}
void motor_stop()
{
    pwm_motor_esc1 = pwm_min;
    kp1=0; kp2=0;
    kd1=0; kd2=0;
    reference=5;
    mymotor1.writeMicroseconds(pwm_min);
    mymotor2.writeMicroseconds(pwm_min);
}
