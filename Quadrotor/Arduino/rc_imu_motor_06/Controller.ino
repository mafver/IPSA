// ................. Global variables por PID controller ................. //
  // Vector component 0 : angle Roll
  // Vector component 1 : angle Pitch
  // Vector component 2 : angle Yaw
volatile float err[3] = {0, 0, 0};             // to measure the error.
float sum_err[3] = {0, 0, 0};         // used on integral part.
float previous_err[3] = {0, 0, 0};    // to store the previous error.
float tsd = 0.05;                     // Sample frequency. 
// Enable interrupt for a frequency of 100
void sample_interrupt()
{
   // initialize timer1 
    cli();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    //OCR1A = 2500;            // compare match register 16MHz/256/25Hz
    OCR1A = 3125;            // compare match register 16MHz/256/20Hz
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS12);    // 256 prescaler 
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    sei();             // enable all interrupts
}
// Into each interruption, update the error values. Take note that the 
// error is measured on radians

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
    cli();                      //stop interrupts;           
    //err[0] = float(ref[PITCH]) - roll_filt_deg;         // Calculate pitch error. 
    err[0] = (float(ref[ROLL]) - roll_k_f_deg)*PI/180;  // Calculate pitch error radians.
    //err[0] = (float(ref[ROLL]) - roll_k_f_deg);  // Calculate pitch error degrees. 
    //err[1] = float(ref[ROLL]) - roll_filt_deg;           // Calculate roll error.
    err[1] = (float(ref[PITCH]) - pitch_k_f_deg)*PI/180;    // Calculate roll error radians.
    //err[1] = (float(ref[PITCH]) - pitch_k_f_deg);    // Calculate roll error degrees.
    //err[2] = float(ref[YAW]) - yaw_filt_deg;             // Calculate yaw error. 
    err[2] = (float(ref[YAW]) - yaw_k_f_deg)*PI/180;     // Calculate yaw error radians. 
    //err[2] = (float(ref[YAW]) - yaw_k_f_deg);     // Calculate yaw error degrees. 
    
    sei();                      //allow interrupts
}
#ifdef debug_error
  void print_error()
  {
      Serial.print("Err Roll: ");
      Serial.print(err[0],5);
      Serial.print(", ");
      Serial.print("Err Pitch: ");
      Serial.print(err[1],5);
      Serial.print(", ");
      Serial.print("Err Yaw: ");
      Serial.println(err[2],5);
  }
#endif
// Define the gains for the controllers.
//float kp[3] = {4.0, 1.3, 1.3};      // Proportional gains.
#ifdef use_PID_controller
  float kp[3] = {0.1, 0.1, 4.5};      // Proportional gains.
  float ki[3] = {0.0, 0.0, 0.01};   // Integral gains.
  float kd[3] = {0.0, 0.0, 0.05};          // Derivative gains.
  float pid_pitch = 0, pid_pitch_map = 0;         // PID controllers for pitch.
  float pid_roll = 0, pid_roll_map = 0;           // PID controllers for roll.
  float pid_yaw = 0, pid_yaw_map = 0;             // PID controllers for yaw.
#endif
#ifdef use_PD_controller
  float kp[3] = {5, 5, 5};      // Proportional gains.
  float kd[3] = {5, 5, 5};          // Derivative gains.
  float pd_pitch = 0, pd_pitch_map = 0;         // PD controllers for pitch.
  float pd_roll = 0, pd_roll_map = 0;           // PD controllers for roll.
  float pd_yaw = 0, pd_yaw_map = 0;             // PD controllers for yaw.
#endif
  
float delta_err[3] = {0.0, 0.0, 0.0};     // Delta (used for calculing derivative part).
// Initialize the controller values.


void controller()
{
    // Update errors only if throttle is more that 0
    if ((timer_pid + tsc) < micros())
    {  
      if (ref[THROTTLE] >= pwm_min && cond_stop == false)
        {
            controller_algorithm();
            timer_pid = micros();     // Update sample time.
        }
        else
        {  
          motor_stop();
          reset_controller();
        }
      
    }  
}
void controller_algorithm()
{
    // Calculate the sum of errors for the calculation of integral components.
    float ts = float(micros()-timer_pid)/1000000;
    //  Serial.println(ts,6);
    
    for (int i= 0; i<3;i++)
    {
       // Calculate delta for calculation of derivative components.
       delta_err[i] = (err[i] - previous_err[i])/ts;  
       previous_err[i] = err[i];  
    }
    #ifdef use_PD_controller
      // Calculate PD controllers.
      // Limit pd controllers.
      // Roll controller.
      //if (err[0]> -3 && err[0]<3)
      //  pd_roll= 0;
      //else
        pd_roll = (err[0] * kp[0]) +  (delta_err[0] * kd[0]);
      // Pitch controller.
      //if (err[1]> -3 && err[1]<3)
      //  pd_pitch= 0;
      //else  
        pd_pitch = (err[1] * kp[1]) + (delta_err[1] * kd[1]);
      //Yaw controller
      //if (err[2]> -3 && err[2]<3)
      //  pd_yaw= 0;
      //else
        pd_yaw = (err[2] * kp[2]) + (delta_err[2] * kd[2]);     
      pd_roll_map = map(pd_roll,-20,20,-100,100);
      pd_pitch_map = map(pd_pitch,-20,20,-100,100);
      pd_yaw_map = map(pd_yaw,-20,20,-450,450);
      // Speed control.
      pwm_motor_esc1 = float(ref[THROTTLE]) - pd_pitch_map;
      pwm_motor_esc2 = float(ref[THROTTLE]) + pd_roll_map;
      pwm_motor_esc3 = float(ref[THROTTLE]) + pd_pitch_map;
      pwm_motor_esc4 = float(ref[THROTTLE]) - pd_roll_map;
//      pwm_motor_esc1 = float(ref[THROTTLE]) - pd_yaw_map;
//      pwm_motor_esc2 = float(ref[THROTTLE]) + pd_yaw_map;  
//      pwm_motor_esc3 = float(ref[THROTTLE]) - pd_yaw_map;
//      pwm_motor_esc4 = float(ref[THROTTLE]) + pd_yaw_map;
    #endif
    #ifdef use_PID_controller
    // Calculate PID controllers.
      pid_roll = (err[0] * kp[0]) + sum_err[0] + (delta_err[0] * kd[0]);
      pid_pitch = (err[1] * kp[1]) + sum_err[1] + (delta_err[1] * kd[1]);
      pid_yaw = (err[2] * kp[2]) + sum_err[2]  + (delta_err[2] * kd[2]);

      pid_roll_map = mapfloat(pid_roll,-20,20,-400,400);
      pid_pitch_map = mapfloat(pid_pitch,-20,20,-400,400);
      pid_yaw_map = mapfloat(pid_yaw,-20,20,-450,450);

      pwm_motor_esc1 = float(ref[THROTTLE]) - pid_pitch_map - pid_yaw_map;
      pwm_motor_esc2 = float(ref[THROTTLE]) + pid_roll_map + pid_yaw_map;  
      pwm_motor_esc3 = float(ref[THROTTLE]) + pid_pitch_map - pid_yaw_map;
      pwm_motor_esc4 = float(ref[THROTTLE]) - pid_roll_map + pid_yaw_map;
//        pwm_motor_esc1 = float(ref[THROTTLE]) - pid_yaw_map;
//        pwm_motor_esc2 = float(ref[THROTTLE]) + pid_yaw_map;  
//        pwm_motor_esc3 = float(ref[THROTTLE]) - pid_yaw_map;
//        pwm_motor_esc4 = float(ref[THROTTLE]) + pid_yaw_map;
      // Update data for Integral part.
      for (int i= 0; i<3;i++)
      {
         if (err[i]> -0.3491 && err[i]<0.3491) 
           sum_err[i] += err[i]*ki[i]*ts;
         else
           sum_err[i] += 0;  
      }
    #endif
     // Saturate values into the PWM range.
     pwm_motor_esc1 = saturation(pwm_motor_esc1,pwm_max,pwm_min);
     pwm_motor_esc2 = saturation(pwm_motor_esc2,pwm_max,pwm_min);
     pwm_motor_esc3 = saturation(pwm_motor_esc3,pwm_max,pwm_min);
     pwm_motor_esc4 = saturation(pwm_motor_esc4,pwm_max,pwm_min);
}
// Debug PID_values.
void print_PID_values()
{
           
   #ifdef debug_derivative_components
    Serial.print("Derivative components: ");
    Serial.print(delta_err[0],4);
    Serial.print(": ");   
    Serial.print(delta_err[1],4);
    Serial.print(": ");   
    Serial.println(delta_err[2],4);
  #endif
  #ifdef  use_PID_controller
  // Debug PID gains.
    #ifdef debug_pid_gains               // Debug only if this option is defined.
       for (int i= 0; i<3;i++)
       {
            switch (i)
            {
              case 0:
              {
                  Serial.print("Roll: ");
                  break;
              }  
              case 1:
              {
                  Serial.print("Pitch: ");
                  break;
              }
              case 2:
              {
                  Serial.print("Yaw: ");
                  break;
              } 
            }
            Serial.print("Kp: ");
            Serial.print(kp[i]);
            Serial.print(" Ki: ");   
            Serial.print(ki[i]);
            Serial.print(" Kd: ");   
            Serial.println(kd[i]);
       }
    #endif
    // Debug integram components.
    #ifdef debug_integral_components
      Serial.print("Integral components: ");
      Serial.print(sum_err[0],4);
      Serial.print(": ");   
      Serial.print(sum_err[1],4);
      Serial.print(": ");   
      Serial.println(sum_err[2],4);
    #endif
    // Debug PID values
    #ifdef debug_pid_values               // Debug only if this option is defined.
       Serial.print("PID roll: ");
       Serial.print(pid_roll,4);
       Serial.print(" PID pitch: ");   
       Serial.print(pid_pitch,4);
       Serial.print(" PID yaw: ");   
       Serial.println(pid_yaw,4);
       Serial.print("PID roll map: ");
       Serial.print(pid_roll_map,4);
       Serial.print(" PID pitch map: ");   
       Serial.print(pid_pitch_map,4);
       Serial.print(" PID yaw map: ");   
       Serial.println(pid_yaw_map,4);
    #endif
  #endif
  
  #ifdef use_PD_controller
    // Debug PID values
    #ifdef debug_pd_gains               // Debug only if this option is defined.
       for (int i= 0; i<3;i++)
       {
            switch (i)
            {
              case 0:
              {
                  Serial.print("Roll: ");
                  break;
              }  
              case 1:
              {
                  Serial.print("Pitch: ");
                  break;
              }
              case 2:
              {
                  Serial.print("Yaw: ");
                  break;
              } 
            }
            Serial.print("Kp: ");
            Serial.print(kp[i]);
            Serial.print(" Kd: ");   
            Serial.println(kd[i]);
       }
      #endif 
      #ifdef debug_pd_values               // Debug only if this option is defined.
       Serial.print("PD roll: ");
       Serial.print(pd_roll);
       Serial.print(" PD pitch: ");   
       Serial.print(pd_pitch);
       Serial.print(" PD yaw: ");   
       Serial.println(pd_yaw);
       Serial.print("PD roll map: ");
       Serial.print(pd_roll_map);
       Serial.print(" PD pitch map: ");   
       Serial.print(pd_pitch_map);
       Serial.print(" PD yaw map: ");   
       Serial.println(pd_yaw_map);
      #endif
  #endif
  // Debug saturated values.
  #ifdef debug_sat_values
    Serial.print("Saturated values ");
    Serial.print(pwm_motor_esc1);
    Serial.print(": ");   
    Serial.print(pwm_motor_esc2);
    Serial.print(": ");   
    Serial.print(pwm_motor_esc3);
    Serial.print(": ");   
    Serial.println(pwm_motor_esc4);
  #endif
}    
long saturation (long data_converted, int max_value, int min_value)
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
// Map points
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Function to reset error values.
void reset_controller()
{
   for (int i= 0; i<3;i++)
   {
      err[i] = 0;
      sum_err[i] = 0;
      previous_err[i] = 0;
   } 
}
