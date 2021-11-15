// ......................... Motor configuration .............. //
void motor_config ()
{
    motor_1.attach(motor1_pin);  // attaches the motor pin to the servo object
    motor_2.attach(motor2_pin);  // attaches the motor pin to the servo object
    motor_3.attach(motor3_pin);  // attaches the motor pin to the servo object
    motor_4.attach(motor4_pin);  // attaches the motor pin to the servo object
    initBrushless();
    //calibration();
    
}
void calibration() {
  Serial.println("Initialization: ");
  Serial.println("Maximun for initialize");
  mov_mot_time (pwm_max,12000);
  Serial.println("Minimum for initialize");
  mov_mot_time (pwm_min,8000);
  Serial.println("Ready");
//  mov_mot_time (1500,3000);
//  mov_mot_time (1100,3000);
//  
  
}
void initBrushless()
{
   Serial.println("Start Motors: ");
   mov_mot_time (pwm_min,15000); 

}
void mov_mot_time (int pwm_value, int time_delay)
{
    unsigned long now = millis();
    while(millis() - now < time_delay)
    {
        motor_1.write(pwm_value);
        motor_2.write(pwm_value);
        motor_3.write(pwm_value);
        motor_4.write(pwm_value);
    }
}
void motor_actuation()
{
//    Serial.print("Sent value: ");
//    Serial.println(ref[0]);
    motor_1.write(pwm_motor_esc1);
    motor_2.write(pwm_motor_esc2);
    motor_3.write(pwm_motor_esc3);
    motor_4.write(pwm_motor_esc4);
}
void motor_stop()
{
    pwm_motor_esc1 = pwm_min;
    pwm_motor_esc2 = pwm_min;
    pwm_motor_esc3 = pwm_min;
    pwm_motor_esc4 = pwm_min;
    motor_1.write(pwm_min);
    motor_2.write(pwm_min);
    motor_3.write(pwm_min);
    motor_4.write(pwm_min);
}
