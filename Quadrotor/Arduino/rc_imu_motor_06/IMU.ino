#ifdef calculate_offset
  #ifdef calculate_accel_offset
    float accel_offset_x, accel_offset_y, accel_offset_z;     // Save offset data.
  #endif
  #ifdef calculate_accel_offset
    float mag_offset_x, mag_offset_y, mag_offset_z;     // Save offset data.
  #endif
  #ifdef calculate_accel_offset
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;     // Save offset data.
  #endif
  float pitch_offset = 0, roll_offset = 0;

  
#endif

#define offset_samples_number 300
#define offset_yaw_samples_number 400
float magnet_x=0, magnet_y=0;
float mag_x, mag_y, mag_z;                  // Variables for correction.
float mag_x_prev=0, mag_y_prev=0, mag_prev=0;                  // Variables for correction.
#define Am 0.4
void imu_configuration()
{
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1);
    
  }
  #ifdef calculate_offset
    calc_sensors_offset();
//    calc_angles_offset();
  #endif
//  #ifdef calculate_yaw_offset
////    calc_yaw_offset();
//  #endif
//   if ( imu.accelAvailable())     // Check accelerometer data available.
//   {
//     imu.readAccel();           
//     a1 = imu.calcAccel(imu.az);
//   }  

}
void check_and_read_data()
{
  // ............................................................ //
  // .... Update the sensor values whenever new data is available //
  // ............................................................ //
  
  if ( imu.gyroAvailable() )      // To read from the gyroscope
    {imu.readGyro();}             // Update gx, gy, and gz with current data.
  
  if ( imu.accelAvailable() )     // To read from the accelerometer
    {imu.readAccel();}            // Update ax, ay, and az with current data.
    
  if ( imu.magAvailable() )       // To read from the magnetometer
    {imu.readMag();}              // Update mx, my, and mz withcurrent data.
  // ............................................................ //
  //....................... Get angles from IMu ................. //
  // ............................................................ //
  // The LSM9DS1's mag x and y axes are opposite to the accelerometer, so my, mx are
  // substituted for each other.
  // Horientation.
//    Accelerometer           Magnetometer 
//       -y  |   /z                 x |   /z
//          |  /                      |  /
//          | /                       | /
// -x <- - - . - - ->  x     y <- - - . - - ->  -y
//          |                         |
//          |                         |
//          |y                        | -x


  #ifdef calculate_offset
    // Correct offset.
    // Accelerometers.
    float accel_x, accel_y, accel_z;                  // Variables for correction.
    #ifdef calculate_accel_offset
      // Magnetometer corrections
      accel_x = imu.calcAccel(imu.ax) - accel_offset_x;            // Coordinate x.
      accel_y = imu.calcAccel(imu.ay) - accel_offset_y;            // Coordinate y.
      accel_z = imu.calcAccel(imu.az) - accel_offset_z + 1;        // Coordinate z considering accelerometer direction.
    #else
      accel_x = imu.calcAccel(imu.ax);            // Coordinate x.
      accel_y = imu.calcAccel(imu.ay);            // Coordinate y.
      accel_z = imu.calcAccel(imu.az);            // Coordinate z.
    #endif
    // Magnetometers
    
    #ifdef calculate_mag_offset
      // Magnetometer corrections
      mag_x = imu.calcMag(imu.mx) - mag_offset_x;            // Coordinate x.
      mag_y = imu.calcMag(imu.my) - mag_offset_y;            // Coordinate x.
      mag_z = imu.calcMag(imu.mz) - mag_offset_z;            // Coordinate x.
    #else
      mag_x = imu.calcMag(imu.mx);            // Coordinate x.
      mag_y = imu.calcMag(imu.my);            // Coordinate y.
      mag_z = imu.calcMag(imu.mz);            // Coordinate z.
    #endif
     // Apply the low pass filter for magnetometers. Ref: https://en.wikipedia.org/wiki/Low-pass_filter    
     float mag_x_f = Am*mag_x+(1-Am)*mag_x_prev;     // Coordinate x
     float mag_y_f = Am*mag_y+(1-Am)*mag_y_prev;
     // Update data
     mag_x_prev = mag_x_f;
     mag_y_prev = mag_y_f;
   // calc_angles(accel_x, accel_y, accel_z,-mag_y, -mag_x, mag_z);
   calc_angles(accel_x, accel_y, accel_z,-mag_y_f, -mag_x_f, mag_z); // Calculate pitch, roll, and yaw. 
  #else
    
    calc_angles(imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az),
               -imu.calcMag(imu.my), -imu.calcMag(imu.mx), imu.calcMag(imu.mz)); // Calculate pitch, roll, and yaw.
  #endif
 
}

// Calculate pitch, roll, and yaw.
void calc_angles(float ax, float ay, float az, float mx, float my, float mz)
{
//    // emergency Stop.
//    a0 = imu.calcAccel(imu.az);
//    diff = abs(a0-a1);
//    // Update acceleration.
//    a1 = a0;  
//    //  Serial.print("diff: ");
//    //  Serial.println(diff);
//    if (diff > 1)
//      {accel_stop = true;}

   // Calculate for the accelerometers.
  roll_acc = atan2(ay, az);                     // Calculate roll angle for accelerometers into the range of [-pi,pi]
  pitch_acc = atan2(-ax, sqrt(ay * ay + az * az));  // Calculate pitch angle for accelerometers into the range of [-pi,pi]
  
  // Determine yaw angle.
  // Correction of the angles.
  mx -= -0.2384;
  my -= 0.043;
  magnet_x = mx;
  magnet_y = my;
  if (my == 0)
    yaw = (mx < 0) ? PI : 0;                    // condition in case of having a division by zero into atan2 function
  else
    yaw = atan2(-mx,my);      
    // yaw = atan2(mx,my);
    

  #ifdef calculate_offset
    pitch_acc -= pitch_offset;
    roll_acc -= roll_offset;
  #endif
  // Limit the angles into the range of [-pi, pi]
  pitch_acc = limit_angles(pitch_acc);          // For pitch angle.
  roll_acc = limit_angles(roll_acc);            // For roll angle.
  yaw = limit_angles(yaw);                  // For yaw angle
  
    
//  if (yaw > PI) yaw -= (2 * PI);              
//  else if (yaw < -PI) yaw += (2 * PI);
//   Consider offset for Pitch and Roll angle
  
  // Escale data obtained
  yaw_deg = yaw*180.0 / PI;
  pitch_deg = pitch_acc*180.0 / PI;
  roll_deg = roll_acc*180.0 / PI;
    
 // Complementary filter.
  #ifdef use_complementary_filter_imu
    if ((cp_loop_time + tscf) < millis())
    {    
      float dt = float(millis()-cp_loop_time)/1000;
      complementary_filter(roll_deg,pitch_deg,yaw_deg,dt);
      cp_loop_time = millis();    
    }
  #endif
  // kalman filter.
  #ifdef use_kalman_filter
    if ((kalman_loop_time + tsk) < micros())
    {    
      float gyro_rate_x = imu.calcGyro(imu.gx);
      float gyro_rate_y = imu.calcGyro(imu.gy);
      float gyro_rate_z = imu.calcGyro(imu.gz);
      float dT = float(micros()- kalman_loop_time)/1000000;
      //Serial.println(dT);
      // Kalman Filter.
        roll_k_f_deg = kalman_filter_roll(roll_deg, gyro_rate_x, dT, 0.005, 0, 0.02);      // Roll angle 
        pitch_k_f_deg = kalman_filter_pitch(pitch_deg, gyro_rate_y, dT, 0.002, 0, 0.01); // Pitch angle
        yaw_k_f_deg = kalman_filter_yaw(yaw_deg, gyro_rate_z, dT, 0.002, 0, 0.05);         // yaw angle 
        kalman_loop_time = micros();   
     }
  #endif
}
#ifdef use_complementary_filter_imu
  void complementary_filter(float phi, float theta, float psi, float dt)
  {
     // Calculate data for gyroscopes
      float gyro_rate_x = imu.calcGyro(imu.gx);
      float gyro_rate_y = imu.calcGyro(imu.gy);
      float gyro_rate_z = imu.calcGyro(imu.gz);
      
      roll_filt = A*(roll_filt + gyro_rate_x*dt);   // HP filter for gyroscope roll angle
      roll_filt += (1-A)*phi;                      // LP filter for accelerometer roll angle
      // For pitch angle 
      pitch_filt = A*(pitch_filt + gyro_rate_y*dt);   // HP filter for gyroscope roll angle
      pitch_filt += (1-A)*theta;                      // LP filter for accelerometer roll angle
      // For yaw angle 
      yaw_filt = Ay*(yaw_filt + gyro_rate_z*dt);   // HP filter for gyroscope roll angle
      yaw_filt += (1-Ay)*psi;                      // LP filter for accelerometer roll angle
   
  /* ...... For avoiding peaks ...... */
      roll_filt = avoid_peaks(roll_filt,roll_ant,50);      // Angle Roll.
      pitch_filt = avoid_peaks(pitch_filt,pitch_ant,5);   // Angle Pitch
      yaw_filt = avoid_peaks(yaw_filt,yaw_ant,50);        // Angle Yaw.
      // Update data
      roll_ant = roll_filt;
      pitch_ant = pitch_filt;
      yaw_ant = yaw_filt;
      // Escale data obtained
  //    yaw_filt_deg = yaw_filt*180.0 / PI;
  //    pitch_filt_deg = pitch_filt*180.0 / PI;
  //    roll_filt_deg = roll_filt*180.0 / PI;
  }
#endif
float avoid_peaks(float actual_data, float previous_data, int limit)  // store result into float.
{
    float data_fixed = 0;                                 // Variable to store data.           
    if(abs(abs(actual_data)-abs(previous_data))>limit)    // comparing change between the actual value and previous one.
      {data_fixed = previous_data;}                       // In case of a peak presented, just consider the previous value obtained.
    else
      {data_fixed = actual_data;}                         // In case other case, save the actual data.
    return data_fixed;                                    // return value obtained.

}
// Subprogram that estimate the offset for the angles calculated by accelerometer and the magnetometer values.
#ifdef calculate_offset
  void calc_sensors_offset()
  {
    #ifdef calculate_accel_offset
      float sum_acc_offset_x = 0, sum_acc_offset_y = 0, sum_acc_offset_z = 0;     // Save offset data from accelerometer.
      int cont = 0;
    #endif
    #ifdef calculate_mag_offset
      float sum_mag_offset_x = 0, sum_mag_offset_y = 0, sum_mag_offset_z = 0;     // Save offset data from magnetometer.
      int cont1 = 0;                              // Components counters  
    #endif
    #ifdef calculate_gyro_offset
      float sum_gyro_offset_x = 0, sum_gyro_offset_y = 0, sum_gyro_offset_z = 0;               // Variables to calculate angles. 
      int cont2 = 0;
    #endif  
    // Calculating accumulated values.
    for (int i=0; i<offset_samples_number; i++)
    {
      // Read IMU data 
      #ifdef calculate_accel_offset
        if ( imu.accelAvailable())     // Check accelerometer data available.
        {
            imu.readAccel();           // Update ax, ay, and az with current data.
            delay (10);
            // Calculate Roll and pitch.
            sum_acc_offset_x += imu.calcAccel(imu.ax);    // Update accel offset for coordiante x.
            sum_acc_offset_y += imu.calcAccel(imu.ay);     // Update accel offset for coordiante y.
            sum_acc_offset_z += imu.calcAccel(imu.az);     // Update accel offset for coordiante y.
            cont ++;                                         // Update counter. 
        }
      #endif
      #ifdef calculate_mag_offset  
        if ( imu.magAvailable() )       // To read from the magnetometer
        {
            imu.readMag();              // Update mx, my, and mz withcurrent data.
            sum_mag_offset_x += imu.calcMag(imu.mx);     // Update accel offset for coordiante z.
            sum_mag_offset_y += imu.calcMag(imu.my);     // Update accel offset for coordiante z.
            sum_mag_offset_z += imu.calcMag(imu.mz);     // Update accel offset for coordiante z. 
            cont1++;                                     // Update counter.
        } 
      #endif
      #ifdef calculate_gyro_offset  
        if ( imu.magAvailable() )       // To read from the magnetometer
        {
            imu.readMag();              // Update mx, my, and mz withcurrent data.
            sum_gyro_offset_x += imu.calcGyro(imu.gx);     // Update accel offset for coordiante z.
            sum_gyro_offset_y += imu.calcGyro(imu.gy);     // Update accel offset for coordiante z.
            sum_gyro_offset_z += imu.calcGyro(imu.gz);     // Update accel offset for coordiante z. 
            cont2++;                                     // Update counter.
        } 
      #endif
                      
    }
    // Calculate mean value.
    // Accelerometer angles.
    #ifdef calculate_accel_offset
      accel_offset_x = sum_acc_offset_x / cont;              // Pitch angle.    
      accel_offset_y = sum_acc_offset_y / cont;              // Roll angle. 
      accel_offset_z = sum_acc_offset_z / cont;              // Pitch angle.    
    #endif
      
    // magnetometer values.
    #ifdef calculate_mag_offset
      mag_offset_x = sum_mag_offset_x / cont1;
      mag_offset_y = sum_mag_offset_y / cont1;
      mag_offset_z = sum_mag_offset_z / cont1;
    #endif

    // gyroscope values.
    #ifdef calculate_gyro_offset
      gyro_offset_x = sum_gyro_offset_x / cont2;
      gyro_offset_y = sum_gyro_offset_y / cont2;
      gyro_offset_z = sum_gyro_offset_z / cont2;
    #endif
    
    // print accel_offset.
    #ifdef Debug_sensor_offsets 
      Serial.println("Offset angles: ");
      #ifdef calculate_accel_offset
        Serial.print("Offset Accelerometer: ");
        Serial.print(accel_offset_x, 3);    // For x
        Serial.print(", ");                 // To separate data
        Serial.print(accel_offset_y, 3);    // For y
        Serial.print(", ");                 // To separate data    
        Serial.println(accel_offset_z, 3);   // For z
      #endif
      #ifdef calculate_mag_offset
        Serial.print("Offset Magnetometer: ");
        Serial.print(mag_offset_x, 3);   // For x
        Serial.print(", ");              // To separate data    
        Serial.print(mag_offset_y, 3);   // For y
        Serial.print(", ");              // To separate data    
        Serial.println(mag_offset_z, 3); // For z
      #endif
      #ifdef calculate_gyro_offset
        Serial.print("Offset Gyroscope: ");
        Serial.print(gyro_offset_x, 3);   // For x
        Serial.print(", ");              // To separate data    
        Serial.print(gyro_offset_y, 3);   // For y
        Serial.print(", ");              // To separate data    
        Serial.println(gyro_offset_z, 3); // For z
      #endif
    #endif  
  }
#endif

#ifdef calculate_yaw_offset
void calc_yaw_offset()
{
  float sum_yaw_offset = 0;
  int cont = 0;
  for (int i=0; i < offset_yaw_samples_number; i++)
  {   
    // Read magnetometer values.
    if ( imu.magAvailable() )       // To read from the magnetometer
    {
      imu.readMag();              // Update mx, my, and mz withcurrent data.
      // Correct magnetometer offset.
      float mag_x, mag_y, mag_z;                  // Variables for correction.
      // Magnetometer corrections
      mag_x = imu.calcMag(imu.mx) - mag_offset_x;            // Coordinate x.
      mag_y = imu.calcMag(imu.my) - mag_offset_y;            // Coordinate y.
      mag_z = imu.calcMag(imu.mz) - mag_offset_z;            // Coordinate z.
        
      yaw = calculate_yaw(-mag_y, -mag_x, mag_z);            // Calculate yaw.
      sum_yaw_offset += yaw;
      cont++;
    }
  }
  yaw_offset = sum_yaw_offset / cont;
  #ifdef Debug_yaw_offset
    Serial.print("Offset yaw: ");
    Serial.println(yaw_offset, 3);    // For pitch angle
  #endif  
    
}
float calculate_yaw(float mag_x, float mag_y, float mag_z)
{
    float yaw_calc = 0;
    // Determine yaw angle.
    if (mag_y == 0)
      yaw_calc = (mag_x < 0) ? PI : 0;                    // condition in case of having a division by zero into atan2 function
    else
      yaw_calc = atan2(mag_x, mag_y);
    
    yaw_calc -= DECLINATION * PI / 180;               // correct value with declination determined 

    return yaw_calc;
}
#endif
float limit_angles(float angle)
{
    float correct_angle = angle;
    // Limit yaw angle into the range of [-pi, pi]
    if (angle > PI) 
      correct_angle -= (2 * PI);              
    if (angle < -PI) 
        correct_angle += (2 * PI);
    return correct_angle;
}
