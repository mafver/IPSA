void print_imu_data()
{
    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
   // Serial.print(',');
    print_angles();   // Print "Roll, Pitch and Yaw"
    lastPrint = millis(); // Update lastPrint time
}

void printGyro()
{
    // Now we can use the gx, gy, and gz variables as we please.
    // Either print them as raw ADC values, or calculated in DPS.
    
  #ifdef debug_calculated_imu_data        // If you want to print calculated angle rates.
    // calcGyro helper function to convert a raw ADC value to DPS. 
    Serial.print("G: ");
    Serial.print(imu.calcGyro(imu.gx), 2);    // Rate x
    Serial.print(", ");                       // To separate data
    Serial.print(imu.calcGyro(imu.gy), 2);    // Rate y
    Serial.print(", ");                       // To separate data
    Serial.print(imu.calcGyro(imu.gz), 2);    // Rate z
    Serial.println(" deg/s");                 // Show units
  #elif defined debug_raw_data                // Print data obtained by sensor.      
    Serial.print("G: ");
    Serial.print(imu.gx);                     // Rate x
    Serial.print(", ");                       // To separate data  
    Serial.print(imu.gy);                     // Rate y
    Serial.print(", ");                       // To separate data
    Serial.println(imu.gz);                   // Rate z
  #elif defined debug_mat_gyro
    Serial.print(imu.calcGyro(imu.gx), 3);    // Rate x
    Serial.print(",");                       // To separate data
    Serial.print(imu.calcGyro(imu.gy), 3);    // Rate y
    Serial.print(",");                       // To separate data
    Serial.print(imu.calcGyro(imu.gz), 3);    // Rate z
  #endif
}

void printAccel()
{  
    // Now we can use the ax, ay, and az variables as we please.
    // Either print them as raw ADC values, or calculated in g's.
    
  #ifdef debug_calculated_imu_data
    // If you want to print calculated values, you can use the
    // calcAccel helper function to convert a raw ADC value to
    // g's. Give the function the value that you want to convert.
    Serial.print("A: ");
    Serial.print(imu.calcAccel(imu.ax), 2);
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.ay), 2);
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.az), 2);
    Serial.println(" g");
  #elif defined debug_raw_data 
    Serial.print("A: ");
    Serial.print(imu.ax);
    Serial.print(", ");
    Serial.print(imu.ay);
    Serial.print(", ");
    Serial.println(imu.az);
  #elif defined debug_mat_acc
    Serial.print(imu.calcAccel(imu.ax) - accel_offset_x , 3);
    Serial.print(",");
    Serial.print(imu.calcAccel(imu.ay) - accel_offset_y, 3);
    Serial.print(",");
    Serial.print(imu.calcAccel(imu.az) - accel_offset_z, 3);
  #endif

}

void printMag()
{  
    // Now we can use the mx, my, and mz variables as we please.
    // Either print them as raw ADC values, or calculated in Gauss.
    
  #ifdef debug_calculated_imu_data          // Print calculated values
    // calcMag function is used to convert a raw ADC value to Gauss. 
    Serial.print("M: ");
    Serial.print(imu.calcMag(imu.mx), 2);   // For x
    Serial.print(", ");                     // To separate data    
    Serial.print(imu.calcMag(imu.my), 2);   // For y
    Serial.print(", ");                     // To separate data    
    Serial.print(imu.calcMag(imu.mz), 2);   // For z
    Serial.println(" gauss");               // To separate data    
  #elif defined debug_raw_data              // Print values obtained by the sensor
    Serial.print("M: ");
    Serial.print(imu.mx);                   // For x  
    Serial.print(", ");                     // To separate data    
    Serial.print(imu.my);                   // For y
    Serial.print(", ");                     // To separate data    
    Serial.println(imu.mz);                 // For z
  #elif defined debug_mat_mag
    //Serial.print(-imu.calcMag(imu.my)+0.2492, 4);   // For x
    Serial.print(magnet_x, 4);   // For x
    Serial.print(",");                      // To separate data    
    //Serial.print(-imu.calcMag(imu.mx)-0.2022, 4);   // For y
    Serial.print(magnet_y, 4);   // For y
    Serial.print(",");                      // To separate data    
    //Serial.println(imu.calcMag(imu.mz), 3); // For z
    Serial.print(imu.calcMag(imu.mz), 4); // For z
  #endif   
}
void print_angles()
{
  #ifdef debug_acc_angles_calculated                  // Print angles calculated by accelerometer 
    Serial.print("Roll_acc, Pitch_acc, Yaw_mag: ");   // Message 
    Serial.print(roll_acc, 2);                        // Print roll angle. 
    Serial.print(", ");                               // To separate data 
    Serial.print(pitch_acc, 2);                       // Print pitch angle. 
    Serial.print(", ");                               // To separate data 
    Serial.println(yaw, 2);                           // Print yaw angle.
  #endif     
  #ifdef debug_gyro_angles_calculated                 // Print angles calculated by gyroscope 
    Serial.print(" Roll_gyro, Pitch_gyro, Yaw_gyro: "); // Message 
    Serial.print(imu.calcGyro(imu.gx)*0.004, 5);     // Print pitch angle. 
    Serial.print(", ");                               // To separate data 
    Serial.print(imu.calcGyro(imu.gy)*0.004, 5);     // Print roll angle. 
    Serial.print(", ");                               // To separate data 
    Serial.println(imu.calcGyro(imu.gz)*0.004, 5);   // Print yaw angle. 
  #endif       
  #ifdef debug_filtered_angles
    Serial.print("Roll_filt: "); 
    Serial.print(roll_filt_deg, 4);                       // Print roll angle. 
    Serial.print(", ");                               // To separate data 
    Serial.print("Pitch_filt: "); // Message 
    Serial.print(pitch_filt_deg, 4);                      // Print pitch angle. 
    Serial.print(", ");                               // To separate data 
    Serial.print("Yaw_filt: ");
    Serial.print(yaw_filt_deg, 4);                      // Print yaw angle. 
  #endif 
  #ifdef debug_kalman_filter_angles      
    Serial.print("Roll_k: "); // Message 
    Serial.print(roll_k_f_deg, 4);                      // Print pitch angle. 
    Serial.print(", pitch_k: ");
    Serial.print(pitch_k_f_deg, 4);                       // Print roll angle. 
    Serial.print(", yaw_k: ");
    Serial.println(yaw_k_f_deg, 4);                      // Print yaw angle. 
  #endif
  #ifdef debug_mat_angles
    Serial.print(roll_deg, 3);                        // Print roll angle. 
    Serial.print(",");                               // To separate data 
    Serial.print(pitch_deg, 3);                       // Print pitch angle. 
    Serial.print(",");                               // To separate data 
    Serial.print(yaw_deg, 3);                           // Print yaw angle. 
  #endif         
  #ifdef debug_mat_filt_angles
    Serial.print(roll_filt_deg, 4);                       // Print roll angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(pitch_filt_deg, 4);                      // Print pitch angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(yaw_filt_deg, 4);                      // Print yaw angle. 
  #endif
  #ifdef debug_mat_filt_and_angles
    Serial.print(roll_deg, 4);                        // Print roll angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(pitch_deg, 4);                       // Print pitch angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(yaw_deg, 4);                         // Print yaw angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(roll_filt_deg, 4);                   // Print roll angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(pitch_filt_deg, 4);                  // Print pitch angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(yaw_filt_deg, 4);                    // Print yaw angle
  #endif

   #ifdef debug_mat_angles_and_rates
    Serial.print(roll_deg, 4);                        // Print roll angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(pitch_deg, 4);                       // Print pitch angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(yaw_deg, 4);                         // Print yaw angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(imu.calcGyro(imu.gx), 4);                  // Print pitch angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(imu.calcGyro(imu.gy), 4);                   // Print roll angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(imu.calcGyro(imu.gz), 4);                    // Print yaw angle
  #endif
  #ifdef debug_mat_kalman_angles
    Serial.print(roll_k_f_deg, 4);                   // Print roll angle.
    Serial.print(",");                                // To separate data 
    Serial.print(pitch_k_f_deg, 4);                  // Print pitch angle.  
    Serial.print(",");                                // To separate data 
    Serial.print(yaw_k_f_deg, 4);                    // Print yaw angle
  #endif   
  #ifdef debug_mat_kalman_and_angles
    Serial.print(roll_deg, 4);                        // Print roll angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(pitch_deg, 4);                       // Print pitch angle.
    Serial.print(",");                                // To separate data 
    Serial.print(yaw_deg, 4);                         // Print yaw angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(roll_k_f_deg, 4);                   // Print roll angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(pitch_k_f_deg, 4);                  // Print pitch angle.
    Serial.print(",");                                // To separate data 
    Serial.println(yaw_k_f_deg, 4);                    // Print yaw angle
  #endif 
  #ifdef debug_mat_all_angles
    Serial.print(roll_deg, 4);                        // Print roll angle.
    Serial.print(",");                                // To separate data 
    Serial.print(pitch_deg, 4);                       // Print pitch angle 
    Serial.print(",");                                // To separate data 
    Serial.print(yaw_deg, 4);                         // Print yaw angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(roll_filt, 4);                   // Print roll angle.  
    Serial.print(",");                                // To separate data 
    Serial.print(pitch_filt, 4);                  // Print pitch angle.
    Serial.print(",");                                // To separate data 
    Serial.print(yaw_filt, 4);                    // Print yaw angle
    Serial.print(",");                                // To separate data 
    Serial.print(roll_k_f_deg, 4);                   // Print roll angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(pitch_k_f_deg, 4);                  // Print pitch angle. 
    Serial.print(",");                                // To separate data 
    Serial.print(yaw_k_f_deg, 4);                    // Print yaw angle
  #endif 
}
