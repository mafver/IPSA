#define data_sample_deb 100
static unsigned long time_data_sample = 0;
void print_data ()
{
   if ((lastPrint + data_sample_deb) < millis())
   {
     #ifdef use_RC_ref                       // In case of using RC
       print_rf_data();       // Print references.
     #endif 
     #ifdef use_Serial_ref                       // In case of using RC
          print_ref_serial_data();
    #endif 
     print_imu_data();      // Print data from IMU
     print_error();         // Print error data.
     print_PID_values();
     Serial.println();
     lastPrint = millis(); // Update lastPrint time
    } 
}
