// ..................... IMU configuration .................. //
#include <Wire.h>             // The SFE_LSM9DS1 library requires both Wire and SPI be
#include <SPI.h>
#include <SparkFunLSM9DS1.h>  // Library for using IMU lsm9ds1

// ........... I2C Setup ..........     //
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M (Serial data output) is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// ............ LSM9DS1 Library Init .......... //
LSM9DS1 imu;                         // LSM9DS1 object for IMU 

// ................................................................ //

// ..............  Servo Configuration   .....................   //
#include <ServoTimer2.h>          // Call Servo library using timer 2
  ServoTimer2 motor_1;       // Servo object for motor 1
  ServoTimer2 motor_2;       // Servo object for motor 2
  ServoTimer2 motor_3;       // Servo object for motor 3
  ServoTimer2 motor_4;       // Servo object for motor 4

#define motor1_pin 3               // PIN used for motor 1
#define motor2_pin 5               // PIN used for motor 2
#define motor3_pin 6               // PIN used for motor 3
#define motor4_pin 11              // PIN used for motor 4

#define pwm_max 2000              // Maximum PWM value.   
#define pwm_min 1000               // Minimum PWM value.
// .......................................................  //

// ************************** Reference settings ************************** //
#define use_RC_ref                 // Get reference data from RC.
//#define use_Serial_ref             // Get reference data from Serial port. 

#define ROLL     1                 // Roll angle related to channel 4.
#define PITCH    0                 // Pitch angle related to channel 1.
#define THROTTLE 2                 // Throttle related to channel 3.
#define YAW      3                 // Yaw angle related to channel 2.
// ---------- Setting in case of using RC -------------------------- //
#ifdef use_RC_ref
    #define CHANNEL1 0                 // Vector component for channel 1
    #define CHANNEL2 1                 // Vector component for channel 2
    #define CHANNEL3 2                 // Vector component for channel 3
    #define CHANNEL4 3                 // Vector component for channel 4
    // ................ Angles Width Range ................. //
    
    #define ROLL_width_max       1944    // Maximum width for yaw angle
    #define ROLL_width_min       1136    // Minimum width for yaw angle
    #define PITCH_width_max     2056    //1925    // Maximum width for pitch angle
    #define PITCH_width_min     1272    // Minimum width for pitch angle
   // #define YAW_width_max      1944    // Maximum width for roll angle
   // #define YAW_width_min      1136    // Minimum width for roll angle
    #define YAW_width_max      1912    // Maximum width for roll angle
    #define YAW_width_min      1112    // Minimum width for roll angle
    #define THROTTLE_width_max  1928    // Maximum width for throttle
    #define THROTTLE_width_min  1128        // Maximum width for throttle
    // ..................................................... //
    
    // ............... Angles Range ....................... //
    
    #define YAW_min             -180  // Maximum yaw angle
    #define YAW_max              180  // Minimum yaw angle
    #define PITCH_min            -33  // Maximum pitch angle
    #define PITCH_max             33  // Minimum pitch angle
    #define ROLL_min             -33  // Maximum roll angle
    #define ROLL_max              33  // Minimum roll angle
    #define THROTTLE_min        1000 // Maximum throttle
    #define THROTTLE_max        1800 // Maximum throttle
    // ..................................................... //
#endif

// **************************************************************** //

// ............... Complementary filter settings  ............. //
#define use_complementary_filter_imu    // In case of appliying a complementary filter.
#ifdef use_complementary_filter_imu
  #define fs 250                        // define sample frequency.
  #define A 0.85                        // Complementary filter coefficient.
  #define Ay 0.85                       // Complementary filter coefficient for yaw angle.
  #define tscf 60                        // [ms] between each calculation.                           
#endif
// .......................................................  //
// ............... Kalman filter settings  ............. //
#define use_kalman_filter
#ifdef use_kalman_filter
  #define tsk 30000                      // [us] between each calculation.                                               
#endif
// .......................................................  //
// ********************** Offset Settings *********************** //
#define calculate_offset                // calculate offset for magnetometer and accelerometer angles  
//#define calculate_yaw_offset            // calculate offset for yae angle.
#define calculate_accel_offset          // calculate the offset for accelerometer measures.
//#define calculate_mag_offset          // calculate the offset for magnetometer measures.
//#define calculate_gyro_offset          // calculate the offset for gyroscope measures.

// ******************* Program debug Settings ******************* //
// ............. IMU ..................  //
#define sample_time_imu 10000                // n [us] between each measure.



// Debug options.

// #define Debug_sensor_offsets
// #define Debug_magnetometer_offsets
// #define Debug_yaw_offset

//#define debug_calculated_imu_data
//#define debug_raw_data
//#define debug_acc_angles_calculated     // Print angles calculated by accelerometer
//#define debug_gyro_angles_calculated    // Print angles calculated by gyroscope
//#define debug_angles                    // To print Roll, pitch and yaw
//#define debug_filtered_angles           // To print complementary filter values
#define debug_kalman_filter_angles      // To print kalman filter values

#define debug_data_matlab                 // Print data to visualize into Matlab.


#ifdef debug_data_matlab
//  #define debug_mat_acc                   // Print values gotten by accelerometer.
//  #define debug_mat_gyro                  // Print values gotten by gyroscope
// #define debug_mat_mag                   // Print values gotten by magnetometer.
//  #define debug_mat_angles                // Print angles calculated by accelerometer and magnetometer
// #define debug_mat_angles_and_rates          // Print angles and the respective rates.
  #ifdef use_kalman_filter
//   #define debug_mat_kalman_angles         // Print values gotten by kalman filter.  
//   #define debug_mat_kalman_and_angles   // Print just angles calculated and kalman values.
    #ifdef use_complementary_filter_imu
//      #define debug_mat_all_angles          // Print all angles for comparison.
    
    #endif    
  #endif
  #ifdef  use_complementary_filter_imu
   // #define debug_mat_filt_angles         // Print angles using the complementary filter.
//    #define debug_mat_filt_and_angles         // Print angles using the complementary filter.
  #endif  
#endif 
// .......................................................  //

// ............................ RF ....................... //
#ifdef use_RC_ref
    #define Debug_RC                  // Print RF width obtained.
    #define Debug_RC_ref            // Print scaled data.

#endif
// .......................................................  //

// ....................... Serial References ............. //
#ifdef use_serial_ref
//    #define Debug_Serial_ref
#endif

// .................PID controller ....................... //
#define tsc 50000                       // [us] between calculations.
#define use_PID_controller
//#define use_PD_controller
#define debug_error                     // to print errors
//#define debug_pid_gains               // to print PID gains for each angle.

#ifdef use_PD_controller
  //#define debug_pd_gains               // to print PID gains for each angle.
  #define debug_pd_values              // to print PID values for each motor.  

#endif

#ifdef use_PID_controller
  #define debug_integral_components     // to print integral components of PID controller.
  //#define debug_pid_gains               // to print PID gains for each angle.
  #define debug_pid_values              // to print PID values for each motor.  
#endif

#define debug_derivative_components   // to print derivative components of PID controller.
#define debug_error                     // to print errors
#define debug_sat_values              // to print vaturated PID values.




// .......................................................  //
    

// ************************************************************* //
  
