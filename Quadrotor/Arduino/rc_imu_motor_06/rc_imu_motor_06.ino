#include "config.h"
// --------------- Working time and emergency stop ------- //
#define working_time 120000
boolean condition = true;
float a1 = 0, a0 = 0, diff = 0;     // To detect change on z acceleration
boolean accel_stop = false;


// *************** Global variables. ********************* //
// -------------------- IMU ----------------------------- //
static unsigned long lastPrint = 0; // Keep track of print time
float roll_acc = 0, pitch_acc = 0;  // Angles to be defined.
float yaw = 0;
float roll_filt = 0, pitch_filt = 0, yaw_filt = 0; // Filtered Angles.
float roll_k_ant = 0, pitch_k_ant = 0, yaw_k_ant = 0; // To store previous kalman angles data

float roll_ant = 0, pitch_ant = 0, yaw_ant = 0; // To store previous data
float roll_deg = 0, pitch_deg = 0, yaw_deg = 0; // Angles on degrees. 
float roll_filt_deg = 0, pitch_filt_deg = 0, yaw_filt_deg = 0; // Filtered angles on degrees. 
float roll_k_f_deg = 0, pitch_k_f_deg = 0, yaw_k_f_deg = 0;    // Kalman filter Angles. 
boolean cond = false;                           // Used to indicate that kalman filter has initial values.
#define DECLINATION 0.91 // Earth magnetic declination (degrees) in Ivry sur seine (Paris).
long cp_loop_time= 0; // variable to save time for each iteration of the complementary filter.
long timer_pid = 0;         // Sample time applied to derivative part of pid controller.
// ----------------------------------------------------------- //
// ------------------ Kalman filter ---------------------- //
#ifdef use_kalman_filter
  long kalman_loop_time= 0;
  // define the parameter for kalman filter.
  float xk_pitch[2] = {0, 0};               // Priori state 
  float xk_pitch1[2] = {0, 0};              // Posteriori state estimated.
  float xk_pitchMinus[2] = {0, 0};          // Posteriori state estimated.
  float pk_pitch[4] = {0.05, 0, 0, 0.01};   // Priori probability kalman values. 
  float pk_pitch1[4] = {0, 0, 0, 0};  // Posteriori probability kalman values. 
  float pk_pitch1Minus[4] = {0, 0, 0, 0};   // Priori probablilty matrix.
  float k_pitch[2] = {0, 0};               // Kalman gains

  // define the parameter for kalman filter.
  float xk_roll[2] = {0, 0};               // Priori state 
  float xk_roll1[2] = {0, 0};              // Posteriori state estimated.
  float xk_rollMinus[2] = {0, 0};          // Posteriori state estimated.
  float pk_roll[4] = {0.05, 0, 0, 0.01};   // Priori probability kalman values. 
  float pk_roll1[4] = {0, 0, 0, 0};  // Posteriori probability kalman values. 
  float pk_roll1Minus[4] = {0, 0, 0, 0};   // Priori probablilty matrix.
  float k_roll[2] = {0, 0};               // Kalman gains

  // define the parameter for kalman filter.
  float xk_yaw[2] = {0, 0};               // Priori state 
  float xk_yaw1[2] = {0, 0};              // Posteriori state estimated.
  float xk_yawMinus[2] = {0, 0};          // Posteriori state estimated.
  float pk_yaw[4] = {0.05, 0, 0, 0.01};   // Priori probability kalman values. 
  float pk_yaw1[4] = {0, 0, 0, 0};  // Posteriori probability kalman values. 
  float pk_yaw1Minus[4] = {0, 0, 0, 0};   // Priori probablilty matrix.
  float k[2] = {0, 0};               // Kalman gains
#endif
// **************** References ******************************* //
int ref[4] = {0, 0, 0, 0};        // Reference vector.
boolean cond_stop = true;         // Stop condition.
// ---------------------------- RF --------------------------- //
#ifdef use_RC_ref
    volatile unsigned long current_time;         // To measure current time of each interruption 
    volatile unsigned long timer[4];             // Vector to save current time for each channel 
    volatile byte previous_state[4];             // Previous state of each channel (HIGH or LOW) 
    // Duration of the pulse on each channel in µs (must be within 1000µs & 2000µs)
    volatile unsigned int pulse_duration[4] = {800, 800, 800, 800};
    volatile unsigned int previous_pulse_duration[4] = {1000, 1000, 1000, 1000};    // To save previous data
    int pulse_duration_modif[4] = {1000, 1000, 1000, 1000};
#endif    
// ......................................................... //

// --------------------- Serial References ---------------- //
#ifdef use_Serial_ref
/* Commands applied:
 *  Start;  Initialize the debugging.
 *  Stop;   Stop the debugging.
 *  Throttle=n;   Modify the value of throttle reference to n value.
 *  Roll=n;       Modify the value of roll reference to n value.
 *  Pitch=n;      Modify the value of pitch reference to n value.
 *  Yaw=n;        Modify the value of yaw reference to n value.
 *  
 */
    String reference_command = "";      // a String to hold incoming data
    bool data_sent = false;             // whether the string is complete
    bool cond_start = false;            // flag to indicate that start command was introduced.
    int location = 0;
#endif
// ------------------------------------------------------- //

// ...... global variable for motors ............. //
long pwm_motor_esc1 = pwm_min;     // Escaled width applied to motor 1
long pwm_motor_esc2 = pwm_min;     // Escaled width applied to motor 2
long pwm_motor_esc3 = pwm_min;     // Escaled width applied to motor 3
long pwm_motor_esc4 = pwm_min;     // Escaled width applied to motor 4


void setup() 
{
  Serial.begin(115200);                   // Set Serial port Baud Rate
  imu_configuration();                    // Configure the IMU  
  motor_config();                         // Configure the motors.
  
  #ifdef use_RC_ref                       // In case of using RC
    rf_interrupt_configuration();         // Configure the interruptions.
  #endif 
  #ifdef use_Serial_ref                   // Get data from Serial port 
    reference_command.reserve(200);       // reserve 200 bytes for the reference string.
  #endif
  sample_interrupt();                     // Configure interruption for PID error sampling.
  // Initialize timers
  cp_loop_time = millis();
  kalman_loop_time = micros();
  timer_pid = micros();
}

void loop()
{
  long counter_time = millis();
  long imu_counter_time = micros();
  #ifdef use_Serial_ref
    serial_references(); 
    condition = cond_start;
    delay(100); 
  #endif
  while (condition)
  { 
      // 1. Read data from IMU according the sample time set
      if (micros()-imu_counter_time > sample_time_imu)
      {
        check_and_read_data();                // Configure and read data from IMU
        imu_counter_time = micros();    
      } 
      // 2. Get reference data.
        references(); 
             
      // 3. Print data.
        print_data();                         // Print data from reference and IMU.
      // 4. PID controller.
      controller();
     
      // 5. Motor actuation.
      motor_actuation();
      #ifdef use_Serial_ref 
        if ( ((millis() - counter_time) > working_time) || cond_stop==true)
        {
            Serial.println("END");
            for (int i=0;i<4;i++)
              ref[i] = 0;
            cond_start=false;
            condition = false; 
        }
      #endif
      #ifdef use_RC_ref 
        if ( ((millis() - counter_time) > working_time))
        {
            Serial.println("END");
            for (int i=0;i<4;i++)
            condition = false; 
        }
      #endif
  }
  motor_stop();
  reset_controller();
}
