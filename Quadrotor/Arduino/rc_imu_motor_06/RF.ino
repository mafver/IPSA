#ifdef use_RC_ref
  // .................. Configuring Interruption pins .......... //
  void rf_interrupt_configuration()
  {
     // Configure interruptions for RC pwm modulation.
      PCICR  |= (1 << PCIE1);  //Set PCIE0 to enable PCMSK0 scan.
      PCMSK1 |= (1 << PCINT8); //Set PCINT8 (analog input A0) to trigger an interrupt on state change.
      PCMSK1 |= (1 << PCINT9); //Set PCINT9 (analog input A1)to trigger an interrupt on state change.
      PCMSK1 |= (1 << PCINT11); //Set PCINT10 (analog input A2)to trigger an interrupt on state change.
      PCMSK1 |= (1 << PCINT12); //Set PCINT11 (analog input A3)to trigger an interrupt on state change.
  }
  // ----------------------------------------------------------- //
  
  // .................................. Arduino Nano interruption function ...................................... //
  ISR(PCINT1_vect)
  {
      current_time = micros();
  
      // Channel 1 -------------------------------------------------
      if (PINC & 0x01) {                                             // Is input A0 high ?
          if (previous_state[CHANNEL1] == 0) {                     // Input A0 changed from 0 to 1 (rising edge)
              previous_state[CHANNEL1] = 1;                       // Save current state
              timer[CHANNEL1]          = current_time;               // Start timer
          }
      } else if(previous_state[CHANNEL1] == 1) {                  // Input A0 changed from 1 to 0 (falling edge)
          previous_state[CHANNEL1] = 0;                            // Save current state
          pulse_duration[CHANNEL1] = current_time - timer[CHANNEL1]; // Stop timer & calculate pulse duration
      }
  
      // Channel 2 -------------------------------------------------
      if (PINC & 0x02) {                                        // Is input A1 high ?
          if (previous_state[CHANNEL2] == 0) {                     // Input A1 changed from 0 to 1 (rising edge)
              previous_state[CHANNEL2] = 1;                       // Save current state
              timer[CHANNEL2]          = current_time;               // Start timer
          }
      } else if(previous_state[CHANNEL2] == 1) {                  // Input A1 changed from 1 to 0 (falling edge)
          previous_state[CHANNEL2] = 0;                            // Save current state
          pulse_duration[CHANNEL2] = current_time - timer[CHANNEL2]; // Stop timer & calculate pulse duration
      }
  
      // Channel 3 -------------------------------------------------
      if (PINC & 0x04) {                                        // Is input A2 high ?
          if (previous_state[CHANNEL3] == 0) {                     // Input A2 changed from 0 to 1 (rising edge)
              previous_state[CHANNEL3] = 1;                       // Save current state
              timer[CHANNEL3]          = current_time;               // Start timer
          }
      } else if(previous_state[CHANNEL3] == 1) {                  // Input A2 changed from 1 to 0 (falling edge)
          previous_state[CHANNEL3] = 0;                            // Save current state
          pulse_duration[CHANNEL3] = current_time - timer[CHANNEL3]; // Stop timer & calculate pulse duration
      }
  
      // Channel 4 -------------------------------------------------
      if (PINC & 0x08) {                                        // Is input A3 high ?
          if (previous_state[CHANNEL4] == 0) {                     // Input 11 changed from 0 to 1 (rising edge)
              previous_state[CHANNEL4] = 1;                       // Save current state
              timer[CHANNEL4]          = current_time;               // Start timer
          }
      } else if(previous_state[CHANNEL4] == 1) {                  // Input A3 changed from 1 to 0 (falling edge)
          previous_state[CHANNEL4] = 0;                            // Save current state
          pulse_duration[CHANNEL4] = current_time - timer[CHANNEL4]; // Stop timer & calculate pulse duration
      }
  }
  // ......................................................................................... //
#endif
// ..................................... RF print data .................................... //
void print_rf_data()
{
#ifdef Debug_RC
    // Print data.
    Serial.print("PITCH: ");
    Serial.print(pulse_duration[PITCH]);                              // Print Channel 1 width 
    Serial.print("; ");
    Serial.print("ROLL: ");
    Serial.print(pulse_duration[ROLL]);                            // Print Channel 4 width 
    Serial.print("; ");
    Serial.print("YAW: ");
    Serial.print(pulse_duration[YAW]);                              // Print Channel 2 width 
    Serial.print("; ");
    Serial.print("THROTTLE: ");
    Serial.println(pulse_duration[THROTTLE]);                              // Print Channel 3 width 
#endif
#ifdef Debug_RC_ref
    Serial.print("PITCH: ");
    Serial.print(ref[PITCH]);                              // Print Channel 1 width 
    Serial.print("; ");
    Serial.print("ROLL: ");
    Serial.print(ref[ROLL]);                            // Print Channel 4 width 
    Serial.print("; ");
    Serial.print("YAW: ");
    Serial.print(ref[YAW]);                              // Print Channel 2 width 
    Serial.print("; ");
    Serial.print("THROTTLE: ");
    Serial.println(ref[THROTTLE]);                              // Print Channel 3 width 
#endif
}
