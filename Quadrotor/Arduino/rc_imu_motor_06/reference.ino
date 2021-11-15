// ...................... Escale data ........................ //

void references()
{
#ifdef use_RC_ref
  boolean cond1 = false, cond2 = false, cond3 = false, cond4 = false;

  cond1 = (pulse_duration[PITCH]>=1632 & pulse_duration[PITCH]<=1668) | pulse_duration[PITCH]<=800;
  cond2 = (pulse_duration[ROLL]>=1492 & pulse_duration[ROLL]<=1528) | pulse_duration[ROLL]<=800;
  cond3 = (pulse_duration[YAW]>=1476 & pulse_duration[YAW]<=1500) | pulse_duration[YAW]<=800;
  cond4 = (pulse_duration[THROTTLE]>=1862 & pulse_duration[THROTTLE]<=1902) | pulse_duration[THROTTLE]<=800;
  cond_stop = cond1 & cond2 & cond3 & cond4;
  if (cond_stop)
  {
     ref[YAW] = 0;
     ref[PITCH] = 0;
     ref[ROLL] = 0;
     ref[THROTTLE] = 1020;
  }
  else
  {
     // Correct sense for YAW and THROTTLE.
      pulse_duration_modif[0] = ROLL_width_min +  ROLL_width_max - pulse_duration[ROLL];       // Correct sense of yaw.
      pulse_duration_modif[1] = THROTTLE_width_min +  THROTTLE_width_max - pulse_duration[THROTTLE];       // Correct sense of throttle.
    // Escale data recieved into angles.
      ref[PITCH]= 0; //map(pulse_duration[PITCH],PITCH_width_min,PITCH_width_max,PITCH_min,PITCH_max);                      // For angle PITCH
      ref[YAW]= 20;//map(pulse_duration[YAW],YAW_width_min,YAW_width_max,YAW_min,YAW_max);                                // For angle YAW
      ref[THROTTLE]= map(pulse_duration_modif[1],THROTTLE_width_min,THROTTLE_width_max,THROTTLE_min,THROTTLE_max);       // For throttle.
      ref[ROLL]= 0;// map(pulse_duration_modif[0],ROLL_width_min,ROLL_width_max,ROLL_min,ROLL_max);                           // For angle ROLL
   }
#endif
#ifdef use_Serial_ref
  serial_references();
#endif
}
// .......................................................... // 
#ifdef use_Serial_ref
void serial_references()
{
  serialEvent();
  if (data_sent) {
    select_data();
    // Erase string sent
    reference_command = "";             // clear the string:
    location = 0;                       
    data_sent = false;
  }
}

void select_data()
{
   String command = "";
   
   location = reference_command.indexOf("=");
   int location1 = reference_command.indexOf("\n");
     if (location1>=0)
     reference_command=reference_command.substring(0,location1);
     if (location<0)    // In case of not presenting the "=" symbol.
     {
        int final_loc = reference_command.indexOf(";");
        command = reference_command.substring(0,final_loc);   // Erase space in case of presenting one.
        Serial.println(command);
        if(command=="Start")
        {  cond_start = true;
          cond_stop = false;
        }
        else 
        {
          if(command=="Stop")
          {
            cond_start = false;
            cond_stop = true;
          }
          else
            Serial.println("Command of initialization not correct");
        }  
     }
     else
     {
       command = reference_command.substring(0,location);     // Get the String in order to salect the parameter.
       Serial.println(command);
       int space_loc = command.indexOf(" ");
       if(space_loc>0)
          command = command.substring(0,space_loc);   // Erase space in case of presenting one.
         
       String ref_num = reference_command.substring(location+1,reference_command.indexOf(";"));    // Get the values. 
       if(ref_num.indexOf(" ")==0)
          ref_num = ref_num.substring(1,reference_command.indexOf(";"));   // Erase space in case of presenting one.
       if(command=="Throttle")
         {
           ref[THROTTLE] = ref_num.toInt();
           //Serial.println(ref[0]);
         }  
         else 
         {     
              if (command=="Pitch")
              {
                   ref[PITCH] = ref_num.toInt();
                   //Serial.println(ref[1]);
              }
              else 
              {    
                  if (command=="Roll")
                  {
                      ref[ROLL] = ref_num.toInt();
                      //Serial.println(ref[2]);
                  }
                  else 
                  {
                      if (command=="Yaw")
                      {
                          ref[YAW] = ref_num.toInt();
                          //Serial.println(ref[3]);
                      }
                      else
                        Serial.println("Command not available");
                  }
              } 
          }
      }
}
   
void serialEvent() {
  while (Serial.available()) {
    
    char inChar = (char)Serial.read();        // get the new byte:
    reference_command += inChar;                    // add it to the reference_command:
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == ';' ||inChar == '\n') {
      data_sent = true;
    }
  }
}
void print_ref_serial_data()
{
 Serial.print("Ref: ");
 for (int i=0;i<4;i++)
    if (i<3)
    {
      Serial.print(ref[i]);
      Serial.print("; ");
    }
    else
      Serial.println(ref[i]);
}

#endif
