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
        if(command=="Start" || command=="start")
        {  cond_start = true;
          cond_stop = false;
        }
        else 
        {
          if(command=="Stop" || command=="stop")
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
       if(command=="Ref" || command=="ref")
         reference = ref_num.toInt();
       if (command=="kp1" || command=="Kp1")
         kp1 = ref_num.toFloat();
       if (command=="kp2" || command=="Kp2")
         kp2 = ref_num.toFloat();
       if (command=="kd1" || command=="Kd1")
         kd1 = ref_num.toFloat();
       if (command=="kd2" || command=="Kd2")
         kd2 = ref_num.toFloat();
       if (command=="kp" || command=="Kp")
       {
         kp1 = ref_num.toFloat();
         kp2 = ref_num.toFloat();
       }
       if (command=="kd" || command=="Kd")
       {
         kd1 = ref_num.toFloat();
         kd2 = ref_num.toFloat();
       }
       if (command=="T1" || command=="t1")
       {
         t1 = ref_num.toFloat();
       }
       if (command=="T2" || command=="t2")
       {
         t2 = ref_num.toFloat();
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
