#ifdef use_kalman_filter
  float kalman_filter_pitch(float ang_acc, float gyro_ang, float deltaT, float Qvar, float Qbias, float R)
  {
      // Kalman matrix.
      float phi[4] = {1, deltaT, 0, 1};   // State transition Matrix
      float psi[2] = {deltaT, 0};        // Control input model.      
      //float Q[4] = {Qvar*Qvar,0, 0, 0};   // Covariance process noise.
      float Q[4] = {Qvar,0, Qbias, 0};   // Covariance process noise.
      float H[2] = {1, 0};               // Observation model.
      // Measure parameters.
      float uk = gyro_ang;              // Control variable
      float zk = ang_acc;               // Estimated variable
      // Print data
//      Serial.print("Angle and gyro: ");
//      Serial.print(zk,4);
//      Serial.print(",");
//      Serial.println(uk,4);
      // --------------- Prediction ---------------- //
      // Predicted a priori state estimate: xk_pitch1Minus = phi.* xk_pitch' + psi* uk;
       xk_pitchMinus[0] = phi[0] * xk_pitch[0] + phi[1] * xk_pitch[1] + psi[0] * uk;
       xk_pitchMinus[1] = phi[2] * xk_pitch[0] + phi[3] * xk_pitch[1] + psi[1] * uk;
//      Serial.print("States predicted: ");
//      Serial.print(xk_pitchMinus[0],4);
//      Serial.print(" ");
//      Serial.println(xk_pitchMinus[1],4);
      // Predicted Error covariance: pk_pitch1Minus = phi.*(pk_pitch.*phi') + Q;
      float alpha[4] = {0, 0, 0, 0};
      alpha[0] = phi[0] * pk_pitch[0] + phi[1] * pk_pitch[2];
      alpha[1] = phi[0] * pk_pitch[1] + phi[1] * pk_pitch[3];
      alpha[2] = phi[2] * pk_pitch[0] + phi[3] * pk_pitch[2];
      alpha[3] = phi[2] * pk_pitch[1] + phi[3] * pk_pitch[3];
      pk_pitch1Minus[0] = alpha[0] * phi[0] + alpha[1] * phi[1] + Q[0];
      pk_pitch1Minus[1] = alpha[0] * phi[2] + alpha[1] * phi[3] + Q[1];
      pk_pitch1Minus[2] = alpha[2] * phi[0] + alpha[3] * phi[1] + Q[2];
      pk_pitch1Minus[3] = alpha[2] * phi[2] + alpha[3] * phi[3] + Q[3];
//      Serial.print("Predicted probablilistic matrix: ");
//      Serial.print(pk_pitch1Minus[0],4);
//      Serial.print(" ");
//      Serial.print(pk_pitch1Minus[1],4);
//      Serial.print(" ");
//      Serial.print(pk_pitch1Minus[2],4);
//      Serial.print(" ");
//      Serial.println(pk_pitch1Minus[3],4);
      // ---------------- Update ------------------- //
      //  Innovation covariance
      float S = (H[0] * pk_pitch1Minus[0] + H[1] * pk_pitch1Minus[2]) *  H[0] 
              + (H[0] * pk_pitch1Minus[1] + H[1] * pk_pitch1Minus[3]) *  H[1] + R;
//      Serial.print("Inonvation covariance: ");
//      Serial.println(S,4);
      //  Kalman gains
      k_pitch[0] = (pk_pitch1Minus[0] * H[0] + pk_pitch1Minus[1] * H[1])/S;
      k_pitch[1] = (pk_pitch1Minus[2] * H[0] + pk_pitch1Minus[3] * H[1])/S; 
//      Serial.print("Kalman gains: ");
//      Serial.print(k_pitch[0],4);
//      Serial.print(" ");
//      Serial.println(k_pitch[1],4);
      //  Innovation measurement.
      float yk = zk - (H[0] * xk_pitchMinus[0] + H[1] * xk_pitchMinus[1]);
      // Updated state estimate.
      xk_pitch1[0] = xk_pitchMinus[0] + k_pitch[0] * yk;
      xk_pitch1[1] = xk_pitchMinus[1] + k_pitch[1] * yk;
      
      // Updated estimate probabilistic matrix.
      pk_pitch1[0] = (1 - k_pitch[0] * H[0]) * pk_pitch1Minus[0] + 
                     (0 - k_pitch[0] * H[1]) * pk_pitch1Minus[2];
      pk_pitch1[1] = (1 - k_pitch[0] * H[0]) * pk_pitch1Minus[1] + 
                     (0 - k_pitch[0] * H[1]) * pk_pitch1Minus[3];
      pk_pitch1[2] = (0 - k_pitch[1] * H[0]) * pk_pitch1Minus[0] + 
                     (1 - k_pitch[1] * H[1]) * pk_pitch1Minus[2];
      pk_pitch1[3] = (0 - k_pitch[1] * H[0]) * pk_pitch1Minus[1] + 
                     (1 - k_pitch[1] * H[1]) * pk_pitch1Minus[3];
      
     
      
      // Reset values for next iteration.
      // States.
      xk_pitch[0] = xk_pitch1[0];
      xk_pitch[1] = xk_pitch1[1];
//      Serial.print("States estimated: ");
//      Serial.print(xk_pitch1[0],4);
//      Serial.print(" ");
//      Serial.println(xk_pitch1[1],4);
      // Probability matrix.
      pk_pitch[0] = pk_pitch1[0];
      pk_pitch[1] = pk_pitch1[1];
      pk_pitch[2] = pk_pitch1[2];
      pk_pitch[3] = pk_pitch1[3];
//      Serial.print("Probabilty Matrix: ");
//      Serial.print(pk_pitch1[0],4);
//      Serial.print(" ");
//      Serial.print(pk_pitch1[1],4);
//      Serial.print(" ");
//      Serial.print(pk_pitch1[2],4);
//      Serial.print(" ");
//      Serial.println(pk_pitch1[3],4);
      
      return xk_pitch[0];
  }
  
  float kalman_filter_roll(float ang_acc, float gyro_ang, float deltaT, float Qvar,float Qbias, float R)
  {
      // Kalman matrix.
      float phi[4] = {1, deltaT, 0, 1};   // State transition Matrix
      float psi[2] = {deltaT, 0};        // Control input model.      
    //  float Q[4] = {Qvar*Qvar,0, 0, 0};   // Covariance process noise.
      float Q[4] = {Qvar,0, Qbias, 0};   // Covariance process noise.
      float H[2] = {1, 0};               // Observation model.
      // Measure parameters.
      float uk = gyro_ang;              // Control variable
      float zk = ang_acc;               // Estimated variable
      // --------------- Prediction ---------------- //
      // Predicted a priori state estimate: xk1Minus = phi.* xk' + psi* uk;
       xk_rollMinus[0] = phi[0] * xk_roll[0] + phi[1] * xk_roll[1] + psi[0] * uk;
       xk_rollMinus[1] = phi[2] * xk_roll[0] + phi[3] * xk_roll[1] + psi[1] * uk;
  
      // Predicted Error covariance: pk1Minus = phi.*(pk.*phi') + Q;
      float alpha[4] = {0, 0, 0, 0};
      alpha[0] = phi[0] * pk_roll[0] + phi[1] * pk_roll[2];
      alpha[1] = phi[0] * pk_roll[1] + phi[1] * pk_roll[3];
      alpha[2] = phi[2] * pk_roll[0] + phi[3] * pk_roll[2];
      alpha[3] = phi[2] * pk_roll[1] + phi[3] * pk_roll[3];
      pk_roll1Minus[0] = alpha[0] * phi[0] + alpha[1] * phi[1] + Q[0];
      pk_roll1Minus[1] = alpha[0] * phi[2] + alpha[1] * phi[3] + Q[1];
      pk_roll1Minus[2] = alpha[2] * phi[0] + alpha[3] * phi[1] + Q[2];
      pk_roll1Minus[3] = alpha[2] * phi[2] + alpha[3] * phi[3] + Q[3];
  
      // ---------------- Update ------------------- //
      //  Innovation covariance
      float S = (H[0] * pk_roll1Minus[0] + H[1] * pk_roll1Minus[2]) *  H[0] 
              + (H[0] * pk_roll1Minus[1] + H[1] * pk_roll1Minus[3]) *  H[1] + R;
      //  Kalman gains
      k_roll[0] = (pk_roll1Minus[0] * H[0] + pk_roll1Minus[1] * H[1])/S;
      k_roll[1] = (pk_roll1Minus[2] * H[0] + pk_roll1Minus[3] * H[1])/S; 
      //  Innovation measurement.
      float yk = zk - (H[0] * xk_rollMinus[0] + H[1] * xk_rollMinus[1]);
      // Updated state estimate.
      xk_roll1[0] = xk_rollMinus[0] + k_roll[0] * yk;
      xk_roll1[1] = xk_rollMinus[1] + k_roll[1] * yk;
      // Updated estimate probabilistic matrix.
      pk_roll1[0] = (1 - k_roll[0] * H[0]) * pk_roll1Minus[0] + 
                    (0 - k_roll[0] * H[1]) * pk_roll1Minus[2];
      pk_roll1[1] = (1 - k_roll[0] * H[0]) * pk_roll1Minus[1] + 
                    (0 - k_roll[0] * H[1]) * pk_roll1Minus[3];
      pk_roll1[2] = (0 - k_roll[1] * H[0]) * pk_roll1Minus[0] + 
                    (1 - k_roll[1] * H[1]) * pk_roll1Minus[2];
      pk_roll1[3] = (0 - k_roll[1] * H[0]) * pk_roll1Minus[1] + 
                    (1 - k_roll[1] * H[1]) * pk_roll1Minus[3];
      // Reset values for next iteration.
      // States.
      xk_roll[0] = xk_roll1[0];
      xk_roll[1] = xk_roll1[1];
      // Probability matrix.
      pk_roll[0] = pk_roll1[0];
      pk_roll[1] = pk_roll1[1];
      pk_roll[2] = pk_roll1[2];
      pk_roll[3] = pk_roll1[3];
      return xk_roll[0];
  }
  
  float kalman_filter_yaw(float ang_acc, float gyro_ang, float deltaT, float Qvar, float Qbias,float R)
  {
      // Kalman matrix.
      float phi[4] = {1, deltaT, 0, 1};   // State transition Matrix
      float psi[2] = {deltaT, 0};        // Control input model.      
      float Q[4] = {Qvar,0, Qbias, 0};   // Covariance process noise.
      float H[2] = {1, 0};               // Observation model.
      // Measure parameters.
      float uk = gyro_ang;              // Control variable
      float zk = ang_acc;               // Estimated variable
      // --------------- Prediction ---------------- //
      // Predicted a priori state estimate: xk1Minus = phi.* xk' + psi* uk;
       xk_yawMinus[0] = phi[0] * xk_yaw[0] + phi[1] * xk_yaw[1] + psi[0] * uk;
       xk_yawMinus[1] = phi[2] * xk_yaw[0] + phi[3] * xk_yaw[1] + psi[1] * uk;
  
      // Predicted Error covariance: pk1Minus = phi.*(pk.*phi') + Q;
      float alpha[4] = {0, 0, 0, 0};
      alpha[0] = phi[0] * pk_yaw[0] + phi[1] * pk_yaw[2];
      alpha[1] = phi[0] * pk_yaw[1] + phi[1] * pk_yaw[3];
      alpha[2] = phi[2] * pk_yaw[0] + phi[3] * pk_yaw[2];
      alpha[3] = phi[2] * pk_yaw[1] + phi[3] * pk_yaw[3];
      pk_yaw1Minus[0] = alpha[0] * phi[0] + alpha[1] * phi[1] + Q[0];
      pk_yaw1Minus[1] = alpha[0] * phi[2] + alpha[1] * phi[3] + Q[1];
      pk_yaw1Minus[2] = alpha[2] * phi[0] + alpha[3] * phi[1] + Q[2];
      pk_yaw1Minus[3] = alpha[2] * phi[2] + alpha[3] * phi[3] + Q[3];
  
      // ---------------- Update ------------------- //
      //  Innovation covariance
      float S = (H[0] * pk_yaw1Minus[0] + H[1] * pk_yaw1Minus[2]) *  H[0] 
              + (H[0] * pk_yaw1Minus[1] + H[1] * pk_yaw1Minus[3]) *  H[1] + R;
      //  Kalman gains
      k[0] = (pk_yaw1Minus[0] * H[0] + pk_yaw1Minus[1] * H[1])/S;
      k[1] = (pk_yaw1Minus[2] * H[0] + pk_yaw1Minus[3] * H[1])/S; 
      //  Innovation measurement.
      float yk = zk - (H[0] * xk_yawMinus[0] + H[1] * xk_yawMinus[1]);
      // Updated state estimate.
      xk_yaw1[0] = xk_yawMinus[0] + k[0] * yk;
      xk_yaw1[1] = xk_yawMinus[1] + k[1] * yk;
      // Updated estimate probabilistic matrix.
      pk_yaw1[0] = (1 - k[0] * H[0]) * pk_yaw1Minus[0] + 
                   (0 - k[0] * H[1]) * pk_yaw1Minus[2];
      pk_yaw1[1] = (1 - k[0] * H[0]) * pk_yaw1Minus[1] + 
                   (0 - k[0] * H[1]) * pk_yaw1Minus[3];
      pk_yaw1[2] = (0 - k[1] * H[0]) * pk_yaw1Minus[0] + 
                   (1 - k[1] * H[1]) * pk_yaw1Minus[2];
      pk_yaw1[3] = (0 - k[1] * H[0]) * pk_yaw1Minus[1] + 
                   (1 - k[1] * H[1]) * pk_yaw1Minus[3];
      // Reset values for next iteration.
      // States.
      xk_yaw[0] = xk_yaw1[0];
      xk_yaw[1] = xk_yaw1[1];
      // Probability matrix.
      pk_yaw[0] = pk_yaw1[0];
      pk_yaw[1] = pk_yaw1[1];
      pk_yaw[2] = pk_yaw1[2];
      pk_yaw[3] = pk_yaw1[3];
      return xk_yaw[0];
  }
#endif
