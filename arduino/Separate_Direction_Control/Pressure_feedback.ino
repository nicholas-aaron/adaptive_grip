/**
 * function: checkPressure - reads the analog input at A0, A2, A3, which detail the amount of force being replied to the sensors
 * parameters: *force - integer array pointer of the force values
 */


void checkPressure(int *(force)){
  force[0] = analogRead(A0);
  force[1] = analogRead(A2);
  force[2] = analogRead(A3);
}

/**
 * function: motorOperation - either stalls or rotates motor depending on force and position value
 * parameters: i - counter, check - status of rotation, *motor - integer array pointer of motor operation, 
 * *force - integer array pointer of the force values
 */

int motorOperation(int i, int check, int *(motor), int *(force)){
  if(positions[i] > limits[i])// check if motor has not fully closed -> keep motor rotating
  {
    if((i == 1 || i ==6) && force[0] < FORCE_THRESHOLD){//check if associated pressure sensor is under the threshold
      motors[i] = i+3;
      check = check + 1;
    }
    else{//threshold has been meet, stall motor
      motors[i] = 0;
    }
    if((i == 2 || i ==4) && force[1] < FORCE_THRESHOLD){
      motors[i] = i+3;
      check = check + 1;
    }
    else{
      motors[i] = 0;
    }
    if((i == 2 || i ==4) && force[1] < FORCE_THRESHOLD){
      motors[i] = i+3;
      check = check + 1;
    }
    else{
      motors[i] = 0;
    }
  }
  else // (positionspi] >= limits[i])//stall motor
     motors[i] = 0;

  return check;
}

