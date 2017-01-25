
/**
 * Function: driveMotor
 * 
 * Rotates the motor(s) by a specified number of degrees at full, half, quarter or eight step speed in
 * either clockwise or counter-clockwise direction
 * 
 * degrees: amount by which the motor is rotated
 * stepStyle: absolute change in angular displacement motor takes with each 'step'
 * highOrLow: the direction that the motor will rotate; CW vs CCW
 */

double selectStepSize(int stpSize){
  double degreesPerStep = 0;
  switch(stpSize){
  case 0: { // Full Step
    //Serial.println("Moving at full step mode.");
    digitalWrite(MS1, LOW); 
    digitalWrite(MS2, LOW);
    degreesPerStep = (double)(1.8/1);
    return degreesPerStep;
    break;
  }
  case 1: { // Half Step
    //Serial.println("Moving at half step mode.");
    digitalWrite(MS1, HIGH); 
    digitalWrite(MS2, LOW);
    degreesPerStep = (double)(1.8/2);
    return degreesPerStep;
    break;
  }
  case 2: { // Quarter Step
    //Serial.println("Moving at quarter step mode.");
    digitalWrite(MS1, LOW); 
    digitalWrite(MS2, HIGH);
    degreesPerStep = (double)(1.8/4);
    return degreesPerStep;
    break;
  }
  default: { // Eighth Step
    //Serial.println("Moving at eigth step mode.");
    digitalWrite(MS1, HIGH); 
    digitalWrite(MS2, HIGH);
    degreesPerStep = (double)(1.8/8);
    return degreesPerStep;
    break;
  }
  }
}

void driveMotor(int mtrID, int degRot, boolean rotDir, double degPerStp, boolean dirSwitch )
{
  int numSteps = (int)(degRot/degPerStp); 
  for(x= 1; x<numSteps; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    if (dirSwitch){
      digitalWrite(DIR_1,HIGH);
      digitalWrite(DIR_2,HIGH);
      digitalWrite(DIR_3,HIGH);
      digitalWrite(DIR_4,HIGH);
      digitalWrite(DIR_5,HIGH);
      digitalWrite(DIR_6,HIGH);
      dirSwitch = !dirSwitch;
    }
    else{
      digitalWrite(DIR_1,LOW);
      digitalWrite(DIR_2,LOW);
      digitalWrite(DIR_3,LOW);
      digitalWrite(DIR_4,LOW);
      digitalWrite(DIR_5,LOW);
      digitalWrite(DIR_6,LOW);
      dirSwitch = !dirSwitch;
    }
    if (rotDir){
      digitalWrite(mtrID,HIGH);
    }
    else{
      digitalWrite(mtrID,LOW);
    }
    
    delay(2);
    digitalWrite(stp, HIGH);
    delay(2);
    digitalWrite(stp,LOW);
  }
}


/**
 * Function: Reset pins
 * 
 * Disables all 6 motors from moving.  Changes motor step size to full step.
 */
void setPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(DIR_1, LOW); // All moving in the same direction
  digitalWrite(DIR_2, LOW);
  digitalWrite(DIR_3, LOW);
  digitalWrite(DIR_4, LOW);
  digitalWrite(DIR_5, LOW);
  digitalWrite(DIR_6, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(EN, LOW);
}

void holdPosition()
{ // Holds a position by vibrating back and forth very, very quickly
  boolean dirSwitch = true;
  for (int i = 0; i < 1000; i++){
    if (dirSwitch){
      digitalWrite(DIR_1,HIGH);
      digitalWrite(DIR_2,HIGH);
      digitalWrite(DIR_3,HIGH);
      digitalWrite(DIR_4,HIGH);
      digitalWrite(DIR_5,HIGH);
      digitalWrite(DIR_6,HIGH);
      dirSwitch = !dirSwitch;
    }
    else{
      digitalWrite(DIR_1,LOW);
      digitalWrite(DIR_2,LOW);
      digitalWrite(DIR_3,LOW);
      digitalWrite(DIR_4,LOW);
      digitalWrite(DIR_5,LOW);
      digitalWrite(DIR_6,LOW);
      dirSwitch = !dirSwitch;
    }

    delay(2);
    digitalWrite(stp,HIGH);
    delay(2);
    digitalWrite(stp, LOW);
  }
}
