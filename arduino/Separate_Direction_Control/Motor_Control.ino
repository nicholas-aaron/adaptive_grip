// -----------------------------Drive Motors -------------------------//

void driveMotor(int (*motors), int degRotation, boolean directions[], int arrayLength )
{
  double degPerStp = (double)(1.8 / 8); // This is used for 1/8th stepping mode
  int numSteps = (int)(degRotation / degPerStp);
  boolean dirSwitch = false;
  for (int x = 0; x < numSteps; x++) //Loop the forward stepping enough times for motion to be visible
  {
    if (dirSwitch) {
      digitalWrite(DIR_1, HIGH);
      digitalWrite(DIR_2, HIGH);
      digitalWrite(DIR_3, HIGH);
      digitalWrite(DIR_4, HIGH);
      digitalWrite(DIR_5, HIGH);
      digitalWrite(DIR_6, HIGH);
      dirSwitch = !dirSwitch;
    }
    else {
      digitalWrite(DIR_1, LOW);
      digitalWrite(DIR_2, LOW);
      digitalWrite(DIR_3, LOW);
      digitalWrite(DIR_4, LOW);
      digitalWrite(DIR_5, LOW);
      digitalWrite(DIR_6, LOW);
      dirSwitch = !dirSwitch;
    }
    for ( int y = 0; y < arrayLength; y++) { //loop through all of the motors 
      if (motors[y] != 0) { // mtr[y] set to zero if motor if not intended on moving
        if (directions[y]) {
          digitalWrite(motors[y], HIGH);
        }
        else {
          digitalWrite(motors[y], LOW);
        }
      }
    }

    delay(2);
    digitalWrite(STP, HIGH);
    delay(2);
    digitalWrite(STP, LOW);
  }
}

void setPins()
{
  digitalWrite(STP, LOW);
  digitalWrite(DIR_1, LOW); // All moving in the same direction
  digitalWrite(DIR_2, LOW);
  digitalWrite(DIR_3, LOW);
  digitalWrite(DIR_4, LOW);
  digitalWrite(DIR_5, LOW);
  digitalWrite(DIR_6, LOW);

}



// ---------------------------- Hold Position -----------------------//

void holdPosition(int mSecs)
{
  for (int x = 0; x < (int)(mSecs / 8); x ++)
  {
    for (int pinOut = DIR_1; pinOut <= DIR_6; pinOut++)
    {
      digitalWrite(pinOut, HIGH);
    }

    digitalWrite(STP, HIGH);
    delay(1);
    digitalWrite(STP, LOW);
    delay(3);

    for (int pinOut = DIR_1; pinOut <= DIR_6; pinOut++)
    {
      digitalWrite(pinOut, LOW);
    }

    digitalWrite(STP, HIGH);
    delay(1);
    digitalWrite(STP, LOW);
    delay(3);
  }
}


//-------------------------Test Motors ------------------------------//
void testAllMotors(int (*motors), boolean directions[]){

  int idx[6] = {0,0,0,0,0,0};

  for (idx[0] = 0; idx[0] <2; idx[0]++){
    for (idx[1] = 0; idx[1] <2; idx[1]++){
      for (idx[2] = 0; idx[2] <2; idx[2]++){
        for (idx[3] = 0; idx[3] <2; idx[3]++){
          for (idx[4] = 0; idx[4] <2; idx[4]++){
            for (idx[5] = 0; idx[5] <2; idx[5]++){
              zeroIntArray(motors,6);
              falseBooleanArray(directions,6);
              setMotors(idx, motors);
              driveMotor(motors,45,directions,6);
            }
          }
        }
      }
    }
  }
}


// ------------------------ Drive Multiple Motors ----------------------//

void multiMotorDrive(int mtr1, boolean dir1, int mtr2, boolean dir2, int rot) {
  double degPerStp = (double)(1.8 / 8); // This is used for 1/8th stepping mode
  int numSteps = (int)(rot / degPerStp);
  boolean dirSwitch = false;
  for (int x = 0; x < rot; x++) {
    if (dirSwitch) {
      digitalWrite(DIR_1, HIGH);
      digitalWrite(DIR_2, HIGH);
      digitalWrite(DIR_3, HIGH);
      digitalWrite(DIR_4, HIGH);
      digitalWrite(DIR_5, HIGH);
      digitalWrite(DIR_6, HIGH);
      dirSwitch = !dirSwitch;
    }
    else {
      digitalWrite(DIR_1, LOW);
      digitalWrite(DIR_2, LOW);
      digitalWrite(DIR_3, LOW);
      digitalWrite(DIR_4, LOW);
      digitalWrite(DIR_5, LOW);
      digitalWrite(DIR_6, LOW);
      dirSwitch = !dirSwitch;
    }
    if (dir1) {
      digitalWrite(mtr1, HIGH);
    }
    else {
      digitalWrite(mtr1, LOW);
    }
    if (dir2) {
      digitalWrite(mtr2, HIGH);
    }
    else {
      digitalWrite(mtr2, LOW);
    }

    delay(2);
    digitalWrite(STP, HIGH);
    delay(2);
    digitalWrite(STP, LOW);
  }
}

// -----------------------------------Set Motors Array ------------------------//

void setMotors(int (*index), int (*motors)){
  for (int x = 0; x < 6; x++){
    if (index[x]==1)
      motors[x] = x+3;//DIR_1 is three, DIR_5 is 7;
    else
      motors[x] = 0;
  }
}



