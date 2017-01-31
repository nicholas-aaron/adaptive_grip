// ----------------------------------------------------------Drive Motors -----------------------------------------------------------------------//

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

// -----------------------------------------------------------Reset Array ----------------------------------------------------------------//
void zeroIntArray(int (*motors), int arrayLength)
{
  for (int x = 0; x < arrayLength; x ++){
    motors[x] = 0;
  } 
}

//------------------------------------------------------------Reset Boolean Array --------------------------------------------------//
void falseBooleanArray(boolean directions[], int arrayLength)
{
  for (int x = 0; x < arrayLength; x++){
    directions[x] = false;
  }
}

// ------------------------------------------------------------------- Hold Position -------------------------------------------------------------//

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

// ----------------------------------------------------------------- Drive Multiple Motors -----------------------------------------------------------//
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

void printArrays(int(*motors), boolean directions[], int arrayLength){
  for (int x = 0; x < arrayLength; x++){
    Serial.print("motors[" + String(x) + "] = " + String(motors[x]) + " | ");
  }
  Serial.println();
  for (int x = 0; x < arrayLength; x++){
    Serial.print("directions[" + String(x) + "] = " + String(directions[x]) + " | ");
  }
  Serial.println();
}



