void driveMotor(int (*motors), int degRotation, boolean directions[], int (*positions), int arrayLength )
{
  double degPerStp = (double)(1.8 / 8); // This is used for 1/8th stepping mode
  int numSteps = (int)(degRotation / degPerStp);
  boolean dirSwitch = false;
  for (int x = 0; x < numSteps; x++)
  {
    if (dirSwitch) { //Sets the motor to open
      digitalWrite(DIR_1, HIGH);
      digitalWrite(DIR_2, HIGH);
      digitalWrite(DIR_3, HIGH);
      digitalWrite(DIR_4, HIGH);
      digitalWrite(DIR_5, HIGH);
      digitalWrite(DIR_6, HIGH);
      dirSwitch = !dirSwitch;
    }
    else { //Sets the motor to close 
      digitalWrite(DIR_1, LOW);
      digitalWrite(DIR_2, LOW);
      digitalWrite(DIR_3, LOW);
      digitalWrite(DIR_4, LOW);
      digitalWrite(DIR_5, LOW);
      digitalWrite(DIR_6, LOW);
      dirSwitch = !dirSwitch;
    }

    // Overwrite directions
    selectMotor(motors, directions, positions);
    delay(2);
    digitalWrite(STP, HIGH);
    delay(2);
    digitalWrite(STP, LOW);
  }
}

void selectMotor(int (*motors), boolean directions[], int (*positions))
{
  for ( int y = 0; y < 6; y++) { //loop through all of the motors
    if (motors[y] != 0) { // mtr[y] set to zero if motor if not intended on moving
      if (directions[y]) { //Open
        if (y == 0 || y == 1 || y == 4) { // These motors close with HIGH direction command
          digitalWrite(motors[y], HIGH);
          positions[y] = positions[y] + 1;
        }
        else {
          if (y == 2 || y == 3 || y == 5) {// These motors close with HIGH directino command
            digitalWrite(motors[y], LOW);
            positions[y] = positions[y] + 1;
          }
          else {
            digitalWrite(motors[y], HIGH);
            Serial.println("Y value is " + String(y));
            Serial.println("ERROR in 'driveMotor', DEBUG");
          }
        }
      }
      else { //Close
        if (y == 0 || y == 1 || y == 4) { // These motors close with LOW direction command
          digitalWrite(motors[y], LOW);
          positions[y] = positions[y] - 1;
        }
        else {
          if (y == 2 || y == 3 || y == 5) {// These motors close with LOW directino command
            digitalWrite(motors[y], HIGH);
            positions[y] = positions[y] - 1;
          }
          else {
            digitalWrite(motors[y], HIGH);
            Serial.println("ERROR in 'driveMotor', wrong y-value selected");
          }
        }
      }
      //debugging
      //Serial.println(positions[y]);

    }
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

// Vibrates everthing
// Watch delays
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

void testAllMotors(int (*motors), boolean directions[], int (*positions)) //test code FOR MOTORS ONLY, do not use when the motors are attached to the robot
{

  int idx[6] = {0, 0, 0, 0, 0, 0};

  for (idx[0] = 0; idx[0] < 2; idx[0]++) {
    for (idx[1] = 0; idx[1] < 2; idx[1]++) {
      for (idx[2] = 0; idx[2] < 2; idx[2]++) {
        for (idx[3] = 0; idx[3] < 2; idx[3]++) {
          for (idx[4] = 0; idx[4] < 2; idx[4]++) {
            for (idx[5] = 0; idx[5] < 2; idx[5]++) {
              zeroIntArray(motors, 6);
              falseBooleanArray(directions, 6);
              setMotors(idx, motors);
              driveMotor(motors, 45, directions, positions, 6);
            }
          }
        }
      }
    }
  }
}

void setMotors(int (*index), int (*motors))
{
  for (int x = 0; x < 6; x++) {
    if (index[x] == 1)
      motors[x] = x + 3; //DIR_1 is three, DIR_5 is 7;
    else
      motors[x] = 0;
  }
}

void setOriginalPosition(int enablePin, int (*positions))
{
  char userInput;
  Serial.println("Please turn the motors to their initial positions, then enter some serial data");
  digitalWrite(enablePin, HIGH);
  while (!Serial.available()) { //Allows the motors to not step - no holding torque
  }
  for (int x = 0; x < 6; x++) {
    positions[x] = 0; //resets the position
  }
  digitalWrite(enablePin, LOW); //Turns the motors back on - draws max current so don't remain in this state for a long time
}


void openAllJoints(int (*motors), boolean directions[], int (*positions), int(*limits))
{
  for (int i = 0; i < 6; i++)
  {
    if (positions[i] < limits[i])
      directions[i] = true; // open
    else
      directions[i] = false; //close
  }

  int check = 1;
  while (check != 0)
  {
    check = 0;
    for (int i = 0; i < 6; i++)
    {
      if (i != 4) //motor 5 does not work
      {
        
        if (i == 4) continue;

        if (positions[i] < limits[i])
        {
          motors[i] = i + 3;
          check = check + 1;
        }
        else // (positionspi] >= limits[i])
          motors[i] = 0;
      }
      else // (positionspi] >= limits[i])
        motors[i] = 0;
    }
    if (check != 0)
      driveMotor(motors, 5, directions, positions, 6);
  }
}

void closeAllJoints(int (*motors), boolean directions[], int (*positions))
{
  for (int i = 0; i < 6; i++)
  {
    if (i == 4) continue;
    
    directions[i] = false;
  }

  int check = 1;
  while (check != 0)
  {
    check = 0;
    for (int i = 0; i < 6; i++)
    {
      if (positions[i] > 20) //leave a little room so that i don't over extend
      {
        motors[i] = i + 3;
        check = check + 1;
      }
      else // (positionspi] >= limits[i])
        motors[i] = 0;
    }
    if (check != 0)
      driveMotor(motors, 5, directions, positions, 6);
  }
}

void grab(int (*motors), boolean directions[], int (*positions), int (*innerLimits), int (*outerLimits))
{
  boolean objectGrabbed = false;
  boolean keepClosingProximals = true;
  boolean keepOpeningProximals = true;

  while(!objectGrabbed) // exits when the force sensor reading surpasses the threshold 
  {
    while(keepClosingProximals)
    {
      keepClosingProximals = closeProximals(motors, positions, directions, innerLimits); //updates based on motor position
      if (keepClosingProximals)
        keepClosingProximals = closeProximalsUpdate(force, objectGrabbed); // updates based on pressure sensing
    }

    while(keepOpeningProximals)
    {
      keepOpeningProximals = openProximals(motors, positions, directions, outerLimits);
    }

    closeDistals();
  }
}


boolean closeProximals(int (*motors), int (*positions), boolean directions[], int (*innerLimits))
{
  int numMotorsMoved = 0;
  boolean closeMore = false;

  for (int i = 3; i < 6; i++)
  {
    directions[i] = false; //set the direction for each of the motors to 'close' : This might be a redundant line. You've already set the direction to false before entering the function
    if (positions[i] > innerLimits[i]) // If the position has not yet reached the target position, move the motor
    {
      numMotorsMoved = numMotorsMoved + 1;
      motors[i] = i+3;
    }
  }

  if (numMotorsMoved != 0)
  {
    driveMotor(motors, 3, directions, positions, 6);
    closeMore = true;
  }

  zeroIntArray(motors, 6);
  return closeMore;
}

boolean closeProximalsUpdate(int (*force), boolean &objectGrabbed)
{
  boolean noPressure = true;
  for (int i = 0; i < 6; i++) // loop through all of the pressure sensors
  {
    if (force[i] > FORCE_THRESHOLD)
    {
      noPressure = false;
      objectGrabbed = true;
    }

  }
  return noPressure; // This value is stored as openProximal - true for yes, open. false for no, do not open
}

boolean openProximals(int (*motors), int (*positions), boolean directions[], int (*outerLimits))
{
  
  int numMotorsMoved = 0;
  boolean openMore = false;
  for (int i = 3; i < 6; i++)
  {
    directions[i] = true; //set the direction for each of the motors to 'open'
    if ( positions[i] < (int)min(outerLimits[i], positions[0])) // don't overextend beyond the position of one of the large joints (kind of a hack here, fix later if there is time)
    {
      numMotorsMoved = numMotorsMoved + 1;
      motors[i] = i+3;
    }
  }
    if (numMotorsMoved != 0)
    {
      driveMotor(motors, 3, directions, positions, 6);
      openMore = true;
    }
    zeroIntArray(motors, 6);
    return openMore;
}

void closeDistals()
{
  int numMotorsMoved = 3;
  while (numMotorsMoved != 0)
  {
    numMotorsMoved = 0;
    for (int i = 0; i < 3; i++)
    {
      if (positions[i] > innerLimits[i]) //want to close the joint
      {
        directions[i] = false;
        motors[i] = i+3;
        numMotorsMoved++;
      }
    }
    if (numMotorsMoved != 0)
      driveMotor(motors, 10, directions, positions, 6);
    zeroIntArray(motors,6);
  }
}

void curlGrab(int(*motors), boolean directions[], int(*positions), int (*limits), int (*force))
{

  for (int i = 0; i < 6; i++)
  {
    directions[i] = false;
  }

  int check = 1;
  while (check != 0)
  {
    check = 0;
    for (int i = 0; i < 6; i++)
    {
      if (positions[i] > limits[i])
      {
        motors[i] = i + 3;
        check = check + 1;
      }
      else // (positionspi] >= limits[i])
        motors[i] = 0;
      checkPressure(force);
      if (i < 3)
      {
        if (force[i] < FORCE_THRESHOLD)
        {
          //check positions
        }
        else {
          switch(i){
            case 0:{
              if (motors[0] != 0)
                check = check -1;
              if (motors[5] != 0)
                check = check -1;
              motors[0] = 0;
              motors[5] = 0;
            }
            case 1:{
              if (motors[1] != 0)
                check = check -1;
              if (motors[3] != 0)
                check = check -1;
              motors[1] = 0;
              motors[3] = 0;
            }
            case 2:{
              if (motors[2] != 0)
                check = check -1;
              if (motors[4] != 0)
                check = check -1;
              motors[2] = 0;
              motors[4] = 0;
            }
            
          }
        }
      }
    }

    if (check != 0)
      driveMotor(motors, 5, directions, positions, 6);
  }

}

