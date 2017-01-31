void driveMotor(int mtrID, int degRot, boolean rotDir )
{
  double degPerStp = (double)(1.8/8); // This is used for 1/8th stepping mode
  int numSteps = (int)(degRot/degPerStp); 
  boolean dirSwitch = false;
  for(int x=0; x<numSteps; x++)  //Loop the forward stepping enough times for motion to be visible
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
    digitalWrite(STP, HIGH);
    delay(2);
    digitalWrite(STP,LOW);
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

void holdPosition(int mSecs)
{
  for (int x = 0; x < (int)(mSecs/10); x ++)
  {
    for (int pinOut = DIR_1; pinOut <= DIR_6; pinOut++)
    {
      digitalWrite(pinOut,HIGH);
    }

    digitalWrite(STP,HIGH);
    delay(1);
    digitalWrite(STP,LOW);
    delay(4);

    for (int pinOut = DIR_1; pinOut <= DIR_6; pinOut++)
    {
      digitalWrite(pinOut,LOW);
    }

    digitalWrite(STP,HIGH);
    delay(1);
    digitalWrite(STP,LOW);
    delay(4);
  }
}
