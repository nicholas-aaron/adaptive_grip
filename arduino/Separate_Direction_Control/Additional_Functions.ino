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

// -----------------------------------Reset Array ----------------------------//
void zeroIntArray(int (*motors), int arrayLength)
{
  for (int x = 0; x < arrayLength; x ++){
    motors[x] = 0;
  } 
}

//------------------------------------Reset Boolean Array --------------------//
void falseBooleanArray(boolean directions[], int arrayLength)
{
  for (int x = 0; x < arrayLength; x++){
    directions[x] = false;
  }
}
