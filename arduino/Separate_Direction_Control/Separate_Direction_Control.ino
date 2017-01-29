#define STP 2 // All motor steps are tied together
#define DIR_1 3 // Each motor has its own direction pin
#define DIR_2 4
#define DIR_3 5
#define DIR_4 6
#define DIR_5 7
#define DIR_6 8

//Declare variables for functions
char userInput;
int desiredDegrees;
int x;
int y;
int state;
int count1;
int count2;
int count3;
int count4;

void setup() {
  pinMode(STP,OUTPUT);
  pinMode(DIR_1,OUTPUT);
  pinMode(DIR_2,OUTPUT);
  pinMode(DIR_3,OUTPUT);
  pinMode(DIR_4,OUTPUT);
  pinMode(DIR_5,OUTPUT);
  pinMode(DIR_6,OUTPUT);

  resetEDPins();
  Serial.begin(9600); //Open Serial connection for debugging
  //Print function list for user selection
  Serial.println("Ready to Test?");
  
  
  Serial.println();
  count1 = 0;
  count2 = 0;
  count3 = 0;
  count4 = 0;
}

//Main loop
void loop() {
      
      userInput = Serial.read(); //Read user input and trigger appropriate function 
      resetEDPins();
      for(x= 0; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
      {
        digitalWrite(STP,HIGH); //Trigger one step forward
        delay(1);
        digitalWrite(STP,LOW); //Pull step pin low so it can be triggered again
        delay(1);
      }

      holdPosition(10000);  
}






//void OriginalPosition()
//{
//  Serial.println("Move back to origianl position");
//  digitalWrite(dir, HIGH); //Pull direction pin high to move "forward"
//  
//  digitalWrite(MS1, LOW); 
//  digitalWrite(MS2, LOW);
//  for(x= 1; x< count1; x++)  //Loop the forward stepping enough times for motion to be visible
//  {
//    digitalWrite(stp,HIGH); //Trigger one step forward
//    delay(1);
//    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
//    delay(1);
//  }
//
//  digitalWrite(MS1, HIGH); 
//  digitalWrite(MS2, LOW);
//  for(x= 1; x< count2; x++)  //Loop the forward stepping enough times for motion to be visible
//  {
//    digitalWrite(stp,HIGH); //Trigger one step forward
//    delay(1);
//    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
//    delay(1);
//  }
//
//  digitalWrite(MS1, LOW); 
//  digitalWrite(MS2, HIGH);
//  for(x= 1; x< count3; x++)  //Loop the forward stepping enough times for motion to be visible
//  {
//    digitalWrite(stp,HIGH); //Trigger one step forward
//    delay(1);
//    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
//    delay(1);
//  }
//
//  digitalWrite(MS1, HIGH); 
//  digitalWrite(MS2, HIGH);
//  for(x= 1; x< count4; x++)  //Loop the forward stepping enough times for motion to be visible
//  {
//    digitalWrite(stp,HIGH); //Trigger one step forward
//    delay(1);
//    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
//    delay(1);
//  }
//  Serial.println("Enter new option");
//  Serial.println();
//}



//Reset Easy Driver pins to default states


