//Define constants for Enable, Step, Direction, MS1 and MS2 pins

//Available digital pins are: 0,1,2,13
#define EN_1 7
#define EN_2 8
#define EN_3 9
#define EN_4 10
#define EN_5 11
#define EN_6 12
#define MS1 3   //Green
#define MS2 4   //Yellow
#define stp 5   //Orange
#define dir 6   //Purple/White


//Declare variables for functions
char user_input;
int x;
int y;
int state;
int count1;
int count2;
int count3;
int count4;

void setup() {
  // put your setup code here, to run once:
  pinMode(EN_1,OUTPUT);
  pinMode(EN_2,OUTPUT);
  pinMode(EN_3,OUTPUT);
  pinMode(EN_4,OUTPUT);
  pinMode(EN_5,OUTPUT);
  pinMode(EN_6,OUTPUT);
  pinMode(stp,OUTPUT);
  pinMode(dir,OUTPUT);
  pinMode(MS1,OUTPUT);
  pinMode(MS2,OUTPUT);

  resetEDPins();
  Serial.begin(9600); //Open Serial connection for debugging
  Serial.println("Begin motor control");
  Serial.println();
  //Print function list for user selection
  Serial.println("Enter number for control option:");
  Serial.println("1. Turn at Full Step.");
  Serial.println("2. Turn at Half Step.");
  Serial.println("3. Turn at Quarter Step .");
  Serial.println("4. Turn at Eighth Step.");
  Serial.println("5. Return to Original Postion.");
  Serial.println();
  count1 = 0;
  count2 = 0;
  count3 = 0;
  count4 = 0;
}

//Main loop
void loop() {
  
    while(Serial.available()){
      user_input = Serial.read(); //Read user input and trigger appropriate function
      //digitalWrite(EN, LOW); //Pull enable pin low to allow motor control
      for (int motorNumber = EN_1; motorNumber <=EN_6; motorNumber++)
    {
      String motorNum = String(motorNumber);
      Serial.println("Moving motor number " + motorNum);
      controlMotor(motorNumber);
      controlStep();
      resetEDPins();
      holdPosition();
    }
  }
}

void controlMotor(int motorID)
{
  switch(motorID)
  {
    //Serial.println("Moving Motor Number" + motorID);
    case EN_1: //(7)NEMA23
    {
      digitalWrite(EN_1,LOW);
      break;
    }
    case EN_2: //(8)NEMA23
    {
      digitalWrite(EN_2,LOW);
      break;
    }
    case EN_3: //(9)NEMA23
    {
      digitalWrite(EN_3,LOW);
      break;
    }
    case EN_4: //(10)NEMA17
    {
      digitalWrite(EN_4,LOW);
      break;
    }
    case EN_5: //(11)NEMA17
    {
      digitalWrite(EN_5,LOW);
      break;
    }
    case EN_6: //(12)NEMA17
    {
      digitalWrite(EN_6,LOW);
      break;
    }
  }
  
}

void controlStep()
{
  if (user_input =='1')
      {
         FullStep();
      }
      else if(user_input =='2')
      {
        HalfStep();
      }
      else if(user_input =='3')
      {
        QuarterStep();
      }
      else if(user_input =='4')
      {
        EighthStep();
      }
      else if(user_input =='5')
      {
        OriginalPosition();
        count1 = 0;
        count2 = 0;
        count3 = 0;
        count4 = 0;
      }
      else
      {
        Serial.println("Invalid option entered.");
      }
}
void holdPosition()
{
  // Keep enable low to hold position
  //digitalWrite(EN,LOW);  
}

void FullStep()
{
  Serial.println("Moving forward at full step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, LOW); 
  digitalWrite(MS2, LOW);
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
    //Serial.println(count);
  }
  count1+=999;
  Serial.println("\nEnter new option");
  Serial.println();
}

void HalfStep()
{
  Serial.print("Moving forward at half step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, HIGH); 
  digitalWrite(MS2, LOW);
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
    count2++;
    
  }
  
  Serial.println("Enter new option");
  Serial.println();
}


void QuarterStep()
{
  Serial.println("Moving forward at quarter step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, LOW); 
  digitalWrite(MS2, HIGH);
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
    count3++;
  }
  Serial.println("Enter new option");
  Serial.println();
}

void EighthStep()
{
  Serial.println("Moving forward at eigth step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, HIGH); 
  digitalWrite(MS2, HIGH);
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
    count4++;
  }
  Serial.println("Enter new option");
  Serial.println();
}

void OriginalPosition()
{
  Serial.println("Move back to origianl position");
  digitalWrite(dir, HIGH); //Pull direction pin high to move "forward"
  
  digitalWrite(MS1, LOW); 
  digitalWrite(MS2, LOW);
  for(x= 1; x< count1; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }

  digitalWrite(MS1, HIGH); 
  digitalWrite(MS2, LOW);
  for(x= 1; x< count2; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }

  digitalWrite(MS1, LOW); 
  digitalWrite(MS2, HIGH);
  for(x= 1; x< count3; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }

  digitalWrite(MS1, HIGH); 
  digitalWrite(MS2, HIGH);
  for(x= 1; x< count4; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  Serial.println("Enter new option");
  Serial.println();
}



//Reset Easy Driver pins to default states
void resetEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(EN_1, HIGH);
  digitalWrite(EN_2, HIGH);
  digitalWrite(EN_3, HIGH);
  digitalWrite(EN_4, HIGH);
  digitalWrite(EN_5, HIGH);
  digitalWrite(EN_6, HIGH);
}

