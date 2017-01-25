//Define constants for Enable, Step, Direction, MS1 and MS2 pins

//Available digital pins are: 0,1,2,13
#define EN 2
#define MS1 3   //Green
#define MS2 4   //Yellow
#define stp 5   //Orange
#define DIR_1 6
#define DIR_2 7
#define DIR_3 8
#define DIR_4 9
#define DIR_5 10
#define DIR_6 11


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
  // put your setup code here, to run once:
  pinMode(EN,OUTPUT);
  pinMode(stp,OUTPUT);
  pinMode(DIR_1,OUTPUT);
  pinMode(DIR_2,OUTPUT);
  pinMode(DIR_3,OUTPUT);
  pinMode(DIR_4,OUTPUT);
  pinMode(DIR_5,OUTPUT);
  pinMode(DIR_6,OUTPUT);
  pinMode(MS1,OUTPUT);
  pinMode(MS2,OUTPUT);

  setPins();
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
  int degrees = 90;
  double degreesPerStep = 0;
  boolean rotationDirection = true;
  boolean dirSwitch = true;

  selectStepSize(1); //choose half stepping
  for(int motorNum = DIR_1; motorNum <= DIR_6; motorNum++){
    driveMotor(motorNum, degrees,rotationDirection, degreesPerStep, dirSwitch); 
  }
  holdPosition();
}







