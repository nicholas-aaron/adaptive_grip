#define STP 2 // All motor steps are tied together
#define DIR_1 3 // Each motor has its own direction pin
#define DIR_2 4
#define DIR_3 5
#define DIR_4 6
#define DIR_5 7
#define DIR_6 8

//Declare variables for functions
char userInput;

void setup() {
  pinMode(STP,OUTPUT);
  pinMode(DIR_1,OUTPUT);
  pinMode(DIR_2,OUTPUT);
  pinMode(DIR_3,OUTPUT);
  pinMode(DIR_4,OUTPUT);
  pinMode(DIR_5,OUTPUT);
  pinMode(DIR_6,OUTPUT);

  setPins();
  Serial.begin(9600); //Open Serial connection for debugging
  //Print function list for user selection
  Serial.println("Test Begins");
  Serial.println();
}

//Main loop
void loop() {
  int degrees = 180;
  boolean rotationDirection = false;
  
  for(int motorNum = DIR_1; motorNum <= DIR_6; motorNum++){
    driveMotor(motorNum, degrees,rotationDirection); 
  }
  holdPosition(5000);
}







