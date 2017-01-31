
#define STP 2 // All motor steps are tied together
#define DIR_1 3 // Each motor has its own direction pin
#define DIR_2 4
#define DIR_3 5
#define DIR_4 6
#define DIR_5 7
#define DIR_6 8

//Declare variables for functions
char userInput;

void setup() { //--------------------------------------------------- SETUP -----------------------------------------------------------------//
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

void loop() { // ------------------------------------------------------- MAIN LOOP --------------------------------------------------//
  int degrees = 180;
  int motors[6];
  boolean directions[6];

  motors[2] = DIR_3;
  zeroIntArray( motors, 6 );
  falseBooleanArray(directions, 6));
  motors[0] = DIR_1;
  motors[1] = DIR_2;
  directions[0] = true;
  directions[1] = true;
  //dirs[2] false;
  driveMotor(motorsPtr, 180, directionsPtr);
  holdPosition(1000);
}







