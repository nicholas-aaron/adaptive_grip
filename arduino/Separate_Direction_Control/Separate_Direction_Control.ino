
#define STP 2 // All motor steps are tied together
#define DIR_1 3 // Each motor has its own direction pin
#define DIR_2 4
#define DIR_3 5
#define DIR_4 6
#define DIR_5 7
#define DIR_6 8
#define ARRAY_SIZE 6

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
}

void loop() { // ------------------------------------------------------- MAIN LOOP --------------------------------------------------//
  int degrees = 180;
  int motors[ARRAY_SIZE] = {1,2,3,4,5,6};
  boolean directions[ARRAY_SIZE] = {false,false,false,true,true,true};
  testAllMotors(motors, directions);//This takes about 30 seconds
  holdPosition(10000);

}







