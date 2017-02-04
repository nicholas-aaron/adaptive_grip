
#define STP 2 // All motor steps are tied together
#define DIR_1 3 // Each motor has its own direction pin
#define DIR_2 4
#define DIR_3 5
#define DIR_4 6
#define DIR_5 7
#define DIR_6 8
#define EN 10
#define ARRAY_SIZE 6

//Declare variables for functions
char userInput;
boolean firstRun = true;
//Arrays
int motors[ARRAY_SIZE] = {1,2,3,4,5,6}; //rotate motor or vibrate in place
boolean directions[ARRAY_SIZE] = {false,false,false,true,true,true};//clockwise or counter clockwise
int positions[ARRAY_SIZE] = {0,0,0,0,0,0};//with respect to 'home' position

void setup() { //--------------------------------------------------- SETUP -----------------------------------------------------------------//
  pinMode(STP,OUTPUT);
  pinMode(DIR_1,OUTPUT);
  pinMode(DIR_2,OUTPUT);
  pinMode(DIR_3,OUTPUT);
  pinMode(DIR_4,OUTPUT);
  pinMode(DIR_5,OUTPUT);
  pinMode(DIR_6,OUTPUT);
  pinMode(EN,OUTPUT);
  setPins();

  Serial.begin(9600); 
  displayMenu();
}

void loop() { // ------------------------------------------------------- MAIN LOOP --------------------------------------------------//
  if (firstRun){
    setOriginalPosition(EN, positions);
    firstRun = false;  
  }
  
  while (Serial.available()) {
    userInput = Serial.read();
    if (userInput =='0'){
      printPositions(positions);
      displayMenu();
    }
    else{
      zeroIntArray(motors,ARRAY_SIZE);
      userMotorChoice(userInput, motors, directions); // Sets new motors array and directions array
      driveMotor(motors, 5, directions, positions, ARRAY_SIZE);
      //debugging
//      for(int i = 0; i < 6; i++){
//        Serial.print("Position of motor ");
//        Serial.print(i+1);
//        Serial.print(" is: ");
//        Serial.println(positions[i]);
//      }
      displayMenu();
    } 
  }
}

void displayMenu(){
  Serial.println("Enter number for control option:");
  Serial.println("0. Print Positions");
  Serial.println("1. Motor 1, Clockwise");
  Serial.println("2. Motor 2, Clockwise");
  Serial.println("3. Motor 3, Clockwise");
  Serial.println("4. Motor 4, Clockwise");
  Serial.println("4. Motor 4, Clockwise");
  Serial.println("5. Motor 5, Clockwise");
  Serial.println("6. Motor 6, Clockwise");
  Serial.println("7. Motor 1, Counter-Clockwise");
  Serial.println("8. Motor 2, Counter-Clockwise");
  Serial.println("a. Motor 3, Counter-Clockwise");
  Serial.println("b. Motor 4, Counter-Clockwise");
  Serial.println("c. Motor 5, Counter-Clockwise");
  Serial.println("d. Motor 6, Counter-Clockwise");
  Serial.println();
}







