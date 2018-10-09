#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <String.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
int leftSpeed = 0;
int rightSpeed = 0;
const int IR_SENSOR1 = A1; //sensor to left of tape
const int MAX_REFLECT = 670; //will need to double check these values
const int MIN_REFLECT = 100;
const int BUFFER = 75;
int MAX_SPEED = 40;
bool running = true;
const int BUTTON = 8;

// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
}

void loop() {
  if (Serial.available() > 0){
    String new_char = Serial.readString();
    int input_command = new_char.substring(1).toInt();
    switch(new_char[0]){
      case '*':
        //Updating MAX_SPEED
        MAX_SPEED = input_command;
        Serial.print("* Message: ");
        Serial.println(input_command);
        break;
      case ',':
        // D const = input_command;
        Serial.print(", Message: ");
        Serial.println(input_command);
        break;
    }
  }

  if (digitalRead(BUTTON)){
    running = !running;
    delay(50);
  }
  if (running) {
    motorLeft->run(FORWARD);
    motorRight->run(BACKWARD);
    int sensor1 = analogRead(IR_SENSOR1);
    leftSpeed = map(sensor1, MIN_REFLECT + BUFFER, MAX_REFLECT - BUFFER, 0, MAX_SPEED);
    rightSpeed = MAX_SPEED - leftSpeed;
    leftSpeed = limitSpeed(leftSpeed, MAX_SPEED, 0);
    rightSpeed = limitSpeed(rightSpeed, MAX_SPEED, 0);
    motorLeft->setSpeed(leftSpeed & 0x00FF);
    motorRight->setSpeed(rightSpeed & 0x00FF);
  }
}

int limitSpeed(int initialSpeed, int maxSpeed, int minSpeed) {
  /*
   * This function is designed to artificially limit any inputs to the motor. If the input speed is
   * not within the specified range, the function automatically clips it such that it fits in the 
   * range. It takes the following arguments:
   *    initialSpeed:   Raw speed
   *    maxSpeed:       Upper bound for acceptable speeds
   *    minSpeed:       Lower bound for acceptable speeds
   */

  // Clip value if greater than maximum 
  if (initialSpeed > maxSpeed){
    return maxSpeed;
  }
  // Clip value if less than minimum
  if (initialSpeed < minSpeed) {
    return minSpeed;
  }
  // Return the original value if it within the range
  return initialSpeed;
}

int calibrate(int scanSpeed, int sensor, int steps, bool findAverage, bool debugMode) {
  /* 
   *  This function is designed to scan the line at the beginning of the course to generate its own
   *  threshold values for the IR sensor. It takes the following arguments:
   *    scanSpeed:    Speed (0-255) of the motor during the scan
   *    sensor:       Pin number of the sensor to be used in the scan
   *    steps:        Number of points used to calculate reflectance of the ground
   *    findAverage:  Toggle to switch between finding the average reflectance of the ground. If
   *                  true, the function will return the average reflectance. If false, the fuction
   *                  will return the minimum reflectance found over the scan interval
   *    debugMode:    Toggle for print statements
   */

  // Initialize Variables
  int lightValues = 0;
    
  // Move off the line
  motorLeft->run(BACKWARD);
  motorLeft->setSpeed(scanSpeed);
  delay(500);

  // Start the scan of the ground
  for (int i = 0; i < steps; i++) {
    motorLeft-> setSpeed(scanSpeed);
    
    if (findAverage) {
      lightValues += (analogRead(sensor) / steps);
    } else if (analogRead(sensor) > lightValues) {
      lightValues = analogRead(sensor);
    }
    delay(20);
  }
  // Return to rough start position
  motorLeft->run(FORWARD);
  motorLeft->setSpeed(scanSpeed);
  delay(500 + 20 * steps);
  motorLeft->setSpeed(0);
  
  if (debugMode) {Serial.print("Measured Light Value is: "); Serial.println(lightValues);}

  return lightValues;
}
