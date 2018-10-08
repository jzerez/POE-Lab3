#include <TFT.h>

/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->  http://www.adafruit.com/products/1438
*/

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
int leftSpeed = 30;
int rightSpeed = 30;
const int IR_SENSOR1 = A0; //sensor to left of tape
const int IR_SENSOR2 = A1; //sensor to right of tape
const int MAX_REFLECT = 990; //will need to double check these values
const int MIN_REFLECT = 800;
const int CALIBRATION = 0;
const int IR_TAPE = 970;
uint8_t maxSpeed = 30;
float Kp = 0.8;
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
        //Updating maxSpeed
        maxSpeed = input_command;
        Serial.print("* Message: ");
        Serial.println(input_command);
        break;
      case '+':
        //Updating Kp 
        Kp = input_command/10.0;
        Serial.print("+ Message: ");
        Serial.println(Kp);
        break;
      case ',':
        // D const = input_command;
        Serial.print(", Message: ");
        Serial.println(input_command);
        break;
    }
  }
  uint8_t i;

  if (digitalRead(BUTTON)){
    running = !running;
    delay(50);
  }
  if (running) {
    motorLeft->run(FORWARD);
    motorRight->run(BACKWARD);
    int sensor1 = analogRead(IR_SENSOR1) + CALIBRATION;
    int sensor2 = analogRead(IR_SENSOR2);
    if(sensor1 > 970){
      //left wheel is on the tape
    }
    else if(sensor2 > 970){
      //right wheel is on the tape 
    }
    float diff = sensor1 - sensor2;
    int deltaSpeed = round(diff * Kp);
    //if diff > 0 (sensor1 > sensor2), then the left sensor is touching the black tape, so the car should turn left.
    //if diff < 0 (sensor1 < sensor2), then the right sensor is touching the black tape, so the car should turn right.
    //new speed of left wheel should always be subtracted by deltaSpeed
    leftSpeed = (30 - deltaSpeed);
    //new speed of right wheel should always by added with deltaSpeed
    rightSpeed = (30 + deltaSpeed);
//    leftSpeed = round(map(analogRead(IR_SENSOR), MIN_REFLECT, MAX_REFLECT, 0, maxSpeed)) & 0x00FF;
//    delay(500);
//    rightSpeed = (maxSpeed - leftSpeed) & 0x00FF;
    leftSpeed = limitSpeed(leftSpeed, maxSpeed, 0);
    rightSpeed = limitSpeed(rightSpeed, maxSpeed, 0);
    Serial.print("Sensor value 1: ");
    Serial.println(sensor1);
    Serial.print("Sensor value 2: ");
    Serial.println(sensor2);
    Serial.print("Diff sensor value and deltaspeed: ");
    Serial.print(diff);
    Serial.print(", ");
    Serial.println(deltaSpeed);
    Serial.print("LEFT SPEED: ");
    Serial.println(leftSpeed);
    Serial.print("RIGHT SPEED: ");
    Serial.println(rightSpeed);
    delay(500);
    motorLeft-> setSpeed(leftSpeed & 0x00FF);
    motorRight-> setSpeed(rightSpeed & 0x00FF);
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
