/* 
This script is a Kp controller for our two sensor setup. It contains a serial interface,
start/stop button, and a limitSpeed helper function.   
*/

#include <TFT.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <String.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();  

// Select which 'port' M1, M2, M3 or M4. We used M1 and M2.
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
const int IR_SENSOR1 = A0; // Sensor to left of tape
const int IR_SENSOR2 = A1; // Sensor to right of tape
int maxSpeed = 20; // Maximum speed set to 20 after testing
int leftSpeed = maxSpeed; // Initialize left motor speed to maxSpeed
int rightSpeed = maxSpeed; // Initialize right motor speed to maxSpeed
float Kp = 1; // Kp constant, set to 1 after extensive testing
bool running = true; // Button state that determines whether to power to motors
const int BUTTON = 8; // Button digital pin at pin 8

void setup() {
  Serial.begin(9600); // Set up Serial library at 9600 bps
  AFMS.begin();  // Create with the default frequency 1.6KHz
}

void loop() {
  // If message sent to serial, then corresponding variable is updated
  // Variable to change is determined by the first character in message 
  if (Serial.available() > 0){
    // Read in serial message as String
    String new_char = Serial.readString();
    // Extract the new number, part of string after first character
    float input_command = new_char.substring(1).toFloat();
    // Determine which variable needs to be updated
    switch(new_char[0]){
      case '*':
        //Updating maxSpeed
        maxSpeed = input_command;
        Serial.print("* Message: ");
        Serial.println(input_command);
        break;
      case '+':
        //Updating Kp 
        Kp = input_command;
        Serial.print("+ Message: ");
        Serial.println(Kp);
        break;
    }
  }
  // If button pressed, then change state of running variable. 
  if (digitalRead(BUTTON)){
    running = !running;
    delay(50); // Delay used to prevent debouncing
  }

  // If running is true, then read in sensor values and calculate motor speeds
  // using the difference in sensor values multiplied by the Kp constant
  if (running) {
    motorLeft->run(FORWARD); // Moving left motor forward
    motorRight->run(BACKWARD); // Since right motor mounted backwards, BACKWARD means moving forward
    int sensor1 = analogRead(IR_SENSOR1); // Read sensor1(left) value
    int sensor2 = analogRead(IR_SENSOR2); // Read sensor2(right) value
    float diff = sensor1 - sensor2; // Calculate difference in sensor values
    int deltaSpeed = round(diff * Kp); // Multiply difference by Kp constant to get difference in speed of motors
    // deltaSpeed refers to left motor speed subtracted by right motor speed
    // if diff > 0 (sensor1 > sensor2), then the left sensor is touching the black tape, so the car should turn left
    // if diff < 0 (sensor1 < sensor2), then the right sensor is touching the black tape, so the car should turn right
    // To find updated leftSpeed, subtract maxSpeed by deltaSpeed
    leftSpeed = (maxSpeed - deltaSpeed);
    // To find updated rightSpeed, add maxSpeed to deltaSpeed
    rightSpeed = (maxSpeed + deltaSpeed);
    // Make sure leftSpeed and rightSpeed are between 0 and maxSpeed by calling limitSpeed
    leftSpeed = limitSpeed(leftSpeed, maxSpeed, 0);
    rightSpeed = limitSpeed(rightSpeed, maxSpeed, 0);
    // Printing data to Serial Monitor, which is saved in txt file
    // Format: sensor1 value,sensor2 value,left speed,right speed
    Serial.print(sensor1);
    Serial.print(",");
    Serial.print(sensor2);
    Serial.print(",");
    Serial.print(leftSpeed);
    Serial.print(",");
    Serial.println(rightSpeed);
    // Update left and right motor speeds
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
  // Return the original value if it is within the range
  return initialSpeed;
}
