/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
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
uint8_t leftSpeed = 30;
uint8_t rightSpeed = 30;
const int IR_SENSOR1 = A0; //sensor to left of tape
const int IR_SENSOR2 = A1; //sensor to right of tape
const int MAX_REFLECT = 990; //will need to double check these values
const int MIN_REFLECT = 800;
uint8_t maxSpeed = 50;
uint8_t Kp = 0.1;
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
        Kp = input_command;
        Serial.print("+ Message: ");
        Serial.println(input_command);
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
    int sensor1 = analogRead(IR_SENSOR1);
    int sensor2 = analogRead(IR_SENSOR2);
    int diff = sensor1 - sensor2;
    uint8_t deltaSpeed = (diff * Kp) & 0x00FF;;
    //if diff > 0 (sensor1 > sensor2), then the left sensor is touching the black tape, so the car should turn left.
    //if diff < 0 (sensor1 < sensor2), then the right sensor is touching the black tape, so the car should turn right.
    //new speed of left wheel should always be subtracted by deltaSpeed
    leftSpeed = leftSpeed - deltaSpeed;
    //new speed of right wheel should always by added with deltaSpeed
    rightSpeed = rightSpeed + deltaSpeed;
//    leftSpeed = round(map(analogRead(IR_SENSOR), MIN_REFLECT, MAX_REFLECT, 0, maxSpeed)) & 0x00FF;
    delay(10);
//    rightSpeed = (maxSpeed - leftSpeed) & 0x00FF;
    leftSpeed = limitSpeed(leftSpeed, maxSpeed, 0);
    rightSpeed = limitSpeed(rightSpeed, maxSpeed, 0);
    Serial.print("Diff sensor value and deltaspeed: ");
    Serial.print(diff);
    Serial.print(", ");
    Serial.println(deltaSpeed);
    Serial.print("LEFT SPEED: ");
    Serial.println(leftSpeed);
    Serial.print("RIGHT SPEED: ");
    Serial.println(rightSpeed);
    delay(100);
    motorLeft-> setSpeed(leftSpeed);
    motorRight-> setSpeed(rightSpeed);
  }
}

int limitSpeed(int initialSpeed, int maxSpeed, int minSpeed) {
  if (initialSpeed > maxSpeed){
    return maxSpeed;
  }
  if (initialSpeed < minSpeed) {
    return minSpeed;
  }
  return initialSpeed;
}
