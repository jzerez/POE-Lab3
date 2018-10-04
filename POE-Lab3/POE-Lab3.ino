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
uint8_t leftSpeed = 0;
uint8_t rightSpeed = 0;
const int IR_SENSOR = A0;
const int MAX_REFLECT = 990;
const int MIN_REFLECT = 800;
const uint8_t MAX_SPEED = 70;
bool running = false;
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
        // Max speed = input_commandand;
        Serial.print("* Message: ");
        Serial.println(input_command);
        break;
      case '+':
        // P const = input_command;
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
    leftSpeed = round(map(analogRead(IR_SENSOR), MIN_REFLECT, MAX_REFLECT, 0, MAX_SPEED)) & 0x00FF;
    delay(10);
    rightSpeed = (MAX_SPEED - leftSpeed) & 0x00FF;
    leftSpeed = limitSpeed(leftSpeed, 0, MAX_SPEED);
    rightSpeed = limitSpeed(rightSpeed, 0, MAX_SPEED);
    Serial.print("Sensor: ");
    Serial.println(analogRead(IR_SENSOR));
    Serial.print("LEFT SPEED: ");
    Serial.println(leftSpeed);
    Serial.print("RIGHT SPEED: ");
    Serial.println(rightSpeed);
    delay(500);
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
