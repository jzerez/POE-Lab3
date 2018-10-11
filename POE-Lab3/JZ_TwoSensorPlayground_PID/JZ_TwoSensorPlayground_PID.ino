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
const int IR_SENSOR1 = A0;
const int IR_SENSOR2 = A1;//sensor to left of tape
const int MAX_REFLECT = 420; //will need to double check these values
const int MIN_REFLECT = 40;
const int BUFFER = 40;
const int MAX_REFLECT_FAR = 830;
const int MIN_REFLECT_FAR = 220;
int MAX_SPEED = 40;
float SPEED_INCREASE_FACTOR = 1.15;
float FIT_POWER = 0.8;
bool running = true;
const int BUTTON = 8;
bool debug = false;

float FIT_CONST = MAX_SPEED/pow((MAX_REFLECT - MIN_REFLECT), FIT_POWER);

// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println();
  Serial.println(FIT_CONST);
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
}

void loop() {
  if (Serial.available() > 0) {
    String new_char = Serial.readString();
    int input_command = new_char.substring(1).toFloat();
    switch (new_char[0]) {
      case '*':
        //Updating MAX_SPEED
        MAX_SPEED = int(input_command);
        FIT_CONST = MAX_SPEED/pow((MAX_REFLECT - MIN_REFLECT), FIT_POWER);
        Serial.print("* Message: ");
        Serial.println(input_command);
        break;
      case ',':
        // Updating SPEED_INCREASE_FACTOR
        SPEED_INCREASE_FACTOR = input_command;
        Serial.print(", Message: ");
        Serial.println(input_command);
        break;
      case 'd':
        debug = !debug;
        break;
      case 'f':
        FIT_POWER = input_command;
        FIT_CONST = MAX_SPEED/pow((MAX_REFLECT - MIN_REFLECT), FIT_POWER);
        break;
    }
  }

  if (digitalRead(BUTTON)) {
    running = !running;
    delay(50);
  }
  if (running) {
    motorLeft->run(FORWARD);
    motorRight->run(BACKWARD);
    int sensor1 = analogRead(IR_SENSOR1);
    int sensor2 = analogRead(IR_SENSOR2);
    float scalingFactor = (1 + float(sensor2 - MIN_REFLECT_FAR) / float(MAX_REFLECT_FAR - MIN_REFLECT_FAR))*SPEED_INCREASE_FACTOR;
    rightSpeed = calcPower(sensor1, scalingFactor);
    leftSpeed = calcPower(MAX_REFLECT-sensor1+MIN_REFLECT, scalingFactor);

    leftSpeed = limitSpeed(leftSpeed, scalingFactor * MAX_SPEED, 0);
    rightSpeed = limitSpeed(rightSpeed, scalingFactor * MAX_SPEED, 0);
    motorLeft->setSpeed(leftSpeed & 0x00FF);
    motorRight->setSpeed(rightSpeed & 0x00FF);

    if (debug) {
      Serial.print("RIGHT SPEED: ");
      Serial.println(rightSpeed);
      Serial.print("LEFT SPEED: ");
      Serial.println(leftSpeed);
      Serial.print("SPEED FACTOR: ");
      Serial.println(scalingFactor);
      Serial.println(FIT_POWER);
      delay(500);
    }
  }
}

int limitSpeed(int initialSpeed, int maxSpeed, int minSpeed) {
  /*
     This function is designed to artificially limit any inputs to the motor. If the input speed is
     not within the specified range, the function automatically clips it such that it fits in the
     range. It takes the following arguments:
        initialSpeed:   Raw speed
        maxSpeed:       Upper bound for acceptable speeds
        minSpeed:       Lower bound for acceptable speeds
  */

  // Clip value if greater than maximum
  if (initialSpeed > maxSpeed) {
    return maxSpeed;
  }
  // Clip value if less than minimum
  if (initialSpeed < minSpeed) {
    return minSpeed;
  }
  // Return the original value if it within the range
  return initialSpeed;
}

int calcPower(int reading, float scalingFactor) {
  return abs(scalingFactor * FIT_CONST * pow((reading - MIN_REFLECT), FIT_POWER));
}
