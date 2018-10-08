#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <String.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
uint8_t leftSpeed = 30;
uint8_t rightSpeed = 30;
const int IR_SENSOR1 = A0; //sensor to left of tape
const int IR_SENSOR2 = A1; //sensor to right of tape
bool measuring = true;
float diff = 0.0;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  AFMS.begin();  // create with the default frequency 1.6KHz
  Serial.print("Starting");
}

void loop() {
  // Run Calibration routine only once.
  if (measuring) {
    motorLeft->run(FORWARD);
    motorRight->run(BACKWARD);

    // Define constants that should probably just be global variables
    const int steps = 100;
    const int delayTime = 20;
    motorLeft->setSpeed(30);
    motorRight->setSpeed(30);

    // Take measurements
    for (int i = 0; i < steps; i++) {
      int sensor1 = analogRead(IR_SENSOR1);
      int sensor2 = analogRead(IR_SENSOR2);
      Serial.print("sensor1 reading is: ");
      Serial.println(sensor1);
      Serial.print("sensor2 reading is: ");
      Serial.println(sensor2);
      diff += sensor1 - sensor2;
      delay(delayTime);
    }
    // Find Average
    diff /= steps;

    // Stop Motors
    motorLeft->setSpeed(0);
    motorRight->setSpeed(0);
    
    //"Turn off" loop
    measuring = false;
    Serial.print("Average Difference is: ");
    Serial.println(diff);
  }
  
}
