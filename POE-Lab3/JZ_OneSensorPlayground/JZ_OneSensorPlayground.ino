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
int MIDDLE = 500;
int THRESH = 10;
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
        //Updating MIDDLE
        MIDDLE = input_command;
        Serial.print("* Message: ");
        Serial.println(input_command);
        break;
      case ',':
        //Updating THRESH
        THRESH = input_command;
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
    
    if (sensor1 > MIDDLE + THRESH) {
      motorRight->setSpeed(30);
      motorLeft->setSpeed(0);
      Serial.print(sensor1);
      Serial.print(",");
      Serial.print(0);
      Serial.print(",");
      Serial.println(30);
    } else if (sensor1 < MIDDLE - THRESH) {
      motorLeft->setSpeed(30);
      motorRight->setSpeed(0);
      Serial.print(sensor1);
      Serial.print(",");
      Serial.print(30);
      Serial.print(",");
      Serial.println(0);
    } else {
      motorLeft->setSpeed(15);
      motorRight->setSpeed(15);
      Serial.print(sensor1);
      Serial.print(",");
      Serial.print(15);
      Serial.print(",");
      Serial.println(15);
    }
  }
}
