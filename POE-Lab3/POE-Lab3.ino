/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
const int IR_SENSOR = A0;
const int MAX_REFLECT = 930;
const int MIN_REFLECT = 630;
const int MAX_SPEED = 50;
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
  uint8_t i;

  if (digitalRead(BUTTON)){
    running = !running;
    delay(50);
  }
  if (running) {
    motorLeft->run(FORWARD);
    motorRight->run(BACKWARD);
    int leftSpeed = round(map(analogRead(IR_SENSOR), MIN_REFLECT, MAX_REFLECT, 0, MAX_SPEED));
    int rightSpeed = MAX_SPEED - leftSpeed;
    motorLeft->setSpeed(leftSpeed);
    motorRight->setSpeed(rightSpeed);
    Serial.print("LEFT SPEED: ");
    Serial.println(leftSpeed);
    Serial.print("RIGHT SPEED: ");
    Serial.println(rightSpeed);
    delay(100);
  }
  
//  
//  
//  
//  for (i=0; i<255; i++) {
//    myMotor->setSpeed(i);
//    myMotor2->setSpeed(i);  
//    delay(10);
//  }
//  for (i=255; i!=0; i--) {
//    myMotor->setSpeed(i); 
//    myMotor2->setSpeed(i);   
//    delay(10);
//  }
//  
//  Serial.print("tock");
//
//  
//  myMotor->run(BACKWARD);
//  myMotor2->run(BACKWARD);
//  for (i=0; i<255; i++) {
//    myMotor->setSpeed(i);  
//    myMotor2->setSpeed(i);  
//    delay(10);
//  }
//  for (i=255; i!=0; i--) {
//    myMotor->setSpeed(i);  
//    myMotor2->setSpeed(i);  
//    delay(10);
//  }
//
//  Serial.print("tech");
//  myMotor->run(RELEASE);
//  myMotor2->run(RELEASE);  
//  delay(1000);
}
