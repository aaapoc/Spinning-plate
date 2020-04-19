
#include <arduino.h>
#include <AccelStepper.h>

const int ledPin = 13;      // the pin that the LED is attached to
uint8_t speed = 0;
float aspeed = 500.0;
int dir = 1;

AccelStepper stepper(AccelStepper::DRIVER);



void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}

void setup() {
  // initialize the serial communication:
  Serial.begin(9600);
  // initialize the ledPin as an output:
  pinMode(ledPin, OUTPUT);
  Serial.print("speed set to ");
  printHex(speed);
  Serial.println();
  stepper.setEnablePin(7);
  stepper.setMaxSpeed(1000);
  stepper.setSpeed(500);
  stepper.enableOutputs();
}

void loop() {
  //delay(1 );
  stepper.runSpeed();
  // check if data has been sent from the computer:
  if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255):
    speed = Serial.read();
    // set the brightness of the LED:
    //analogWrite(ledPin, brightness);
    Serial.print("speed set to ");
    printHex(speed);
    Serial.println();
    if (speed<64) {
      dir = -1;
    } else {
      dir = 1;
    }
    //aspeed = abs(speed - 64.0) * 4.0 * dir;
    aspeed = pow(abs(speed - 64.0),1.6)*dir;
    stepper.setSpeed(aspeed);
  }
}