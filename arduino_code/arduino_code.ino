#include <Wire.h>

const int dirPinLeft = 7;
const int dirPinRight = 8;

const bool cw = true;
const bool ccw = false;

const int pwmPinLeft = 9;    // PWM output pin 1
const int pwmPinRight = 10;  // PWM output pin 2

#define DIR_MASK 0b10000000    // 1 bit for direction (left or right)
#define FB_MASK 0b01000000     // 1 bit for forward or backward
#define SPEED_MASK 0b00111111  // 6 bits for speed value

int waitTime=1000;
long prevMillis=0;

// Define the I2C address for this slave device
#define I2C_ADDRESS 0x08

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveByte);
  pinMode(dirPinLeft, OUTPUT);
  pinMode(dirPinRight, OUTPUT);
  pinMode(pwmPinLeft, OUTPUT);
  pinMode(pwmPinRight, OUTPUT);

  // Set up Timer1 for 25 kHz PWM
  TCCR1A = 0;  // Reset control registers
  TCCR1B = 0;

  TCCR1A = (1 << WGM11) | (1 << WGM10);  // Set to Fast PWM mode
  TCCR1B = (1 << WGM12) | (1 << CS10);   // Fast PWM, no prescaler

  // Set frequency to 25 kHz
  OCR1A = 319;  // (16MHz / (25kHz * 2)) - 1
}

void setDutyCycle(int pin, int dutyCycle) {
  if (pin == pwmPinLeft) {
    OCR1A = map(dutyCycle, 0, 64, 0, 319);
  } else if (pin == pwmPinRight) {
    OCR1B = map(dutyCycle, 0, 64, 0, 319);
  }
}

void loop(){
  // stop motors if jetson nano stops sending packets for waitTime milliseconds
  if (millis()-prevMillis>waitTime){
    setDutyCycle(pwmPinLeft,0);
    setDutyCycle(pwmPinRight,0);
    Serial.println("Stopping Motors");
    prevMillis=millis();
  }
}

void receiveByte(int byteCount) {
  prevMillis=millis();
  while (Wire.available()) {
    byte receivedByte = Wire.read();

    // Extracting the left/right bit (most significant bit)
    bool rightBool = (receivedByte & DIR_MASK) >> 7;

    // Extracting the forward/backward bit (second most significant bit)
    bool forwardBool = (receivedByte & FB_MASK) >> 6;

    // Extracting the speed (last 6 bits)
    int speed = receivedByte & SPEED_MASK;

    //control right wheel spin
    if (rightBool) {
      if (forwardBool) {
        digitalWrite(dirPinRight, cw);
      } else {
        digitalWrite(dirPinRight, ccw);
      }
      setDutyCycle(pwmPinRight, speed);
    }

    //control left wheel spin
    else {
      if (forwardBool) {
        digitalWrite(dirPinLeft, ccw);
      } else {
        digitalWrite(dirPinRight, cw);
      }
      setDutyCycle(pwmPinLeft, speed);
    }

    // Displaying the results
    Serial.print("Left/Right: ");
    Serial.print(rightBool ? "Right" : "Left");
    Serial.print(", Forward/Backward: ");
    Serial.print(forwardBool ? "Forward" : "Backward");
    Serial.print(", Speed: ");
    Serial.println(speed);
  }
}
