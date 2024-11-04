#include <Wire.h>

const int dirPinLeft = 7;
const int dirPinRight = 8;

const bool cw = false;
const bool ccw = true;

const int pwmPinLeft = 9;    // PWM output pin 1
const int pwmPinRight = 10;  // PWM output pin 2

#define DIR_MASK 0b10000000    // 1 bit for direction (left or right)
#define FB_MASK 0b01000000     // 1 bit for forward or backwards
#define SPEED_MASK 0b00111111  // 6 bits for speed value

int waitTime=100;
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

  // Configure Timer1 for PWM_PIN_L
  TCCR1A = 0;  // Reset Timer1 configuration
  TCCR1B = 0;

  // Set Fast PWM mode with ICR1 as TOP
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  // Set prescaler to 1 and start PWM
  TCCR1B |= (1 << CS10);

  // Set TOP value for 25 kHz frequency
  ICR1 = 640;  // (16 MHz / (1 * 25 kHz)) - 1

  // Set non-inverting mode for both channels
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
}

void setDutyCycle(int pin, int dutyCycle) {
  if (pin == pwmPinLeft) {
    OCR1A = map(dutyCycle, 0, 64, 0, ICR1);
  } else if (pin == pwmPinRight) {
    OCR1B = map(dutyCycle, 0, 64, 0, ICR1);
  }
}

void loop(){
  // stop motors if jetson nano stops sending packets for waitTime milliseconds
  if (millis()-prevMillis>waitTime){
    setDutyCycle(pwmPinLeft,0);
    setDutyCycle(pwmPinRight,0);
    //Serial.println("Stopping Motors");
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
    //Serial.print("Left/Right: ");
    //Serial.print(rightBool ? "Right" : "Left");
    //Serial.print(", Forward/Backward: ");
    //Serial.print(forwardBool ? "Forward" : "Backward");
    //Serial.print(", Speed: ");
    //Serial.println(speed);
  }
}
