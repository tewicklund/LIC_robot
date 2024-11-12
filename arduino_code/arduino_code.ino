#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

// Define the CE and CSN pins for the nRF24L01+ module
#define CE_PIN 2
#define CSN_PIN 3

// Create an RF24 object
RF24 radio(CE_PIN, CSN_PIN);

// Define the address for communication (must match transmitter)
const byte address[6] = "00001";  

const int dirPinLeft = 7;
const int dirPinRight = 8;

const bool cw = false;
const bool ccw = true;

const int pwmPinLeft = 9;    // PWM output pin 1
const int pwmPinRight = 10;  // PWM output pin 2

#define DIR_MASK 0b10000000    // 1 bit for direction (left or right)
#define FB_MASK 0b01000000     // 1 bit for forward or backwards
#define SPEED_MASK 0b00111111  // 6 bits for speed value

int waitTime = 100;
long prevMillis = 0;

// Define the I2C address for this slave device
#define I2C_ADDRESS 0x08

void printByteAsBits(byte b) {
  for (int i = 7; i >= 0; i--) {
    Serial.print(bitRead(b, i));
  }
  Serial.println();
}

void setupNRF24() {
  radio.begin();
  radio.openReadingPipe(0, address);  // Set the address to read from
  radio.setPALevel(RF24_PA_LOW);      // Set power level (use LOW for short distances)
  radio.startListening();             // Set the module as a receiver
}

void byte_to_speed(byte command_byte) {
  // Extracting the left/right bit (most significant bit)
  bool rightBool = (command_byte & DIR_MASK) >> 7;

  // Extracting the forward/backward bit (second most significant bit)
  bool forwardBool = (command_byte & FB_MASK) >> 6;

  // Extracting the speed (last 6 bits)
  int speed = command_byte & SPEED_MASK;

  //control right wheel spin
  if (rightBool) {
    if (forwardBool) {
      digitalWrite(dirPinRight, cw);
    } else {
      digitalWrite(dirPinRight, ccw);
    }
    if (speed != 0) {
      speed += 64;
    }
    setDutyCycle(pwmPinRight, speed);
  }

  //control left wheel spin
  else {
    if (forwardBool) {
      digitalWrite(dirPinLeft, ccw);
    } else {
      digitalWrite(dirPinLeft, cw);
    }
    if (speed != 0) {
      speed += 64;
    }
    setDutyCycle(pwmPinLeft, speed);
  }
}

void setDutyCycle(int pin, int dutyCycle) {
  if (pin == pwmPinLeft) {
    OCR1A = map(dutyCycle, 0, 255, 0, ICR1);
  } else if (pin == pwmPinRight) {
    OCR1B = map(dutyCycle, 0, 255, 0, ICR1);
  }
}

void handle_I2C(int byteCount){
  while (Wire.available()){
    byte received_byte_I2C=Wire.read();
    Serial.println("I2C Packet Received!");
    prevMillis = millis();
    printByteAsBits(received_byte_I2C);
    byte_to_speed(received_byte_I2C);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(handle_I2C);
  setupNRF24();

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

void loop() {
  // stop motors if jetson nano stops sending packets for waitTime milliseconds
  if (millis() - prevMillis > waitTime) {
    setDutyCycle(pwmPinLeft, 0);
    setDutyCycle(pwmPinRight, 0);
    //Serial.println("Stopping Motors");
    prevMillis = millis();
  }

  //handle incoming SPI packets from RF module
  if (radio.available()) {
    Serial.println("SPI Packet Received!");
    prevMillis = millis();
    byte received_byte_SPI;
    radio.read(&received_byte_SPI, sizeof(received_byte_SPI));
    printByteAsBits(received_byte_SPI);
    byte_to_speed(received_byte_SPI);
  }
}
