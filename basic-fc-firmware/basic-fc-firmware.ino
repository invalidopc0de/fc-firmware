#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

// Teensy 3.0 has the LED on pin 13
const int ledPin = 13;

// set this to the hardware serial port you wish to use
#define GPSSERIAL Serial1 //TODO: NEED TO VERIFY
#define RADIOSERIAL Serial2

// the setup() method runs once, when the sketch starts
void setup() {
  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);
  
  Serial.begin(9600);
  GPSSERIAL.begin(38400); // TODO: Set correct value for GPS
  RADIOSERIAL.begin(9600);
}

// the loop() methor runs over and over again,
// as long as the board has power
void loop() {
    int incomingByte;
        
	if (Serial.available() > 0) {
		incomingByte = Serial.read();
        Serial.write(incomingByte);
        GPSSERIAL.write(incomingByte);
	}
	if (GPSSERIAL.available() > 0) {
		incomingByte = GPSSERIAL.read();
		Serial.write(incomingByte);
        RADIOSERIAL.write(incomingByte);
	}
}

