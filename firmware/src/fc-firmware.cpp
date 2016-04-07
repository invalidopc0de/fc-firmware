#include "Arduino.h"

#include "algorithm.h"

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
  This example code is in the public domain.
 */
 
DeviceSpec spec;

void fire_charge(int)
{
    digitalWrite(13, HIGH);
}

void setup() {                
	// initialize the digital pin as an output.
	// Pin 13 has an LED connected on most Arduino boards:
	pinMode(13, OUTPUT);     
    
    // Should initalize devices here
    
    spec.fire_charge = fire_charge;
    
    init_algorithm(&spec);
}

void loop() {
    eval_algorithm(&spec);
	delay(1000);     
}