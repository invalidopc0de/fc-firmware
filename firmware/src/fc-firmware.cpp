#include "algorithm.h"

#include "Arduino.h"

 
DeviceSpec spec;

void fire_charge(int)
{
    digitalWrite(13, HIGH);
}

void setup() {       
    //Serial.begin(9600);
     
	// initialize the digital pin as an output.
	// Pin 13 has an LED connected on most Arduino boards:
	pinMode(13, OUTPUT);     
    
    // Should initalize devices here
    
    spec.fire_charge = fire_charge;
    
    //init_algorithm(&spec);
}

void loop() {
     //Serial.println("Hello");
    
    //eval_algorithm(&spec);
    digitalWrite(13, LOW);
	delay(1000);   
    digitalWrite(13, LOW);
    delay(1000);  
}