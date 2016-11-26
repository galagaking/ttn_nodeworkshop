/*
  Button
 
 Turns on and off a light emitting diode(LED) connected to digital  
 pin 10, when board is moved due to ball switch on pin 6. 
 
 
 The circuit:
 * LED attached from pin 10 to ground 
 * Tilt / ball switch on pin 6
 
 http://www.arduino.cc/en/Tutorial/Button
 */

const int ballPin = 6;     // the number of the pushbutton pin
const int ledPin =  10;      // the number of the LED pin

// variables will change:
int ballState = 0;         // variable for reading the ball switch status

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);      
  // initialize the ball switch pin as an input, using internal PULLUP:
    pinMode(ballPin, INPUT_PULLUP);  
}

void loop(){
  // read the state of the ball switch:
  ballState = digitalRead(ballPin);

  // check if the ball switch is moved.
  // if it is, the ballState is HIGH:
  if (ballState == HIGH) {     
    // turn LED on:    
    digitalWrite(ledPin, HIGH);  
  } 
  else {
    // turn LED off:
    digitalWrite(ledPin, LOW); 
  }
}
