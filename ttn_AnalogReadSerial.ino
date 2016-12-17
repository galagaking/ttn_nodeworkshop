/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/
#define BatteryThreshold 265

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  analogReference(INTERNAL); //reference will be 1,1V internal on 3.3V Arduino
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Serial.print(sensorValue);
  // Vout = Vin * (R2/(R1+R2))
  // Vout = Vin * (4700/(100000+4700)), 0.135V - 3V battery
  // Vout = Vin * (10000/(100000+10000)), 0,273V - 3V battery
  // Vout = Vin * (11000/(91000+11000)), 0,323V - 3V battery
  // INTERNAL: 1.1V -> 0.001V per count
  // 3V battery -> 0.135V -> 135 (4k7 on R2)
  // 3V battery -> 0.273V -> 273 (10K on R2)
  // 3V battery -> 0.324V -> 324 (11K/91K)

  if (sensorValue>BatteryThreshold)
  {
    Serial.println(" - Battery OK");
  }
  else
  {
    Serial.println(" - Battery LOW");
  }
  
  delay(100);        // delay in between reads for stability
}
