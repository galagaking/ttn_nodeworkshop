#include <TheThingsNetwork.h>
#include <Wire.h>

#include <Adafruit_BMP280.h>

//Use with The Things Network:
/*
Use the initial code to get the EUI of the module. Copy this value when adding a device in the TTN Console to devEUI
Generate an appKey in the console and copy the value in 'appKey' below.
appEui is 0000000000000000, as well as joinEui.
Use the code below as 'uplink payload formatter' // custom javascript formatter

// TTNV3 Payload Formatter Uplink V0.1
function decodeUplink(input) {

if ((input.fPort > 0) && (input.fPort < 223))
{
  var decodedTemp = 0;
  var decodedPress = 0;

// seperate raw data from payload
  var rawTemp = input.bytes[0] + input.bytes[1] * 256;
  var rawPress = input.bytes[2] + input.bytes[3] * 256;

// decode raw data to values
  decodedTemp = ((rawTemp/10)-20.0) ; // adjust to one decimal, correct offset
  decodedPress = rawPress /10; //adjust to HectoPascal /mBar
  
// definition of the decimal places
  decodedTemp = decodedTemp.toFixed(1);
  decodedPress = decodedPress.toFixed(1);


// return values
  return {
    data: {
      Temperature: decodedTemp,
      Pressure: decodedPress
          },
    warnings: [],
    errors: []
  };
}
else {
    return {
      data: {},
      warnings: [],
      errors: ["Invalid data received"]
    };
  
}
}
//end of payload formatter


*/


Adafruit_BMP280 bmp; // I2C

// Set your AppEUI and AppKey
const char *appEui = "0000000000000000";
const char *appKey = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";

#define GROUNDLEVEL_PRESSURE 997.1 //pressure at groundlevel to calibrate height

#define loraSerial Serial1
#define debugSerial Serial

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  // Wait a maximum of 10s for Serial Monitor - native USB
  while (!debugSerial && millis() < 10000);

  debugSerial.println(F("BMP280 test"));
  unsigned status;
  status = bmp.begin(0x76,0x60);
  //status = bmp.begin();
  if (!status) {
    debugSerial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    debugSerial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    debugSerial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    debugSerial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    debugSerial.print("        ID of 0x60 represents a BME 280.\n");
    debugSerial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */




  debugSerial.println("-- STATUS");
  ttn.showStatus();

  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);

  ttn.showStatus();
  debugSerial.println("Setup for The Things Network complete");
}

void loop()
{

    debugSerial.print(F("Temperature = "));
    debugSerial.print(bmp.readTemperature());
    uint16_t temperature=10*(bmp.readTemperature()+20.0);  //convert float to int, add 20 degrees to eliminate negative values (unsigned int)
    debugSerial.println(" *C");

    debugSerial.print(F("Pressure = "));
    debugSerial.print(bmp.readPressure());
    uint16_t pressure=0.1*bmp.readPressure(); //convert float to int, measurement is in Pascal, so divede by 100 to get HectoPascal ('mBar'). First divide by 10 here
    debugSerial.println(" Pa");

    debugSerial.print(F("Approx altitude = "));
    debugSerial.print(bmp.readAltitude(GROUNDLEVEL_PRESSURE)); /* Adjusted to local forecast! */
    debugSerial.println(" m");

  // Split word (16 bits) into 2 bytes of 8
  byte payload[3];
  payload[0] = lowByte(temperature);
  payload[1] = highByte(temperature);
  payload[2] = lowByte(pressure);
  payload[3] = highByte(pressure);

  debugSerial.print("Transmitting sensor values ");
  debugSerial.print(F("temperature: "));
  debugSerial.print(temperature);
  debugSerial.print(F(" - pressure: "));
  debugSerial.println(pressure);

  ttn.sendBytes(payload, sizeof(payload)+1);

  delay(600000); //once per 10 minutes, do not sent to frequently because of bandwidth limitations of LoRaWAN
}
