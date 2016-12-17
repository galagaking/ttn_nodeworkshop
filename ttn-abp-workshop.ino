/*******************************************************************************
 * Code adapted for the Node Building Workshop using a modified LoraTracker board
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include <Wire.h>
#include "i2c.h"

#include "i2c_SI7021.h"
SI7021 si7021;
#include <Arduino.h>

int sleepcycles = 7;  // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec
bool joined = false;
bool sleeping = false;
#define LedPin 10     // pin 13 LED is not used, because it is connected to the SPI port
#define BatteryThreshold 320  // depending on voltage divider and experiments


// LoRaWAN NwkSKey, network session key
// Copy from TTN Console MSB first!
static const PROGMEM u1_t NWKSKEY[16] = { 0x01, 0x02, 0x03, 0x04, 0x58, 0xBD, 0xFE, 0x47, 0x45, 0x34, 0x00, 0x70, 0x55, 0x73, 0x9C, 0x20 };

// LoRaWAN AppSKey, application session key
// Copy from TTN Console MSB first!
static const u1_t PROGMEM APPSKEY[16] = { 0x01, 0x02, 0x03, 0x04, 0x4D, 0x31, 0x35, 0x4D, 0xA5, 0x65, 0x7E, 0xD5, 0x4A, 0xD2, 0x13, 0xB3 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x12345678 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping is hardware specific. below as used in the Loratracker.uk board
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2,5, LMIC_UNUSED_PIN}, //DIO0 and DIO1 connected
};


void onEvent (ev_t ev) {
  int i,j;
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
                sleeping = true;
        if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        Serial.print(F("Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
        i=(LMIC.frame[LMIC.dataBeg]);
        // i (0..255) can be used as data for any other application
        // like controlling a relay, showing a display message etc.
        if (i>10){
          i=10;     // maximum number of BLINKs
        }
          for(j=0;j<i;j++)
          {
            digitalWrite(LedPin,HIGH);
            delay(200);
            digitalWrite(LedPin,LOW);
            delay(400);
          }
      }
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      delay(50);  // delay to complete Serial Output before Sleeping

      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
      break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j) {
  byte buffer[2];  
  float humi,temp;
  int16_t t_value, h_value, s_value;
    si7021.getHumidity(humi);
    si7021.getTemperature(temp);
    si7021.triggerMeasurement();
    // getting sensor values
    Serial.print(F("TEMP: "));
    Serial.print(temp);
    Serial.print(F(" HUMI: "));
    Serial.print(humi);
    temp = constrain(temp,-24,40);  //temp in range -24 to 40 (64 steps)
    humi=constrain(humi,20,100);    //humi in range 20 to 100 (80 steps)
    int sensorValue = analogRead(A0); // battery voltage
  // print out the value you read:
    Serial.print(F(" Battery: "));
    Serial.println(sensorValue);
  // Vout = Vin * (R2/(R1+R2))
  // Vout = Vin * (4700/(100000+4700)), 0.135V - 3V battery
  // Vout = Vin * (10000/(100000+10000)), 0,273V - 3V battery
  // Vout = Vin * (11000/(91000+11000)), 0,323V - 3V battery
  // INTERNAL: 1.1V -> 0.001V per count
  // 3V battery -> 0.135V -> 135 (4k7 on R2)
  // 3V battery -> 0.273V -> 273 (10K on R2)
  // 3V battery -> 0.324V -> 324 (11K/91K)

        t_value=int16_t((temp*(100/6.25)+2400/6.25)); //0.0625 degree steps with offset
                                                      // no negative values
        Serial.print(F("decoded TEMP: "));
        Serial.print(t_value,HEX);
        h_value=int16_t((humi-20)/5); //5% steps, offset 20.
                Serial.print(F(" decoded HUMI: "));
        Serial.print(h_value,HEX);
        s_value=(h_value<<10) + t_value;  // putting the bits in the right place
        Serial.print(F(" decoded sent: "));
        Serial.println(s_value,HEX);
        buffer[0]=s_value&0xFF; //lower byte
        buffer[1]=s_value>>8;   //higher byte
        if (sensorValue>BatteryThreshold)
        {
          Serial.println(F("Battery OK"));
         }
         else
        {
          Serial.println(F("Battery LOW"));
          buffer[1]=buffer[1]+0x80; //warning in bit 15.
         }
        
    // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (uint8_t*) buffer, 2 , 0);
    Serial.println(F("Sending: "));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));
  Serial.print(F("Probe SI7021: "));
  if (si7021.initialize()) Serial.println(F("Sensor found!"));
    else
    {
        Serial.println(F("Sensor missing"));
        while(1) {}; // Program will stop if there is no sensor
    }
   analogReference(INTERNAL); //reference will be 1,1V internal on 3.3V Arduino
  // LED is connected to pin 10, if this port is NOT set as output before
  // SPI initialization, it will be used as SS (Slave Select) and controlled by the SPI module
      pinMode(LedPin, OUTPUT);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
   // do_send(&sendjob);
}

void loop() {
   do_send(&sendjob);    // Sent sensor values
      while(sleeping == false)
      {
        os_runloop_once();
      }
      sleeping = false;
      for (int i=0;i<sleepcycles;i++)
      {
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
      }
}

