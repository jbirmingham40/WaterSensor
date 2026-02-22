// Built using a 3.3V Arduino Pro Mini running at 8MHz
// Be sure to select Optiboot on 32-pin CPUs, atmega328p, 8MHz
// Do NOT select Optiboot 328p -- this will not work

#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include "RunningMedian.h"

#include <RFM69.h>
#include <SPI.h>

// Make sure to program with the processor set to the 3.3V 8 MHz version
// Cable to sensor - Brown is GND and Red/Orange is VCC
// Baud rate is 115200 to debug

#define DEBUG  // comment me out to disable Serial Debug
#ifdef DEBUG
#define DEBUG_BEGIN(x) Serial.begin(x)  // in your case you are using Serial1
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_BEGIN(x)
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define BATTERY_PIN A0
#define DONE_PIN A2

#define TX_PIN 6  // White
#define RX_PIN 7  // Yellow Divide

#define MAX_BATTERY_VOLTAGE 3.3                    // maximum battery voltage
#define HOLE_DEPTH          12.5                   // depth of the hole in inches

// **** Radio Settings *****
#define RFM69_INT_PIN 2 // Green
#define RFM69_CS_PIN 10 // Purple  
#define RFM69_RST_PIN 3 // Yellow
#define RF69_FREQ 915.0  // Change to 434.0 or other frequency, must match RX's freq!
#define SENSOR_NODE 1
#define CONTROLLER_NODE 2

RFM69 radio;

struct PayloadStruct {
  float waterLevel;
  float batteryVoltage;
};
PayloadStruct payload;

SoftwareSerial sensorSerial(TX_PIN, RX_PIN);  // RX, TX

void setup() {
  float voltage, waterLevel;

  DEBUG_BEGIN(9600);
  DEBUG_PRINTLN("Woke up from sleep");

  wdt_enable(WDTO_8S);  // enable watchdog timer

  initRadio();               // enable radio
  sensorSerial.begin(9600);  // enable sensor

  getBatteryLevel(voltage);
  getWaterLevel(waterLevel);
  sendMessage(voltage, waterLevel);

#ifdef DEBUG
  Serial.flush();
#endif

  DEBUG_PRINTLN("Going to sleep");
  sendDone();
}

void loop() {
  // // TODO: add a way to know if it is still running?
  // float voltage, waterLevel;
  // //  getBatteryLevel(voltage);
  //  getWaterLevel(waterLevel);
  //  delay(250);
  // // sendDone();
}

void sendDone() {
  wdt_reset();
  pinMode(DONE_PIN, OUTPUT);
  digitalWrite(DONE_PIN, LOW);
  delay(50);
  digitalWrite(DONE_PIN, HIGH);
}

void initRadio() {
  // initialize RF69 with default settings
  DEBUG_PRINT(F("Initializing radio... "));

  pinMode(RFM69_RST_PIN,OUTPUT);
  digitalWrite(RFM69_RST_PIN,LOW);

  radio.setIrq(RFM69_INT_PIN);
  radio.setCS(RFM69_CS_PIN);

  if (radio.initialize(RF69_FREQ, SENSOR_NODE, 0)) {
    Serial.println(F("succeeded!"));
    radio.setHighPower();
    radio.encrypt("TOPSECRETPASSWRD");
  } else {
    Serial.println(F("failed, code "));
  }
}


void sendMessage(float voltage, float waterLevel) {
  wdt_reset();
  payload.batteryVoltage = voltage;
  payload.waterLevel = waterLevel;

  int sleepTime = 100;  // start at 100 ms
  bool sendSuccess = false;
  do {
    Serial.println("Sending message");
    sendSuccess = radio.sendWithRetry(CONTROLLER_NODE, &payload, sizeof(payload));

    if(sendSuccess)
      Serial.println("ACK received!");
    else
      Serial.println("no ACK received");


    if(!sendSuccess) {
      // utilize an exponential decay delay
      wdt_reset();
      delay(sleepTime);
      sleepTime *= 2;
    }
  } while(!sendSuccess && sleepTime < 8000);
}

void getBatteryLevel(float &voltage) {
  const byte NUM_SAMPLES = 10;

  pinMode(BATTERY_PIN, INPUT);

  RunningMedian runningMedian = RunningMedian(NUM_SAMPLES);
  for (int i = 0; i < NUM_SAMPLES; i++) {
    runningMedian.add(analogRead(BATTERY_PIN));
    delay(10);
  }
  voltage = (float)(runningMedian.getAverage(3) / 1024.0) * MAX_BATTERY_VOLTAGE;

  DEBUG_PRINT(F("Battery voltage normalized is: "));
  DEBUG_PRINTLN(voltage);
}

int readSensorInMM() {
  const int MIN_SENSOR_READING = 30;
  unsigned char sensorValues[4] = {};

  // read sensor data
  do {
    for (int i = 0; i < 4; i++) {
      sensorValues[i] = sensorSerial.read();
      wdt_reset();  // Failed readings take a long time, several will exceed the watchdog timeout
    }
  } while (sensorSerial.read() == 0xff);

  sensorSerial.flush();

  // check for valid data
  if (sensorValues[0] == 0xff) {  // good reading
    int sum = (sensorValues[0] + sensorValues[1] + sensorValues[2]) & 0x00FF;
    if (sum == sensorValues[3]) {
      int reading = (sensorValues[1] << 8) + sensorValues[2];  // sensor reading in millimeters
      if(reading >= MIN_SENSOR_READING) {   // we can only trust readings from the sensor that are 30 millimeters or greater
        return reading;
      }
    }
  }

  return -1;
}

float accurateSensorReadInIN() {
  const int MAX_READINGS = 15;
  const int MIN_GOOD_READINGS = 5;
  const float MM_TO_INCHES = 0.03937;
  const int MIN_SENSOR_READING = 30;
  const int ONE_INCH_IN_MM = 26;

  int numGoodReadings = 0;

  RunningMedian runningMedian = RunningMedian(MAX_READINGS);
  for (int i=0; i < MAX_READINGS; i++) {
    int sensorReading = readSensorInMM();

    //DEBUG_PRINT("Sensor reading ");
    //DEBUG_PRINTLN(sensorReading);
    
    if(sensorReading > -1) {
        runningMedian.add(sensorReading);
        numGoodReadings++;
    }

    delay(100);
  }
  
  if(numGoodReadings >= MIN_GOOD_READINGS) {
    int median = runningMedian.getMedian();   // get the middle value which should be -1 or an integer between 30 and some upper number like 512
    int avg = runningMedian.getAverage(5);    // get the average of the 5 middle values which should be -1 if all values or negative or an integer between 30 and some upper number like 512
    int distribution = abs(median - avg);     // determine how far off our median is from the average of the 5 middle values

    // DEBUG_PRINTLN("median, avg, distribution");
    // DEBUG_PRINTLN(median);
    // DEBUG_PRINTLN(avg);
    // DEBUG_PRINTLN(distribution);

    // only allow averages that produce numbers in the valid range of 30 to some upper number like 512
    // only allow values where the median is within 26 millimeters of the average or 1 inch
    if(avg > MIN_SENSOR_READING && distribution < ONE_INCH_IN_MM) {
      //DEBUG_PRINT("Settled on sensor reading of ");
      //DEBUG_PRINTLN(median);
      return median * MM_TO_INCHES;   // return the result in inches
    }
  }

  
  DEBUG_PRINTLN("Unable to get a valid sensor reading.");
  return -1;
}

void getWaterLevel(float &waterLevel) {
  float distanceToWaterInIN = accurateSensorReadInIN();
  
  DEBUG_PRINT(F("Accurate sensor reading "));
  DEBUG_PRINTLN(distanceToWaterInIN);

  if (distanceToWaterInIN < 0) {
    waterLevel = -1;
    DEBUG_PRINTLN(F("Sensor returned a negative number.   Too close?"));
  } else if (distanceToWaterInIN > HOLE_DEPTH) {
    waterLevel = HOLE_DEPTH;
    DEBUG_PRINTLN(F("Sensor returned a distance greater than the hole distance.  Too far?"));
  } else {
    waterLevel = HOLE_DEPTH - (float)distanceToWaterInIN;
    DEBUG_PRINT(F("Water Level: "));
    DEBUG_PRINTLN(waterLevel);
  }
}