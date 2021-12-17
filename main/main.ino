
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Wire.h>
#include <SoftwareSerial.h>

#define RX 2
#define TX 3

//Device Information
#define DEVICE_NAME "Wearable 1"
#define DEVICE_ID 1234

//Sub-device IDs
#define IMU_SENSOR        1
#define HEART_RATE_SENSOR 2

Adafruit_MPU6050 imu;

//Heart Rate Setup
MAX30105 heartRateSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

SoftwareSerial Bluetooth(RX, TX);


void setup() {
  InitializeSerial();
  InitializeBluetooth();
  InitializeIMU();
  //InitializeHeartRateSensor();

}

void loop() {

  SendXML(IMU_SENSOR);
  //SendXML(HEART_RATE_SENSOR);
  //ReadHeartRate();
  //delay(100);
}

void InitializeSerial() {
  Serial.begin(9600);
}

void InitializeBluetooth() {
  Bluetooth.begin(9600);
}

void InitializeIMU() {
  if (!imu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void InitializeHeartRateSensor() {
  if (!heartRateSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  heartRateSensor.setup(); //Configure sensor with default settings
  heartRateSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  heartRateSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

// Reads current values of IMU and returns an XML string
String ReadIMU() {
  String xml;

  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  xml = "<roll>" + String(a.acceleration.z) + "</roll>" +
        "<pitch>" + String(a.acceleration.x) + "</pitch>" +
        "<yaw>" + String(a.acceleration.y) + "</yaw>";

  return xml;
}

// Reads current IR value of heart rate sensor, calculates BPM, and returns XML string
String ReadHeartRate() {
  Serial.println("Reading heart rate...");
  String xml;

  long irValue = heartRateSensor.getIR();

  Serial.println("got above if");
  if (checkForBeat(irValue) == true)
  {
    Serial.println("Got to IF");
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
  } else {
    Serial.println("Skipped IF");
  }


}

// Generates XML string from template and sends over serial
void SendXML(int sub_device_id) {

  String deviceXML;
  String opcode;
  //read device data and generate command XML
  switch (sub_device_id) {
    case 1:
      deviceXML = ReadIMU();
      opcode = "IMU";
      break;
    case 2:
      break;
  }

  String XML;
  //generate message
  XML =   "<message>" +
          String("<header>") +
          String("<name>") + DEVICE_NAME + "</name>" +
          String("<id>") + DEVICE_ID + "</id>" +
          String("<timestamp>") + 123456 + "</timestamp>" +
          String("<sub_device_id>") + sub_device_id + "</sub_device_id>" +
          String("</header>") +
          String("<command>") +
          String("<opcode>") + opcode + "</opcode>" +
          String("<args>") + deviceXML + "</args>" +
          String("</command></message>");

  Serial.println(XML);
}
