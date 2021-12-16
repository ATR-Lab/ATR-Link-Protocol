
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define RX 2
#define TX 3

//
#define DEVICE_NAME "Wearable 1"
#define DEVICE_ID 1234

//Sub-device IDs
#define IMU       1
#define HEARTBEAT 2

Adafruit_MPU6050 imu;
SoftwareSerial Bluetooth(RX, TX);


void setup() {
  InitializeSerial();
  InitializeBluetooth();
  InitializeIMU();

}

void loop() {

  SendXML(IMU);
//  SendXML(HEART_RATE_SENSOR);

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

// Reads current values of IMU and returns an XML string
String ReadIMU(){
  String xml;
  
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  xml = "<roll>" + String(a.acceleration.z) + "</roll>" +
        "<pitch>" + String(a.acceleration.x) + "</pitch>" +
        "<yaw>" + String(a.acceleration.y) + "</yaw>";
        
  return xml;
}

// Generates XML string from template and sends over serial
void SendXML(int sub_device_id){

  String deviceXML;
  String opcode;
  //read device data and generate command XML
  switch(sub_device_id){
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
