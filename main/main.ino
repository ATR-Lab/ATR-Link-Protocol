
//Libraries
#include <Adafruit_MPU6050.h> //IMU
#include <Adafruit_Sensor.h>  //IMU
//#include "SSD1306Ascii.h"     //OLED
//#include "SSD1306AsciiWire.h" //OLED
//#include "MAX30105.h"         //HEART
//#include "heartRate.h"        //HEART
#include <Wire.h>             //I2C
#include "atr_wearable.h"
#include "constants.h"
#include "TinyXML.h"
//#include <SoftwareSerial.h> //BLUETOOTH

//Device Information
#define DEVICE_NAME "Wearable 1"
#define DEVICE_ID 1234



//IMU setup
Adafruit_MPU6050 imu;
long roll = 0;
long pitch = 0;
long yaw = 0;

//Heart Rate Setup
//MAX30105 heartRateSensor;
//const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
//byte rates[RATE_SIZE]; //Array of heart rates
//byte rateSpot = 0;
//long lastBeat = 0; //Time at which the last beat occurred
//long irValue = 0;
//float beatsPerMinute = 0;
//int beatAvg = 0;

char opcode[2];

//SSD1306AsciiWire oled;

char buffer_[32];
TinyXML xmlParser;

void setup() {
  //InitializeOLED();
  InitializeSerial();
  InitializeIMU();
  //InitializeHeartRateSensor();
  xmlParser.init((uint8_t*)&buffer_,32,&XML_callback);
  //vibration motor setup
  pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
}

void loop() {
  //Serial.println("loop");
  int mByte;
  mByte = Serial.read();
  if (mByte != -1){
    if(mByte != '%' && mByte != 10){
      //Serial.println((char)mByte);
      xmlParser.processChar((char)mByte);
    }
    else if(mByte == '%'){
      if(opcode[0] == '1' && opcode[1] == '0'){ // IMU
        ReadIMU();
        SendXML(1);
      }
    }
  }

  /* Read Devices */
  
  
//  ReadIMU();
//  ReadHeartRate();
//
//  /* Send XML for Devices */
//  // SendXML(IMU_SENSOR);
//  SendXML(HEART_RATE_SENSOR);
//  // SendXML(VIBRATION_MOTOR);
//
//  // Maybe add a timer for this call so that the OLED doesn't flicker
//  UpdateOLED();

}

void InitializeSerial() {
  Serial.begin(9600);
}



void InitializeIMU() {
  if (!imu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}
/*
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
*/
/*
void InitializeOLED() {
  oled.begin(&Adafruit128x32, SCREEN_ADDRESS);
  oled.setFont(System5x7);
  oled.clear();
}

void UpdateOLED() {
  oled.clear();
  oled.setCursor(0,0);
  oled.print("BPM: ");
  oled.println(beatsPerMinute);
  oled.println();
  oled.println();
  oled.print("X: ");
  oled.print(roll);
  oled.print(" Y: ");
  oled.print(pitch);
  oled.print(" Z: ");
  oled.print(yaw);
}
*/
// Reads current values of IMU
void ReadIMU() {

  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  roll = a.acceleration.z;
  pitch = a.acceleration.x;
  yaw = a.acceleration.y;

}
/*
// Reads current IR value of heart rate sensor, calculates BPM, and returns XML string
void ReadHeartRate() {
  irValue = heartRateSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
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

  }
}
*/
// Generates XML string from template and sends over serial
void SendXML(int sub_device_id) {

  String deviceXML;
  String opcode;
  //read device data and generate command XML
  switch (sub_device_id) {
    case IMU_SENSOR:
      // To save memory, I guess
      Serial.print("<message>");
        Serial.print("<header>");
        Serial.print("</header>");
        Serial.print("<data>");
          Serial.print("<");
          Serial.print(XML_TAG_DATA_IMU);
          Serial.print(">");
            Serial.print("<roll>");
              Serial.print(String(roll));
            Serial.print("</roll>");
            Serial.print("<pitch>");
              Serial.print(String(pitch));
            Serial.print("</pitch>");
            Serial.print("<yaw>");
              Serial.print(String(pitch));
            Serial.print("</yaw>");
        Serial.print("</");
        Serial.print(XML_TAG_CMD_IMU);
        Serial.print(">");
      Serial.print("<data>");
      Serial.print("</message>");
      break;
    /*
    case 2:
      deviceXML = "<bpm>" + String(beatsPerMinute) + "</bpm><avg_bpm>" + String(beatAvg) + "</avg_bpm>";
      opcode = HEART_BEAT_SENSOR_CMD;
      break;
    case 3:
      deviceXML = "<motor_status>" + String(digitalRead(VIBRATION_MOTOR_PIN)) + "</motor_status>";
      opcode = VIBRATION_MOTOR_CMD;
      break;
     */
  }
}

void SetVibrationMotor(bool b){
  digitalWrite(VIBRATION_MOTOR_PIN, b);
}


void XML_callback( uint8_t statusflags, char* tagName, uint16_t tagNameLen, char* data, uint16_t dataLen){
  //Serial.println("parsing");
  if (statusflags & STATUS_START_TAG) {
    if (tagNameLen) {
      //Serial.print("Start tag ");
      //Serial.println(tagName);
    }
  }
  else if  (statusflags & STATUS_END_TAG){
    //Serial.println(tagName);
  
  }
  else if  (statusflags & STATUS_TAG_TEXT){
    //Serial.print("Tag:");
    //Serial.print(tagName);
    //Serial.print(" text:");
    //Serial.println(data);

    if(String(tagName) == String("/message/command/opcode")){
      opcode[0] = data[0];
      opcode[1] = data[1];
    }
  }
  else if  (statusflags & STATUS_ATTR_TEXT){
    //Serial.print("Attribute:");
    //Serial.print(tagName);
    //Serial.print(" text:");
    //Serial.println(data);
  }
  else if  (statusflags & STATUS_ERROR){
    //Serial.print("XML Parsing error  Tag:");
    //Serial.print(tagName);
    //Serial.print(" text:");
    //Serial.println(data);
  }


}
