
//Libraries
#include <Adafruit_MPU6050.h> //IMU
#include <Adafruit_Sensor.h>  //IMU
#include "SSD1306Ascii.h"     //OLED
#include "SSD1306AsciiWire.h" //OLED
#include "MAX30105.h"         //HEART
#include "heartRate.h"        //HEART
#include <Wire.h>             //I2C
#include "atr_wearable.h"
#include "constants.h"
//#include <SoftwareSerial.h> //BLUETOOTH
#define SCREEN_ADDRESS 0x3C

//Device Information
#define DEVICE_NAME "Wearable 1"
#define DEVICE_ID 1234



//IMU setup
Adafruit_MPU6050 imu;
long roll = 0;
long pitch = 0;
long yaw = 0;


MAX30105 heartRateSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
long irValue = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

char opcode[2];
int opcodeidx = 0;
SSD1306AsciiWire oled;


void setup() {
  //InitializeOLED();
  Serial.begin(9600);
  Serial.println("Setup");Serial.flush();
  InitializeIMU();
  //InitializeHeartRateSensor();
  //vibration motor setup
  //pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
  int opcodeidx = 0;
  //delay(2000);
}




void InitializeIMU() {
  if (!imu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      //Serial.println("ATke");
      delay(10);
    }
  }
  //Serial.println("Paise");
  //while(1);
  imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void InitializeHeartRateSensor() {
  if (!heartRateSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    //Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  heartRateSensor.setup(); //Configure sensor with default settings
  heartRateSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  heartRateSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}


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

// Reads current values of IMU
void ReadIMU() {
  //Serial.println("Asche 1");Serial.flush();
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);
  
  roll = a.acceleration.z;
  pitch = a.acceleration.x;
  yaw = a.acceleration.y;

  float accelerationX = (int16_t)(a.acceleration.x * CONVERSIONG);
  float accelerationY = (int16_t)(a.acceleration.y * CONVERSIONG);
  float accelerationZ = (int16_t)(a.acceleration.z * CONVERSIONG);
  //pitch = 180 * atan2 (accelerationX,sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/M_PI;
  roll = 180 * atan2 (-accelerationY, accelerationZ)/M_PI;
  //yaw = 180 * atan2 (accelerationZ,sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI;
  //Serial.println("asche");Serial.flush();
  SendXML(1);
}

// Reads current IR value of heart rate sensor, calculates BPM.
void ReadHeartRate() {
  irValue = heartRateSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    //Serial.print("asche");
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
    //Serial.println(beatAvg);
  }
  SendXML(2);
  
}

// Generates XML string from template and sends over serial
void SendXML(int sub_device_id) {

  String deviceXML;
  String opcode;
  //read device data and generate command XML
  switch (sub_device_id) {
    case IMU_SENSOR:
      // To save memory, I guess
        Serial.print("{");Serial.flush();
        Serial.print("\'type\': \'IMU\',");Serial.flush();
        Serial.print("\'roll\':"); Serial.flush();
        Serial.print(String(roll));Serial.flush();
        //Serial.print(",");Serial.flush();
        //Serial.print("\'pitch\':"); Serial.flush();
        //Serial.print(String(pitch));Serial.flush();
        //Serial.print(",");Serial.flush();
        //Serial.print("\'yaw\':"); Serial.flush();
        //Serial.print(String(yaw));Serial.flush();
        Serial.print("}%"); Serial.flush();
    break;
    case 2:
        Serial.print("{"); Serial.flush();
        Serial.print("\'type\': \'BPM\',");Serial.flush();
        Serial.print("\'bpm\':"); Serial.flush();
        Serial.print(String(beatAvg)); Serial.flush();
        Serial.print("}%"); Serial.flush();
      break;
    /*
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


void loop() {
  //Serial.println("loop");Serial.flush();
  ReadIMU();
  //ReadHeartRate();
  //UpdateOLED();
  //delay(100);
  /*
  int mByte;
  mByte = Serial.read();
  
  if (mByte != -1){
    Serial.print(mByte,DEC);Serial.flush();
      opcode[opcodeidx++] = mByte;
    }
    else if(mByte == '%'){
      //Serial.println("dhukse3");
      Serial.println(opcode[0],DEC);
      Serial.println(opcode[1],DEC);
      if(opcode[0] == '1' && opcode[1] == '0'){ // IMU
        //ReadIMU();
        Serial.println("dhukse4");
        SendXML(1);
      }
      else if(opcode[0] == '1' && opcode[1] == '1'){ // HearBeat
        //Serial.println("dhukse2");
        //ReadHeartRate();
        SendXML(2);
      }
      else{
        //Serial.println("dhuke nai");
        //Serial.println(mByte,DEC);
      }
      opcodeidx = 0;
    }
    else if(mByte == 94){
      opcodeidx = 0;
      Serial.print("%");
      Serial.flush();
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
