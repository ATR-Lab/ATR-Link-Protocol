/////////////////////////////////////////////////////////
//
//  Data 1st HR
//  Data 2nd GSR Vb
//
//
//
/////////////////////////////////////////////////////////
// TPU 6050
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;


// PIR Sensor Node 02
#include <avr/sleep.h>
#define interruptPin 2

// PIR
int ledPin = 13;                // LED
int pirPin = 3;                 // PIR Out pin
int pirStat = 0;                // PIR status
int beatsPerMinute_pre = 0;
int beatsPerMinute_flag = 0;

// PPG
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
MAX30105 particleSensor;
void PPG_Sensor_raw();
void PPG_Sensor();

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

// Timer for 10 s
int timer_10s = 1;
int timer_counts = 0;

volatile int seconds = 0;

// ADC

int analogPin_v0 = A3; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V
int analogPin_vb = A1; // potentiometer wiper (middle terminal) connected to analog pin 3
int val = 0;  // variable to store the value read

byte curr_v0 = 0; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V
byte curr_vb = 0; // potentiometer wiper (middle terminal) connected to analog pin 3


int Sensor_order_counts = 0;
int Sensor_flag = 0;

// OLED

// 128x32_OLED_Hello_World_Adafruitlib
// based on Adafruit libraries supporting graphic displays
// public domain

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

void oled(void);

void setup()
{
    Serial.begin(115200);
  //Serial.println("Initializing...");
  digitalWrite(4, HIGH);

  // TPU 6050

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  calculate_IMU_error();
  delay(20);
  // OLED
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
oled();
  
  
  // PIR
  //pinMode(ledPin, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(pirPin, INPUT);
  pinMode(interruptPin, INPUT_PULLUP);

  // PPG

  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    //while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  // interrupt
  int frequency = 1; // in hz
  //Interupt Service Routine and timer setup
  noInterrupts();// kill interrupts until everybody is set up
  //We use Timer 1 b/c it's the only 16 bit timer
  TCCR1A = B00000000;//Register A all 0's since we're not toggling any pins
  // TCCR1B clock prescalers
  // 0 0 1 clkI/O /1 (No prescaling)
  // 0 1 0 clkI/O /8 (From prescaler)
  // 0 1 1 clkI/O /64 (From prescaler)
  // 1 0 0 clkI/O /256 (From prescaler)
  // 1 0 1 clkI/O /1024 (From prescaler)
  TCCR1B = B00001100;//bit 3 set for CTC mode, will call interrupt on counter match, bit 2 set to divide clock by 256, so 16MHz/256=62.5KHz
  TIMSK1 = B00000010;//bit 1 set to call the interrupt on an OCR1A match
  OCR1A  = (unsigned long)((62500UL / frequency) - 1UL);//our clock runs at 62.5kHz, which is 1/62.5kHz = 16us
  interrupts();//restart interrupts

}


void PPG_Sensor()
{
  long irValue = particleSensor.getIR();

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

  Serial.print(beatsPerMinute);

  byte kkk = byte(beatsPerMinute);


  if (beatsPerMinute_pre != kkk)
  {
    beatsPerMinute_pre = kkk;
    beatsPerMinute_flag = 1;
    Sensor_order_counts ++;
    if (Sensor_order_counts == 3) Sensor_order_counts = 1;
  }
}

void PPG_Sensor_raw()
{
  long irValue = particleSensor.getIR();
  Serial.println(irValue);
}


void ACC_GYR()
{
    // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(yaw);
  //Serial.println();
}

void ACC_GYR_raw()
{
    // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
//  // Print the values on the serial monitor
  Serial.print(AccX);
  Serial.print(",");
  Serial.print(AccY);
  Serial.print(",");
  Serial.print(AccZ);
  //Serial.println();
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
//  // Print the error values on the Serial Monitor
//  Serial.print("AccErrorX: ");
//  Serial.println(AccErrorX);
//  Serial.print("AccErrorY: ");
//  Serial.println(AccErrorY);
//  Serial.print("GyroErrorX: ");
//  Serial.println(GyroErrorX);
//  Serial.print("GyroErrorY: ");
//  Serial.println(GyroErrorY);
//  Serial.print("GyroErrorZ: ");
//  Serial.println(GyroErrorZ);
}

void loop()
{
  //PPG_Sensor_raw();
  ACC_GYR_raw();
  //ACC_GYR();
  //PPG_Sensor();
  Serial.println();
  
}

ISR(TIMER1_COMPA_vect) { //Interrupt Service Routine, Timer/Counter1 Compare Match A
  seconds++;
  if (seconds >= 60) { //set to however many seconds you want
    timer_10s = 1;
    //Serial.println(micros());           // This code is what happens
    seconds = 0;                        // after 'x' seconds
    //digitalWrite(13, !digitalRead(13)); //
    Sensor_order_counts = 1;
  } else if (seconds >= 4) 
  {
    Sensor_flag = 1;
    }
}

void oled(void)
{
display.clearDisplay (); // clear display
display.setCursor (10,5); // position the cursor
display.setTextSize (2); // medium size font
display.setTextColor (WHITE); // white is not default !
display.print ("Hello");

display.setCursor (10,23);
display.setTextSize (1); // smallest font
display.print ("World");
delay (1000); // wait a second

display.drawCircle (110,15,15, WHITE); // head contour
display.drawCircle (105,14,4, WHITE); // left eye
display.drawCircle (115,14,4, WHITE); // right eye

display.drawPixel (103,22, WHITE); // mouth
display.drawPixel (118,22, WHITE); // mouth
display.drawPixel (104,23, WHITE); // mouth
display.drawPixel (117,23, WHITE); // mouth
display.drawPixel (105,24, WHITE); // mouth
display.drawPixel (116,24, WHITE); // mouth
display.drawLine (106,25,115,25, WHITE); // mouth
display.display ();

delay (1000);

display.clearDisplay (); // clear display
display.setCursor (10,5); // position the cursor
display.setTextSize (2); // medium size font
display.setTextColor (WHITE); // white is not default !
display.print ("SCI Lab");

display.setCursor (10,23);
display.setTextSize (1); // smallest font
display.print ("The Best!");
delay (1000); // wait a second

display.drawCircle (110,15,15, WHITE); // head contour
display.drawCircle (105,14,4, WHITE); // left eye
display.drawCircle (115,14,4, WHITE); // right eye

display.drawPixel (103,22, WHITE); // mouth
display.drawPixel (118,22, WHITE); // mouth
display.drawPixel (104,23, WHITE); // mouth
display.drawPixel (117,23, WHITE); // mouth
display.drawPixel (105,24, WHITE); // mouth
display.drawPixel (116,24, WHITE); // mouth
display.drawLine (106,25,115,25, WHITE); // mouth
display.display ();

delay (1000);

}
