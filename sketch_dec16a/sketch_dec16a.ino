#include <Arduino_LSM6DS3.h>
#include <MadgwickAHRS.h>

// Madgwick
Madgwick filter;
// sensor's sample rate is fixed at 119 Hz:
const float sensorRate = 104;

float roll, pitch, heading;

long previousMillis = 0;  // last timechecked, in ms
unsigned long micros_per_reading, micros_previous;

void setup() {
  Serial.begin(9600);    // initialize serial communication

  //pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  /*
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  */

  // Setup bluetooth
  //BLE.setLocalName("ArduinoIMU");
  //BLE.setAdvertisedService(imuService);
  //imuService.addCharacteristic(imuCharacteristic);
  //BLE.addService(imuService);

  // start advertising
  //BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  // start the filter to run at the sample rate:
  //filter.begin(119);

  //delay(10000);

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
  //Serial.print("Magnetic field sample rate = ");
  //Serial.print(IMU.magneticFieldSampleRate());
  //Serial.println(" uT");
  //Serial.println();
  //Serial.println("Magnetic Field in uT");
  //Serial.println("X\tY\tZ");

  micros_per_reading = 1000000 / 104;
  micros_previous = micros();
}

/*
// send IMU data
void sendSensorData() {

  float ax, ay, az; // Acceleration
  float gx, gy, gz; // Gyroscope
  float mx, my, mz; // Magnometer

  // read orientation x, y and z eulers
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  //IMU.readMagneticField(mx, my, mz);

  filter.update(gx, gy, gz, ax, ay, az); //for all 3
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);

  // Send 3x eulers over bluetooth as 1x byte array
  float data[3];
  data[0] = heading;
  data[1] = pitch;
  data[2] = roll;
  //imuCharacteristic.setValue((byte *) &data, 12);

}
*/

String send_imu_message(float x, float y, float z){
  String sx(x);
  String sy(y);
  String sz(z);
  
  String xml_ = 
  String("<message>") + 
    String("<command>") +
      String("<opcode> imu </opcode>") + 
      String("<args>") + 
        String("<x>") + 
          sx + 
       String("</x>") +
       String("<y>") +
          sy + 
      String("</y>") +
      String("<z>") + 
        sz + 
      String("</z>") + 
     String("</args>") + 
    String("</command>") + 
    String("</message>%");
    return xml_;   

  
}
void loop() {
  float ax, ay, az; // Acceleration
  float gx, gy, gz; // Gyroscope
  float mx, my, mz; // Magnometer

  // read orientation x, y and z eulers
  if(IMU.accelerationAvailable()){
    IMU.readAcceleration(ax, ay, az);
    double roll = atan2(ay , az) * 180.0 / PI;
    double pitch =atan2(-ax , sqrt(ay * ay + az * az)) * 180.0 / PI; //account for roll already applied
    Serial.println(send_imu_message(roll, pitch, 0));
    delay(1000);
  }
  
}
