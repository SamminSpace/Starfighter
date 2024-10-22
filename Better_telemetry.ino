#include <Wire.h> //I2C
#include <SPI.h> //OLED
#include <SD.h> 
#include <Adafruit_Sensor.h>
//#include <utility/imumaths.h>
#include <107-Arduino-BMP388.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include "SparkFun_Ublox_Arduino_Library.h" //idk if I have right library


//BMP pins
int BMP_sensor = 19;
int output_pin = 18;

//BMP3XX
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

//BNO055
double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 1000; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees


Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055();
SFE_UBLOX_GPS myGPS;
//SFE_UBLOX_GNSS myGNSS; HELP NO WORK


//TELEMETRY 
String TeamID = "Starfighter";
String payload_state = "launch";
float MissionTime = 0; 
float UTCtime = 0; 
int Packetnum = 0;


//variables
int temperature = 0;
float altitude = 0; 
float previous_alt = 0;
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;
float gps_lat = 0;
float gps_long = 0;
float gps_alt = 0;
float velocity = 0;

//Timer for altitude rate of change AKA vertical velocity
double long time1 = 0;
double long time2 = 0;

//Timer for waiting 10 seconds and 2 mins for states to update 
long previousMillis = 0;
long interval = 10000; //10 seconds
long fivemin = 120000; //2 minutes in milliseconds

//LED loop timer
int ledState = LOW;
int ledPin = 2;
long current_time = 0; 
long previous_time = 0;
int blinkrate = 500;

//SD CARD
File dataFile; 
const int chipSelect = 10;

//stabilization
int solenoid_left = 3; //CW
int solenoid_right = 4; //CCW
float angular_velocity = 0;


void setup() {
  Serial.begin(9600);
  pinMode(BMP_sensor, INPUT);
  pinMode(output_pin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(solenoid_left, OUTPUT);
  pinMode(solenoid_right, OUTPUT);


  // WRITING THE HEADER
  Serial.print("Writing Header...");

  // see if the card is present and can be initialized:
  while (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    delay(1000);
      // No SD card, so don't do anything more - stay stuck here
  }
  
  Serial.println("and it worked!");

  //the actual heading
  File dataFile = SD.open("telemetry.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("Team ID, Mission Time, UTC Time, Packet Count, "
    "Payload State, Altitude, Temperature, XAccerlation, YAcceleration, "
    "ZAccerlation, XGyroscope, YGyroscope, ZGyroscope, GPS Lattitude, "
    "GPS Longitude, GPS Altitude");
    dataFile.close();
  }
  //prints if sucessful 
   Serial.println("Team ID, Mission Time, UTC Time, Packet Count, "
  "Payload State, Altitude, Temperature, XAccerlation, YAcceleration, "
  "ZAccerlation, XGyroscope, YGyroscope, ZGyroscope, GPS Lattitude, "
  "GPS Longitude, GPS Altitude");
  

  // Set up oversampling and filter initialization (tbh IDK)
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  //calibration for BNO055
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
}



void loop() {
  LED_Blink();
  Read_BMP();
  Read_IMU();
  TelemertyLoop(); // MAKE SURE TELEMETRY LOG IS BEFORE UPDATE ALTITUDE

  // ADD A LAUNCH CATEGORY 


  while (payload_state == "Ascenion"){
    Serial.print("We going up");
    LED_Blink();
    Read_BMP();
    Read_IMU();
    TelemertyLoop();
    Update_Altitude();
     

  long currentMillis = millis();
  if ((altitude >= 20000) && (velocity > 0)) { // above 20km and going up
    if (currentMillis - previousMillis >= interval) { //10 second timer
      payload_state = "Stabilization";
  } else {
    previousMillis = currentMillis; //reset timer so its ready for the next time
     }
   }
  }


  while (payload_state == "Stabilization"){
    Serial.print("Do stabilization things");
    LED_Blink();
    Read_BMP();
    Read_IMU();
    TelemertyLoop();
    Update_Altitude();

  long currentMillis = millis();
  if ((altitude >= 1) && (velocity < 0)) { // negative vertical velocity
    if (currentMillis - previousMillis >= fivemin) { //2 min  timer
      payload_state = "Descent";
  } else {
    previousMillis = currentMillis; //reset timer 
     }
   }
  }

  while (payload_state == "Descent"){
    Serial.print("Going down");
    LED_Blink();
    Read_BMP();
    Read_IMU();
    TelemertyLoop();
    Update_Altitude();

    long currentMillis = millis();
    if ((altitude < 1) && (velocity < 1)) { //maybe just if v < 1
      if (currentMillis - previousMillis >= interval) { 
        payload_state = "Landing";
  } else {
    previousMillis = currentMillis; 
   }
   
  }

  while (payload_state == "Landing" ){
    LED_Blink();
    Serial.print("Payload has sucessfully landed");
    Serial.print("no need to collect data");
  }
  }
}


//function to loop
void TelemertyLoop ()
{
  MissionTime = (millis() / 1000);

  File dataFile = SD.open("telemetry.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(TeamID + "," + MissionTime + "," + UTCtime + 
    "," + Packetnum + "," + payload_state + "," + altitude + ","
    + temperature + "," + accel_x + "," + accel_y + "," + accel_z +
    "," + gyro_x + "," + gyro_y + "," + gyro_z + "," + gps_lat + 
    "," + gps_long + "," + gps_alt);
    dataFile.close();
  }

  Packetnum = Packetnum + 1;
}

void LED_Blink()
{
  long current_time = millis();

  if (current_time - previous_time >= blinkrate) {
    // save the last time you blinked the LED
    previous_time = current_time;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}



void Read_BMP()
{
  temperature = bmp.temperature; 
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
}



void Read_IMU ()
{
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    accel_x = acc.x();
    accel_y = acc.y();
    accel_z = acc.z();
    gyro_x = gyro.x();
    gyro_y = gyro.y();
    gyro_z = gyro.z();
}


void Update_Altitude ()
{
  time2 = (millis() /1000);    
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  velocity = ((altitude - previous_alt) / (time2 - time1));
  previous_alt = altitude;
  time1 = time2;
}


void Read_Ang_Velocity () //basically same fucntion as Read_IMU so maybe combine?
{
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    angular_velocity = gyro.z();  
}


void Read_GPS()
{
    gps_lat = myGPS.getLatitude();
    gps_long = myGPS.getLongitude();
    gps_alt = myGPS.getAltitude();
    //UTCtime =(myGPS.getHour() + ":" + myGPS.getMinute() + ":" + myGPS.getSecond());
    //Serial.print(myGNSS.getMinute());
   
}
