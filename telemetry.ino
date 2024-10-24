#include <Wire.h> //I2C
#include <SPI.h> //OLED
#include <SD.h> 
#include <Adafruit_Sensor.h>
//#include <utility/imumaths.h>
#include <107-Arduino-BMP388.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include "SparkFun_Ublox_Arduino_Library.h" //idk if I have right library


//BMP388
int BMP_sensor = 19;
int output_pin = 18;
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


//TELEMETRY 
String TeamID = "Starfighter";
String payload_state = "launch";
float MissionTime = 0; 
String UTCtime = "0-0-0:0:0:0"; 
int Packetnum = 0;


//variables
float temperature = 0.0;
float altitude = 0.0; 
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

//Timer sampling 
long previousMillis = 0;
long currentMillis = 0;
long sesnor_interval = 100;


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

int state = 0;
int falltime = 30000;
int fallprevioustime;
int current_alt;
float previous_alt = 0.0;
float linaccel;







void setup() {
  Serial.begin(9600);
  pinMode(BMP_sensor, INPUT);
  pinMode(output_pin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(solenoid_left, OUTPUT);
  pinMode(solenoid_right, OUTPUT);


  if (!bmp.begin_I2C()) {
    Serial.println("Could not find BMP3 sensor.");
  } 
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
  // put your main code here, to run repeatedly:

  if(state == 0){
    state = 1;
    Update_State();
  }else if (state == 1){
    //Launch
    LED_Blink();
    Read_BMP();
    Read_IMU();
    Read_GPS();
    TelemertyLoop();

    if(altitude > 1000){
      //Go to ascent
      state = 2;
      Update_State();
    }

  } else if(state == 2){
    //Ascent 
    LED_Blink();
    Read_BMP();
    Read_IMU();
    Read_GPS();
    TelemertyLoop();

    if(altitude > 20000 && velocity > 0){
      state = 3;
      Update_State();
      fallprevioustime = millis();
      previous_alt = current_alt;
    }

  } else if (state == 3){
    //Stabilization
    LED_Blink();
    Read_BMP();
    Read_IMU();
    Read_GPS();
    TelemertyLoop();

    //actual stabilization; maybe put in function 
    Read_Ang_Velocity();
    if(angular_velocity >= 8){
      digitalWrite(solenoid_left, HIGH); //CC
    } else if ((angular_velocity <= 1) && (angular_velocity > 0)){
      digitalWrite(solenoid_left, LOW);
    }
  
    if (angular_velocity <= (0-8)){
      digitalWrite(solenoid_right, HIGH);
   } else if((angular_velocity > (0-1)) && (angular_velocity < 0)){
      digitalWrite(solenoid_right, LOW);
    }
    
    //get out conditions
    if(millis() - fallprevioustime > falltime){
      //go to descent
      if(previous_alt > current_alt){
        state = 4;
        Update_State();
      }
    } else{
        fallprevioustime = millis();
        previous_alt = current_alt;
    }

    if (velocity < 0 || altitude < 18000){
      //goes to descent 
      state = 4;
      Update_State();
    }

  } else if (state == 4){
      //Descent
      LED_Blink();
      Read_BMP();
      Read_IMU();
      Read_GPS();
      TelemertyLoop();

    if(altitude < 1000 && velocity < 1){
      //goes to landing
      state = 5;
      Update_State();
    }

  } else if (state == 5){
    //Landing
    LED_Blink();
  }

}









//function to loop
void TelemertyLoop()
{
  MissionTime = (millis() / 1000);

  File dataFile = SD.open("telemetry.csv", FILE_WRITE);
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
  Serial.print(temperature);
}



void Read_IMU()
{
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    accel_x = acc.x();
    accel_y = acc.y();
    accel_z = acc.z();
    gyro_x = gyro.x();
    gyro_y = gyro.y();
    gyro_z = gyro.z();
}


void Update_Altitude()
{
  time2 = (millis() /1000);    
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  velocity = ((altitude - previous_alt) / (time2 - time1));
  previous_alt = altitude;
  time1 = time2;
}


void Read_Ang_Velocity() //basically same fucntion as Read_IMU so maybe combine?
{
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    angular_velocity = gyro.z();  
}


void Read_GPS() //reads GPS
{
    gps_lat = myGPS.getLatitude();
    gps_long = myGPS.getLongitude();
    gps_alt = myGPS.getAltitude();
    UTCtime = (String(myGPS.getYear()) + "-" + String(myGPS.getMonth()) + 
    "-" + String(myGPS.getDay()) + ":" + String(myGPS.getHour()) + ":" + 
    String(myGPS.getMinute()) + ":" + String(myGPS.getSecond()));
}



void Update_State() //updates title; doesn't switch state
{
  if(state == 0){
    payload_state = "Launch";
  } else if(state == 1){
    payload_state = "Launch";
  } else if(state == 2){
    payload_state = "Ascent";
  } else if(state == 3){
    payload_state = "Stabilization";
  } else if(state == 4){
    payload_state = "Descent";
  } else if(state == 5){
    payload_state = "Landing";
  } else{
    payload_state = "unknown";
  }

  Serial.println(payload_state);
}


void Logger_Error() { //if SD no work, LED blink slowwww
  if (!SD.begin(chipSelect)) {
   blinkrate = 2000;
  }
}


void BMP_Error() { //if sensors not detected, LED blink fastttt
 if (!bmp.begin_I2C()) {
    blinkrate = 200;
 }
}


void IMU_Error (){
  if (!bno.begin()){
    blinkrate = 500; 
  }
}


void Sample_Timer() { //LETS DO THIS
  currentMillis = millis();

  if (currentMillis - previousMillis >= sesnor_interval) {
    previousMillis = currentMillis;
    Read_BMP();
    Read_IMU();
    Read_GPS();
  }
}

