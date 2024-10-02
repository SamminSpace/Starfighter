#include <SD.h> //SD library
#include <Wire.h> //I2C
#include <SPI.h> //OLED
using namespace std; 

//TELEMETRY 
String TeamID = "Starfighter";
String payload_state = "launch";
float MissionTime = 0; 
float UTCtime = 0; 
int Packetnum = 0;


//variables
int temperature = 0;
float altitude = 0; //Rori is right :(
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;
float gps_lat = 0;
float gps_long = 0;
float gps_alt = 0;


//set up the pins
int SDcard = 4;
int BMPpin = 8;

//THE FILE
File telemetry;



void setup() {
  Serial.begin (9600);
  pinMode(SDcard, OUTPUT);
  pinMode(BMPpin, INPUT);
  
  Serial.print("Initializing SD card...");
  while (!Serial) { 
    ; //wait for serial port to connect
  }
  
 // if SD card is not in right pin 
  if (!SD.begin(SDcard)){
  Serial.println("Could not initialize SD card."); 
}
//if SD is in right pin
  Serial.print("initialization done!");

//sets up headers for telemetry 
  telemetry = SD.open("telemetry.txt", FILE_WRITE);
  Serial.println("Team ID, Mission Time, UTC Time, Packet Count, "
  "Payload State, Altitude, Temperature, XAccerlation, YAcceleration, "
  "ZAccerlation, XGyroscope, YGyroscope, ZGyroscope, GPS Lattitude, "
  "GPS Longitude, GPS Altitude");
  
  /* ok so for right now I will just have the variables in, but make sure for the 
  time and things that change get updated */
  if (telemetry){
    Serial.println("Starting telemetry loop.");
    telemetry.println(TeamID + ", " + MissionTime + ", " + UTCtime + 
    ", " + Packetnum + ", " + payload_state + ", " + altitude + ", "
    + temperature + ", " + accel_x + ", " + accel_y + ", " + accel_z +
    ", " + gyro_x + ", " + gyro_y + ", " + gyro_z + ", " + gps_lat + 
    ", " + gps_long + ", " + gps_alt);
    telemetry.close(); 
    Serial.print("DONE");
  } else{
    Serial.print("File could not open due to error.");
  }
}




void loop() {
  Packetnum = packetnum + 1
  /* functions read sensors 
  checking alt > 1 km  (switch to acension)
  check alt > 20 km (switch on stabilization)
  check alt > 1 AND moving negtive (descent)
  check alt < 1 AND Moving neg (landing)
  check velocity = 0 (touchdown)  */
 
//then read sensors and update variables, send back into the telemetry 
}
