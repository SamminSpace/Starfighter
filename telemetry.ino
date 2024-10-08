#include <Wire.h> //I2C
#include <SPI.h> //OLED


//TELEMETRY 
String TeamID = "Starfighter";
String payload_state = "launch";
float MissionTime = 0; 
float UTCtime = 0; 
int Packetnum = 0;


//variables
int temperature = 0;
float altitude = 0;
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
//int int BMP_sensor = 19;
//int output_pin = 18;


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  //pinMode(BMP_sensor, INPUT); 
  //pinMode(output_pin, OUTPUT)
  
 //sets up headers for telemetry 
  Serial1.println("Team ID, Mission Time, UTC Time, Packet Count, "
  "Payload State, Altitude, Temperature, XAccerlation, YAcceleration, "
  "ZAccerlation, XGyroscope, YGyroscope, ZGyroscope, GPS Lattitude, "
  "GPS Longitude, GPS Altitude");
  
  /* ok so for right now I will just have the variables in, but make sure for the 
  time and things that change get updated */
    Serial1.println("Starting telemetry loop.");
    Serial1.println(TeamID + ", " + MissionTime + ", " + UTCtime + 
    ", " + Packetnum + ", " + payload_state + ", " + altitude + ", "
    + temperature + ", " + accel_x + ", " + accel_y + ", " + accel_z +
    ", " + gyro_x + ", " + gyro_y + ", " + gyro_z + ", " + gps_lat + 
    ", " + gps_long + ", " + gps_alt);
    Serial1.print("DONE");
}




void loop() {
// here is the loop
}
