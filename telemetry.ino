#include <Wire.h> //I2C
#include <SPI.h> //OLED
#include <SD.h> 

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
float velocity = 0;


//set up the pins
const int chipSelect = 10;
int BMP_sensor = 19;
int output_pin = 18;

//SD CARD
File dataFile; 

void setup() {
  Serial.begin(9600);
  pinMode(BMP_sensor, INPUT);
  pinMode(output_pin, OUTPUT);
  
  
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
  File dataFile = SD.open("telemetry.txt", FILE_WRITE);
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
  
}



void loop() {
  TelemertyLoop ();
  Update_State (); //checks altitude and updates the state machine

  while (payload_state == "Ascenion"){
    Serial.print("We going up");
    TelemertyLoop ();
    Update_State (); 
  }

  while (payload_state == "Stabilization"){
    Serial.print("Do stabilization things");
    TelemertyLoop ();
    Update_State (); 
  }

  while (payload_state == "Decent"){
    Serial.print("Going down");
    TelemertyLoop ();
    Update_State (); 
  }

  while (payload_state == "Landing" ){
    Serial.print("Payload has sucessfully landed");
    Serial.print("no need to collect data");
  }
}



//function to loop
void TelemertyLoop ()
{
  MissionTime = (millis() / 1000);

  File dataFile = SD.open("telemetry.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(TeamID + ", " + MissionTime + ", " + UTCtime + 
    ", " + Packetnum + ", " + payload_state + ", " + altitude + ", "
    + temperature + ", " + accel_x + ", " + accel_y + ", " + accel_z +
    ", " + gyro_x + ", " + gyro_y + ", " + gyro_z + ", " + gps_lat + 
    ", " + gps_long + ", " + gps_alt);
    dataFile.close();
  }

  Packetnum = Packetnum + 1;
}


void Update_State ()
{
  if (altitude >= 20000){
    payload_state = "Stabilization"; 
  }
  else if (altitude >= 1){
    payload_state = "Asencsion"; 
  }
  else if ((altitude <=20000) && (velocity < 0)){
    payload_state = "Descent"; 
  }
  else if (altitude <= 1 && (velocity < 0)){
    payload_state = "Landing"; 
  }
}

