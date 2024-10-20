#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <107-Arduino-BMP388.h>
#include "Adafruit_BMP3XX.h"

//TELEMETRY 
String TeamID = "Starfighter";
String payload_state = "launch";
float MissionTime = 0; 
float UTCtime = 0; 
int Packetnum = 0;


//variables
float temperature = 0;
float pressure = 0;
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

//BMP3XX
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;

//File 
File dataFile;


void setup()
{
  pinMode(BMP_sensor, INPUT);
  pinMode(output_pin, OUTPUT);
  Serial.begin(9600); //this might need to change 152500 or whtvr
  while (!Serial) {
    ; // wait for serial port to connect.
  }


  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
  Serial.println("card initialized.");


  if (!bmp.begin_I2C()) {
    // this sets up I2C
    Serial.println("Could not find BMP3 sensor.");
     while (1); 
  }

  // Set up oversampling and filter initialization (tbh IDK)
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
}




void loop()
{
  Serial.print("Temperature: ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");
  temperature = bmp.temperature; 

  Serial.print("Pressure: ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");
  pressure = (bmp.pressure / 100); 

  Serial.print("Aprox Altitude: ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  Serial.println();

 TelemertyLoop();
 delay(1000);
}




void TelemertyLoop ()
{
  MissionTime = (millis() / 1000); //get time before send into telemtry

  File dataFile = SD.open("BMP.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(TeamID + ", " + MissionTime + ", " + UTCtime + 
    ", " + Packetnum + ", " + payload_state + ", " + altitude + ", "
    + temperature + ", " + pressure);
    dataFile.close();
  }

  Packetnum = Packetnum + 1;
}



