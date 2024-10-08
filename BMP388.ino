#include <SD.h> //SD library
#include <Wire.h> //I2C
#include <SPI.h> //OLED
#include <107-Arduino-BMP388.h> // the actual BMP library
#include "Adafruit_BMP3XX.h"

//BMP3XX
float SEALEVELPRESSURE_HPA = 1013.25;
Adafruit_BMP3XX bmp;

//variables needing to be defined
float temperature = 0;
float altitude = 0;
float pressure = 0;

// all pin set ups
int SD_card = 4;
int BMP_sensor = 4;
int output_pin = 8;

File myFile;

void setup() {
  Serial.begin (9600);
  pinMode(BMP_sensor, INPUT);
  pinMode(output_pin, OUTPUT);
  pinMode(SD_card, OUTPUT);

  if (!bmp.begin_I2C()) {
    // this sets up I2C
    Serial.println("Could not find BMP3 sensor."); 
    while (1);
  }

//trying to set up SD
if (!SD.begin(SD_card)){
  Serial.println("Could not initialize SD card.");
}
  
if (SD.exists("file.txt")){
  Serial.println("File exists.");
  if (SD.remove("file.txt") == true){
    Serial.println("Sucessfully removed file.");
  } else {
    Serial.println("Could not remove file.");
  }
}
}


void loop() {
 if (!bmp.performReading()) {
   Serial.println("Failed to read.");
   return;
 }
  
  Serial.print("Tempertaure: ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");
  //bmp.temperature = temperature;

  Serial.print("Pressure: ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Aprox Altitude: ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.println(" m");

  Serial.println();
  /* make sure after the test that the format matches telemetry
  this is for testing to make sure it works */
  
  delay(3000);
}


