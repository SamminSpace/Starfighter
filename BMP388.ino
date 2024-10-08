#include <SD.h> //SD library
#include <Wire.h> //I2C
#include <SPI.h> //OLED
#include <107-Arduino-BMP388.h> // the actual BMP library
#include "Adafruit_BMP3XX.h"

//BMP3XX
float SEALEVELPRESSURE_HPA = 1013.25;
Adafruit_BMP3XX bmp;

// all pin set ups
int SD_card = 4;
int BMP_sensor = 4;
int output_pin = 8;

//SD card
File telemetry;


void setup() {
  Serial.begin (9600);
  pinMode(BMP_sensor, INPUT);
  pinMode(output_pin, OUTPUT);
  pinMode(SD_card, OUTPUT);

  if (!bmp.begin_I2C()) {  // this sets up I2C
    Serial.println("Could not find BMP3 sensor."); 
    while (1);
  }

 Serial.print("Initializing SD card...");
  while (!Serial) { 
    ; //wait for serial port to connect
  }
  if (!SD.begin(SDcard)){  // if SD card is not in right pin 
  Serial.println("Could not initialize SD card."); 
}
//if SD is in right pin
  Serial.print("initialization done!");

}

void writeFile() {  //writes to SD Card
  telemetry = SD.open("BMP.txt", FILE_WRITE); //opens the file 
  if (telemetry) {
    telemetry.print(bmp.temperature);
    telemetry.print(bmp.pressure);
    telemetry.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    telemetry.close();
  } else {
      Serial.println("Could not open file.");
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

  Serial.print("Pressure: ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Aprox Altitude: ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.println(" m");

  Serial.println();
  /* make sure after the test that the format matches telemetry
  this is for testing to make sure it works */

  writeFile();
  delay(3000);
}


