#include <SD.h> //SD library
#include <Wire.h> //I2C
#include <SPI.h> //OLED
#include <107-Arduino-BMP388.h> // the actual BMP library
#include "Adafruit_BMP3XX.h"

//BMP3XX
float SEALEVELPRESSURE_HPA = 1013.25;
Adafruit_BMP3XX bmp;

// all pin set ups
int BMP_sensor = 19;
int output_pin = 18;




void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); //check buad rate
  pinMode(BMP_sensor, INPUT);
  pinMode(output_pin, OUTPUT);

  if (!bmp.begin_I2C()) {  // this sets up I2C
    Serial.println("Could not find BMP3 sensor."); 
    while (1);
  }

}


void writeFile() {  //writes to SD Card
  Serial1.print("Tempertaure: ");
  Serial1.print(bmp.temperature);
  Serial1.println(" *C");

  Serial1.print("Pressure: ");
  Serial1.print(bmp.pressure / 100.0);
  Serial1.println(" hPa");

  Serial1.print("Aprox Altitude: ");
  Serial1.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial1.println(" m");
  }
}

void loop() {
/* if (!bmp.performReading()) {
   Serial.println("Failed to read.");
   return;
 } */
  
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


