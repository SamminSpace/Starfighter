#include <SD.h> //SD library
#include <Wire.h> //I2C
#include <SPI.h> //OLED
#include <107-Arduino-BMP388.h>

//BMP3XX


//variables needing to be defined
float celsius = 0;
float farenheight = 0;
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


void readTemp() //read sensor
{
  int analogBit = analogRead(output_pin); //read sesnor and store in variable

  celsius = analogBit;
  farenheight = (celsius * 9 /5 + 32);
  Serial.print(celsius);
  Serial.println(" C, ");
  Serial.print(farenheight); 
  Serial.println(" F"); 
  Serial.println(bmp388.io().read(BMP_sensor), HEX);
}

void writeFile() //writes to SD Card
{
  myFile = SD.open("temp.txt", FILE_WRITE); //opens the file 
  if (myFile) {
    myFile.println(celsius, 2);
    myFile.close();
    Serial.print(celsius, 2); //for debugging to see if writes to serial

  } else {
      Serial.println("Could not open file.");
  }
}



void loop() {
readTemp(); 
writeFile();
delay(3000);
}
