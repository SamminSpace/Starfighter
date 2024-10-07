
int solenoid1 = 3;
int solenoid2 = 5;

void setup() {
  pinMode(solenoid1, OUTPUT);
  pinMode(solenoid2, OUTPUT);

  digitalWrite(solenoid1, LOW); //starts closed
  digitalWrite(solenoid2, LOW); 
}

void loop() {
  Serial.print("Solenoid 1: ");
  Serial.println(digitalRead(solenoid1));
  digitalWrite(solenoid1, HIGH);  

  Serial.print("Solenoid 2: ");
  Serial.println(digitalRead(solenoid2));
  digitalWrite(solenoid2, HIGH);  
  delay(3000);

  Serial.println("Solenoid 1: ");
  Serial.println(digitalRead(solenoid1));
  digitalWrite(solenoid1, LOW);

  Serial.println("Solenoid 2: ");
  Serial.println(digitalRead(solenoid2));
  digitalWrite(solenoid2, LOW);
  delay(3000);
}
