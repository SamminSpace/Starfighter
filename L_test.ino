// Define the pin number for the LED
const int ledPin = 2; // Change to your LED pin if using an external LED

// Define the blink interval in milliseconds
const int blinkInterval = 1000; // 1000 ms = 1 second

void setup() {
    // Initialize the LED pin as an output
    pinMode(ledPin, OUTPUT);
}

void loop() {
    // Turn the LED on
    digitalWrite(ledPin, HIGH);
    // Wait for the specified interval
    delay(blinkInterval);
    // Turn the LED off
    digitalWrite(ledPin, LOW);
    // Wait for the specified interval
    delay(blinkInterval);
}
