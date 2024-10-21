#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

int gyro_x = 0;

// Create an instance of the BNO055 class
Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
    Serial.begin(9600); // Start serial communication for debugging
    Wire.begin(); // Initialize I2C communication

    // Initialize the BNO055
    if (!bno.begin()) {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    // Set the BNO055 to normal mode
    bno.setExtCrystalUse(true);
}

void loop() {
    // Variables to hold the sensor data
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    // Print the data to the Serial Monitor
    Serial.print("Accelerometer: ");
    Serial.print("X: "); Serial.print(acc.x());
    Serial.print(" | Y: "); Serial.print(acc.y());
    Serial.print(" | Z: "); Serial.println(acc.z());

    Serial.print("Gyroscope: ");
    Serial.print("X: "); Serial.print(gyro.x());
    gyro_x = gyro.x();
    Serial.print(" | Y: "); Serial.print(gyro.y());
    Serial.print(" | Z: "); Serial.println(gyro.z());

    Serial.print("Magnetometer: ");
    Serial.print("X: "); Serial.print(mag.x());
    Serial.print(" | Y: "); Serial.print(mag.y());
    Serial.print(" | Z: "); Serial.println(mag.z());

    Serial.print("Euler Angles: ");
    Serial.print("Heading: "); Serial.print(euler.x());
    Serial.print(" | Roll: "); Serial.print(euler.y());
    Serial.print(" | Pitch: "); Serial.println(euler.z());

    Serial.print("Linear Acceleration: ");
    Serial.print("X: "); Serial.print(linacc.x());
    Serial.print(" | Y: "); Serial.print(linacc.y());
    Serial.print(" | Z: "); Serial.println(linacc.z());

    Serial.print("Gravity Vector: ");
    Serial.print("X: "); Serial.print(grav.x());
    Serial.print(" | Y: "); Serial.print(grav.y());
    Serial.print(" | Z: "); Serial.println(grav.z());

    // Delay for a short period to avoid flooding the Serial Monitor
    delay(500); // Adjust as necessary
}
