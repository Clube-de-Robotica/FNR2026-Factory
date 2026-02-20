#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

// Create sensor objects
LSM6 imu;        // Accelerometer + Gyroscope
LIS3MDL mag;     // Magnetometer

// Sampling rate control
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 20; // 50Hz sampling rate (adjustable)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for serial port to connect
  }
  
  Serial.println("Pololu MinIMU-9 v5 Data Transmitter");
  Serial.println("Initializing...");

  // Initialize I2C communication
  Wire.begin();
  
  // Initialize LSM6DS33 (Accelerometer + Gyroscope)
  if (!imu.init()) {
    Serial.println("Failed to detect LSM6DS33 (accelerometer + gyroscope)");
    while (1) {
      delay(10);
    }
  }
  imu.enableDefault();
  Serial.println("LSM6DS33 initialized!");
  
  // Initialize LIS3MDL (Magnetometer)
  if (!mag.init()) {
    Serial.println("Failed to detect LIS3MDL (magnetometer)");
    while (1) {
      delay(10);
    }
  }
  mag.enableDefault();
  Serial.println("LIS3MDL initialized!");

  Serial.println();
  Serial.println("Sensor Configuration:");
  Serial.println("LSM6DS33 (Accel + Gyro):");
  Serial.println("  Accelerometer: ±2g full scale");
  Serial.println("  Gyroscope: ±245 dps full scale");
  Serial.println("  Output Data Rate: 1.66 kHz");
  Serial.println("LIS3MDL (Magnetometer):");
  Serial.println("  Full scale: ±4 gauss");
  Serial.println("  Output Data Rate: 80 Hz");
  Serial.println();

  Serial.println("Starting data transmission...");
  Serial.println("Format: TIME,AX,AY,AZ,GX,GY,GZ,MX,MY,MZ");
  delay(100);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check if it's time to sample
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;
    
    // Read sensor data
    imu.read();
    mag.read();

    // Transmit data in CSV format
    // Format: timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z
    Serial.print(currentTime);
    Serial.print(",");
    
    // Accelerometer data (raw values)
    // LSM6DS33: ±2g scale, 16-bit signed
    // To convert to m/s²: (raw / 16384.0) * 9.80665
    Serial.print(imu.a.x);
    Serial.print(",");
    Serial.print(imu.a.y);
    Serial.print(",");
    Serial.print(imu.a.z);
    Serial.print(",");
    
    // Gyroscope data (raw values)
    // LSM6DS33: ±245 dps scale, 16-bit signed
    // To convert to dps: raw / 114.29
    // To convert to rad/s: (raw / 114.29) * (PI / 180)
    Serial.print(imu.g.x);
    Serial.print(",");
    Serial.print(imu.g.y);
    Serial.print(",");
    Serial.print(imu.g.z);
    Serial.print(",");
    
    // Magnetometer data (raw values)
    // LIS3MDL: ±4 gauss scale, 16-bit signed
    // To convert to gauss: raw / 6842.0
    Serial.print(mag.m.x);
    Serial.print(",");
    Serial.print(mag.m.y);
    Serial.print(",");
    Serial.println(mag.m.z);
  }
}