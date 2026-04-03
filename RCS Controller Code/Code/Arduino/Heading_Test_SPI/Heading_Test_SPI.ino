#include <SPI.h>
#include <Adafruit_LSM6DSOX.h>
#include <MadgwickAHRS.h>

#define PIN_CS 14

Adafruit_LSM6DSOX lsm6ds;
Madgwick filter;

float roll, pitch, yaw, accel_X_bias = 0, accel_Y_bias = 0, accel_Z_bias = 0, gyro_X_bias = 0, gyro_Y_bias = 0, gyro_Z_bias = 0;
const float sensorRate = 104; // IMU sample rate in Hz 
const int SPI_rate = 50000; // defined in Hz

void setup() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH); // do this for clean data transfers (I don't quite understand why, but this can help maintain a clean signal if done before initializing SPI protocol)

  Serial.begin(115200);
  while (!Serial);

  // Pass SPI object and CS pin to sensor
  if (!lsm6ds.begin_SPI(PIN_CS, &SPI, SPI_rate)) { // SPI is defined in the standard SPI library
    Serial.println("Failed to initialize LSM6DSOX over SPI");
    while (1) delay(10);
  }

  Serial.println("LSM6DSOX SPI initialized");

  lsm6ds_data_rate_t dataRate; // set sensor data rate based on single user input
  switch ((int)sensorRate) {
    case 12:
      dataRate = LSM6DS_RATE_12_5_HZ;
      Serial.println("IMU data rate set to 12.5 Hz");
      break;
    case 26:
      dataRate = LSM6DS_RATE_52_HZ;
      Serial.println("IMU data rate set to 26 Hz");
      break;
    case 104:
      dataRate = LSM6DS_RATE_104_HZ;
      Serial.println("IMU data rate set to 104 Hz");
      break;
    case 833:
      dataRate = LSM6DS_RATE_833_HZ;
      Serial.println("IMU data rate set to 833 Hz");
      break;
    case 1660:
      dataRate = LSM6DS_RATE_1_66K_HZ;
      Serial.println("IMU data rate set to 1660 Hz");
      break;
    default:
      Serial.println("INVALID IMU DATA RATE");
  }

  // Configure IMU settings (adjust as needed)
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  lsm6ds.setAccelDataRate(dataRate);
  lsm6ds.setGyroDataRate(dataRate);

  
  Serial.println("IMU SETTLING IN PROGRESS"); // The IMU apparently requires a few seconds to "settle" to its true bias values. Do this before sampling.
  delay(2000);
  Serial.println("SAMPLING BIAS --- ENSURE DEVICE IS ABSOLUTELY STATIONARY");
  sensors_event_t accel, gyro, temp;
  const int samples = 1000;
  float sum_gyroX = 0, sum_gyroY = 0, sum_gyroZ = 0, sum_accelX = 0, sum_accelY = 0, sum_accelZ = 0;
  for (int i = 0; i < samples; i++) {
      lsm6ds.getEvent(&accel, &gyro, &temp);
      sum_accelX += accel.acceleration.x;
      sum_accelY += accel.acceleration.y;
      sum_accelZ += accel.acceleration.z;
      sum_gyroX += gyro.gyro.x;
      sum_gyroY += gyro.gyro.y;
      sum_gyroZ += gyro.gyro.z;
      delay(5);
  }
  accel_X_bias = sum_accelX / samples;
  accel_Y_bias = sum_accelY / samples;
  accel_Z_bias = (sum_accelZ / samples) - 9.81; // account for gravity
  gyro_X_bias = sum_gyroX / samples;
  gyro_Y_bias = sum_gyroY / samples;
  gyro_Z_bias = sum_gyroZ / samples;
  Serial.print("Accel X bias: ");
  Serial.print(accel_X_bias, 6);
  Serial.print(" Accel Y bias: ");
  Serial.print(accel_Y_bias, 6);
  Serial.print(" Accel Z bias: ");
  Serial.println(accel_Z_bias, 6);
  Serial.print("Gyro X bias: ");
  Serial.print(gyro_X_bias, 6);
  Serial.print(" Gyro Y bias: ");
  Serial.print(gyro_Y_bias, 6);
  Serial.print(" Gyro Z bias: ");
  Serial.println(gyro_Z_bias, 6);

  // Initialize Madgwick filter
  filter.begin(sensorRate);
}

void loop() {
  // Get IMU data
  static sensors_event_t accel, gyro, temp;
  static float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
  lsm6ds.getEvent(&accel, &gyro, &temp);

  // Convert to units expected by Madgwick (accel: g, gyro: deg/s)
  accelX = (accel.acceleration.x - accel_X_bias) / 9.81; // Convert m/s^2 to g
  accelY = (accel.acceleration.y - accel_Y_bias) / 9.81;
  accelZ = (accel.acceleration.z - accel_Z_bias) / 9.81;
  gyroX = (gyro.gyro.x - gyro_X_bias) * 180.0 / PI; // Convert rad/s to deg/s
  gyroY = (gyro.gyro.y - gyro_Y_bias) * 180.0 / PI;
  gyroZ = (gyro.gyro.z - gyro_Z_bias) * 180.0 / PI;

  filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);

  // Get Euler angles
  // roll = filter.getRoll();
  // pitch = filter.getPitch();
  yaw = filter.getYaw();

  // Print heading (yaw in degrees, 0-360)
  Serial.print("Heading: ");
  Serial.print(yaw);
  Serial.println(" degrees");

  delay(1000 / sensorRate); // Match the sensor rate
}
