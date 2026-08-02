#include <Adafruit_LSM6DSOX.h>
#include <Arduino.h>
#include <WiFi.h>

// Connecting to the wifi
// UDP settings
WiFiUDP udp;
const unsigned int udpPort = 1234;
const char* ssid = "ALIENWARE1";
const char* password = "alienware";

float OGIMUGyroX = 0;
float OGIMUGyroY = 0;
float OGIMUGyroZ = 0;
#define PI 3.1415926535897932384626433832795
#define CW_SOLENOID_PIN 33
#define CCW_SOLENOID_PIN 27

float roll, pitch, yaw, accel_X_bias = 0, accel_Y_bias = 0, accel_Z_bias = 0, gyro_X_bias = 0, gyro_Y_bias = 0, gyro_Z_bias = 0;
const float sensorRate = 208.0;
long start_time = 0;
sensors_event_t accel, gyro, temp;

Adafruit_LSM6DSOX IMU;

void setup() {
	Serial.begin(115200);
	while (!Serial) delay(10); // Wait for Serial to initialize

	pinMode(CW_SOLENOID_PIN, OUTPUT); // connect solenoid pins to output and pull down
	pinMode(CCW_SOLENOID_PIN, OUTPUT);
	digitalWrite(CW_SOLENOID_PIN, LOW);
	digitalWrite(CCW_SOLENOID_PIN, LOW);

	while (Serial.available() < 0 && Serial.read() == "s") {
		delay(10);
	}

	if (!IMU.begin_I2C()) { // check if IMU is communicating with board
		Serial.println("Failed to find LSM6DSOX chip");
		while (1) delay(10); // TODO CHANGE THIS TO RESET CHIP, TRY AGAIN, OR THROW ERRORS
	}
	Serial.println("LSM6DSOX initialized successfully");
	lsm6ds_data_rate_t dataRate; // set sensor data rate based on single user input
	dataRate = LSM6DS_RATE_208_HZ;

	IMU.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
	IMU.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
	IMU.setAccelDataRate(dataRate);
	IMU.setGyroDataRate(dataRate);

	Serial.println("MADE 1");

	Serial.print("Connecting to ");
	Serial.println(ssid);
	WiFi.begin(ssid, password);

	int attempts = 0;
	while (WiFi.status() != WL_CONNECTED && attempts < 20) {
		delay(500);
		Serial.print(".");
		attempts++;
	}

	if (WiFi.status() != WL_CONNECTED) {
		Serial.println("\nWiFi connection failed! Rebooting...");
		ESP.restart();
	}
	Serial.println("\nWiFi connected.");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	udp.begin(udpPort);
	Serial.print("UDP listening on port ");
	Serial.println(udpPort);
		// calibrate IMU bias values (shouldn't be necessary for roll and pitch axes with Madgwick, but doesnt seem to work without???)
	Serial.println("IMU SETTLING IN PROGRESS"); // The IMU requires a few seconds to "settle" to its true bias values. Do this before sampling.
	delay(1000);
	Serial.println("SAMPLING BIAS --- ENSURE DEVICE IS ABSOLUTELY STATIONARY");
	const int samples = 250;
	float sum_gyroX = 0, sum_gyroY = 0, sum_gyroZ = 0, sum_accelX = 0, sum_accelY = 0, sum_accelZ = 0;
	for (int i = 0; i < samples; i++) {
		IMU.getEvent(&accel, &gyro, &temp);
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
		
	IMU.getEvent(&accel, &gyro, &temp);
	OGIMUGyroX = gyro_X_bias;
	OGIMUGyroY = gyro_Y_bias;
	OGIMUGyroZ = gyro_Z_bias;

	pinMode(14, OUTPUT);
	digitalWrite(14, HIGH);
}


float rotatedX = 0, rotatedY = 0, rotatedZ = 0;
float sumX = 0, sumY = 0, sumZ = 0;
int samples = 0;

void loop() {
	if (millis() < 5000) {
		return;
	} 
	
	if (start_time == 0) {
		digitalWrite(CW_SOLENOID_PIN, HIGH);
		start_time = millis();
	}

	IMU.getEvent(&accel, &gyro, &temp);

	sumX += gyro.gyro.x;
	sumY += gyro.gyro.y;
	sumZ += gyro.gyro.z;
	samples += 1;

	rotatedX = sumX / samples;
	rotatedY = sumY / samples;
	rotatedZ = sumZ / samples;

	if (fabs(rotatedX - OGIMUGyroX) >= (PI/36)) {
		auto delay_time = millis() - start_time;
		Serial.print("X Delay Time = ");
		Serial.println(delay_time);
	}

	if (fabs(rotatedY - OGIMUGyroY) >= (PI/36)) {
		auto delay_time = millis() - start_time;
		Serial.print("Y Delay Time = ");
		Serial.println(delay_time);
	}

	if (fabs(rotatedZ - OGIMUGyroZ) >= (PI/36)) {
		auto delay_time = millis() - start_time;
		Serial.print("Z Delay Time = ");
		Serial.println(delay_time);
	}
}