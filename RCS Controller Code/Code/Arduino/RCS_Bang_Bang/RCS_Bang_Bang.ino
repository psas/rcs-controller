#include <Adafruit_LSM6DSOX.h>
#include <Arduino.h>

#define PI 3.1415926535897932384626433832795
#define CW_SOLENOID_PIN 14 // provides impulse such that the rocket turns in the clockwise direction
#define CCW_SOLENOID_PIN 32 // provides impulse such that the rocket turns in the counter-clockwise direction
#define DEADBAND 2 // degrees

// declared globally for access in setup and loop
Adafruit_LSM6DSOX sox;
float gyro_bias = 0;

void setup() {
    Serial.begin(115200);
    delay(1500); // wait for connections

    pinMode(CW_SOLENOID_PIN, OUTPUT); // connect solenoid pins to output
    pinMode(CCW_SOLENOID_PIN, OUTPUT);
    digitalWrite(CW_SOLENOID_PIN, LOW); // ensure pins are pulled down on initialization
    digitalWrite(CCW_SOLENOID_PIN, LOW);

    while (!Serial) {
        delay(10); // will pause while connecting to serial
    }

    if (!sox.begin_I2C()) { // check if IMU is communicating with board
        // if (!sox.begin_SPI(LSM_CS)) {
        // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
        Serial.println("Failed to find LSM6DSOX chip");
        while (1) {
            delay(10); // TODO CHANGE THIS TO RESET CHIP, TRY AGAIN, OR THROW ERRORS
        }
    }
    Serial.println("IMU SETTLING IN PROGRESS"); // The IMU apparently requires a few seconds to "settle" to its true bias values. Do this before sampling.
    delay(1000);
    Serial.println("SAMPLING BIAS --- ENSURE DEVICE IS ABSOLUTELY STATIONARY");
    sensors_event_t accel, gyro, temp;
    const int samples = 250;
    float sum = 0;
    for (int i = 0; i < samples; i++) {
        sox.getEvent(&accel, &gyro, &temp);
        sum += gyro.gyro.z;
        delay(5);
    }
    gyro_bias = sum / samples;
    Serial.print("Gyro bias: ");
    Serial.println(gyro_bias, 6);
}

void loop() {
    static sensors_event_t accel, gyro, temp; // IMU variables
    static float current_angular_velocity, target_angular_velocity = 0/180*PI, gyro_read; // radians/s
    static float deadband = DEADBAND/180.0*PI; // convert deadband to radians
    sox.getEvent(&accel, &gyro, &temp); // get angular velocity
    gyro_read = gyro.gyro.z;
    current_angular_velocity = gyro_read-gyro_bias; // save to temporary variable for readability. Sensor value read in radians/s
    Serial.println("HELLO LINE HERE");
    if ((current_angular_velocity-target_angular_velocity) < -deadband){
        digitalWrite(CCW_SOLENOID_PIN, HIGH);
        digitalWrite(CW_SOLENOID_PIN, LOW);
    }
    else if ((current_angular_velocity-target_angular_velocity) > deadband){
        digitalWrite(CW_SOLENOID_PIN, HIGH);
        digitalWrite(CCW_SOLENOID_PIN, LOW);
    }
    else{
        digitalWrite(CCW_SOLENOID_PIN, LOW);
        digitalWrite(CW_SOLENOID_PIN, LOW);
    }
    // Serial.print(" bias: ");
    // Serial.print(gyro_bias, 4);
    // Serial.print(" test: ");
    // Serial.print(test, 4);
    // Serial.print(" Accel: ");
    // Serial.print(current_angular_velocity, 4);
    // Serial.print(" Diff: ");
    // Serial.print((current_angular_velocity-target_angular_velocity));
    // Serial.print("\n"); 
}