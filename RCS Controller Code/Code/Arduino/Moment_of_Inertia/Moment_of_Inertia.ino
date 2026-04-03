#include <Adafruit_LSM6DSOX.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
// #include <Adafruit_LSM6DSOX.h> // Uncomment if using LSM6DSOX

#define PI 3.1415926535897932384626433832795
#define CW_SOLENOID_PIN 27 // provides impulse such that the rocket rotates in the clockwise direction
#define CCW_SOLENOID_PIN 33 // provides impulse such that the rocket rotates in the counter-clockwise direction

// const char* ssid = "CenturyLink1484";
// const char* password = "jfy5yxqf8jdb6t";

const char* ssid = "ALIENWARE1";
const char* password = "alienware";

// IPAddress laptopIP(192, 168, 137, 1); // ← replace with your laptop's IP
const unsigned int laptopPort = 1235; // ← use a different port than the receiving one

float Kp = 0, Ki = 0, Kd = 0;
int angle_deadband = 2, control_enable = 0;

// UDP settings
WiFiUDP udp;
const unsigned int udpPort = 1234;

// declared globally for access in setup and loop
Adafruit_LSM6DSOX sox;
float gyro_bias = 0;

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(1500);

  pinMode(CW_SOLENOID_PIN, OUTPUT); // connect solenoid pins to output
  pinMode(CCW_SOLENOID_PIN, OUTPUT);
  digitalWrite(CW_SOLENOID_PIN, LOW); // ensure pins are pulled down on initialization
  digitalWrite(CCW_SOLENOID_PIN, LOW);

  if (!sox.begin_I2C()) {
    Serial.println("Failed to initialize LSM6DSOX!");
    ESP.restart(); // Software reset
  }

  Serial.println("LSM6DSOX initialized successfully");
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

  // Connect to WiFi
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

  IPAddress laptopIP = WiFi.gatewayIP();  // <-- Auto-detect laptop IP
  Serial.print("Laptop IP (Gateway): ");
  Serial.println(laptopIP);

  // Start UDP listener
  udp.begin(udpPort);

  /*
  Gather results and send back to PC:
  */

  int valveOpenTime = 2000;
  float final_angular_velocity, gyro_read;

  digitalWrite(CW_SOLENOID_PIN, HIGH); // open valve
  delay(valveOpenTime);
  digitalWrite(CW_SOLENOID_PIN, LOW); // close valve

  sox.getEvent(&accel, &gyro, &temp); // get angular velocity
  gyro_read = gyro.gyro.z; // save to temporary variable for readability. Sensor value read in radians/s
  final_angular_velocity = gyro_read-gyro_bias;

  char resultBuffer[64];  // Declare as char[] for compatibility with snprintf
  snprintf(resultBuffer, sizeof(resultBuffer), "Radians/s: %.8f", final_angular_velocity);

  udp.beginPacket(laptopIP, laptopPort);
  udp.write((const uint8_t*)resultBuffer, strlen(resultBuffer));  // Cast only here
  udp.endPacket();

  Serial.println("Sent data back to laptop.");
}

void loop() {
  // static sensors_event_t accel, gyro, temp; // IMU variables
  // static float current_angular_velocity, target_angular_velocity = 0/180*PI, gyro_read; // radians/s
  // static float deadband = angle_deadband/180.0*PI; // convert deadband to radians
  // static bool control_active = false;
  // static char incomingPacket[64];
  // static int packetSize;

  // packetSize = udp.parsePacket();
  // if (packetSize) {
  //   if (packetSize >= sizeof(incomingPacket)) {
  //     Serial.println("Packet too large!");
  //     return;
  //   }

  //   int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
  //   incomingPacket[len] = '\0';

  //   Serial.print("Received: ");
  //   Serial.println(incomingPacket);

  //   // Parse comma-separated values
  //   char* token = strtok(incomingPacket, ",");
  //   if (token) control_enable = atoi(token);
  //   token = strtok(NULL, ",");
  //   if (token){
  //     angle_deadband = atoi(token); // convert to Radians
  //     deadband = angle_deadband / 180.0 * PI; // convert for internal use with IMU readings, but keep original for printing new input
  //   }
  //   token = strtok(NULL, ",");
  //   if (token) Kp = atof(token);
  //   token = strtok(NULL, ",");
  //   if (token) Ki = atof(token);
  //   token = strtok(NULL, ",");
  //   if (token) Kd = atof(token);

  //   Serial.printf("PID: control_enable=%d, angle_deadband=%d, Kp=%.5f, Ki=%.5f, Kd=%.5f\n", control_enable, angle_deadband, Kp, Ki, Kd);
  // }

  // if (control_enable){ // only enter control loop if set to active
  //   sox.getEvent(&accel, &gyro, &temp); // get angular velocity
  //   gyro_read = gyro.gyro.z;
  //   current_angular_velocity = gyro_read-gyro_bias; // save to temporary variable for readability. Sensor value read in radians/s

  //   if ((current_angular_velocity-target_angular_velocity) < -deadband){
  //       digitalWrite(CCW_SOLENOID_PIN, HIGH);
  //       digitalWrite(CW_SOLENOID_PIN, LOW);
  //   }
  //   else if ((current_angular_velocity-target_angular_velocity) > deadband){
  //       digitalWrite(CW_SOLENOID_PIN, HIGH);
  //       digitalWrite(CCW_SOLENOID_PIN, LOW);
  //   }
  //   else{
  //       digitalWrite(CCW_SOLENOID_PIN, LOW);
  //       digitalWrite(CW_SOLENOID_PIN, LOW);
  //   }
  // }
  // else{
  //   digitalWrite(CCW_SOLENOID_PIN, LOW);
  //   digitalWrite(CW_SOLENOID_PIN, LOW);
  //   // integral = 0; // reset integral when disabled
  // }
}