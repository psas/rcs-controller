#define CW_PIN 33
#define CCW_PIN 27

// #include <WiFi.h>
// #include <ArduinoOTA.h>
// #include <Adafruit_LSM6DSOX.h> // Uncomment if using LSM6DSOX

// const char* ssid = "CenturyLink1484";
// const char* password = "jfy5yxqf8jdb6t";


const char* ssid = "ALIENWARE1";
const char* password = "alienware";

// const char* ssid = "Sam's Galaxy A53 5G";
// const char* password = "xazp6526";


const int ledPin = 33;
float Kp = 0, Ki = 0, Kd = 0;
int angle_deadzone = 2, LED_enable = 0;

// UDP settings
// WiFiUDP udp;
// const unsigned int udpPort = 1234;

void setup() {
  Serial.begin(115200);
  delay(1500);

  // Initialize LED pin
  pinMode(CW_PIN, OUTPUT);
  pinMode(CCW_PIN, OUTPUT);
  digitalWrite(CW_PIN, LOW); // LED off initially
  digitalWrite(CCW_PIN, LOW); // LED off initially

  // // Connect to WiFi
  // Serial.print("Connecting to ");
  // Serial.println(ssid);
  // WiFi.begin(ssid, password);

  // int attempts = 0;
  // while (WiFi.status() != WL_CONNECTED && attempts < 20) {
  //   delay(500);
  //   Serial.print(".");
  //   attempts++;
  // }

  // if (WiFi.status() != WL_CONNECTED) {
  //   Serial.println("\nWiFi connection failed! Rebooting...");
  //   ESP.restart();
  // }

  // Serial.println("\nWiFi connected.");
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());

  // // Start UDP listener
  // udp.begin(udpPort);
  // Serial.print("UDP listening on port ");
  // Serial.println(udpPort);
}

void loop() {

  // static int packetSize;
  // static char incomingPacket[64];

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
  //   if (token) LED_enable = atoi(token);
  //   token = strtok(NULL, ",");
  //   if (token) angle_deadzone = atoi(token);
  //   token = strtok(NULL, ",");
  //   if (token) Kp = atof(token);
  //   token = strtok(NULL, ",");
  //   if (token) Ki = atof(token);
  //   token = strtok(NULL, ",");
  //   if (token) Kd = atof(token);

  //   Serial.printf("PID: enable=%d, angle_deadzone=%d, Kp=%.5f, Ki=%.5f, Kd=%.5f\n", LED_enable, angle_deadzone, Kp, Ki, Kd);
  // }
  
  digitalWrite(CW_PIN, HIGH); // LED off initially
  digitalWrite(CCW_PIN, LOW); // LED off initially

  delay(1000);

  digitalWrite(CW_PIN, LOW); // LED off initially
  digitalWrite(CCW_PIN, LOW); // LED off initially

  delay(500);

  digitalWrite(CW_PIN, LOW); // LED off initially
  digitalWrite(CCW_PIN, HIGH); // LED off initially

  delay(1000);

  digitalWrite(CW_PIN, LOW); // LED off initially
  digitalWrite(CCW_PIN, LOW); // LED off initially

  delay(500);



}
