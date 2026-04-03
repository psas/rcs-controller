#include <Adafruit_LSM6DSOX.h>
#include <MadgwickAHRS.h>
#include <Arduino.h>
#include <WiFi.h>

// UDP settings
WiFiUDP udp;
const unsigned int udpPort = 1234;
const char* ssid = "ALIENWARE1";
const char* password = "alienware";

#define PI 3.1415926535897932384626433832795
#define CW_SOLENOID_PIN 33
#define CCW_SOLENOID_PIN 27

#define RCS_F 1
#define RCS_ARM 1
#define KP 0
#define KI 0
#define KD 0
#define CYCLE_TIME 0.1 // PID control loop time (dt) in seconds
#define INTEGRAL_MAX 1
#define INTEGRAL_MIN -1
#define MIN_FIRE 0.06 // minimum fire time in seconds

hw_timer_t *control_timer = NULL, *filter_timer = NULL; // control loop timer
volatile bool controlLoopFlag = false, filterUpdateFlag = false; // determines if the PID should be activated in any given iteration, and when filter should be updated with new IMU values
float roll, pitch, yaw, accel_X_bias = 0, accel_Y_bias = 0, accel_Z_bias = 0, gyro_X_bias = 0, gyro_Y_bias = 0, gyro_Z_bias = 0;
const float sensorRate = 208.0, filterRate = 100.0; // IMU sample rate in Hz

typedef struct { // PID controller structure
    float Kp, Ki, Kd;       // PID gains
    float integral;         // Running sum for integral term
    float prev_error;       // Previous error for derivative term
    float dt;               // Sampling period (seconds)
    float integral_min, integral_max; // Anti-windup limits
    unsigned long thrust_time; // thruster on time (PWM control)
    unsigned long thrust_start_time; // begin of control input is relative to PID calculations
    bool CW_thruster_on = false; // thruster on or off (in order disengage thrust if target reached earlier than expected)
    bool CCW_thruster_on = false;
    int deadband; // maximum heading error within the RCS doesn't need to fire
    int target; // target heading
} PID_data;

void pid_init(PID_data *PID, float kp, float ki, float kd, float cycle_time, float int_max, float int_min){
    PID->Kp = kp;
    PID->Ki = ki;
    PID->Kd = kd;
    PID->integral = 0.0;
    PID->prev_error = 0.0;
    PID->dt = cycle_time;
    PID->integral_max = int_max;
    PID->integral_min = int_min;
    PID->deadband = 10;
    PID->target = 180;
}

void pid_update(PID_data *PID, float kp, float ki, float kd, int deadband, int target_heading){
    PID->Kp = kp;
    PID->Ki = ki;
    PID->Kd = kd;
    PID->deadband = deadband;
    PID->target = target_heading;
}

void IRAM_ATTR controlLoopTimer() { // Interrupt Service Routine (ISR)
  controlLoopFlag = true; // Signal control loop to run
}

void IRAM_ATTR filterUpdateTimer() { // Interrupt Service Routine (ISR)
  filterUpdateFlag = true; // Signal control loop to run
}

float PID_controller(PID_data *PID, float error){
    static float P, I, D;
    
    error = error*PI/180; // convert error to radians (within this function only) for mathematical operations

    P = PID->Kp * error; 
    PID->integral += error * PID->dt; // TODO CONSIDER REFINING dt PRECISION BY ACTUALLY TIMING
    I = PID->Ki * PID->integral;
    D = PID->Kd * (error - PID->prev_error) / PID->dt;
    
    // if (pid->integral > pid->integral_max) pid->integral = pid->integral_max; // ensure integral term doesn't go over max
    // if (pid->integral < pid->integral_min) pid->integral = pid->integral_min; // ensure integral term doesn't go under min

    PID->prev_error = error;
    return P + I + D;
}

void set_thrust(PID_data *PID, float current_heading){
    static float error, tau_output, target_heading = 180; // TODO create logic for changing target heading
    static float tau_rcs = RCS_F * RCS_ARM; // defined by RCS hardware specifications 
    static float duty_cycle, fire_time;

    target_heading = PID->target;
    error = target_heading - current_heading; // error in set and measured heading
    Serial.print("Current Heading: ");
    Serial.print(current_heading);
    Serial.print(" error ");
    Serial.println(error);
    // account for rollover and shortest path
    if (error > 180){
        error -= 360;
    } else if (error < -180){
        error += 360;
    }

    // TODO CONSIDER ERROR SET TO 0 IF WITHIN DEADBAND AND RESETTING INTEGRAL
    if (fabs(error) < PID->deadband) {
        error = 0;
        PID->integral = 0;
    }

    tau_output = PID_controller(PID, error); // calculate desired torque
    duty_cycle = tau_output/tau_rcs; // calculate percentage of duty_cycle for desired torque given RCS output
    if (duty_cycle > 1.0) duty_cycle = 1.0; // saturate in case greater torque required than can be output for a single given duty cycle, thrust commands will rollover into next cycle

    fire_time = duty_cycle*PID->dt*1e6; // convert firing time to microseconds for later comparisons with start time

    // if (fire_time < (MIN_FIRE*1e6)){ // don't fire thrusters if fire time below minimum solenoid actuation time, time converted to microseconds for comparison with firing time
    //     PID->CW_thruster_on = false;
    //     PID->CCW_thruster_on = false;

    //     if (fabs(error) < PID->deadband) { // if angle is within deadband, and thrusters off, we can assume we are in a stable state and can "turn off" the PID controller, requiring a reset of the accumulated error
    //         PID->integral = 0.0;
    //     }
    //     return;
    // }
    
    PID->thrust_time = fire_time; // set fire time for appropriate thruster
    PID->thrust_start_time = micros(); // set beginning of thrust time now, as it is relative to PID calculations
    if (tau_output > 0){
        PID->CW_thruster_on = true; // turn on CW thruster
        PID->CCW_thruster_on = false; // turn off CCW thruster
    } else if (tau_output < 0){
        PID->CW_thruster_on = false; // turn off CCW thruster
        PID->CCW_thruster_on = true; // turn on CW thruster
    }
}

void thrust_controller(PID_data *PID){
    static bool CW_thruster_state = false, CCW_thruster_state = false; // keep track of current states to avoid continually sending redundant digitalWrite commands

    if (CW_thruster_state != PID->CW_thruster_on){ // check if thruster state matches target state
        if (PID->CW_thruster_on && ((micros() - PID->thrust_start_time) < PID->thrust_time)){ // if thruster should be on and firing time not exceeded, activate thruster
            digitalWrite(CW_SOLENOID_PIN, HIGH);
            CW_thruster_state = true; // set current thruster state
        } else { // else thruster should not be on
            digitalWrite(CW_SOLENOID_PIN, LOW);
            CW_thruster_state = false; // set current thruster state
        }
    }

    if (CCW_thruster_state != PID->CCW_thruster_on){ // check if thruster state matches target state
        if (PID->CCW_thruster_on && ((micros() - PID->thrust_start_time) < PID->thrust_time)){ // if thruster should be on and firing time not exceeded, activate thruster
            digitalWrite(CCW_SOLENOID_PIN, HIGH);
            CCW_thruster_state = true; // set current thruster state
        } else { // else thruster should not be on
            digitalWrite(CCW_SOLENOID_PIN, LOW);
            CCW_thruster_state = false; // set current thruster state
        }
    }  
}

// declared globally for access in setup and loop
PID_data PID;
Adafruit_LSM6DSOX IMU;
Madgwick filter;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10); // Wait for Serial to initialize

    pinMode(CW_SOLENOID_PIN, OUTPUT); // connect solenoid pins to output and pull down
    pinMode(CCW_SOLENOID_PIN, OUTPUT);
    digitalWrite(CW_SOLENOID_PIN, LOW);
    digitalWrite(CCW_SOLENOID_PIN, LOW);

    while (!Serial) {
        delay(10); // will pause while connecting to serial
    }

    if (!IMU.begin_I2C()) { // check if IMU is communicating with board
        Serial.println("Failed to find LSM6DSOX chip");
        while (1) delay(10); // TODO CHANGE THIS TO RESET CHIP, TRY AGAIN, OR THROW ERRORS
    }
    Serial.println("LSM6DSOX initialized successfully");

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
        case 208:
        dataRate = LSM6DS_RATE_208_HZ;
        Serial.println("IMU data rate set to 208 Hz");
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
    IMU.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    IMU.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    IMU.setAccelDataRate(dataRate);
    IMU.setGyroDataRate(dataRate);
    
    Serial.println("MADE 1");

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

    // Start UDP listener
    udp.begin(udpPort);
    Serial.print("UDP listening on port ");
    Serial.println(udpPort);

    // calibrate IMU bias values (shouldn't be necessary for roll and pitch axes with Madgwick, but doesnt seem to work without???)
    Serial.println("IMU SETTLING IN PROGRESS"); // The IMU requires a few seconds to "settle" to its true bias values. Do this before sampling.
    delay(1000);
    Serial.println("SAMPLING BIAS --- ENSURE DEVICE IS ABSOLUTELY STATIONARY");
    sensors_event_t accel, gyro, temp;
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

    filter.begin(filterRate); // Initialize Madgwick filter

    pid_init(&PID, KP, KI, KD, CYCLE_TIME, INTEGRAL_MAX, INTEGRAL_MIN); // Initialize PID values
    control_timer = timerBegin(1e6); // 1 MHz, means we use microseconds to measure interrupt timing
    timerAttachInterrupt(control_timer, &controlLoopTimer); // Attach ISR
    timerAlarm(control_timer, PID.dt*1e6, true, 0); // 100 ms, convert to microseconds from PID.dt (in seconds) by multiplying by 1e6
    
    filter_timer = timerBegin(1e6); // 1 MHz, means we use microseconds to measure interrupt timing
    timerAttachInterrupt(filter_timer, &filterUpdateTimer); // Attach ISR
    timerAlarm(filter_timer, 1e6/filterRate, true, 0); // convert filter rate to microseconds by dividing 1e6 us by filterrate
    
    pinMode(14, OUTPUT);
    digitalWrite(14, HIGH);
    // timerStart(control_timer); // Start control loop timer
    // timerStart(filter_timer); // Start filter update timer
}

void loop() {
    // declare once with static, prevents deallocation and reallocation each iteration
    static sensors_event_t accel, gyro, temp; // IMU variables
    static unsigned long current_time, last_time; // times
    static float heading = 0, dt_heading; // variables for heading calculation
    static float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
    
    // // get current angular acceleration and update heading
    // IMU.getEvent(&accel, &gyro, &temp); // get angular velocity
    // current_time = micros(); // current time
    // dt_heading = (current_time - last_time)/1e6; // delta-t since last update, represented in seconds
    // heading += dt_heading * (gyro.gyro.x * 180/PI); // calculate new heading based on angular velocity and time since last read, converting from radians to degrees in same line CHECK WHICH AXIS WE NEED
    // last_time = current_time; // set time for next loop   

    // // account for heading rollover 
    // if (heading > 360) heading = heading - 360;// these could be replaced with while loops in order to account for massive spikes which might cause heading to exceed, for instance, 720 degrees
    // else if (heading < 0) heading = heading + 360;

    static int packetSize;
    static char incomingPacket[64];
    static int control_enable = 0, angle_deadband, target_heading;
    static float Kp, Ki, Kd;

    packetSize = udp.parsePacket();
    if (packetSize) {
        if (packetSize >= sizeof(incomingPacket)) {
        Serial.println("Packet too large!");
        return;
        }

        int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
        incomingPacket[len] = '\0';
        Serial.print("Received: ");
        Serial.println(incomingPacket);

        // Parse comma-separated values
        char* token = strtok(incomingPacket, ",");
        if (token) control_enable = atoi(token);
        token = strtok(NULL, ",");
        if (token) angle_deadband = atoi(token);
        token = strtok(NULL, ",");
        if (token) Kp = atof(token);
        token = strtok(NULL, ",");
        if (token) Ki = atof(token);
        token = strtok(NULL, ",");
        if (token) Kd = atof(token);
        token = strtok(NULL, ",");
        if (token) target_heading = atoi(token);

        Serial.printf("PID: enable=%d, angle_deadband=%d, Kp=%.5f, Ki=%.5f, Kd=%.5f target=%d\n", control_enable, angle_deadband, Kp, Ki, Kd, target_heading);
        Serial.println(filter.getYaw());
        pid_update(&PID, Kp, Ki, Kd, angle_deadband, target_heading);
    }
    
    if (filterUpdateFlag){
        filterUpdateFlag = false; // turn off flag for next loop
        IMU.getEvent(&accel, &gyro, &temp); // get IMU readings
        
        // Convert to units expected by Madgwick (accel: g, gyro: deg/s)
        accelX = (accel.acceleration.x - accel_X_bias) / 9.81; // Convert m/s^2 to g
        accelY = (accel.acceleration.y - accel_Y_bias) / 9.81;
        accelZ = (accel.acceleration.z - accel_Z_bias) / 9.81;
        gyroX = (gyro.gyro.x - gyro_X_bias) * 180.0 / PI; // Convert rad/s to deg/s
        gyroY = (gyro.gyro.y - gyro_Y_bias) * 180.0 / PI;
        gyroZ = -(gyro.gyro.z - gyro_Z_bias) * 180.0 / PI;

        filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ); // update filter values
    }

    if (controlLoopFlag){ // if interrupt timer was activated, enter PID controller 
        controlLoopFlag = false; // turn off flag for next loop
        heading = filter.getYaw();
        set_thrust(&PID, heading); // determine thrust inputs for thrust_controller later
    }

    if (control_enable){
        thrust_controller(&PID); // actuate thrusters
    }
    else {
        digitalWrite(CW_SOLENOID_PIN, LOW); // ensure valves remain closed if PID controller is turned off
        digitalWrite(CCW_SOLENOID_PIN, LOW);
    }
}