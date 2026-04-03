#include <Adafruit_LSM6DSOX.h>
#include <Arduino.h>

#define PI 3.1415926535897932384626433832795
#define CW_SOLENOID_PIN 13
#define CCW_SOLENOID_PIN 14

#define RCS_F 1
#define RCS_ARM 1
#define KP 1
#define KI 1
#define KD 1
#define INTEGRAL_MAX 1
#define INTEGRAL_MIN -1
#define MIN_FIRE 0.06 // minimum fire time in seconds

hw_timer_t *control_timer = NULL; // control loop timer
volatile bool controlLoopFlag = false; // determines if the PID should be activated in any given iteration
float alpha = 0.2; // smoothing factor (0 < alpha <= 1)

typedef struct { // PID controller structure
    float Kp, Ki, Kd;       // PID gains
    float integral;         // Running sum for integral term
    float prev_error;       // Previous error for derivative term
    float integral_min, integral_max; // Anti-windup limits
    unsigned long last_PID_time; // keep track of last PID calculation time
    unsigned long thrust_time; // thruster on time (PWM control)
    unsigned long thrust_start_time; // begin of control input is relative to PID calculations
    bool CW_thruster_on = false; // thruster on or off (in order disengage thrust if target reached earlier than expected)
    bool CCW_thruster_on = false;
} PID_data;

void pid_init(PID_data *PID, float kp, float ki, float kd, float int_max, float int_min){
    PID->Kp = kp;
    PID->Ki = ki;
    PID->Kd = kd;
    PID->integral = 0.0;
    PID->prev_error = 0.0;
    PID->integral_max = int_max;
    PID->integral_min = int_min;
}

float PID_controller(PID_data *PID, float error){
    static float P, I, D;
    static unsigned long dt, current_time;
    
    error = error*PI/180; // convert error to radians (within this function only) for mathematical operations

    current_time = micros();
    dt = current_time - PID->last_PID_time; // calculate time since last calculation
    P = PID->Kp * error; 
    PID->integral += error * dt;
    I = PID->Ki * PID->integral;
    D = PID->Kd * (error - PID->prev_error) / dt;
    
    // if (pid->integral > pid->integral_max) pid->integral = pid->integral_max; // ensure integral term doesn't go over max
    // if (pid->integral < pid->integral_min) pid->integral = pid->integral_min; // ensure integral term doesn't go under min

    PID->prev_error = error;
    PID->last_PID_time = current_time;
    return P + I + D;
}

void set_thrust(PID_data *PID, float current_heading){
    static float error, tau_output, target_heading = 90; // TODO create logic for changing target heading
    static float tau_rcs = RCS_F * RCS_ARM; // defined by RCS hardware specifications 
    static float duty_cycle, fire_time;

    error = target_heading - current_heading; // error in set and measured heading
    // account for rollover and shortest path
    if (error > 180){
        error -= 360;
    } else if (error < -180){
        error += 360;
    }

    tau_output = PID_controller(PID, error); // calculate desired torque
    duty_cycle = tau_output/tau_rcs; // calculate percentage of duty_cycle for desired torque given RCS output
    if (duty_cycle > 1.0) duty_cycle = 1.0; // saturate in case greater torque required than can be output for a single given duty cycle, thrust commands will rollover into next cycle

    fire_time = duty_cycle*PID->dt*1e6; // convert firing time to microseconds for later comparisons with start time

    if (fire_time < (MIN_FIRE*1e6)){ // don't fire thrusters if fire time below minimum solenoid actuation time, time converted to microseconds for comparison with firing time
        PID->CW_thruster_on = false;
        PID->CCW_thruster_on = false;

        if (fabs(error) < 2) { // if angle is within deadband, and thrusters off, we can assume we are in a stable state and can "turn off" the PID controller, requiring a reset of the accumulated error
            PID->integral = 0.0;
        }
        return;
    }
    
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
Adafruit_LSM6DSOX sox;

void setup() {
    Serial.begin(115200);
    delay(1500); // wait for connections

    pinMode(CW_SOLENOID_PIN, OUTPUT); // connect solenoid pins to output
    pinMode(CCW_SOLENOID_PIN, OUTPUT);

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

    pid_init(&PID, KP, KI, KD, INTEGRAL_MAX, INTEGRAL_MIN); // Initialize PID values
}

void loop() {
    // declare once with static, prevents deallocation and reallocation each iteration
    static sensors_event_t accel, gyro, temp; // IMU variables
    static unsigned long current_time, last_time; // times
    static float heading = 0, dt_heading; // variables for heading calculation

    // get current angular acceleration and update heading
    sox.getEvent(&accel, &gyro, &temp); // get angular velocity
    current_time = micros(); // current time
    dt_heading = (current_time - last_time)/1e6; // delta-t since last update, represented in seconds
    heading += dt_heading * (gyro.gyro.x * 180/PI); // calculate new heading based on angular velocity and time since last read, converting from radians to degrees in same line CHECK WHICH AXIS WE NEED
    last_time = current_time; // set time for next loop   
    
    // account for heading rollover 
    if (heading > 360) heading = heading - 360;// these could be replaced with while loops in order to account for massive spikes which might cause heading to exceed, for instance, 720 degrees
    else if (heading < 0) heading = heading + 360;
    
    set_thrust(&PID, heading); // determine thrust inputs for thrust_controller
    thrust_controller(&PID); // determines thrust inputs and outputs
}