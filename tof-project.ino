
// arduino-cli compile --fqbn esp32:esp32:nodemcu-32s tof-project.ino && arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:nodemcu-32s tof-project.ino && arduino-cli monitor -p /dev/cu.usbserial-0001


#include <AccelStepper.h>
//#include <ArduinoJson.h>
#include <PID_v1.h>

#include <HTTPClient.h>
#include <WebServer.h>
#include <WiFi.h>

#include <Arduino.h>

//#include <Wire.h>
//#include <VL53L0X.h>

#include <Wire.h>
#include <VL53L0X.h>

// local
#include "config.h"
#include "tof_setup.h"

// function prototype defined in tof-setup 
void tof_setup(VL53L0X &sensor_0, VL53L0X &sensor_1, VL53L0X &sensor_2, VL53L0X &sensor_3);

// wifi
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;


WebServer server(80);


//WiFiClient browser;
//IPAddress ip("ESP);
IPAddress ip(ESP_32_IP);
// local ip gateway
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);



// ==========  motors =============
class Motor {
    /*
       handle motor movment/ position/ tof

     */
    public:
        AccelStepper m;
        String motor_name;
        int step_pin;
        int dir_pin;
        String step_unit;
        const float steps_per_unit;
        const float min_pos;
        const float max_pos;
        const float max_speed;

    Motor(
        AccelStepper _m,
        String _motor_name,
        int _step_pin,
        int _dir_pin,
        String _step_unit,
        float _steps_per_unit,
        const float _min_pos,
        const float _max_pos,
        const float _max_speed
        ):

    m(_m),
    motor_name(_motor_name),
    step_pin(_step_pin), dir_pin(_dir_pin),
    step_unit(_step_unit), steps_per_unit(_steps_per_unit),
    min_pos(_min_pos), max_pos(_max_pos),
    max_speed(_max_speed),
    kp_(0.8), ki_(0.1), kd_(0.5), integral_(0.0), prev_error_(0.0), last_time_(0)

    {

    };

    void set_speed(float motor_speed){
        // set motor speed
        m.setMaxSpeed(motor_speed * steps_per_unit);
    };

    void set_accel(float motor_accel){
        /*
        set accel
        */
        m.setAcceleration(motor_accel * steps_per_unit);
    };

    void move_to_position(float target_pos){
        /* blocking, move motor to position in units */
        // make sure doesnt exceed limits
        target_pos = min(max_pos, max(min_pos, target_pos));
        m.runToNewPosition(target_pos * steps_per_unit);
    };

    float current_pos(){
        /*
        current pos in units
        */
        return m.currentPosition() / steps_per_unit;
    };

    void set_kp(double kp){
        /* set kp weight */
        kp_ = kp;
        Serial.print(motor_name);
        Serial.print(" kp: ");
        Serial.println(kp_);
    };
    void set_ki(double ki){
        /* set ki weight */
        ki_ = ki;
        Serial.print(motor_name);
        Serial.print(" ki: ");
        Serial.println(ki_);
    };
    void set_kd(double kd){
        /* set kp weight */
        kd_ = kd;
        Serial.print(motor_name);
        Serial.print(" kd: ");
        Serial.println(kd_);
    };

    void move_to(float move_target){
        /* 
           set absolute target for accel stepper run() function
           must be called to drive motor once set
           move_target, in units not steps 
         */
        m.moveTo(move_target * steps_per_unit);

    };


    // motor pid
    float update_pid(float setpoint, float pv) {
        unsigned long now = millis();
        float dt = (now - last_time_) / 1000.0; // Convert milliseconds to seconds

        if (last_time_ == 0) {
            dt = 1.0; // Initial dt value in case this is the first update
        }
        last_time_ = now;

        // Calculate error
        float error = setpoint - pv;

        // Proportional term
        float P = kp_ * error;

        // Integral term
        integral_ += error * dt;
        float I = ki_ * integral_;

        // Derivative term
        float derivative = (error - prev_error_) / dt;
        float D = kd_ * derivative;

        // Update previous error
        prev_error_ = error;

        // Calculate control variable (motor speed)
        float control = P + I + D;

        // constrain the control variable to the valid range
        control = min(max_speed, abs(control));

        return control;
    };


    private:
        float kp_;  // Proportional gain
        float ki_;  // Integral gain
        float kd_;  // Derivative gain
        float integral_;  // Integral error
        float prev_error_;  // Previous error
        unsigned long last_time_;  // Time when the last update was called

}; // end Motor


class TOFassembly {
    /*
       tof assembly
    */
    public:
       float tof_xz_ang; // tof sensor angle offset
       const float short_linkage_len; // physical length of short linkage section, with laser/ sensor
       const float long_linkage_len; // physical length of long linkage section
       const double hingeCenterDist; // distance form the center tof sensor to the hinge pivot point
       const double tofHingeDist; // distance from the outer tof sensors to their respective hinge pivot point

    TOFassembly():

    short_linkage_len(36),
    long_linkage_len(50),
    hingeCenterDist(25),
    tofHingeDist(14.75),
    tof_xz_ang(15.00)
    {};


    float set_tof_angle(float angle_degrees) {
        // Calculate the unknown side length for a given sensor angle
        
        angle_degrees = 90 - angle_degrees;
        
        // Convert angle from degrees to radians
        float angle_radians = angle_degrees * DEG_TO_RAD;
        
        // Calculate the length of leg A using the cosine of the angle
        float short_len_a = short_linkage_len * cos(angle_radians);
        
        // Calculate the length of leg B using the sine of the angle
        // First triangle height
        float b = short_linkage_len * sin(angle_radians);

        // Length along y axis
        float long_len_a = sqrt(pow(long_linkage_len, 2) - pow(b, 2));
        
        // tof_motor y len
        return short_len_a + long_len_a;
    };


    struct Vector {
        double x, y, z;
    };
    struct Vector crossProduct(const Vector& a, const Vector& b) {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    };
    double norm(const Vector& v) {
        return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    };


    float calculateAngles(double sensorData[], int dataSize=4) {
        /* 
           calculate angle around z axis (theta), and angle around x axis (phi)
         */

        // distance in mm reported by the center tof sensor
        double centerPoint = sensorData[0];
        
        // distacne in mm of the three outer tof sensors
        double magnitudes[dataSize - 1];
        for (int i = 1; i < dataSize; i++) {
            magnitudes[i-1] = sensorData[i];
        }

        // angle between center vector, and each of the three outer vectors
        double phi = radians(tof_xz_ang);

        Vector hingePoints[dataSize - 1];
        Vector tofPoints[dataSize - 1];
        Vector vectors[dataSize - 1];
        Vector endpoints[dataSize - 1];

        // each of the three tof sensors are arranged equally arounf the center sensor
        for (int i = 0; i < dataSize - 1; i++) {
            double theta = radians((i * -120) + 90);

            // vector from the center tof sensor to each hinge pivot point
            Vector hingePoint = {hingeCenterDist * cos(theta), 0, hingeCenterDist * sin(theta)};
            hingePoints[i] = hingePoint;

            // vector from each hinge pivot point to its respective outer tof sensor
            Vector tofPoint = {(tofHingeDist * cos(phi) + hingeCenterDist) * cos(theta), -tofHingeDist * sin(phi), (tofHingeDist * cos(phi) + hingeCenterDist) * sin(theta)};
            tofPoints[i] = tofPoint;

            // reported distance in mm of each outer tof sensor
            double magnitude = magnitudes[i];
            vectors[i] = {magnitude * (sin(phi) * cos(theta)) + tofPoint.x, magnitude * cos(phi) + tofPoint.y, magnitude * sin(phi) * sin(theta) + tofPoint.z};
            endpoints[i] = vectors[i];
        }

        // vector of two pairs of outer tof vecotrs
        Vector v = {vectors[1].x - vectors[0].x, vectors[1].y - vectors[0].y, vectors[1].z - vectors[0].z};
        Vector w = {vectors[2].x - vectors[0].x, vectors[2].y - vectors[0].y, vectors[2].z - vectors[0].z};

        // normal vector 
        Vector n = crossProduct(v, w);

        Vector centroid = {(vectors[0].x + vectors[1].x + vectors[2].x) / 3, (vectors[0].y + vectors[1].y + vectors[2].y) / 3, (vectors[0].z + vectors[1].z + vectors[2].z) / 3};

        double nMagnitude = norm(n);
        double theta = atan2(n.y, n.x);
        double phiAngle = acos(n.z / nMagnitude);

        // around z axis
        float thetaDegrees = -degrees(theta) + 90;
        // around x axis
        float phiDegrees = -degrees(phiAngle) + 90;

        return thetaDegrees;

    };

}; // end TOFassembly

TOFassembly tof_assembly;

// yaw
//const int yaw_limit_pin = 34;
const int yaw_step_pin = 13;
const int yaw_dir_pin = 14;
const float yaw_steps_per_rev = 3200*1;
const float yaw_gear_mult = 50/8;
const float yaw_steps_per_unit = (yaw_steps_per_rev * yaw_gear_mult)/360;
const float YAW_MIN = -80.0;
const float YAW_MAX = 80.0;
const float yaw_max_speed = 300;
AccelStepper _m_yaw(1, yaw_step_pin, yaw_dir_pin); // pin 23 = step, pin 22 = direction
Motor m_yaw(_m_yaw, "yaw", yaw_step_pin, yaw_dir_pin, "degrees", yaw_steps_per_unit, YAW_MIN, YAW_MAX, yaw_max_speed);


// y
//const float drive_teeth = 96;
const int y_limit_pin = 34;
const int y_step_pin = 27;
const int y_dir_pin = 12;
//const float y_steps_per_rev = 3200*2;
const float y_steps_per_rev = 3200;
const float y_gear_mult = 128;
const float y_mm_per_rev = y_gear_mult *2;
const float y_steps_per_unit = (y_steps_per_rev / y_mm_per_rev);
const float Y_MIN = 10;
const float Y_MAX = 650;
const float y_max_speed = 500;
AccelStepper _m_y(1, y_step_pin, y_dir_pin);
Motor m_y(_m_y, "y", y_step_pin, y_dir_pin, "mm", y_steps_per_unit, Y_MIN,  Y_MAX, y_max_speed);



// tof ceenter motor
const int tof_limit_pin = 35;
const int tof_step_pin = 16;
const int tof_dir_pin = 17;
const float mm_per_rev = 8;
const float tof_steps_per_rev = 2*3200;
const float tof_steps_per_unit = (tof_steps_per_rev / mm_per_rev);
const float tof_min = 20;
const float tof_max = 60;
const float tof_max_speed = 40;
AccelStepper _m_tof(1, tof_step_pin, tof_dir_pin);
Motor m_tof(_m_tof,"tof", tof_step_pin, tof_dir_pin, "mm", tof_steps_per_unit, tof_min, tof_max, tof_max_speed);


// around z axis
//double thetaDegrees;
// around x axis
//double phiDegrees;

// toggle red laser power pin
const int laser_pin = 19;

#define LONG_RANGE
#define HIGH_ACCURACY

VL53L0X sensor_0;
VL53L0X sensor_1;
VL53L0X sensor_2;
VL53L0X sensor_3;

// three tof angles
//float TOF_ANGLE = 0;

float YAW_TARGET_ANG = 0;
float Y_TARGET_DIST = 600;

double s0;
double s1;
double s2;
double s3;



void start_sensors(){
    // timeing budget in microseconds, min 20 miliseconds
    long timing_budget = 12 * 10000;


    sensor_0.setMeasurementTimingBudget(timing_budget);
    sensor_1.setMeasurementTimingBudget(timing_budget);
    sensor_2.setMeasurementTimingBudget(timing_budget);
    sensor_3.setMeasurementTimingBudget(timing_budget);

    // etting a lower limit increases the potential range of the sensor but also increases the likelihood of getting an inaccurate reading because of reflections from objects other than the intended target. This limit is initialized to 0.25 MCPS by default.

    //float rate_limit = 0.1;
    float rate_limit = 0.25;
    sensor_0.setSignalRateLimit(rate_limit);
    sensor_1.setSignalRateLimit(rate_limit);
    sensor_2.setSignalRateLimit(rate_limit);
    sensor_3.setSignalRateLimit(rate_limit);

    //int pre_range = 18;
    //sensor_0.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, pre_range);
    //sensor_1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, pre_range);
    //sensor_2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, pre_range);
    //sensor_3.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, pre_range);

    int final_range = 14; 
    sensor_0.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, final_range);
    sensor_1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, final_range);
    sensor_2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, final_range);
    sensor_3.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, final_range);


    sensor_0.startContinuous();
    sensor_1.startContinuous();
    sensor_2.startContinuous();
    sensor_3.startContinuous();
};



void setup() {

    // forces to use the fix IP
    WiFi.config(ip, gateway, subnet);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        //Serial.print(("."));
    }
    Serial.begin(9600);
    Serial.println("");
    Serial.println("---");
    Serial.println("ESP32 Connected to Wifi on IP address : http://");
    Serial.print(WiFi.localIP()); // Print the IP address
    Serial.println("");

    delay(1000);
    Serial.println("ESP32 Started");
    Wire.begin();
    delay(500);

    pinMode(laser_pin, OUTPUT); 

    pinMode(tof_step_pin, OUTPUT); 
    pinMode(tof_dir_pin, OUTPUT); 
    pinMode(tof_limit_pin, INPUT); 

    pinMode(yaw_step_pin, OUTPUT); 
    pinMode(yaw_dir_pin, OUTPUT); 
    //pinMode(yaw_limit_pin, INPUT); 

    pinMode(y_step_pin, OUTPUT); 
    pinMode(y_dir_pin, OUTPUT); 
    pinMode(y_limit_pin, INPUT); 

    // yaw axis
    m_yaw.m.setPinsInverted(true);
    m_yaw.m.setMinPulseWidth(20);
    m_yaw.m.setCurrentPosition(0); 
    //m_yaw.setMaxSpeed(3000);
    //m_yaw.setAcceleration(20000);
        
    m_yaw.set_speed(100);
    m_yaw.set_accel(200);

    //m0.setPinsInverted(true);
    // tof triangulation
    m_tof.m.setMinPulseWidth(20);
    m_tof.m.setCurrentPosition(0); 
    m_tof.m.setMaxSpeed(3000);
    m_tof.m.setAcceleration(20000);


    // y axis
    m_y.m.setPinsInverted(true);
    m_y.m.setCurrentPosition(0); 
    m_y.m.setMinPulseWidth(30);
    m_y.m.setMaxSpeed(1000);
    m_y.m.setAcceleration(20000);



    //set tof addresses
    tof_setup(sensor_0, sensor_1, sensor_2, sensor_3);
    Serial.println("--------------------------");
    Serial.print("sensor_0 adr: ");
    Serial.println(sensor_0.getAddress());
    Serial.print("sensor_1 adr: ");
    Serial.println(sensor_1.getAddress());
    Serial.print("sensor_2 adr: ");
    Serial.println(sensor_2.getAddress());
    Serial.print("sensor_3 adr: ");
    Serial.println(sensor_3.getAddress());


    delay(100);
    start_sensors();

    //create second task on core 0
    xTaskCreatePinnedToCore(
        loop_2, /* Function to implement the task */
        "loop_2", /* Name of the task */
        30000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        1,  /* Priority of the task */
        NULL,  /* Task handle. */
        0); /* Core where the task should run */

}







void handle_setup_all(){

};





// home tof motor
void home_tof(){
    m_tof.set_speed(20);
    m_tof.set_accel(100);

    // move twords limit swithch
    m_tof.m.moveTo(300* tof_steps_per_unit);
    
    long fast_trig_pos;
    int check_count = 0;
    int limit_val = 0;

    while(limit_val != 1){
        for (int i=0; i<50; i++){
            m_tof.m.run();
        };
        limit_val = digitalRead(tof_limit_pin);
        // chack for limit signal
        if (limit_val == 1){
            check_count = 0;
            // must have 50 consecutive positive pin readings before exit
            while ((limit_val == 1) and (check_count<10)) {
                // chack there was no erronous signal
                limit_val = digitalRead(tof_limit_pin);
                check_count++;
                delay(1);
            };
        }; // end if limit
    }; // end while

    m_tof.m.setCurrentPosition(60.5 * tof_steps_per_unit); 
    // backoff end stop
    m_tof.m.runToNewPosition(50 * tof_steps_per_unit);
    //m0.setCurrentPosition(0); 
    m_tof.set_speed(40);

};

void home_yaw(){
    m_yaw.m.setCurrentPosition(0 * yaw_steps_per_unit); 
    m_yaw.set_speed(15);
    m_yaw.set_accel(200);
};


// home y motor
void home_y(){
    m_y.set_speed(80);
    m_y.set_speed(100);

    // move twords limit swithch
    m_y.m.moveTo(-2000* y_steps_per_unit);
    
    long fast_trig_pos;
    int check_count = 0;
    int limit_val = 0;

    while(limit_val != 1){
        for (int i=0; i<50; i++){
            m_y.m.run();
        };
        limit_val = digitalRead(y_limit_pin);
        // chack for limit signal
        if (limit_val == 1){
            check_count = 0;
            // must have 50 consecutive positive pin readings before exit
            while ((limit_val == 1) and (check_count<10)) {
                // chack there was no erronous signal
                limit_val = digitalRead(y_limit_pin);
                check_count++;
                delay(1);
            };
        }; // end if limit
    }; // end while

    m_y.m.setCurrentPosition(0 * y_steps_per_unit); 
    // backoff end stop
    m_y.m.runToNewPosition(500 * y_steps_per_unit);

    m_y.m.setMaxSpeed(250 * y_steps_per_unit);
    //m0.setCurrentPosition(0); 
};


int start_home = 0;
void handle_home(){
    start_home = 1;
    delay(500);
    server.send(200, "text/plain", "1");
};


int wrap_home(){
    if (start_home == 1){
        String motor_name = server.arg("motor_name");
            Serial.println("start home");

        float unit_mult;
        AccelStepper* m = NULL;
        // select motor
        if (motor_name == "m_tof"){
            // set value with pointer
            home_tof();
            m = &m_tof.m;
        } else if (motor_name == "m_yaw"){
            home_yaw();
            m = &m_yaw.m;
        } else if (motor_name == "m_y"){
            home_y();
            m = &m_y.m;
        } else {
            Serial.print("NO MOTOR NAMED: ");
            Serial.println(motor_name);
            server.send(200, "text/plain");
            // set to 0 after home finished
            start_home = 0;
            return 0;
        };
        // set to 0 after home finished
        start_home = 0;

    };

    return 1;
};



int start_move = 0;
void handle_move(){
    start_move = 1;
    server.send(200, "text/plain");
};

void wrap_move(){
    if (start_move == 1){
        String motor = server.arg("motor");
        float steps = server.arg("position").toFloat();

        Serial.println("start run move: ");
        Serial.println(motor);
        Serial.println(" to:  ");
        Serial.println(steps);

        move(steps, motor);
        start_move = 0;

    };

};

int move(float move_position, String motor_name){
    // pointer
    float unit_mult;
    float min_pos;
    float max_pos;
    Motor* m = NULL;
    // select motor
    if (motor_name == "m_tof"){
        // set value with pointer
        m = &m_tof;
    } else if (motor_name == "m_yaw"){
        m = &m_yaw;
    } else if (motor_name == "m_y"){
        m = &m_y;
    } else {
        Serial.print("NO MOTOR NAMED: ");
        Serial.println(motor_name);
        server.send(200, "text/plain");
        return 0;
    };

    Serial.print("move to: ");
    Serial.println(move_position);
    m->move_to_position(move_position);

    return 1;

};

int update_motor_settings(){
    // update motor settings based on url string args

    String motor_name = server.arg("motor");
    Serial.print("setting motor: ");
    Serial.println(motor_name);

    // motor steps per unit multiple, mm or degrees
    // pointer
    Motor* m = NULL;
    // select motor
    if (motor_name == "m_tof"){
        // set value with pointer
        m = &m_tof;
    } else if (motor_name == "m_yaw"){
        m = &m_yaw;
    } else if (motor_name == "m_y"){
        m = &m_y;
    } else {
        Serial.print("NO MOTOR NAMED: ");
        Serial.println(motor_name);
        server.send(200, "text/plain");
        return 0;
    };

    // set pid kd
    if (server.hasArg("kp") == true){
        float kp = server.arg("kp").toFloat();
        m->set_kp(kp);
        Serial.print("kp: ");
        Serial.println(kp);
    };
    // set pid ki
    if (server.hasArg("ki") == true){
        float ki = server.arg("ki").toFloat();
        m->set_ki(ki);
        Serial.print("ki: ");
        Serial.println(ki);
    };
    // set pid ki
    if (server.hasArg("kd") == true){
        float kd = server.arg("kd").toFloat();
        m->set_kd(kd);
        Serial.print("kd: ");
        Serial.println(kd);
    };

    // set speed
    if (server.hasArg("speed") == true){
        float m_speed = server.arg("speed").toFloat();
        m->set_speed(m_speed);
        Serial.print("new speed: ");
        Serial.println(m_speed);
    };

    // set accel
    if (server.hasArg("accel") == true){
        float m_accel = server.arg("accel").toFloat();
        m->set_accel(m_accel);
        Serial.print("new accel: ");
        Serial.println(m_accel);
    };

    server.send(200, "text/plain");
    return 1;

};







int update_global_settings(){
    /*
       update global settings/params/state
    */

    // set y target ang
    if (server.hasArg("y_target_dist") == true){
        Y_TARGET_DIST = server.arg("y_target_dist").toFloat();
        Serial.print("Y_TARGET_DIST: ");
        Serial.println(Y_TARGET_DIST);
    }; // end target dist


    // toggle red laser
    if (server.hasArg("laser_state") == true){

        float laser_state = server.arg("laser_state").toInt();

        if (laser_state == 1) {
            Serial.println("LASER ON");
            digitalWrite(laser_pin, HIGH);
        } 
        else {
            Serial.println("LASER OFF");
            digitalWrite(laser_pin, LOW);
        };

    };// end laser


    if (server.hasArg("tof_angle") == true){

        float tof_angle = server.arg("tof_angle").toFloat();

        float tof_y_len = tof_assembly.set_tof_angle(tof_angle);

        m_tof.move_to_position(tof_y_len);

        Serial.print("tof_angle: ");
        Serial.print(tof_angle);
        Serial.print(" -> tof_y_len: ");
        Serial.println(tof_y_len);

    }; // end tof_ang




    server.send(200, "text/plain");
    return 1;

}; // end update_global





// TODO replace with semaphore or mutex, somthing better than global 
int run_keep_angle = 0;
void wrap_keep_angle() {
    /*
       called from first core
    */
    while(run_keep_angle==1){
        for (int i=1; i < 10000; i++){

            // run yaw absolute
            m_yaw.m.run();

            // run y absolute
            m_y.m.run();
        };
        vTaskDelay(2); 
    };
};



void handle_keep_angle(){
    run_keep_angle = int(server.arg(0).toFloat());
    Serial.print("keep_angle: ");
    Serial.println(run_keep_angle);
    server.send(200, "text/plain");

};



int display_data = 0;
void handle_display_data(){
    display_data = int(server.arg(0).toFloat());
    server.send(200, "text/plain");
};




// response
String res = "";
void get_sensor_data(){
    /*
       update sensor data
    */

    // linear distance reported by each sensor
    s0 = sensor_0.readRangeContinuousMillimeters();
    s1 = sensor_1.readRangeContinuousMillimeters();
    s2 = sensor_2.readRangeContinuousMillimeters();
    s3 = sensor_3.readRangeContinuousMillimeters();

    if(display_data == 1){
        res = "";
        res.concat(s0);
        res.concat(",");
        res.concat(s1);
        res.concat(",");
        res.concat(s2);
        res.concat(",");
        res.concat(s3);
        res.concat(";");
        //res.concat(thetaDegrees);
        //res.concat(",");
        //res.concat(phiDegrees);
        Serial.println(res);

    };

    // TODO fix, probably keep each sensor val gloabl, pass ref
    double sensor_data[4] = {s0, s1, s2, s3};

    YAW_TARGET_ANG = tof_assembly.calculateAngles(sensor_data);

    // value toggled by url arg
    if (run_keep_angle == 1){
    
        // current yaw motor angle 
        float current_ang = m_yaw.current_pos();

        // absolute position yaw motor should move to 
        float yaw_move = current_ang + YAW_TARGET_ANG;

        // prevent over shoot
        yaw_move = min(max(YAW_MIN, yaw_move), YAW_MAX);

        // pid yaw speed
        float yaw_speed = m_yaw.update_pid(0, abs(YAW_TARGET_ANG));

        m_yaw.set_speed(yaw_speed);

        // move yaw absolute
        m_yaw.move_to(yaw_move);

        // difference between current tof measured distance, and y target distance, mm
        float y_move_diff = s0 - Y_TARGET_DIST;

        // y current motor position plus new target position, mm
        float motor_move = m_y.current_pos() + y_move_diff;

        // prevent over shoot
        motor_move = min(max(Y_MIN, motor_move), Y_MAX);

        // pid y motor speed
        float y_speed = m_y.update_pid(0, y_move_diff);

        //float y_speed = linear_pid_speed(y_move_diff, 500, 2, 500);
        m_y.set_speed(y_speed);

        // set y absolute target
        m_y.move_to(motor_move);
    };


};




void loop() {

    wrap_home();
    vTaskDelay(3); 

    wrap_move();
    vTaskDelay(3); 

    wrap_keep_angle();
    vTaskDelay(3); 

};


void routing(){

    Serial.print("routing started on core: ");
    Serial.print(xPortGetCoreID());

    server.on("/setup_all", handle_setup_all);
    server.on("/home", handle_home);

    server.on("/update_motor_settings", update_motor_settings);
    server.on("/update_global_settings", update_global_settings);

    server.on("/run_move", handle_move);
    server.on("/display_data", handle_display_data);
    server.on("/keep_angle", handle_keep_angle);

    server.begin();

};



// runs on core 0, main loop on core 1
void loop_2(void * params){
    /*
     second loop on core 0, runs continious spline moves
    */

    Serial.print("loop_2 started on core: ");
    Serial.print(xPortGetCoreID());
    Serial.print("  with priority = ");
    Serial.println(uxTaskPriorityGet(NULL));
    routing();
    while(1){
        server.handleClient();
        get_sensor_data();
        delay(2);
    };// end while(1)


};






















