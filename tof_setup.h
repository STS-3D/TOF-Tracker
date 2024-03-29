// set addresses for multiple tof sesnors with same default address

#include <Wire.h>
#include <VL53L0X.h>

// ESP32 pins
// WIRE_SCL Green 22
// WIRE_SDA Blue 21

const int tof_0_shutoff = 32;
const int tof_1_shutoff = 33;
const int tof_2_shutoff = 25;
const int tof_3_shutoff = 26;

byte get_addr() {
    byte error, address;
    int n_devices;
    n_devices = 0;
    for(address=1; address<127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            return address;
            if (address<16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
            n_devices++;
        }
        else if (error==4) {
            Serial.print("Error at address 0x");
            if (address<16) {
                Serial.print("0");
            }
            Serial.println(address,HEX);
        }    
    }
    if (n_devices == 0) {
        Serial.println("No devices found\n");
    }
    return 0;

}



// TODO probably use array of pointers here
// pass references of sensor objects defined in main ino
void tof_setup(VL53L0X &sensor_0, VL53L0X &sensor_1, VL53L0X &sensor_2, VL53L0X &sensor_3){

    VL53L0X sensors[4] = {sensor_0, sensor_1, sensor_2, sensor_3};

    const int tof_pins[4] = {tof_0_shutoff, tof_1_shutoff, tof_2_shutoff, tof_3_shutoff};

    // sensors have save i2c address by default, set to unique values
    const int addrs[4] = {42, 43, 44, 45};


    // set pin mode for shutoff pins
    for (int i=0;i<4;i++){
        pinMode(tof_pins[i], OUTPUT); 
    };

    // turn all sensors off
    for (int i=0;i<4;i++){
        digitalWrite(tof_pins[i], LOW);
    };

    // tof i2c address  
    uint8_t current_addr;
    byte addr;
    // turn all sensors off
    for (int i=0;i<4;i++){
        digitalWrite(tof_pins[i], HIGH);
        delay(250);
        addr = get_addr();
        Serial.print("tof: ");
        Serial.print(i);
        Serial.print(" ");
        Serial.print(addr);
        Serial.print(", ");
        Serial.print(addr, HEX);
        Serial.println();

        //current_addr = sensors[i].getAddress();
        //Serial.print("start addr: ");
        //Serial.println(current_addr);
        //sensors[i].setAddress(uint8_t(tof_pins[i]));
        //current_addr = sensors[i].getAddress();
        //Serial.print("end addr: ");
        //Serial.println(current_addr);
        //sensors[i].init();


        // TODO handle in loop
        if (i==0){
            current_addr = sensor_0.getAddress();
            Serial.print("start addr: ");
            Serial.println(current_addr);
            sensor_0.setAddress(uint8_t(tof_pins[i]));
            current_addr = sensor_0.getAddress();
            Serial.print("end addr: ");
            Serial.println(current_addr);
            sensor_0.init();
        };

        if (i==1){
            current_addr = sensor_1.getAddress();
            Serial.print("start addr: ");
            Serial.println(current_addr);
            sensor_1.setAddress(uint8_t(tof_pins[i]));
            current_addr = sensor_1.getAddress();
            Serial.print("end addr: ");
            Serial.println(current_addr);
            sensor_1.init();
        };

        if (i==2){
            current_addr = sensor_2.getAddress();
            Serial.print("start addr: ");
            Serial.println(current_addr);
            sensor_2.setAddress(uint8_t(tof_pins[i]));
            current_addr = sensor_2.getAddress();
            Serial.print("end addr: ");
            Serial.println(current_addr);
            sensor_2.init();
        };
        if (i==3){
            current_addr = sensor_3.getAddress();
            Serial.print("start addr: ");
            Serial.println(current_addr);
            sensor_3.setAddress(uint8_t(tof_pins[i]));
            current_addr = sensor_3.getAddress();
            Serial.print("end addr: ");
            Serial.println(current_addr);
            sensor_3.init();
        };


    };

    // turn all sensors on
    for (int i=0;i<4;i++){
        digitalWrite(tof_pins[i], HIGH);
    };



}; // end tof_setup








