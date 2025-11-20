#include <Arduino.h>
#include <Wire.h>
#include "bmm350.h"

// -----------------------------
// Configuration
// -----------------------------
#define TCA9548A_ADDR 0x70
#define NUM_SENSORS 5
const uint8_t SENSOR_CHANNELS[NUM_SENSORS] = {0, 1, 3, 4, 7};
#define BMM350_I2C_ADDR 0x14
#define SELF_TEST_THRESHOLD 130.0

// -----------------------------
// Global variables
// -----------------------------
struct bmm350_dev sensors[NUM_SENSORS];
uint8_t dev_addr = BMM350_I2C_ADDR;
uint16_t counter = 0;
bool data_logging_active = false;

// -----------------------------
// TCA9548A Multiplexer Control
// -----------------------------
void selectMuxChannel(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delayMicroseconds(100);
}

void disableMuxChannels() {
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(0);
    Wire.endTransmission();
}

// -----------------------------
// I2C Helper Functions
// -----------------------------
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) return -1;
    Wire.requestFrom((int)addr, (int)len);
    for (uint32_t i = 0; i < len && Wire.available(); i++) data[i] = Wire.read();
    return 0;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(addr);
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++) Wire.write(data[i]);
    return Wire.endTransmission();
}

void user_delay_us(uint32_t period, void *intf_ptr) { delayMicroseconds(period); }

void bmm350_error_codes_print_result(const char *api_name, int8_t rslt, uint8_t sensor_id) {
    if (rslt != BMM350_OK) Serial.printf("# [ERR] Sensor %d - %s -> code %d\n", sensor_id, api_name, rslt);
}

// -----------------------------
// Self-Test
// -----------------------------
int check_self_test(uint8_t sensor_idx) {
    struct bmm350_mag_temp_data mag_before;
    struct bmm350_self_test self_test_out;
    int8_t rslt;

    selectMuxChannel(SENSOR_CHANNELS[sensor_idx]);
    rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_before, &sensors[sensor_idx]);
    if(rslt != BMM350_OK) { Serial.printf("# Sensor %d: Failed before self-test\n", sensor_idx); return 0; }

    rslt = bmm350_perform_self_test(&self_test_out, &sensors[sensor_idx]);
    if(rslt != BMM350_OK) { Serial.printf("# Sensor %d: Self-test API failed\n", sensor_idx); return 0; }

    float deltaX = self_test_out.out_ust_x - mag_before.x;
    float deltaY = self_test_out.out_ust_y - mag_before.y;

    Serial.printf("# Sensor %d Self-Test: ΔX=%.2f µT, ΔY=%.2f µT -> %s\n", sensor_idx, deltaX, deltaY,
                  (deltaX >= SELF_TEST_THRESHOLD && deltaY >= SELF_TEST_THRESHOLD) ? "PASSED ✅" : "FAILED ❌");
    return (deltaX >= SELF_TEST_THRESHOLD && deltaY >= SELF_TEST_THRESHOLD) ? 1 : 0;
}

// -----------------------------
// Initialize single sensor
// -----------------------------
bool initSensor(uint8_t sensor_idx) {
    selectMuxChannel(SENSOR_CHANNELS[sensor_idx]);
    sensors[sensor_idx].read = user_i2c_read;
    sensors[sensor_idx].write = user_i2c_write;
    sensors[sensor_idx].delay_us = user_delay_us;
    sensors[sensor_idx].intf_ptr = &dev_addr;

    int8_t rslt = bmm350_init(&sensors[sensor_idx]);
    bmm350_error_codes_print_result("bmm350_init", rslt, sensor_idx);
    if (rslt != BMM350_OK) return false;

    bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_8, &sensors[sensor_idx]);
    bmm350_set_powermode(BMM350_NORMAL_MODE, &sensors[sensor_idx]);
    sensors[sensor_idx].delay_us(10000, sensors[sensor_idx].intf_ptr);
    bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &sensors[sensor_idx]);

    return true;
}

// -----------------------------
// Arduino Setup
// -----------------------------
void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10);

    Wire.begin();
    Wire.setClock(400000);

    // Initialize sensors
    bool all_ok = true;
    for(uint8_t i=0; i<NUM_SENSORS; i++) {
        if(!initSensor(i)) all_ok = false;
    }

    // CSV header
    Serial.println("Timestamp(ms),"
                   "S0_X,S0_Y,S0_Z,S0_T,"
                   "S1_X,S1_Y,S1_Z,S1_T,"
                   "S2_X,S2_Y,S2_Z,S2_T,"
                   "S3_X,S3_Y,S3_Z,S3_T,"
                   "S4_X,S4_Y,S4_Z,S4_T,"
                   "tS0_start,tS0_end,"
                   "tS1_start,tS1_end,"
                   "tS2_start,tS2_end,"
                   "tS3_start,tS3_end,"
                   "tS4_start,tS4_end,"
                   "Δ01,Δ12,Δ23,Δ34,Δ04_total");

    data_logging_active = true;
    delay(500);
}

// -----------------------------
// Arduino Loop
// -----------------------------
void loop() {
    unsigned long timestamp = millis();
    struct bmm350_mag_temp_data mag_data[NUM_SENSORS];
    bool read_success[NUM_SENSORS];
    unsigned long t_start[NUM_SENSORS];
    unsigned long t_end[NUM_SENSORS];

    // Read all sensors
    for(uint8_t i=0; i<NUM_SENSORS; i++) {
        t_start[i] = micros();
        selectMuxChannel(SENSOR_CHANNELS[i]);
        int8_t rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_data[i], &sensors[i]);
        read_success[i] = (rslt == BMM350_OK);
        t_end[i] = micros();
    }

    // Calculate inter-sensor deltas based on start times
    unsigned long delta[NUM_SENSORS-1];
    for(uint8_t i=0; i<NUM_SENSORS-1; i++) delta[i] = t_start[i+1] - t_start[i];

    unsigned long delta04_total = t_start[4] - t_start[0];

    // Print CSV
    Serial.print(timestamp);
    for(uint8_t i=0; i<NUM_SENSORS; i++) {
        if(read_success[i])
            Serial.printf(",%.2f,%.2f,%.2f,%.2f", mag_data[i].x, mag_data[i].y, mag_data[i].z, mag_data[i].temperature);
        else
            Serial.print(",ERR,ERR,ERR,ERR");
    }

    for(uint8_t i=0; i<NUM_SENSORS; i++) Serial.printf(",%lu,%lu", t_start[i], t_end[i]);
    for(uint8_t i=0; i<NUM_SENSORS-1; i++) Serial.printf(",%lu", delta[i]);
    Serial.printf(",%lu", delta04_total);
    Serial.println();

    counter++;
    if(counter >= 100) {
        for(uint8_t i=0; i<NUM_SENSORS; i++) check_self_test(i);
        counter = 0;
    }

    delay(200);
}
