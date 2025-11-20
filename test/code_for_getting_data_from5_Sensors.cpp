#include <Arduino.h>
#include <Wire.h>
#include "bmm350.h"




/* this code gets data from 5 sensors and plot them to csv and an alert log file 
to tell if there is suspected errors */






// TCA9548A Multiplexer Configuration
#define TCA9548A_ADDR 0x70
#define NUM_SENSORS 5

// Sensor channels on the multiplexer
const uint8_t SENSOR_CHANNELS[NUM_SENSORS] = {0, 1, 3, 4, 7};

// BMM350 Configuration
#define BMM350_I2C_ADDR 0x14
#define SELF_TEST_THRESHOLD 130.0

// Global device structs for each sensor
struct bmm350_dev sensors[NUM_SENSORS];
uint8_t dev_addr = BMM350_I2C_ADDR;

// Counter for periodic self-test
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
    if (Wire.endTransmission(false) != 0)
        return -1;

    Wire.requestFrom((int)addr, (int)len);
    for (uint32_t i = 0; i < len && Wire.available(); i++) {
        data[i] = Wire.read();
    }
    return 0;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(addr);
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    return Wire.endTransmission();
}

void user_delay_us(uint32_t period, void *intf_ptr) {
    delayMicroseconds(period);
}

void bmm350_error_codes_print_result(const char *api_name, int8_t rslt, uint8_t sensor_id) {
    if (rslt != BMM350_OK) {
        Serial.printf("# [ERR] Sensor %d - %s -> code %d\n", sensor_id, api_name, rslt);
    }
}

// -----------------------------
// Self-Test Function
// -----------------------------
int check_self_test(uint8_t sensor_idx) {
    struct bmm350_mag_temp_data mag_before;
    struct bmm350_self_test self_test_out;
    int8_t rslt;

    selectMuxChannel(SENSOR_CHANNELS[sensor_idx]);

    rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_before, &sensors[sensor_idx]);
    if(rslt != BMM350_OK) {
        Serial.printf("# Sensor %d: Failed to read before self-test\n", sensor_idx);
        return 0;
    }

    rslt = bmm350_perform_self_test(&self_test_out, &sensors[sensor_idx]);
    if(rslt != BMM350_OK) {
        Serial.printf("# Sensor %d: Self-test API failed\n", sensor_idx);
        return 0;
    }

    float deltaX = self_test_out.out_ust_x - mag_before.x;
    float deltaY = self_test_out.out_ust_y - mag_before.y;

    Serial.printf("# Sensor %d Self-Test:\n", sensor_idx);
    Serial.printf("#   Before: X=%.2f Y=%.2f\n", mag_before.x, mag_before.y);
    Serial.printf("#   During: X=%.2f Y=%.2f\n", self_test_out.out_ust_x, self_test_out.out_ust_y);
    Serial.printf("#   Delta:  ΔX=%.2f µT, ΔY=%.2f µT\n", deltaX, deltaY);

    if(deltaX >= SELF_TEST_THRESHOLD && deltaY >= SELF_TEST_THRESHOLD) {
        Serial.printf("#   Result: PASSED ✅\n");
        return 1;
    } else {
        Serial.printf("#   Result: FAILED ❌\n");
        return 0;
    }
}

// -----------------------------
// Initialize Single Sensor
// -----------------------------
bool initSensor(uint8_t sensor_idx) {
    selectMuxChannel(SENSOR_CHANNELS[sensor_idx]);
    
    sensors[sensor_idx].read = user_i2c_read;
    sensors[sensor_idx].write = user_i2c_write;
    sensors[sensor_idx].delay_us = user_delay_us;
    sensors[sensor_idx].intf_ptr = &dev_addr;

    int8_t rslt = bmm350_init(&sensors[sensor_idx]);
    bmm350_error_codes_print_result("bmm350_init", rslt, sensor_idx);
    
    if (rslt != BMM350_OK) {
        Serial.printf("# Sensor %d: Initialization FAILED!\n", sensor_idx);
        return false;
    }

    Serial.printf("# Sensor %d: Chip ID = 0x%02X\n", sensor_idx, sensors[sensor_idx].chip_id);

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
    
    Serial.println("# ========== BMM350 Multi-Sensor System ==========");
    Serial.printf("# Number of sensors: %d\n", NUM_SENSORS);
    Serial.println("# Multiplexer: TCA9548A at 0x70");
    Serial.println("# ===============================================");
    Serial.println("#");
    Serial.println("# INSTRUCTIONS FOR DATA LOGGING:");
    Serial.println("# 1. Use a serial terminal that can log to file:");
    Serial.println("#    - PuTTY: Session > Logging > 'All session output'");
    Serial.println("#    - Arduino IDE: Tools > Serial Monitor (copy data)");
    Serial.println("#    - CoolTerm: Connection > Capture to Text File");
    Serial.println("#    - screen: Start with 'screen /dev/ttyUSB0 115200 -L'");
    Serial.println("#");
    Serial.println("# 2. Lines starting with '#' are comments (can be filtered)");
    Serial.println("# 3. CSV data lines contain actual sensor readings");
    Serial.println("#");

    Wire.begin();
    Wire.setClock(400000);

    // Initialize all sensors
    bool all_sensors_ok = true;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.printf("# --- Initializing Sensor %d (Channel %d) ---\n", i, SENSOR_CHANNELS[i]);
        if (!initSensor(i)) {
            all_sensors_ok = false;
            Serial.printf("# ERROR: Sensor %d failed to initialize!\n", i);
        } else {
            Serial.printf("# Sensor %d initialized successfully!\n", i);
        }
        delay(100);
    }

    if (!all_sensors_ok) {
        Serial.println("# ⚠️  WARNING: Some sensors failed to initialize!");
        delay(2000);
    }

    // Read initial samples
    Serial.println("#");
    Serial.println("# ********** INITIAL READINGS **********");
    for (uint8_t sample = 0; sample < 5; sample++) {
        Serial.printf("# Sample %d:\n", sample + 1);
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            selectMuxChannel(SENSOR_CHANNELS[i]);
            struct bmm350_mag_temp_data mag_temp_data;
            int8_t rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &sensors[i]);
            if (rslt == BMM350_OK) {
                Serial.printf("#   S%d: X=%.2f Y=%.2f Z=%.2f T=%.2f\n",
                              i, mag_temp_data.x, mag_temp_data.y,
                              mag_temp_data.z, mag_temp_data.temperature);
            }
        }
        delay(100);
    }

    // Run self-test on all sensors
    Serial.println("#");
    Serial.println("# ********** RUNNING SELF-TEST ON ALL SENSORS **********");
    bool all_passed = true;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        int result = check_self_test(i);
        if (result == 0) {
            all_passed = false;
            Serial.printf("# ⚠️  Sensor %d failed self-test! Retrying...\n", i);
            
            delay(1000);
            result = check_self_test(i);
            if (result == 0) {
                Serial.printf("# ❌ Sensor %d failed self-test again!\n", i);
            }
        }
    }

    if (all_passed) {
        Serial.println("# ✅ All sensors passed self-test!");
    } else {
        Serial.println("# ⚠️  Some sensors failed self-test. Check connections.");
    }

    Serial.println("#");
    Serial.println("# ********** STARTING DATA COLLECTION **********");
    Serial.println("# Data format: CSV with header below");
    Serial.println("# ===============================================");
    Serial.println("#");
    
    // Print CSV header (without # so it's part of the data)
    Serial.println("Timestamp(ms),S0_X,S0_Y,S0_Z,S0_T,S1_X,S1_Y,S1_Z,S1_T,S2_X,S2_Y,S2_Z,S2_T,S3_X,S3_Y,S3_Z,S3_T,S4_X,S4_Y,S4_Z,S4_T");
    
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

    // Read from all sensors
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        selectMuxChannel(SENSOR_CHANNELS[i]);
        int8_t rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_data[i], &sensors[i]);
        read_success[i] = (rslt == BMM350_OK);
    }

    // Print CSV data (NO # prefix - this is actual data)
    Serial.print(timestamp);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (read_success[i]) {
            Serial.printf(",%.2f,%.2f,%.2f,%.2f",
                         mag_data[i].x,
                         mag_data[i].y,
                         mag_data[i].z,
                         mag_data[i].temperature);
        } else {
            Serial.print(",ERR,ERR,ERR,ERR");
        }
    }
    Serial.println();

    counter++;

    // Periodic self-test every 100 readings
    if (counter >= 100) {
        Serial.println("#");
        Serial.println("# --- Running Periodic Self-Test ---");
        Serial.printf("# Total readings collected: %d\n", counter);
        
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            int result = check_self_test(i);
            if (result == 0) {
                Serial.printf("# ⚠️  WARNING: Sensor %d failed periodic self-test!\n", i);
            }
        }
        
        Serial.println("# --- Resuming Data Collection ---");
        Serial.println("#");
        
        counter = 0;
    }

    delay(200); // ~5Hz reading rate
}