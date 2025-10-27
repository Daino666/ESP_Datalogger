#include <Arduino.h>
#include <Wire.h>
#include "bmm350.h"

// I2C address of BMM350
#define BMM350_I2C_ADDR  0x14

// Datasheet self-test threshold in µT
#define SELF_TEST_THRESHOLD 130.0

// Global device struct
struct bmm350_dev dev;
uint8_t dev_addr = BMM350_I2C_ADDR;

// -----------------------------
// I2C helper functions
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

void bmm350_error_codes_print_result(const char *api_name, int8_t rslt) {
    if (rslt != BMM350_OK) {
        Serial.print("[ERR] ");
        Serial.print(api_name);
        Serial.print(" -> code ");
        Serial.println(rslt);
    }
}

// -----------------------------
// Self-test function
// -----------------------------
void check_self_test() {
    struct bmm350_mag_temp_data mag_before;
    struct bmm350_self_test self_test_out;
    int8_t rslt;

    // Read X/Y/Z before self-test
    rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_before, &dev);
    if(rslt != BMM350_OK) {
        Serial.println("Failed to read magnetometer before self-test");
        return;
    }

    // Run self-test (API handles suspend mode)
    rslt = bmm350_perform_self_test(&self_test_out, &dev);
    if(rslt != BMM350_OK) {
        Serial.println("Self-test API failed");
        return;
    }

    // Compute delta values
    float deltaX = self_test_out.out_ust_x - mag_before.x;
    float deltaY = self_test_out.out_ust_y - mag_before.y;

    // Print results
    Serial.printf("Before Self-Test: X=%.2f Y=%.2f\n", mag_before.x, mag_before.y);
    Serial.printf("During Self-Test: X=%.2f Y=%.2f\n", self_test_out.out_ust_x, self_test_out.out_ust_y);
    Serial.printf("ΔX=%.2f µT, ΔY=%.2f µT\n", deltaX, deltaY);

    // Check against datasheet threshold
    if(deltaX >= SELF_TEST_THRESHOLD && deltaY >= SELF_TEST_THRESHOLD) {
        Serial.println("Self-Test PASSED ✅");
    } else {
        Serial.println("Self-Test FAILED ❌");
    }
}

// -----------------------------
// Arduino setup()
// -----------------------------
void setup() {
    Serial.begin(115200);
    Wire.begin();  // SDA=21, SCL=22 on ESP32 by default

    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;
    dev.intf_ptr = &dev_addr;

    int8_t rslt;
    struct bmm350_mag_temp_data mag_temp_data;
    struct bmm350_pmu_cmd_status_0 pmu_cmd_stat_0;
    uint8_t err_reg_data = 0;

    // Initialize sensor
    rslt = bmm350_init(&dev);
    bmm350_error_codes_print_result("bmm350_init", rslt);

    Serial.print("BMM350 Chip ID: 0x");
    Serial.println(dev.chip_id, HEX);

    // Check PMU status
    rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, &dev);
    bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0", rslt);
    Serial.print("PMU cmd busy: ");
    Serial.println(pmu_cmd_stat_0.pmu_cmd_busy);

    // Error register
    rslt = bmm350_get_regs(BMM350_REG_ERR_REG, &err_reg_data, 1, &dev);
    bmm350_error_codes_print_result("bmm350_get_error_reg_data", rslt);
    Serial.print("Error reg: 0x");
    Serial.println(err_reg_data, HEX);

    // Configure sensor
    bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_8, &dev);
    bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
    dev.delay_us(10000, dev.intf_ptr);
    bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);

    // Read a few samples before self-test
    Serial.println("\n********** MAGNETOMETER READINGS **********");
    for (uint8_t i = 0; i < 10; i++) {
        delay(100);
        rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &dev);
        if (rslt == BMM350_OK) {
            Serial.printf("%lu ms: X=%.2f Y=%.2f Z=%.2f T=%.2f\n",
                          millis(), mag_temp_data.x, mag_temp_data.y,
                          mag_temp_data.z, mag_temp_data.temperature);
        }
    }

    // Run self-test according to datasheet
    Serial.println("\n********** RUNNING SELF-TEST **********");
    check_self_test();
}

// -----------------------------
// Arduino loop()
// -----------------------------
void loop() {
    // Empty — everything runs in setup for demo
}
