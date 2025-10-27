#include <Arduino.h>
#include <Wire.h>
#include "bmm350.h"

#define BMM350_I2C_ADDR 0x14  // double-check your module address

struct bmm350_dev dev;

// -------- I2C helpers --------
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    Wire.beginTransmission(BMM350_I2C_ADDR);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) return -1;

    Wire.requestFrom(BMM350_I2C_ADDR, (uint8_t)len);
    for (uint32_t i = 0; i < len && Wire.available(); i++)
        reg_data[i] = Wire.read();
    return 0;
}

int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    Wire.beginTransmission(BMM350_I2C_ADDR);
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++) Wire.write(reg_data[i]);
    return Wire.endTransmission();
}

void delay_us(uint32_t period, void *intf_ptr) {
    delayMicroseconds(period);
}

// -------- Arduino setup --------
void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin();  // SDA=21, SCL=22 on ESP32 DevKit

    // Init device struct
    dev.read = i2c_read;
    dev.write = i2c_write;
    dev.delay_us = delay_us;
    dev.intf_ptr = NULL;

    if (bmm350_init(&dev) != BMM350_OK) {
        Serial.println("BMM350 init failed!");
        while (1);
    }
    Serial.println("BMM350 initialized successfully!");

    // Enable all axes
    bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
    // Set output data rate and performance
    bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_8, &dev);
    // Set normal power mode
    bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
}

// -------- Arduino loop --------
void loop() {
    struct bmm350_mag_temp_data mag_temp;

    // Read data directly
    if (bmm350_get_compensated_mag_xyz_temp_data(&mag_temp, &dev) == BMM350_OK) {
        Serial.print("X: "); Serial.print(mag_temp.x, 2);
        Serial.print("  Y: "); Serial.print(mag_temp.y, 2);
        Serial.print("  Z: "); Serial.print(mag_temp.z, 2);
        Serial.print("  Temp: "); Serial.println(mag_temp.temperature, 2);
    } else {
        Serial.println("Error reading BMM350!");
    }

    delay(200);  // adjust polling speed as needed
}
