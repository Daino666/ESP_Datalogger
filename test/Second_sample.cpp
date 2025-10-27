#include <Arduino.h>
#include <Wire.h>
#include "bmm350.h"

// -----------------------------
// I2C address of BMM350
// -----------------------------
#define BMM350_I2C_ADDR  0x14   // default

// -----------------------------
// Forward declarations
// -----------------------------
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void   user_delay_us(uint32_t period, void *intf_ptr);
void   bmm350_error_codes_print_result(const char *api_name, int8_t rslt);

// -----------------------------
// Global BMM350 device struct
// -----------------------------
struct bmm350_dev dev;
uint8_t dev_addr = BMM350_I2C_ADDR;

void setup()
{
    Serial.begin(115200);
    Wire.begin();  // SDA, SCL defaults

    // Setup BMM350 device struct
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;
    dev.intf_ptr = &dev_addr;

    // Initialize sensor
    int8_t rslt = bmm350_init(&dev);
    bmm350_error_codes_print_result("bmm350_init", rslt);

    if (rslt == BMM350_OK) {
        Serial.println("BMM350 initialized successfully!");
    } else {
        Serial.println("BMM350 init failed!");
        while (1) delay(1000);
    }

    // Set output data rate and performance
    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_100HZ, BMM350_AVERAGING_4, &dev);
    bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

    // Enable axes
    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
    bmm350_error_codes_print_result("bmm350_enable_axes", rslt);

    // Set power mode
    rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
    bmm350_error_codes_print_result("bmm350_set_powermode", rslt);
}

void loop()
{
    struct bmm350_mag_temp_data mag_temp_data;
    int8_t rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &dev);

    if (rslt == BMM350_OK) {
        Serial.print("X: ");
        Serial.print(mag_temp_data.x, 2);
        Serial.print("  Y: ");
        Serial.print(mag_temp_data.y, 2);
        Serial.print("  Z: ");
        Serial.print(mag_temp_data.z, 2);
        Serial.print("  Temp: ");
        Serial.println(mag_temp_data.temperature, 2);
    } else {
        Serial.println("Failed to read data!");
    }

    delay(200); // print at ~5Hz
}

// -----------------------------
// Helper functions
// -----------------------------

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0)
        return -1;

    Wire.requestFrom((int)dev_addr, (int)len);
    for (uint32_t i = 0; i < len && Wire.available(); i++) {
        data[i] = Wire.read();
    }
    return 0;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    return Wire.endTransmission();
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
    delayMicroseconds(period);
}

void bmm350_error_codes_print_result(const char *api_name, int8_t rslt)
{
    if (rslt != BMM350_OK) {
        Serial.print("API ");
        Serial.print(api_name);
        Serial.print(" returned error code: ");
        Serial.println(rslt);
    }
}
    