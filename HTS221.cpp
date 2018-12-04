#include "HTS221.h"

HTS221::HTS221(const char* devicePath, uint8_t deviceAddress)
{
    if ((device = open(devicePath, O_RDWR)) < 0) {
        throw std::runtime_error("Failed to open the I2C bus.");
    }

    if (ioctl(device, I2C_SLAVE, deviceAddress) < 0) {
        close(device);
        throw std::runtime_error("Failed to configure the I2C device.");
    }

    int32_t data = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_WHO_AM_I);

    if (data != 0xBC) {
        throw std::runtime_error("Failed to verify the device identity.");
    }

    data = 0;

    data |= HTS221_BDU_ENABLE;
    data |= HTS221_DATARATE_1_HZ;

    i2c_smbus_write_byte_data(device, HTS221_REGISTER_CTRL_REG1, (uint8_t)data);

    data = 0;

    data |= HTS221_AVERAGE_HUMIDITY_4_SAMPLES;
    data |= HTS221_AVERAGE_TEMPERATURE_2_SAMPLES;

    i2c_smbus_write_byte_data(device, HTS221_REGISTER_AV_CONF, (uint8_t)data);
}

void HTS221::calibrate()
{
    int32_t data = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_H0_RH_X2);
    H0_RH_X2 = data;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_H1_RH_X2);
    H1_RH_X2 = data;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_T0_DEGC_X8);
    T0_DEGC_X8 = data;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_T1_T0_MSB);
    T0_DEGC_X8 |= (data & 0x03) << 8;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_T1_DEGC_X8);
    T1_DEGC_X8 = data;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_T1_T0_MSB);
    T1_DEGC_X8 |= (data & 0x0C) << 6;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_H0_T0_OUT_L);
    H0_T0 = data;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_H0_T0_OUT_H);
    H0_T0 |= data << 8;

    if (H0_T0 > 32768) {
        H0_T0 -= 65536;
    }

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_H1_T0_OUT_L);
    H1_T0 = data;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_H1_T0_OUT_H);
    H1_T0 |= data << 8;

    if (H1_T0 > 32768) {
        H1_T0 -= 65536;
    }

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_T0_OUT_L);
    T0_OUT = data;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_T0_OUT_H);
    T0_OUT |= data << 8;

    if (T0_OUT > 32768) {
        T0_OUT -= 65536;
    }

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_T1_OUT_L);
    T1_OUT = data;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CALIB_T1_OUT_H);
    T1_OUT |= data << 8;

    if (T1_OUT > 32768) {
        T1_OUT -= 65536;
    }
}

void HTS221::disableHeater()
{
    int32_t data = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CTRL_REG2);

    data &= ~(0b1 << 1);
    data |= HTS221_HEATER_DISABLE;

    i2c_smbus_write_byte_data(device, HTS221_REGISTER_CTRL_REG2, (uint8_t)data);
}

void HTS221::enableHeater()
{
    int32_t data = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CTRL_REG2);

    data &= ~(0b1 << 1);
    data |= HTS221_HEATER_ENABLE;

    i2c_smbus_write_byte_data(device, HTS221_REGISTER_CTRL_REG2, (uint8_t)data);
}

double HTS221::getHumidity()
{
    int32_t data = 0;
    int32_t humidity = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_STATUS_REG);

    if (!(data & 0b10)) {
        throw std::runtime_error("The humidity sensor is not ready.");
    }

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_HUMIDITY_OUT_L);
    humidity = data;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_HUMIDITY_OUT_H);
    humidity |= data << 8;

    if (humidity > 32768) {
        humidity -= 65536;
    }

    return H0_RH_X2 / 2.0 + (humidity - H0_T0) * (H1_RH_X2 - H0_RH_X2) / 2.0 / (H1_T0 - H0_T0);
}

double HTS221::getTemperature()
{
    int32_t data = 0;
    int32_t temperature = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_STATUS_REG);

    if (!(data & 0b01)) {
        throw std::runtime_error("The temperature sensor is not ready.");
    }

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_TEMP_OUT_L);
    temperature = data;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_TEMP_OUT_H);
    temperature |= data << 8;

    if (temperature > 32768) {
        temperature -= 65536;
    }

    return T0_DEGC_X8 / 8.0 + (temperature - T0_OUT) * (T1_DEGC_X8 - T0_DEGC_X8) / 8.0 / (T1_OUT - T0_OUT);
}

void HTS221::powerDown()
{
    int32_t data = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CTRL_REG1);

    data &= ~(0b1 << 7);
    data |= HTS221_POWER_DOWN;

    i2c_smbus_write_byte_data(device, HTS221_REGISTER_CTRL_REG1, (uint8_t)data);
}

void HTS221::powerUp()
{
    int32_t data = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CTRL_REG1);

    data &= ~(0b1 << 7);
    data |= HTS221_POWER_UP;

    i2c_smbus_write_byte_data(device, HTS221_REGISTER_CTRL_REG1, (uint8_t)data);
}

void HTS221::setAverageHumiditySamples(HTS221AverageHumiditySamples_t averageHumiditySamples)
{
    int32_t data = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_AV_CONF);

    data &= ~0b111;
    data |= averageHumiditySamples;

    i2c_smbus_write_byte_data(device, HTS221_REGISTER_AV_CONF, (uint8_t)data);
}

void HTS221::setAverageTemperatureSamples(HTS221AverageTemperatureSamples_t averageTemperatureSamples)
{
    int32_t data = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_AV_CONF);

    data &= ~(0b111 << 3);
    data |= averageTemperatureSamples;

    i2c_smbus_write_byte_data(device, HTS221_REGISTER_AV_CONF, (uint8_t)data);
}

void HTS221::setDataRate(HTS221DataRate_t dataRate)
{
    int32_t data = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CTRL_REG1);

    data &= ~0b11;
    data |= dataRate;

    i2c_smbus_write_byte_data(device, HTS221_REGISTER_CTRL_REG1, (uint8_t)data);
}

void HTS221::triggerMeasurement()
{
    int32_t data = 0;

    data = i2c_smbus_read_byte_data(device, HTS221_REGISTER_CTRL_REG1);

    if ((data & 0b11) == HTS221_DATARATE_ONE_SHOT) {
        i2c_smbus_write_byte_data(device, HTS221_REGISTER_CTRL_REG2, 0b1);
    }
}

HTS221::~HTS221()
{
    close(device);
}
