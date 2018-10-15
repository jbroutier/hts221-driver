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

	data = i2c_smbus_read_byte_data(device, HTS221_WHO_AM_I_REG);

	if (!(data & HTS221_WHO_AM_I_RESPONSE)) {
		throw std::runtime_error("Failed to verify the device identity.");
	}

	i2c_smbus_write_byte_data(device, HTS221_CTRL_REG1, 0x00);
	i2c_smbus_write_byte_data(device, HTS221_CTRL_REG1, HTS221_ODR_SET + HTS221_BDU_SET);
	i2c_smbus_write_byte_data(device, HTS221_AVG_REG, HTS221_AVG_SET);
}

void HTS221::calibrate()
{
	int32_t data = 0;

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_H0_RH_X2);
	H0_RH_X2 = data;

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_H1_RH_X2);
	H1_RH_X2 = data;

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_T0_DEGC_X8);
	T0_DEGC_X8 = data;

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_T1_T0_MSB);
	T0_DEGC_X8 |= (data & 0x03) << 8;

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_T1_DEGC_X8);
	T1_DEGC_X8 = data;

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_T1_T0_MSB);
	T1_DEGC_X8 |= (data & 0x0C) << 6;

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_H0_T0_L);
	H0_T0 = data;

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_H0_T0_H);
	H0_T0 |= data << 8;

	if (H0_T0 > 32768) {
		H0_T0 -= 65536;
	}

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_H1_T0_L);
	H1_T0 = data;

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_H1_T0_H);
	H1_T0 |= data << 8;

	if (H1_T0 > 32768) {
		H1_T0 -= 65536;
	}

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_T0_OUT_L);
	T0_OUT = data;

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_T0_OUT_H);
	T0_OUT |= data << 8;

	if (T0_OUT > 32768) {
		T0_OUT -= 65536;
	}

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_T1_OUT_L);
	T1_OUT = data;

	data = i2c_smbus_read_byte_data(device, HTS221_CALIB_T1_OUT_H);
	T1_OUT |= data << 8;

	if (T1_OUT > 32768) {
		T1_OUT -= 65536;
	}
}

double HTS221::getHumidity()
{
	int32_t data = 0;
	int32_t humidity = 0;

	data = i2c_smbus_read_byte_data(device, HTS221_STATUS_REG);

	if (!(data & HTS221_HUMIDITY_READY)) {
		throw std::runtime_error("The humidity sensor is not ready.");
	}

	data = i2c_smbus_read_byte_data(device, HTS221_HUMIDITY_L_REG);
	humidity = data;

	data = i2c_smbus_read_byte_data(device, HTS221_HUMIDITY_H_REG);
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

	data = i2c_smbus_read_byte_data(device, HTS221_STATUS_REG);

	if (!(data & HTS221_TEMPERATURE_READY)) {
		throw std::runtime_error("The temperature sensor is not ready.");
	}

	data = i2c_smbus_read_byte_data(device, HTS221_TEMPERATURE_L_REG);
	temperature = data;

	data = i2c_smbus_read_byte_data(device, HTS221_TEMPERATURE_H_REG);
	temperature |= data << 8;

	if (temperature > 32768) {
		temperature -= 65536;
	}

	return T0_DEGC_X8 / 8.0 + (temperature - T0_OUT) * (T1_DEGC_X8 - T0_DEGC_X8) / 8.0 / (T1_OUT - T0_OUT);
}

void HTS221::initialize()
{
	int32_t data = 0;

	data = i2c_smbus_read_byte_data(device, HTS221_CTRL_REG1);

	data |= HTS221_POWER_UP;

	i2c_smbus_write_byte_data(device, HTS221_CTRL_REG1, (uint8_t)data);

	calibrate();
}

void HTS221::shutdown()
{
	int32_t data = 0;

	data = i2c_smbus_read_byte_data(device, HTS221_CTRL_REG1);

	data &= ~HTS221_POWER_UP;

	i2c_smbus_write_byte_data(device, HTS221_CTRL_REG1, (uint8_t)data);
}

HTS221::~HTS221()
{
	close(device);
}