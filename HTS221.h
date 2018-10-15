#ifndef HTS221_H
#define HTS221_H

#include <stdexcept>

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

static constexpr auto HTS221_DEVICE_ADDRESS		= 0x5F;

static constexpr auto HTS221_WHO_AM_I_REG		= 0x0F;
static constexpr auto HTS221_WHO_AM_I_RESPONSE	= 0xBC;

static constexpr auto HTS221_POWER_UP			= 0x80;
static constexpr auto HTS221_ODR_SET			= 0x01;
static constexpr auto HTS221_BDU_SET			= 0x04;
static constexpr auto HTS221_AVG_SET			= 0x1B;

static constexpr auto HTS221_AVG_REG			= 0x10;
static constexpr auto HTS221_CTRL_REG1			= 0x20;
static constexpr auto HTS221_CTRL_REG2			= 0x21;
static constexpr auto HTS221_CTRL_REG3			= 0x22;

static constexpr auto HTS221_STATUS_REG			= 0x27;
static constexpr auto HTS221_HUMIDITY_L_REG		= 0x28;
static constexpr auto HTS221_HUMIDITY_H_REG		= 0x29;
static constexpr auto HTS221_TEMPERATURE_L_REG	= 0x2A;
static constexpr auto HTS221_TEMPERATURE_H_REG	= 0x2B;

static constexpr auto HTS221_TEMPERATURE_READY	= 0x01;
static constexpr auto HTS221_HUMIDITY_READY		= 0x02;

static constexpr auto HTS221_CALIB_H0_RH_X2		= 0x30;
static constexpr auto HTS221_CALIB_H1_RH_X2		= 0x31;
static constexpr auto HTS221_CALIB_T0_DEGC_X8	= 0x32;
static constexpr auto HTS221_CALIB_T1_DEGC_X8	= 0x33;
static constexpr auto HTS221_CALIB_T1_T0_MSB	= 0x35;
static constexpr auto HTS221_CALIB_H0_T0_L		= 0x36;
static constexpr auto HTS221_CALIB_H0_T0_H		= 0x37;
static constexpr auto HTS221_CALIB_H1_T0_L		= 0x3A;
static constexpr auto HTS221_CALIB_H1_T0_H		= 0x3B;
static constexpr auto HTS221_CALIB_T0_OUT_L		= 0x3C;
static constexpr auto HTS221_CALIB_T0_OUT_H		= 0x3D;
static constexpr auto HTS221_CALIB_T1_OUT_L		= 0x3E;
static constexpr auto HTS221_CALIB_T1_OUT_H		= 0x3F;

class HTS221
{
public:
	explicit HTS221(const char* devicePath, uint8_t deviceAddress = HTS221_DEVICE_ADDRESS);
	virtual ~HTS221();

	void initialize();
	void shutdown();

	double getHumidity();
	double getTemperature();

protected:
	void calibrate();

private:
	int device;
	int32_t H0_RH_X2, H1_RH_X2, T0_DEGC_X8, T1_DEGC_X8, T1_T0_MSB, H0_T0, H1_T0, T0_OUT, T1_OUT;
};

#endif // #ifndef HTS221_H