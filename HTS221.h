#ifndef HTS221_H
#define HTS221_H

#include <stdexcept>

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

static constexpr auto HTS221_DEVICE_ADDRESS = 0x5F;

class HTS221
{
public:
    explicit HTS221(const char* devicePath, uint8_t deviceAddress = HTS221_DEVICE_ADDRESS);
    virtual ~HTS221();

    typedef enum
    {
        HTS221_REGISTER_WHO_AM_I                = 0x0F,
        HTS221_REGISTER_AV_CONF                 = 0x10,
        HTS221_REGISTER_CTRL_REG1               = 0x20,
        HTS221_REGISTER_CTRL_REG2               = 0x21,
        HTS221_REGISTER_CTRL_REG3               = 0x22,
        HTS221_REGISTER_STATUS_REG              = 0x27,
        HTS221_REGISTER_HUMIDITY_OUT_L          = 0x28,
        HTS221_REGISTER_HUMIDITY_OUT_H          = 0x29,
        HTS221_REGISTER_TEMP_OUT_L              = 0x2A,
        HTS221_REGISTER_TEMP_OUT_H              = 0x2B,
        HTS221_REGISTER_CALIB_H0_RH_X2          = 0x30,
        HTS221_REGISTER_CALIB_H1_RH_X2          = 0x31,
        HTS221_REGISTER_CALIB_T0_DEGC_X8        = 0x32,
        HTS221_REGISTER_CALIB_T1_DEGC_X8        = 0x33,
        HTS221_REGISTER_CALIB_T1_T0_MSB         = 0x35,
        HTS221_REGISTER_CALIB_H0_T0_OUT_L       = 0x36,
        HTS221_REGISTER_CALIB_H0_T0_OUT_H       = 0x37,
        HTS221_REGISTER_CALIB_H1_T0_OUT_L       = 0x3A,
        HTS221_REGISTER_CALIB_H1_T0_OUT_H       = 0x3B,
        HTS221_REGISTER_CALIB_T0_OUT_L          = 0x3C,
        HTS221_REGISTER_CALIB_T0_OUT_H          = 0X3D,
        HTS221_REGISTER_CALIB_T1_OUT_L          = 0x3E,
        HTS221_REGISTER_CALIB_T1_OUT_H          = 0x3F,
    } HTS221Registers_t;

    typedef enum
    {
        HTS221_AVERAGE_HUMIDITY_4_SAMPLES       = 0b000,
        HTS221_AVERAGE_HUMIDITY_8_SAMPLES       = 0b001,
        HTS221_AVERAGE_HUMIDITY_16_SAMPLES      = 0b010,
        HTS221_AVERAGE_HUMIDITY_32_SAMPLES      = 0b011,
        HTS221_AVERAGE_HUMIDITY_64_SAMPLES      = 0b100,
        HTS221_AVERAGE_HUMIDITY_128_SAMPLES     = 0b101,
        HTS221_AVERAGE_HUMIDITY_256_SAMPLES     = 0b110,
        HTS221_AVERAGE_HUMIDITY_512_SAMPLES     = 0b111,
    } HTS221AverageHumiditySamples_t;

    typedef enum
    {
        HTS221_AVERAGE_TEMPERATURE_2_SAMPLES    = (0b000 << 3),
        HTS221_AVERAGE_TEMPERATURE_4_SAMPLES    = (0b001 << 3),
        HTS221_AVERAGE_TEMPERATURE_8_SAMPLES    = (0b010 << 3),
        HTS221_AVERAGE_TEMPERATURE_16_SAMPLES   = (0b011 << 3),
        HTS221_AVERAGE_TEMPERATURE_32_SAMPLES   = (0b100 << 3),
        HTS221_AVERAGE_TEMPERATURE_64_SAMPLES   = (0b101 << 3),
        HTS221_AVERAGE_TEMPERATURE_128_SAMPLES  = (0b110 << 3),
        HTS221_AVERAGE_TEMPERATURE_256_SAMPLES  = (0b111 << 3),
    } HTS221AverageTemperatureSamples_t;

    typedef enum
    {
        HTS221_DATARATE_ONE_SHOT                = 0b00,
        HTS221_DATARATE_1_HZ                    = 0b01,
        HTS221_DATARATE_7_HZ                    = 0b10,
        HTS221_DATARATE_12_5_HZ                 = 0b11,
    } HTS221DataRate_t;

    typedef enum
    {
        HTS221_BDU_DISABLE                      = (0b0 << 2),
        HTS221_BDU_ENABLE                       = (0b1 << 2),
    } HTS221DataUpdateMode_t;

    typedef enum
    {
        HTS221_POWER_DOWN                       = (0b0 << 7),
        HTS221_POWER_UP                         = (0b1 << 7),
    } LPS25HPowerMode_t;

    double getHumidity();
    double getTemperature();

    void calibrate();
    void triggerMeasurement();

    void powerDown();
    void powerUp();

    void setAverageHumiditySamples(HTS221AverageHumiditySamples_t averageHumiditySamples);
    void setAverageTemperatureSamples(HTS221AverageTemperatureSamples_t averageTemperatureSamples);
    void setDataRate(HTS221DataRate_t dataRate);

private:
    int device;
    int32_t H0_RH_X2, H1_RH_X2, T0_DEGC_X8, T1_DEGC_X8, T1_T0_MSB, H0_T0, H1_T0, T0_OUT, T1_OUT;
};

#endif // #ifndef HTS221_H