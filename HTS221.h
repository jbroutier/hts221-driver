#ifndef HTS221_H
#define HTS221_H

#include <stdexcept>

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "HTS221_Registers.h"
#include "HTS221_Types.h"

static constexpr auto HTS221_DEVICE_ADDRESS = 0x5F;

class HTS221
{
public:
    explicit HTS221(const char* devicePath, uint8_t deviceAddress = HTS221_DEVICE_ADDRESS);
    virtual ~HTS221();

    void calibrate();
    void disableHeater();
    void enableHeater();

    double getHumidity();
    double getTemperature();

    void powerDown();
    void powerUp();

    void setAverageHumiditySamples(HTS221AverageHumiditySamples_t averageHumiditySamples);
    void setAverageTemperatureSamples(HTS221AverageTemperatureSamples_t averageTemperatureSamples);
    void setDataRate(HTS221DataRate_t dataRate);
    
    void triggerMeasurement();

private:
    int device;
    int32_t H0_RH_X2, H1_RH_X2, T0_DEGC_X8, T1_DEGC_X8, T1_T0_MSB, H0_T0, H1_T0, T0_OUT, T1_OUT;
};

#endif // #ifndef HTS221_H
