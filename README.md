# HTS221 driver

## Description

A C++ driver for the HTS221 humidity and temperature sensor manufactured by STMicroelectronics and used in the
RaspberryPi Sense Hat board.

## License

This piece of software is available under the terms of the [MIT License](LICENSE).

## Changelog

**r1.0.0**

* Initial release.

**r1.0.1**

* Fixed a misnamed enum.

## Example

```cpp
#include "HTS221.h"
#include <iostream>

int main(int argc, char** argv)
{
    double humidity, temperature;

    HTS221 sensor("/dev/i2c-1");
    sensor.calibrate();

    humidity = sensor.getHumidity();
    std::cout << "Humidity: " << humidity << " % RH" << std::endl;

    temperature = sensor.getTemperature();
    std::cout << "Temperature: " << temperature << " °C" << std::endl;

    return 0;
}
```

## Documentation

#### `HTS221::HTS221(const char* devicePath, uint8_t deviceAddress = HTS221_DEVICE_ADDRESS)`
___
Creates a HTS221 object.

The *devicePath* parameter corresponds to the path of the I2C bus on the system.  
The optional parameter *deviceAddress* allows you to override the address of the sensor, in case you are using a
standalone sensor or a modified version of the Sense Hat board.

Upon creation of the object, the sensor is configured with the following options:

|Option|Value|
|--|--|
|Output data rate|HTS221_DATARATE_1_HZ|
|Humidity samples|HTS221_AVERAGE_HUMIDITY_4_SAMPLES|
|Temperature samples|HTS221_AVERAGE_TEMPERATURE_2_SAMPLES|

Throws an `std::runtime_error` exception if the communication with the sensor fails for any reason.

### `HTS221::~HTS221()`
___
Destroys the object, closing the connection to the sensor.

### `void HTS221::calibrate()`
___
Calibrates the sensor.

You must call this method at least once before any measurement attempt.

### `void HTS221::disableHeater()`
___
Turns off the internal heating element of the sensor.

### `void HTS221::enableHeater()`
___
Turns on the internal heating element of the sensor.

The heating element can be used to speed up the sensor recovery time in case of condensation.  
You shouldn't try to read humidity and temperature values while the heating element is working.

### `double HTS221::getHumidity()`
___
Returns the humidity value measured by the sensor, in percentage of relative humidity.

Throws an `std::runtime_error` exception if the humidity value can't be read.

### `double HTS221::getTemperature()`
___
Returns the temperature value measured by the sensor, in Celsius degrees.

Throws an `std::runtime_error` exception if the temperature value can't be read.

### `void HTS221::powerDown()`
___
Turns the sensor off, to reduce power consumption.

### `void HTS221::powerUp()`
___
Turns the sensor on.

### `void HTS221::setAverageHumiditySamples(HTS221AverageHumiditySamples_t averageHumiditySamples)`
___
Sets the number of samples used to compute the humidity value.

The *averageHumiditySamples* parameter can take one of the following values:

|Value|
|--|
|HTS221_AVERAGE_HUMIDITY_4_SAMPLES|
|HTS221_AVERAGE_HUMIDITY_8_SAMPLES|
|HTS221_AVERAGE_HUMIDITY_16_SAMPLES|
|HTS221_AVERAGE_HUMIDITY_32_SAMPLES|
|HTS221_AVERAGE_HUMIDITY_64_SAMPLES|
|HTS221_AVERAGE_HUMIDITY_128_SAMPLES|
|HTS221_AVERAGE_HUMIDITY_256_SAMPLES|
|HTS221_AVERAGE_HUMIDITY_512_SAMPLES|

### `void HTS221::setAverageTemperatureSamples(HTS221AverageTemperatureSamples_t averageTemperatureSamples)`
___
Sets the number of samples used to compute the temperature value.

The *averageTemperatureSamples* parameter can take one of the following values:

|Value|
|--|
|HTS221_AVERAGE_TEMPERATURE_2_SAMPLES|
|HTS221_AVERAGE_TEMPERATURE_4_SAMPLES|
|HTS221_AVERAGE_TEMPERATURE_8_SAMPLES|
|HTS221_AVERAGE_TEMPERATURE_16_SAMPLES|
|HTS221_AVERAGE_TEMPERATURE_32_SAMPLES|
|HTS221_AVERAGE_TEMPERATURE_64_SAMPLES|
|HTS221_AVERAGE_TEMPERATURE_128_SAMPLES|
|HTS221_AVERAGE_TEMPERATURE_256_SAMPLES|

### `void HTS221::setDataRate(HTS221DataRate_t dataRate)`
___
Sets the output data rate of the sensor.

The *dataRate* parameter can take one of the following values:

|Value|Measurement interval|
|--|--|
|HTS221_DATARATE_ONE_SHOT|Manual|
|HTS221_DATARATE_1_HZ|1000 ms|
|HTS221_DATARATE_7_HZ|142.8 ms|
|HTS221_DATARATE_12_5_HZ|80 ms|

Trying to read the humidity or temperature values more frequently than permitted by the measurement interval will raise
an exception.

### `void HTS221::triggerMeasurement()`
___
Manually triggers a measurement of humidity and temperature values.

If the output data rate of the sensor is set to `HTS221_DATARATE_ONE_SHOT`, you must call this method to trigger a
measurement before any attempt to read the humidity or temperature values.

## Useful resources

[HTS221 datasheet](https://www.st.com/resource/en/datasheet/hts221.pdf)
