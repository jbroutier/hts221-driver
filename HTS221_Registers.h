#ifndef HTS221_REGISTERS_H
#define HTS221_REGISTERS_H

static constexpr auto HTS221_REGISTER_WHO_AM_I          = 0x0F;
static constexpr auto HTS221_REGISTER_AV_CONF           = 0x10;
static constexpr auto HTS221_REGISTER_CTRL_REG1         = 0x20;
static constexpr auto HTS221_REGISTER_CTRL_REG2         = 0x21;
static constexpr auto HTS221_REGISTER_CTRL_REG3         = 0x22;
static constexpr auto HTS221_REGISTER_STATUS_REG        = 0x27;
static constexpr auto HTS221_REGISTER_HUMIDITY_OUT_L    = 0x28;
static constexpr auto HTS221_REGISTER_HUMIDITY_OUT_H    = 0x29;
static constexpr auto HTS221_REGISTER_TEMP_OUT_L        = 0x2A;
static constexpr auto HTS221_REGISTER_TEMP_OUT_H        = 0x2B;
static constexpr auto HTS221_REGISTER_CALIB_H0_RH_X2    = 0x30;
static constexpr auto HTS221_REGISTER_CALIB_H1_RH_X2    = 0x31;
static constexpr auto HTS221_REGISTER_CALIB_T0_DEGC_X8  = 0x32;
static constexpr auto HTS221_REGISTER_CALIB_T1_DEGC_X8  = 0x33;
static constexpr auto HTS221_REGISTER_CALIB_T1_T0_MSB   = 0x35;
static constexpr auto HTS221_REGISTER_CALIB_H0_T0_OUT_L = 0x36;
static constexpr auto HTS221_REGISTER_CALIB_H0_T0_OUT_H = 0x37;
static constexpr auto HTS221_REGISTER_CALIB_H1_T0_OUT_L = 0x3A;
static constexpr auto HTS221_REGISTER_CALIB_H1_T0_OUT_H = 0x3B;
static constexpr auto HTS221_REGISTER_CALIB_T0_OUT_L    = 0x3C;
static constexpr auto HTS221_REGISTER_CALIB_T0_OUT_H    = 0X3D;
static constexpr auto HTS221_REGISTER_CALIB_T1_OUT_L    = 0x3E;
static constexpr auto HTS221_REGISTER_CALIB_T1_OUT_H    = 0x3F;

#endif // #ifndef HTS221_REGISTERS_H
