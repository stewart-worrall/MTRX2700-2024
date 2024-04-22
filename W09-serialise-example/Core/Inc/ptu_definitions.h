/*
 * ptu_definitions.h
 *
 *  Created on: Apr 14, 2023
 *      Author: stew
 */

#ifndef INC_PTU_DEFINITIONS_H_
#define INC_PTU_DEFINITIONS_H_

#include <stdint.h>

#define laser_wr  0xc4
#define laser_rd  0xc5

#define gyro_wr 0xD2
#define gyro_rd 0xD3

#define accel_wr 0xA6    //
#define accel_rd 0xA7    //

#define ADXL345_TO_READ 6

#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATAX0 0x32
#define ADXL345_DATA_FORMAT 0x31

#define ADXL345_OFSX 0x1E
#define ADXL345_OFSY 0x1F
#define ADXL345_OFSZ 0x20

//#define ALPHA 0.5

#define magnet_wr  0x3C
#define magnet_rd  0x3D

#define HM5883_MODE_REG 0x02
#define HM5883_DATAX0 0x03



#define L3G4200D_WHO_AM_I      0x0F

#define L3G4200D_CTRL_REG1     0x20
#define L3G4200D_CTRL_REG2     0x21
#define L3G4200D_CTRL_REG3     0x22
#define L3G4200D_CTRL_REG4     0x23
#define L3G4200D_CTRL_REG5     0x24
#define L3G4200D_REFERENCE     0x25
#define L3G4200D_OUT_TEMP      0x26
#define L3G4200D_STATUS_REG    0x27

#define L3G4200D_OUT_X_L       0x28
#define L3G4200D_OUT_X_H       0x29
#define L3G4200D_OUT_Y_L       0x2A
#define L3G4200D_OUT_Y_H       0x2B
#define L3G4200D_OUT_Z_L       0x2C
#define L3G4200D_OUT_Z_H       0x2D

#define L3G4200D_FIFO_CTRL_REG 0x2E
#define L3G4200D_FIFO_SRC_REG  0x2F

#define L3G4200D_INT1_CFG      0x30
#define L3G4200D_INT1_SRC      0x31
#define L3G4200D_INT1_THS_XH   0x32
#define L3G4200D_INT1_THS_XL   0x33
#define L3G4200D_INT1_THS_YH   0x34
#define L3G4200D_INT1_THS_YL   0x35
#define L3G4200D_INT1_THS_ZH   0x36
#define L3G4200D_INT1_THS_ZL   0x37
#define L3G4200D_INT1_DURATION 0x38

#define L3G4200D_OUT_XYZ_CONT  0xA8



#define HAL_I2C_TIMEOUT_MAX 100  // timeout in milliseconds

#define gyro_wr 0xD2
#define gyro_rd 0xD3

#define I2CSlaveAddressWrite 0x32
#define I2CSlaveAddressRead 0x32


typedef struct GYRO_CFG_STRUCT {
  uint8_t ctl_register;
  uint8_t ctl_value;
} GYRO_CFG_STRUCT;


// data structures containing the raw values
typedef struct AccelRaw {
  uint16_t x;
  uint16_t y;
  uint16_t z;
} AccelRaw;

// structure containing the config parameters for the accelerometer
typedef struct ACCELEROMETER_CFG_STRUCT {
  uint8_t power_ctl_register;
  uint8_t power_ctl_value;
  uint8_t data_format_register;
  uint8_t data_format_value;
} ACCELEROMETER_CFG_STRUCT;



// LIDAR lite code

#define LIDARLITE_ADDR_DEFAULT 0x62
#define LIDAR_WR 0xC4
#define LIDAR_RD 0xC5


#endif /* INC_PTU_DEFINITIONS_H_ */
