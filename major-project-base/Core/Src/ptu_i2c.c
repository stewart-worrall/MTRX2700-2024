#include "ptu_i2c.h"

#include "ptu_definitions.h"

ACCELEROMETER_CFG_STRUCT accelerometer_cfg = {ADXL345_POWER_CTL, 0x08, ADXL345_DATA_FORMAT, 0x08};

void initialise_ptu_i2c(I2C_HandleTypeDef *i2c) {

	HAL_StatusTypeDef return_value = 0x00;

	uint8_t reg_1 = 0b00001111;
	// Enable x, y, z and turn off power down:
	return_value = HAL_I2C_Mem_Write(i2c, gyro_wr, L3G4200D_CTRL_REG1, 1, &reg_1, 1, 10);

	uint8_t reg_2 = 0b00000000;
	// If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
	return_value = HAL_I2C_Mem_Write(i2c, gyro_wr, L3G4200D_CTRL_REG2, 1, &reg_2, 1, 10);

	// Configure CTRL_REG3 to generate data ready interrupt on INT2
	// No interrupts used on INT1, if you'd like to configure INT1
	// or INT2 otherwise, consult the datasheet:
	return_value = HAL_I2C_Mem_Write(i2c, gyro_wr, L3G4200D_CTRL_REG3, 1, &reg_2, 1, 10);

	// CTRL_REG4 controls the full-scale range, among other things:
	//if(scale == 250){
	////////HAL_I2C_Mem_Write(&hi2c1, gyro_wr, L3G4200D_CTRL_REG4, 1, &reg_2, 1, 10);
	//}else if(scale == 500){
	//writeRegister(gyro_Address, gyro_CTRL_REG4, 0b00010000);
	//}else{
	//writeRegister(gyro_Address, gyro_CTRL_REG4, 0b00110000);
	//}

	// CTRL_REG5 controls high-pass filtering of outputs, use it
	// if you'd like:
	//HAL_I2C_Mem_Write(&hi2c1, gyro_wr, L3G4200D_CTRL_REG5, 1, &reg_2, 10);

	// reset lidar board
	uint8_t reset_value = 0x00;
	return_value = HAL_I2C_Mem_Write(i2c, LIDAR_WR, 0x00, 1, &reset_value, 1, 10);
}
