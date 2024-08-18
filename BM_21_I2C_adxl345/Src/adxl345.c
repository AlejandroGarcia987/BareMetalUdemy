/*
 * adxl345.c
 *
 *  Created on: Aug 18, 2024
 *      Author: alexg
 */

#include "adxl345.h"

char data;
uint8_t data_rec[6];

void adxl_read_address (uint8_t reg){

	I2C1_byteRead(DEVICE_ADDR, reg, &data);

}

void adxl_write (uint8_t reg, char value){

	char data[1];
	data[0] = value;

	I2C1_burstWrite(DEVICE_ADDR, reg, 1, data);

}

void adxl_read_values (uint8_t reg)
{
	I2C1_burstRead(DEVICE_ADDR, reg, 6,(char*)data_rec);
}

void adxl_init(void)
{
	/*Iniciamos el I2C*/
	I2C1_init();
	/*Leemos el ID del acc*/
	/*Debería devolver 0xE5*/
	adxl_read_address(DEVID_R);

	/*ponemos el acc a medir en +-4g*/
	adxl_write(DATA_FORMAT_R, FOUR_G);

	/*Reseteamos todos los bits*/
	adxl_write(POWER_CTRL_R, RESET);

	/*Configuramos elbit de power control*/
	adxl_write(POWER_CTRL_R, SET_MEASURE_R);


}
