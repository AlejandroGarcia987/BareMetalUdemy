/*
 * Lo que hemos hecho en este programa es separar el código y hacerlo más modular.
 * Para ello hemos creado el uart.c, que contiene las funciones de uart escritas anteriormente, y el uart.h, que se
 * utiliza para llamar a la función UART de transmisión.
 */

#include "stdio.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "uart.h"
#include "adxl345.h"


int16_t x, y, z;
float xg, yg, zg;

extern uint8_t data_rec[6];


int main(void)
{
	adxl_init();

	while(1)
	{
		adxl_read_values(DATA_START_ADDR);

		x = ((data_rec[1]<<8) |data_rec[0]);
		y = ((data_rec[3]<<8) |data_rec[2]);
		z = ((data_rec[5]<<8) |data_rec[4]);

		xg = x * 0.0078;
		yg = y * 0.0078;
		zg = z * 0.0078;
	}
}
