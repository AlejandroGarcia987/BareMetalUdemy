/*
 * i2c.c
 *
 *  Created on: Aug 18, 2024
 *      Author: alexg
 */

#include "stm32f4xx.h"

#define GPIOBENR (1U<<1)
#define I2CENR (1U<<21)

#define I2C_100KHZ					80	//0B 0101 0000 = Decimal = 80
#define SD_MODE_MAX_RISE_TIME 		17
#define CR1_PE (1U<<0)

//#define S2R_BUSY   (1U<<1)
#define SR2_BUSY   (1U<<1)
#define CR1_START  (1U<<8)
#define SR1_SB	   (1U<<0)
#define SR1_ADDR   (1U<<1)
#define SR1_TXE    (1U<<7)
#define CR1_ACK    (1U<<10)
#define CR1_STOP   (1U<<9)
#define SR1_RXNE   (1U<<6)
#define SR1_BTF	   (1U<<2)

/*
 * Pinout:
 * PB8---> SCL
 * PB9---> SDA
 */
void I2C1_init(void)
{
	/*Enable clock access to GPIOB (We use PB8 and PB9 for I2C1)*/
	RCC->APB1ENR |= GPIOBENR;

	/*Set PB8 and PB9 Mode to alternate function*/
	/*PB8*/
	GPIOB->MODER &=~ (1U<<16);
	GPIOB->MODER |= (1U<<17);

	/*PB9*/
	GPIOB->MODER &=~ (1U<<18);
	GPIOB->MODER |= (1U<<19);

	/*Set PB8 and PB9 output type to open drain*/
	GPIOB->OTYPER |= (1U<<8);
	GPIOB->OTYPER |= (1U<<9);

	/*Enable pullup for PB8 and PB9*/
	/*PB8*/
	GPIOB->PUPDR |= (1U<<16);
	GPIOB->PUPDR &=~ (1U<<17);

	/*PB9*/
	GPIOB->PUPDR |= (1U<<18);
	GPIOB->PUPDR &=~ (1U<<19);

	/*Configurar PB8 y PB9 en modo AF4 para trabajar con I2C*/

	/*PB8*/
	GPIOB->AFR[1] &=~(1U<<0);
	GPIOB->AFR[1] &=~(1U<<1);
	GPIOB->AFR[1] |= (1U<<2);
	GPIOB->AFR[1] &=~(1U<<3);
	/*PB9*/
	GPIOB->AFR[1] &=~(1U<<4);
	GPIOB->AFR[1] &=~(1U<<5);
	GPIOB->AFR[1] |= (1U<<6);
	GPIOB->AFR[1] &=~(1U<<7);


	/*Enable clock access to I2C1*/
	RCC->APB1ENR |= I2CENR;

	/*Enable REST mode*/
	I2C1->CR1 |= (1U<<15);

	/*Come out of reset mode*/
	I2C1->CR1 &=~ (1U<<15);

//	/*Set peripheral clock freq to 16Mhz*/
	I2C1->CR2 |= (1U<<4);

	/*Set I2C to standar mode, 100Khz*/
	I2C1->CCR = I2C_100KHZ;
	//I2C1->CCR = I2C_400KHZ;

	/*Set Rise time*/
	I2C1->TRISE = SD_MODE_MAX_RISE_TIME;

	/*Enable I2C1 module*/
	I2C1->CR1 |= CR1_PE;

}


/*Creamos una función para leer un byte. Necesitamos los argumentos de la dirección del esclavo, dirección del maestro y los datos leídos*/

void I2C1_byteRead(char saddr, char maddr, char* data)
{
	volatile int tmp;

	//Esperamos hasta que el BUS no esté ocupado
	while(I2C1->SR2 & (SR2_BUSY)){}

	/*Generate start condition*/
	I2C1->CR1 |= CR1_START;

	/*Esperamos hasta que el flag es set*/
	while (!(I2C1->SR1 & (SR1_SB))){}
	//while ((I2C1->SR1 & (SR1_SB))){}

	/*Transmitir la dirección del esclavo + escritura*/
	I2C1->DR = saddr << 1;

	/*Esperar hasta que el flag de la dirección se establezca*/
	while((I2C1->SR1 & (SR1_ADDR))){}

	/*Limpiar el flag de addr*/
	tmp = I2C1->SR2;

	/*Enviar la dirección de memoria*/
	I2C1->DR = maddr;

	/*Esperar hasta que el envío esté vacío*/
	while(!(I2C1->SR1 & SR1_TXE)){}

	/*Generar el reinicio*/
	I2C1->CR1 |= CR1_START;

	/*Esperar hasta el flag de inicio se establezca*/
	while(!(I2C1->SR1 & SR1_SB)){}

	/*Transmitir la dirección del esclavo y leer*/
	I2C1->DR = saddr << 1 | 1 ;

	/*Esperar hasta el flag de dirección se establezca*/
	while(!(I2C1->SR1 & SR1_ADDR)){}

	/*Deshabilitar el acknowledge*/
	I2C1->CR1 &=~ CR1_ACK;

	/*Limpiar el flag de addr*/
	tmp = I2C1->SR2;

	/*Generate stop after data received*/
	I2C1->CR1 |= CR1_STOP;

	/*Esperar hasta que el flag RXNE esté establecido*/
	while(!(I2C1->SR1 & SR1_RXNE)){}

	/*Leer los datos desde DR*/
	*data++ = I2C1->DR;
}

void I2C1_burstRead(char saddr, char maddr,int n,char* data)
{
	volatile int tmp;

	//Esperamos hasta que el BUS no esté ocupado
	while(I2C1->SR2 & (SR2_BUSY)){}

	/*Generate start condition*/
	I2C1->CR1 |= CR1_START;

	/*Esperamos hasta que el flag es set*/
	while (!(I2C1->SR1 & (SR1_SB))){}

	/*Transmitir la dirección del esclavo + escritura*/
	I2C1->DR = saddr << 1;

	/*Esperar hasta que el flag de la dirección se establezca*/
	while(!(I2C1->SR1 & (SR1_ADDR))){}

	/*Limpiar el flag de addr*/
	tmp = I2C1->SR2;

	/*Esperar hasta que el envío esté vacío*/
	while(!(I2C1->SR1 & SR1_TXE)){}

	/*Enviar la dirección de memoria*/
	I2C1->DR = maddr;

	/*Esperar hasta que el envío esté vacío*/
	while(!(I2C1->SR1 & SR1_TXE)){}

	/*Generar el reinicio*/
	I2C1->CR1 |= CR1_START;

	/*Esperar hasta el flag de inicio se establezca*/
	while(!(I2C1->SR1 & SR1_SB)){}

	/*Transmitir la dirección del esclavo y leer*/
	I2C1->DR = saddr << 1 | 1 ;

	/*Esperar hasta el flag de dirección se establezca*/
	while(!(I2C1->SR1 & SR1_ADDR)){}

	/*Limpiar el flag de addr*/
	tmp = I2C1->SR2;

	/*Habilitar el acknowledge*/
	I2C1->CR1 |= CR1_ACK;

	while (n > 0U)
	{
		/*Si un byte*/
		if(n == 1U)
		{
			/*Deshabilitar acknowledge*/
			I2C1->CR1 &=~ CR1_ACK;

			/*Generar una parada*/
			I2C1->CR1 |= CR1_STOP;

			/*Wait for the RXNE flag is set*/
			while(!(I2C1->SR1 & SR1_RXNE)){}

			/*Leer los datos del DR*/
			*data++ = I2C1->DR;

			break;
		}
		else
		{
			while(!(I2C1->SR1 & SR1_RXNE)){}

			/*Leer los datos del DR*/
			(*data++) = I2C1->DR;

			n--;
		}
	}
}

void I2C1_burstWrite(char saddr, char maddr, int n, char* data)
{
	volatile int tmp;

	//Esperamos hasta que el BUS no esté ocupado
	while(I2C1->SR2 & (SR2_BUSY)){}

	/*Generate start condition*/
	I2C1->CR1 |= CR1_START;

	/*Esperamos hasta que el flag es set*/
	while (!(I2C1->SR1 & (SR1_SB))){}

	/*Transmitir la dirección del esclavo + escritura*/
	I2C1->DR = saddr << 1;

	/*Esperar hasta que el flag de la dirección se establezca*/
	while(!(I2C1->SR1 & (SR1_ADDR))){}

	/*Limpiar el flag de addr*/
	tmp = I2C1->SR2;

	/*Esperar hasta que el envío esté vacío*/
	while(!(I2C1->SR1 & SR1_TXE)){}

	/*Enviar la dirección de memoria*/
	I2C1->DR = maddr;

	for(int i = 0; i < n; i++)
	{
		/*Esperar hasta que el data register esté vacío*/
		while(!(I2C1->SR1 & SR1_TXE)){}

		/*Transmitir la dirección de memoria*/
		I2C1->DR = *data++;

	}

	while(!(I2C1->SR1 & (SR1_BTF))){}

	/*Generar la parada*/
	I2C1->CR1 |= CR1_STOP;


}
