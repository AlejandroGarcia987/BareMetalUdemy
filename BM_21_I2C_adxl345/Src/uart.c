#include "uart.h"

//#define GPIODEN			(1U<<3)
#define GPIOAEN			(1U<<0) //Pongo el bit de la posición 0 a 1 para activar el RCC para el GPIOA. El botón está conectado al bus A.
#define UART2EN			(1U<<17) // EL UART2 está en la posición 17 del RCC_APB1ENR

#define CR1_TE			(1U<<3)// Activamos el bit de la posición 3, TE, de Control register 1 (USART_CR1) (Transmision)
#define CR1_RE			(1U<<2)// Activamos el bit de la posición 2, RE, de Control register 1 (USART_CR1) (Recepcion)
#define CR1_UE			(1U<<13)// Activamos el bit de la posición 13, UE (UART Enable), de Control register 1 (USART_CR1)
#define SR_TXE			(1U<<7) //Preparamos el bit 7 de Status register (USART_SR) que es TXE (TX Empty) para saber si está vacío
#define SR_RXNE			(1U<<5) //Bit para comprobar que la recepción no esté vacía en el USART2 USART Status register.
#define CR1_RXNEIE		(1U<<5)


#define SYS_FREQ		16000000 //Si no configuramos nada, por defecto el reloj del sistema son 16Mhz
#define APB1_CLK        SYS_FREQ //Si no configuramos nada, los divisores y demás son 1, por lo que APB1 tendrá la frecuencia de SYS_FREQ

#define UART_BAUDRATE 	115200 //Definimos este BAUDRATE por defecto

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_baudrate(uint32_t PeriphClk, uint32_t BaudRate);


void uart2_write(int ch);

/*Escribimos esta función para poder usar el printf*/
int __io_putchar(int ch)
{
	uart2_write(ch);
	return ch;
}

void uart2_rxtx_init(void) /*
HEmos modificado el nombre de la función, ya que ahora
vamos a inicializar el sistema como envío y recepción.
*/
{
	/*configurar el GPIO del UART para hacer TX*/
	/*1. Activar el clock para GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*2. SET PA2 (Uart2 TX) mode to alternate function mode (modo de función alternativa*/
	/*Hemos visto que el pin TX del USART2 está en el pin2 del GPIOA, que corresponde
	 * a las posiciones 4 y 5 del GPIOx_MODER. Tenemos que ponerlo en modo funcionamiento alternativo.*/
	GPIOA->MODER &=~ (1U<<4);
	GPIOA->MODER |=  (1U<<5);

	/*3. Set PA2 modo función alternativa a UART_TX (Es lo mismo que AF7 ya que lo pone en la tabla del datasheet)*/
	/*Tenemos que usar la sección GPIO alternate function low register (GPIOx_AFRL) (x = A..I/J/K)
	 *para poder configurar el pin2, que es usart2 tx, tenemos que usar AFRL2[3:0] que ocupa los bits
	 *para 8, 9, 10, 11.
	 *Debajo vemos que la tabla dice que, dependiendo del tipo de función alternativa que queramos usar, tenemos que
	 *Debajo configurar los bits de una manera específica y, para usar el AF7 que hemos visto antes, la combinación es
	 *0111*/
	/*Tenemos que usar AFR[0] primero para seleccionar AFRLX, si usasemos AFR[1] sería AFRHX. EN los includes no tenemos
	 * AFRL y AFRH*/
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &=~ (1U<<11);

	/*Ahora tenemos que poner PA3 en modo UART_RX*/
	/*
	 * 1. Ponemos el MODE REGISTER 3 en modo funcionamiento alterntivo. Corresponderá al PA3. MODE REG 3
	 * corresponde a los pines 6 y 7.
	 */
	GPIOA->MODER &=~ (1U<<6);
	GPIOA->MODER |=  (1U<<7);

	/*
	 * 2. BUscamos el GPIO alternate function. Como es el pin 3 necesitaremos AFRL3 que ocupa los bits 12-13-14-15.
	 * De nuevo tenemos que ponerlo en modo AF7 (recuerda que AFR[0] es para LOW REGISTER y AFR[1] para high register
	 */
	GPIOA->AFR[0] |= (1U<<12);
	GPIOA->AFR[0] |= (1U<<13);
	GPIOA->AFR[0] |= (1U<<14);
	GPIOA->AFR[0] &=~ (1U<<15);


	/*Primero activamos el reloj, luego activamos el modo función alternativa y, una vez activado dicho modo, le decimos que es UART_TX*/

	/*Configurar modulo UART*/
	/*1. Habilitar el acceso de reloj para el UART2*/
	//EL USART2 está conectado al APB1 bus según nuestro datasheet.
	RCC->APB1ENR |= UART2EN;

	 /* 2. Configurar el baudrate del UART */
	uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);

	 /* 3. Configurar la dirección de transmisión */ /*TAmbién configuramos la dirección de recepción*/
	USART2->CR1 = CR1_TE; //No poenmos |= y sólo usamos = para limpiar todo el registro y poner a 1 sólo el bit que nos interesa
	USART2->CR1 |= CR1_RE; //POngo |= ya que quiero mantener la configuración que ya tengo para TE

	//USART2->CR1 = (CR1_TE | CR1_RE); //Con esto habilitamos las dos.

	 /* 4. Activar el modulo UART */
	USART2->CR1 |= CR1_UE; //Aquí si usamos |= ya que ya hemos configurado el bit TE y queremos mantener el estado.

}

void uart2_rx_interrupt_init(void) /*
HEmos modificado el nombre de la función, ya que ahora
vamos a inicializar el sistema como envío y recepción.
*/
{
	/*configurar el GPIO del UART para hacer TX*/
	/*1. Activar el clock para GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*2. SET PA2 (Uart2 TX) mode to alternate function mode (modo de función alternativa*/
	/*Hemos visto que el pin TX del USART2 está en el pin2 del GPIOA, que corresponde
	 * a las posiciones 4 y 5 del GPIOx_MODER. Tenemos que ponerlo en modo funcionamiento alternativo.*/
	GPIOA->MODER &=~ (1U<<4);
	GPIOA->MODER |=  (1U<<5);

	/*3. Set PA2 modo función alternativa a UART_TX (Es lo mismo que AF7 ya que lo pone en la tabla del datasheet)*/
	/*Tenemos que usar la sección GPIO alternate function low register (GPIOx_AFRL) (x = A..I/J/K)
	 *para poder configurar el pin2, que es usart2 tx, tenemos que usar AFRL2[3:0] que ocupa los bits
	 *para 8, 9, 10, 11.
	 *Debajo vemos que la tabla dice que, dependiendo del tipo de función alternativa que queramos usar, tenemos que
	 *Debajo configurar los bits de una manera específica y, para usar el AF7 que hemos visto antes, la combinación es
	 *0111*/
	/*Tenemos que usar AFR[0] primero para seleccionar AFRLX, si usasemos AFR[1] sería AFRHX. EN los includes no tenemos
	 * AFRL y AFRH*/
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &=~ (1U<<11);

	/*Ahora tenemos que poner PA3 en modo UART_RX*/
	/*
	 * 1. Ponemos el MODE REGISTER 3 en modo funcionamiento alterntivo. Corresponderá al PA3. MODE REG 3
	 * corresponde a los pines 6 y 7.
	 */
	GPIOA->MODER &=~ (1U<<6);
	GPIOA->MODER |=  (1U<<7);

	/*
	 * 2. BUscamos el GPIO alternate function. Como es el pin 3 necesitaremos AFRL3 que ocupa los bits 12-13-14-15.
	 * De nuevo tenemos que ponerlo en modo AF7 (recuerda que AFR[0] es para LOW REGISTER y AFR[1] para high register
	 */
	GPIOA->AFR[0] |= (1U<<12);
	GPIOA->AFR[0] |= (1U<<13);
	GPIOA->AFR[0] |= (1U<<14);
	GPIOA->AFR[0] &=~ (1U<<15);


	/*Primero activamos el reloj, luego activamos el modo función alternativa y, una vez activado dicho modo, le decimos que es UART_TX*/

	/*Configurar modulo UART*/
	/*1. Habilitar el acceso de reloj para el UART2*/
	//EL USART2 está conectado al APB1 bus según nuestro datasheet.
	RCC->APB1ENR |= UART2EN;

	 /* 2. Configurar el baudrate del UART */
	uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);

	 /* 3. Configurar la dirección de transmisión */ /*TAmbién configuramos la dirección de recepción*/
	USART2->CR1 = CR1_TE; //No poenmos |= y sólo usamos = para limpiar todo el registro y poner a 1 sólo el bit que nos interesa
	USART2->CR1 |= CR1_RE; //POngo |= ya que quiero mantener la configuración que ya tengo para TE

	/*Activar RXNE interrupción*/
	USART2->CR1 |= CR1_RXNEIE;

	/*Activar UART2 interrupt en NVIC*/
	NVIC_EnableIRQ(USART2_IRQn);

	//USART2->CR1 = (CR1_TE | CR1_RE); //Con esto habilitamos las dos.

	 /* 4. Activar el modulo UART */
	USART2->CR1 |= CR1_UE; //Aquí si usamos |= ya que ya hemos configurado el bit TE y queremos mantener el estado.

}

char uart2_read(void)
{
	/*
	 * Función para recibir por UART desde el pc
	 */

	//1. Comprobar que el registro no esté vacío. Nos vamos a USART_SR y vemos que el flag de recepción esté activado para
	// que no esté vacío.
	while(!(USART2->SR & SR_RXNE)){} //Mientras no esté listo, esperamos
	USART2->DR; //En cuanto esté ilsto, leemos el uart2

}

void uart2_write(int ch)
{
	/*Queremos estar seguros de que la transmision del registro está vacía antes de nada*/
	while(!(USART2->SR & SR_TXE)){} //Con esto vemos que, si el bit TXE de SR NO es 1, significa que el USART no está disponible
						// y el código se quedaría ahi.
	/*Luego, escirbimos en el transmit DR*/
	USART2->DR = (ch & 0xFF); //Con esto transmitimos 8 bits
}



//Función para hacer el SET del baudrate calculado en el UART.
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
	USARTx->BRR = compute_uart_baudrate(PeriphClk, BaudRate);
}

//Escribimos una ecuación para calcular el Baudrate.
static uint16_t compute_uart_baudrate(uint32_t PeriphClk, uint32_t BaudRate)
{
	return ((PeriphClk + (BaudRate/2U))/BaudRate);
}
