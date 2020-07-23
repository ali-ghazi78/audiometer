

#include "stm32f1xx_hal_spi.h"

extern SPI_HandleTypeDef hspi1;


#ifndef ____ad9833____
#define  ____ad9833____
#endif





enum ad9833_mode{
	ad9833_sin=0,
	ad9833_tri=1
};


void my_spi_write16(SPI_HandleTypeDef *hspi,uint16_t my_data,GPIO_TypeDef *SLAVE_GPIO,uint16_t SLAVE_GPIO_PIN)
{
	HAL_GPIO_WritePin(SLAVE_GPIO, SLAVE_GPIO_PIN, 0);
	HAL_Delay(10);
	HAL_SPI_Transmit(hspi, &my_data, 2, 800);
	HAL_Delay(5);
	HAL_GPIO_WritePin(SLAVE_GPIO, SLAVE_GPIO_PIN, 1);

}
void ad9833_send_command(uint16_t command)
{
	my_spi_write16(&hspi1,command, GPIOA,GPIO_PIN_6);

}
#define ad9833_send_data ad9833_send_command


void ad9833_set_mode_and_freq(char mode,uint32_t freq )
{
	if(!mode)
		ad9833_send_command( 1<<13 ); ///sin mode
	else
		ad9833_send_command( 1<<13 | 1<<1); //tri mode


	freq = (float)freq*10.73741824;

	//send lsb first
	ad9833_send_data((freq%(0xffff))| 1<<14);
	ad9833_send_data((freq/(0xffff))| 1<<14);


	//#268435456/25000000=10.73741824
	//   freg*(fref/2^28 )  // in my case fref is 25mhz
}



