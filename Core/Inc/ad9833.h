

#include "stm32f1xx_hal_spi.h"

extern SPI_HandleTypeDef hspi1;


#ifndef ____ad9833____
#define  ____ad9833____
#endif





enum ad9833_mode{
	ad9833_sin=0,
	ad9833_tri=1,
	ad9833_pulse=2,
	ad9833_pulse2om=3

};


void my_spi_write16(SPI_HandleTypeDef *hspi,uint16_t my_data,GPIO_TypeDef *SLAVE_GPIO,uint16_t SLAVE_GPIO_PIN)
{


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	HAL_SPI_Transmit(hspi, &my_data, 1, 800);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);





}
void ad9833_send_command(uint16_t command)
{
	my_spi_write16(&hspi1,command, GPIOB,GPIO_PIN_0);

}
#define ad9833_send_data ad9833_send_command



void ad9833_disable( )
{
	ad9833_send_command(  (1<<8)|(1<<6) ); ///sin mode


}


void ad9833_set_mode_and_freq(char mode,uint32_t freq )
{


	uint32_t temp = 0;
	if(mode==0)
		ad9833_send_command( (1<<13)  ); ///sin mode
	else if (mode==1)
		ad9833_send_command( (1<<13) | (1<<1)); //tri mode
	else if (mode==2)
			ad9833_send_command( (1<<13) |(1<<5)| (1<<3)); //pulse mode
	else if (mode==3)
			ad9833_send_command( (1<<13) |(1<<5)); //pulse2om mode


	freq =(float) freq*10.73741824;
	temp = freq;
	//temp = 10000;
	//send lsb first
	ad9833_send_command((temp%(1<<14))| 1<<14);
	ad9833_send_command((temp>>14)| 1<<14);



	//#268435456/25000000=10.73741824
	//   freg*(fref/2^28 )  // in my case fref is 25mhz
}



