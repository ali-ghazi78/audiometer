

#ifndef __spi_SOF_
#define __spi_SOF_
#endif






void spi_init
void spi_send_16(GPIO_TypeDef * PORT_SPI,uint8_t sck,uint8_t mosi, uint8_t ss, uint16_t data )
{
	HAL_GPIO_WritePin(PORT_SPI, sck, 1);
	HAL_GPIO_WritePin(PORT_SPI, mosi, data&(1<<16));
	ASM_NOP();
	HAL_GPIO_WritePin(PORT_SPI, ss, 0);
	ASM_NOP();
	for (int i=0; i<16;i++)
	{
		HAL_GPIO_WritePin(PORT_SPI, sck, 0);
		ASM_NOP();
		HAL_GPIO_WritePin(PORT_SPI, mosi, data&(1<<15-i));
		HAL_GPIO_WritePin(PORT_SPI, sck, 1);
		ASM_NOP();
	}


}
