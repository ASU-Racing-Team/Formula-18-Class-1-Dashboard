#include <stm32f4xx_hal_conf.h>
#include "lcd.h"
// LCD_D0 -> PF12
// LCD_D1 -> PD15
// LCD_D2 -> PF15
// LCD_D3 -> PE13
// LCD_D4 -> PE14
// LCD_D5 -> PE11
// LCD_D6 -> PE9
// LCD_D7 -> PF13
// LCD_RD -> PA3
// LCD_RS -> PC3
// LCD_WR -> PC0
// LCD_CS -> PF3
// LCD_RST -> PF5
void init_lcd_gpio(){

	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Pull = GPIO_NOPULL;
	
	gpio.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
	
	HAL_GPIO_Init(GPIOF, &gpio);
	
	gpio.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
	
	HAL_GPIO_Init(GPIOE, &gpio);
	
	gpio.Pin = GPIO_PIN_0 | GPIO_PIN_3;
	
	HAL_GPIO_Init(GPIOC, &gpio);

	gpio.Pin = GPIO_PIN_3;
	
	HAL_GPIO_Init(GPIOA, &gpio);
	
	gpio.Pin =  GPIO_PIN_15;

	HAL_GPIO_Init(GPIOD, &gpio);
	
}

