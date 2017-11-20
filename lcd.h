#ifndef LCD_H
#define LCD_H
#include <stm32f4xx_hal_conf.h>
#define disp_x_size 239
#define disp_y_size 319
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

void init_lcd_gpio();
inline GPIO_PinState byte_to_pinstate(uint8_t byte,uint8_t bit){
	if((byte>>bit)&1){
		return GPIO_PIN_SET;
	}else{
		return GPIO_PIN_RESET;
	}
}
inline void lcd_rst_active(){
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5,GPIO_PIN_RESET);
}
inline void lcd_rst_idle(){
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5,GPIO_PIN_SET);
}
inline void lcd_cs_active(){
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3,GPIO_PIN_RESET);
}
inline void lcd_cs_idle(){
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3,GPIO_PIN_SET);
}
inline void lcd_wr_active(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_RESET);
}
inline void lcd_wr_idle(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_SET);
}
inline void lcd_rs_active(){
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_RESET);
}
inline void lcd_rs_idle(){
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_SET);
}
inline void lcd_rd_active(){
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_RESET);
}
inline void lcd_rd_idle(){
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_SET);
}

inline void lcd_write_pins(uint8_t d){
	
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, byte_to_pinstate(d,0));
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, byte_to_pinstate(d,1));
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, byte_to_pinstate(d,2));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, byte_to_pinstate(d,3));
	
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, byte_to_pinstate(d,4));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, byte_to_pinstate(d,5));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  byte_to_pinstate(d,6));
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, byte_to_pinstate(d,7));
	
}
inline void lcd_write_bus(uint16_t d){

		lcd_write_pins(d>>8);
		lcd_wr_active();
		lcd_wr_idle();
		lcd_write_pins(d&0xFF);
		lcd_wr_active();
		lcd_wr_idle();

}
inline void lcd_set_dir_in(){
	
	GPIOF->MODER &= ~(GPIO_MODER_MODER3_Msk | 
										GPIO_MODER_MODER5_Msk | 
										GPIO_MODER_MODER12_Msk | 
										GPIO_MODER_MODER13_Msk | 
										GPIO_MODER_MODER15_Msk);
	GPIOE->MODER &= ~(GPIO_MODER_MODER9_Msk | 
										GPIO_MODER_MODER11_Msk | 
										GPIO_MODER_MODER13_Msk | 
										GPIO_MODER_MODER14_Msk);
	GPIOC->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER3_Msk);
	GPIOD->MODER &= ~(GPIO_MODER_MODER15_Msk);
	GPIOA->MODER &= ~(GPIO_MODER_MODER3_Msk);
	
}
inline void lcd_set_dir_out(){
	GPIOF->MODER |= (GPIO_MODER_MODER3_0 | 
										GPIO_MODER_MODER5_0 | 
										GPIO_MODER_MODER12_0 | 
										GPIO_MODER_MODER13_0 | 
										GPIO_MODER_MODER15_0);
	GPIOE->MODER |= (GPIO_MODER_MODER9_0| 
										GPIO_MODER_MODER11_0| 
										GPIO_MODER_MODER13_0| 
										GPIO_MODER_MODER14_0);
	GPIOC->MODER |= (GPIO_MODER_MODER0_0| GPIO_MODER_MODER3_0);
	GPIOD->MODER |= (GPIO_MODER_MODER15_0);
	GPIOA->MODER |= (GPIO_MODER_MODER3_0);
	GPIOF->MODER &= ~(GPIO_MODER_MODER3_1| 
										GPIO_MODER_MODER5_1| 
										GPIO_MODER_MODER12_1| 
										GPIO_MODER_MODER13_1| 
										GPIO_MODER_MODER15_1);
	GPIOE->MODER &= ~(GPIO_MODER_MODER9_1| 
										GPIO_MODER_MODER11_1| 
										GPIO_MODER_MODER13_1| 
										GPIO_MODER_MODER14_1);
	GPIOC->MODER &= ~(GPIO_MODER_MODER0_1| GPIO_MODER_MODER3_1);
	GPIOD->MODER &= ~(GPIO_MODER_MODER15_1);
	GPIOA->MODER &= ~(GPIO_MODER_MODER3_1);
}
inline void lcd_write_cmd(uint8_t cmd){
	lcd_rs_active();
	lcd_write_bus((uint16_t)cmd);
}
inline void lcd_write_data(uint16_t data){
	lcd_rs_idle();
	lcd_write_bus(data);
}
#endif
