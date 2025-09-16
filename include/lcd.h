#ifndef INC_LCD_H_
#define INC_LCD_H_

void Lcd_Init(void);
void Lcd_Port(char data, int rs);
void Lcd_Send_Cmd(char cmd);
void Lcd_Send_Char(char data);
void Lcd_Send_String(char *str);
void Lcd_Set_Cursor(int row, int col);
void Lcd_Clear(void);
void Lcd_Shift_Right(void);
void Lcd_Shift_Left(void);
void Lcd_Blink(void);
void Lcd_NoBlink(void);
void Lcd_CGRAM_CreateChar(unsigned char pos, const char*msg);
void Lcd_CGRAM_WriteChar(char pos);
uint32_t HAL_Delay_Us_Init(void);

__STATIC_INLINE void HAL_Delay_Us(volatile uint32_t microseconds)
{
	uint32_t clk_cycle_start = DWT->CYCCNT;
	microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
	while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

#endif
