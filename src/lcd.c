#include "main.h"
#include "lcd.h"

uint32_t HAL_Delay_Us_Init(void)
{
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
	CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
	DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;
	DWT->CYCCNT = 0;

	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");

    return (DWT -> CYCCNT) ? 0 : 1;
}

void Lcd_Port(char data, int rs)
{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, rs);
	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ((data>>3) & 0x01));
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ((data>>2) & 0x01));
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ((data>>1) & 0x01));
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ((data>>0) & 0x01));
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, 1);
	HAL_Delay_Us(20);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, 0);
	HAL_Delay_Us(20);
}

void Lcd_Send_Cmd(char cmd)
{
    char datatosend;
    datatosend = ((cmd>>4) & 0x0F);
    Lcd_Port(datatosend,0);
    datatosend = ((cmd) & 0x0F);
    Lcd_Port(datatosend, 0);
}

void Lcd_Send_Char(char data)
{
	char datatosend;
	datatosend = ((data>>4) & 0x0F);
	Lcd_Port(datatosend, 1);
	datatosend = ((data) & 0x0F);
	Lcd_Port(datatosend, 1);
}

void Lcd_Clear(void)
{
	Lcd_Send_Cmd(0x01);
	HAL_Delay(2);
}

void Lcd_Set_Cursor(int row, int col)
{
	uint8_t address = 0x00;
	switch(row)
	{
		case 1:
			address = 0x00;
			break;
		case 2:
			address = 0x40;
			break;
		case 3:
			address = 0x14;
			break;
		case 4:
			address = 0x54;
			break;
	}
	address += col - 1;
	Lcd_Send_Cmd(0x80 | address);

}

void Lcd_Init(void)
{
	HAL_Delay_Us_Init();
	HAL_Delay(50);
	Lcd_Send_Cmd(0x30);
	HAL_Delay(5);
	Lcd_Send_Cmd(0x30);
	HAL_Delay(1);
	Lcd_Send_Cmd(0x30);
	HAL_Delay(10);
	Lcd_Send_Cmd(0x20);
	HAL_Delay(10);
	Lcd_Send_Cmd(0x28);
	HAL_Delay(1);
	Lcd_Send_Cmd(0x08);
	HAL_Delay(1);
	Lcd_Send_Cmd(0x01);
	HAL_Delay(1);
	HAL_Delay(1);
	Lcd_Send_Cmd(0x06);
	HAL_Delay(1);
	Lcd_Send_Cmd(0x0C);
}

void Lcd_Send_String(char *str)
{
	while(*str) Lcd_Send_Char(*str++);
}

void Lcd_Shift_Right(void)
{
	Lcd_Send_Cmd(0x1C);
}

void Lcd_Shift_Left(void)
{
	Lcd_Send_Cmd(0x18);
}

void Lcd_Blink(void)
{
	Lcd_Send_Cmd(0x0F);
}

void Lcd_NoBlink(void)
{
	Lcd_Send_Cmd(0x0C);
}

void Lcd_CGRAM_CreateChar(unsigned char pos, const char*msg)
{
    if(pos < 8)
    {
        Lcd_Send_Cmd(0x40 + (pos*8));
        for(unsigned char i=0; i<8; i++)
        {
            Lcd_Send_Char(msg[i]);
        }
    }
}

void Lcd_CGRAM_WriteChar(char pos)
{
    Lcd_Send_Char(pos);
}
