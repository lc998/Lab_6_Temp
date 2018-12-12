/*
 * lcdDisplay.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Lucas
 */
#include "lcdDisplay.h"
#include "delay.h"

void TextLCD_Strobe(TextLCDType *lcd);
void TextLCD_Data(TextLCDType *lcd, uint8_t data);
void TextLCD_Cmd(TextLCDType *lcd, uint8_t cmd);

//extern volatile uint32_t usekTick;
extern TIM_HandleTypeDef htim11;

//kalla strukten till riktiga pinnar,
void TextLCD_Init(TextLCDType *lcd, GPIO_TypeDef *controlPort, uint16_t rsPin, uint16_t rwPin, uint16_t enPin, GPIO_TypeDef *dataPort, uint16_t dataPins, uint8_t rows, uint8_t cols){
	//POWER ON
	lcd->controlPort = controlPort;
	lcd->rsPin = rsPin;
	lcd->rwPin = rwPin;
	lcd->dataPort = dataPort;
	lcd->dataPins = dataPins;
	lcd->strbPin = enPin;
	lcd->cols = cols;
	lcd->rows = rows;
	//TODO:: add lcd->bits = bits;

	//DELAY > 15ms
		HAL_Delay(15);
	//38HEX
	TextLCD_Cmd(lcd,0x38);
		HAL_Delay(5); // > 4.1ms
	//38HEX
	TextLCD_Cmd(lcd,0x38);
		delay_us(100);// > 100us
	//38HEX
	TextLCD_Cmd(lcd,0x38);
		//moved to Strobe // > 40us
	//38HEX
	TextLCD_Cmd(lcd,0x38);
		//moved to Strobe// > 40us
	//06HEX
	TextLCD_Cmd(lcd,0x06);
		//moved to Strobe // > 40us
	//0EHEX
	TextLCD_Cmd(lcd,0x0E);
		//moved to Strobe // > 40us
	//01HEX
	TextLCD_Cmd(lcd,0x01);
		HAL_Delay(2);// > 1.64ms

//END OF INITIALIZATION

	//80HEX
	TextLCD_Cmd(lcd,0x80);	// > 40us

}

//väcker lcd displayen
void TextLCD_Strobe(TextLCDType *lcd){
	delay_us(40);
	//skriv en 1:a till E porten
	HAL_GPIO_WritePin(lcd->controlPort, lcd->strbPin, 1);
	//vänta
	delay_us(40);
	//skriv en nolla till E porten
	HAL_GPIO_WritePin(lcd->controlPort, lcd->strbPin, 0);
	//vänta
	delay_us(40);
}

//writes command to screen
void TextLCD_Cmd(TextLCDType *lcd, uint8_t cmd){
	//R/S R/W 00
	HAL_GPIO_WritePin(lcd->controlPort, lcd->rsPin, 0);
	HAL_GPIO_WritePin(lcd->controlPort, lcd->rwPin, 0);

	lcd->dataPort->ODR &=~(0xff);
	lcd->dataPort->ODR |=(cmd);

	TextLCD_Strobe(lcd);
	delay_us(40);
}

//writes data to screen
void TextLCD_Data(TextLCDType *lcd, uint8_t data){
	//data

	HAL_GPIO_WritePin(lcd->controlPort, lcd->rsPin, 1);
	HAL_GPIO_WritePin(lcd->controlPort, lcd->rwPin, 0);

	lcd->dataPort->ODR &=~(0xff);
	lcd->dataPort->ODR |= (data);

	TextLCD_Strobe(lcd);
	delay_us(40); //+
}

//set cur. pos. to (0,0)
void TextLCD_Home (TextLCDType *lcd){
	TextLCD_Cmd(lcd, 0x02);
	//delay 1.6ms
	HAL_Delay(2);
}

//clear display
void TextLCD_Clear (TextLCDType *lcd){
	TextLCD_Cmd(lcd, 0x01);
	//delay 1.64ms
	HAL_Delay(2);
}

//set position to (x,y)
void TextLCD_Position (TextLCDType *lcd, int x, int y){
	//int width = lcd->rows;
	//TODO::add code for use off diffferent dsplay then 2x16
//	if(x > lcd->cols)
//		x = 0;
//	if(y > lcd->rows)
//		y = (y+1) % lcd->rows;

	TextLCD_Cmd(lcd, 0x80 | 0x40*x | 0x01*y);
	delay_us(40);
}

//prints a char to screen
void TextLCD_Putchar (TextLCDType *lcd, uint8_t data){
	TextLCD_Data(lcd, data);
	delay_us(40);
}

//prints a string to screen
void TextLCD_Puts (TextLCDType *lcd, char *string){
	int i = 0;
	while(*string != '\0'){
		TextLCD_Putchar(lcd, *string);
		string++;
		i++;
	}
	delay_us(40);
}

void TextLCD_PutInt(TextLCDType *lcd, int i){
	if(i < 10 && i >= 0){
		TextLCD_Putchar(lcd, 0x30+i);
		return;
	}

	if(i < 0){
		TextLCD_Putchar(lcd, '-');
		TextLCD_PutInt(lcd, -i);
	}else{
		TextLCD_PutInt(lcd, i/10);
		TextLCD_Putchar(lcd, 0x30+(i%10));
	}
	delay_us(40);
}

//
void TextLCD_Printf (TextLCDType *lcd, char *message, ...){

}
