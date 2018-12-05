#include "stm32f4xx_hal.h"
#include "delay.h"


typedef struct {
	uint8_t bits, rows, cols;
	GPIO_TypeDef *controlPort, *dataPort;
	uint16_t rsPin, strbPin, rwPin;
	uint16_t dataPins;
} TextLCDType;

//void TextLCD_Init(TextLCDType *lcd, GPIO_TypeDef *controlPort, uint16_t rsPin, uint16_t rwPin, uint16_t enPin, GPIO_TypeDef *dataPort, uint16_t dataPins);
void TextLCD_Init(TextLCDType *lcd, GPIO_TypeDef *controlPort, uint16_t rsPin, uint16_t rwPin, uint16_t enPin, GPIO_TypeDef *dataPort, uint16_t dataPins, uint8_t rows, uint8_t cols);
void TextLCD_Home (TextLCDType *lcd);
void TextLCD_Clear (TextLCDType *lcd);
void TextLCD_Position (TextLCDType *lcd, int x, int y);
void TextLCD_Putchar (TextLCDType *lcd, uint8_t data);
void TextLCD_Puts (TextLCDType *lcd, char *string);
void TextLCD_Printf (TextLCDType *lcd, char *message, ...);
