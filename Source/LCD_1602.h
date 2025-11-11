/**
 * @file    LCD_1602.h
 * @author  Electronic6502
 * @version V1.0
 * @date    9 Oct 2025
 * @brief   HD44780 based LCD1602 Library Header File
 */

#ifndef __LCD_1602_H__
#define __LCD_1602_H__

#include "3rd_Lib/stm8s.h"

#define PCF8574_Address 0x27

#define _MODE_DATA      1
#define _MODE_COMMAND   0
#define _4bit           0
#define _8bit           1

#define _RS_PIN         0x01            /* LCD RS pin connected to PCF8574 pin0 */
#define _RW_PIN         0x02            /* LCD RW pin connected to PCF8574 pin1 */
#define _EN_PIN         0x04            /* LCD EN pin connected to PCF8574 pin2 */
#define _BACKLIGHT_PIN  0x08            /* LCD Backlight Connected to PCF8574 pin3 */

void LCD_Init (void);

void LCD_Clear (void);

void LCD_GotoXY (uint8_t x, uint8_t y);

void LCD_PutChar (uint8_t Chr);

void LCD_PutString (char String[]);


/*============= Internal Functions =============*/

void I2C_Config (void);

void I2C_Start (void);

void I2C_WriteAddress (uint8_t Addr);

void I2C_WriteData (uint8_t Data);

void I2C_Stop (void);

/* Send Half Byte (4bit) Data or Command to LCD */
void _LCD_Send4bit (uint8_t Value, uint8_t Mode);       

/* Send Full Byte (8bit) Data or Command to LCD */
void _LCD_SendByte (uint8_t Value, uint8_t Mode);


#endif