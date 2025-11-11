/**
 * @file    LCD_1602.c
 * @author  Electronic6502
 * @version V1.0
 * @date    9 Oct 2025
 * @brief   HD44780 based LCD1602 Library Source File
 */

#include  "LCD_1602.h"

extern void Delay_ms (uint16_t ms);

uint8_t _Data_Buffer = 0;                       /* Buffer of pins State, 0 = All pins in Low State */

void LCD_Init (void){
    I2C_Config ();
    _Data_Buffer |= _BACKLIGHT_PIN;             /* set Backlight bit on Buffer  */
    Delay_ms (50);                              /* Delay Needed for HD44780 LCD after Power On */
    I2C_Start ();
    I2C_WriteAddress (PCF8574_Address);         /* Write PCF8574 Address */
    I2C_WriteData (_Data_Buffer | _EN_PIN);     /* set High EN pin and Turn on Backlight */
    Delay_ms (1);
    I2C_WriteData (_Data_Buffer &= (~_EN_PIN)); /* set Low EN pin */
    I2C_Stop ();

    _LCD_Send4bit (0x2, _MODE_COMMAND);         /* Set to 4bit Operation */
    _LCD_SendByte (0x28,_MODE_COMMAND);         /* Set to 4bit Operation and Select 2 Line Mode */
    _LCD_SendByte (0x0C,_MODE_COMMAND);         /* Turn on Display, without Cursor and Blink */ 
    _LCD_SendByte (0x01, _MODE_COMMAND);        /* Clear LCD */
    Delay_ms (5);                               /* Clearing the LCD took 1.5 ms */
    _LCD_SendByte (0x06,_MODE_COMMAND);         /* Cursor Direction is Increment */
}

void LCD_Clear (void){
    _LCD_SendByte (0x01, _MODE_COMMAND);
    Delay_ms (5);
}

void LCD_GotoXY (uint8_t x, uint8_t y){
    if (y == 0){
        _LCD_SendByte ((0x80 | x), _MODE_COMMAND);
    } else{
        _LCD_SendByte ((0xC0 | x), _MODE_COMMAND);
    }
}

void LCD_PutChar (uint8_t Chr){
    _LCD_SendByte (Chr, _MODE_DATA);
}

void LCD_PutString (char String[]){
    uint8_t i = 0;
    while (String [i] != '\0'){
        if (String[i] == '\n'){
            _LCD_SendByte (0xC0, _MODE_COMMAND);    /* Handle Line Feed */
        } else {
            _LCD_SendByte (String[i], _MODE_DATA);
        }
        i++;
    }
}


/*==========================================================================================*/

/*=================================== Internal Functions ===================================*/

void _LCD_Send4bit (uint8_t Value, uint8_t Mode){
    I2C_Start ();
    I2C_WriteAddress (PCF8574_Address);
    if (Mode == _MODE_DATA){
        _Data_Buffer |= _RS_PIN;
    }
    else {                          /* Command Mode */
        _Data_Buffer &= ~_RS_PIN;
    }
    Value <<= 4;
    _Data_Buffer &= 0x0F;           /* Clear MSB 4 bit */
    _Data_Buffer |= Value;          /* Transfer First 4 bit */
    I2C_WriteData (_Data_Buffer | _EN_PIN);
    Delay_ms (1);
    I2C_WriteData (_Data_Buffer &= ~_EN_PIN);
    I2C_Stop ();
}

void _LCD_SendByte (uint8_t Value, uint8_t Mode){
    _LCD_Send4bit ((Value >> 4), Mode);     /* Send First 4 bit MSB */
    _LCD_Send4bit (Value, Mode);            /* Send Last 4 bit LSB */
}

void I2C_Config (void){
    I2C->CR1    = I2C_CR2_SWRST;            /* Reset I2C Peripheral Registers */
    I2C->FREQR  = 16;                       /* I2C Input Clock in MHz */
    I2C->TRISER = 17;                       /* Maximum Rise Time = FREQR + 1 */
    I2C->CCRL   = 80;                       /* I2C SCL Frequency = 1 / (2 * CCR * tMaster) */
    I2C->CR1    = I2C_CR1_PE;               /* Enable I2C Peripheral */
}

void I2C_Start (void){
    I2C->CR2    = I2C_CR2_START;
    while ((I2C->SR1 & I2C_SR1_SB) == 0);   /* Wait for Start Generation */
}

void I2C_WriteAddress (uint8_t Addr){
    I2C->DR = Addr << 1;
    while ((I2C->SR1 & I2C_SR1_ADDR) == 0); /* Wait for Transmit Address */
    Addr = I2C->SR3;                        /* Clear EV6 (Address Transmit) Event Flag */
}

void I2C_WriteData (uint8_t Data){
    I2C->DR = Data;
    while (((I2C->SR1) & I2C_SR1_TXE) == 0);/* Wait for Transmit Data */
}

void I2C_Stop (void){
    I2C->CR2    = 0x02;
    while (I2C->SR3 & I2C_SR3_MSL);         /* Wait for Stop Generation */
}

/* End of File */