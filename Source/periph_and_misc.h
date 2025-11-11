/**
 * @file    periph_and_misc.h
 * @author  Electronic6502
 * @version V1.0
 * @date    9 Oct 2025
 * @brief   STM8 Microcontroller Peripherals Configuration and Some Misc Functions.
 */

#ifndef __PERIPH_AND_MISC_H__
#define __PERIPH_AND_MISC_H__

#include "3rd_Lib/stm8s.h"
#include "LCD_1602.h"

#define VBAT_ADC    2
#define CAP_ADC     0

#define BUTTON_PORT GPIOC
#define BUTTON_PIN  7

#define REL_PORT    GPIOD
#define REL2_PIN    3
#define REL1_PIN    2

#define CAP_PORT            GPIOC
#define CAP_HIGH_PORT       GPIOE
#define CAP_CHG_HIGH_PIN    5
#define CAP_CHG_MED_PIN     4
#define CAP_CHG_LOW_PIN     5
#define CAP_DISCHG_PIN      6

#define EEP_CAP_ZERO_ADDR   0
#define EEP_ESR_ZERO_ADDR   16
#define EEP_L1_ADDR         32
#define EEP_FREQ0_ADDR      64

#define VREF                5.0f        /* ADC Reference Voltage */

#define PI_POW2_4x          39.4784     /*  4 x (PI ^ 2)  */

#define PinHigh(PORT,PIN)       (PORT->ODR |= (1 << PIN))
#define PinLow(PORT,PIN)        (PORT->ODR &= ~(1 << PIN))
#define PinOutput(PORT,PIN)     (PORT)->DDR |= (1 << PIN);  (PORT)->CR1 |= (1 << PIN)
#define PinInput(PORT,PIN)      (PORT)->DDR &= ~(1 << PIN); (PORT)->CR1 &= ~(1 << PIN)
#define ReadPin(PORT,PIN)       (PORT->IDR & (1 << PIN))

typedef enum{
    MODE_Frequency      = 1,
    MODE_Capacitance    = 2,
    MODE_Inductance     = 3
}Modes_Enum;

/* Configure GPIOs as INPUT or OUTPUT and Enable Selected Pins Interrupt */
void GPIO_Config (void);

/* Select HSE (External Crystal) as Clock Source */
/* if the Function Returned 1, the External Crystal is Broken */
uint8_t Clock_Config (void);

/* Configure ADC */
void ADC_Config (void);

/* Read Selected ADC Channel and Return Value */
uint16_t ADC_Read (uint8_t Channel);

/* Read Pre-Selected ADC Channel in Continuous Conversion Mode */
uint16_t ADC_Read_CONT (void);

/* Config Timer1 for Selected Task */
void TIM1_Config_for (uint8_t Mode);

/* Config Timer2 for us and ms Delay */
void TIM2_Config_for_Delay (void);

/* Micro Second Delay */
void Delay_us (uint16_t us);

/* Mili Second Delay */
void Delay_ms (uint16_t ms);

/* Save Float Number in the Selected Address of EEPROM */
float EEPROM_Read_Float (uint8_t Address);

/* Read Float Number from Selected Address in EEPROM */
void EEPROM_Save_Float (float Value, uint8_t Address);

void TIM1_ResetCounter (void);

float TIM1_ReadCount (void);

float TIM1_PulseCounter (uint16_t GateTime_ms);

/*===================================== Misc Functions =====================================*/

void Print_Number (uint32_t l_Number);

/* Print Float Number to LCD */
/* for Saving FLASH Memory ( Alternate of %f in sprintf )*/
void Print_Float (float fl_Number);

#endif