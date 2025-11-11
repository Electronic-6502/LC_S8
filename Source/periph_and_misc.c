/**
 * @file    periph_and_misc.c
 * @author  Electronic6502
 * @version V1.0
 * @date    9 Oct 2025
 * @brief   STM8 Microcontroller Peripherals Configuration and Some Misc Functions.
 */

#include "periph_and_misc.h"

extern volatile uint16_t TIM2_OVF_Count;
extern volatile uint16_t TIM1_OVF_Count;

void GPIO_Config (void){
    GPIOA->DDR = 0x00;                  /* GPIOA All pins as Input */
    GPIOC->DDR = 0x70;                  /* GPIOC pin 4,5,6 as OUTPUT */ 
    GPIOD->DDR = 0x2C;                  /* GPIOD pin 2,3,5 as OUTPUT */
    GPIOE->DDR = 0x20;                  /* GPIOE pin 5 as OUTPUT */

    GPIOC->CR1 = 0x70;                  /* GPIOC pin 4,5,6 Mode Push-Pull */
    GPIOD->CR1 = 0x2C;                  /* GPIOD pin 2,3,5 Mode Push-Pull */
    GPIOE->CR1 = 0x20;                  /* GPIOE pin 5 Mode Push-Pull */

    GPIOC->CR2 = 0x80;                  /* GPIOC pin 7 Enable Interrupt */
    EXTI->CR1  = 0x10;                  /* GPIOC Interrupt on Rising Edge */

    GPIOC->ODR = 0x10;                  /* GPIOC pin 4 set to High */
    GPIOE->ODR = 0x20;                  /* GPIOE pin 5 set to High */
}

uint8_t Clock_Config (void){
    uint8_t Error_Flag = 0;
	CLK->CKDIVR = 0x00;				    /* Run CPU and Peripherals at 16MHz */		            
	CLK->ECKR = 0x01;				    /* Enable External Crystal Clock Source */
	CLK->SWR = 0xB4;				    /* Select External Crystal at Master Clock */
    Delay_ms (10);                      /* Delay needed for Stablization External Crystal */
	if(CLK->SWCR & 0x08){			    /* if External Crystal Available */
		CLK->SWCR &= 0xF7;			    /* Clear SWIF Flag */			
		CLK->SWCR |= 0x02;			    /* Switch Master Clock to External Crystal */
	}
	else{							    /* if External Crystal UnAvailable, Switch Back to HSI */
		CLK->SWR = 0xE1;
		CLK->ECKR = 0x00;
        Error_Flag = 1;                 /* Set External Crystal Error Flag */
	}    
    return Error_Flag;
}

void ADC_Config (void){
    ADC1->CR1   = 0x10;                                 /* ADC Clock = fMaster / 3 */
    ADC1->CR2   = ADC1_CR2_ALIGN;                       /* ADC Data in Right Alignment Mode */
    ADC1->TDRL  = (1 << VBAT_ADC) | (1 << CAP_ADC);     /* Disable ADC Channel Schemitt Trigger */
    ADC1->CR1   |= ADC1_CR1_ADON;                       /* Wakeup ADC from Sleep */
}

static uint16_t Result = 1;                             /* Static Variable for Fastest Access Speed */

inline uint16_t ADC_Read_CONT (void){
    while ((ADC1->CSR & ADC1_CSR_EOC) == 0);
    Result  = ADC1->DRL;
    Result |= (ADC1->DRH << 8);
    ADC1->CSR = 0;
    return Result;
}

uint16_t ADC_Read (uint8_t Channel){
    ADC1->CSR  = Channel;
    ADC1->CR1 |= ADC1_CR1_ADON;                   /* Start ADC Conversion */
    while ((ADC1->CSR & ADC1_CSR_EOC) == 0);        /* Wait for End Conversion */
    Result  = ADC1->DRL;
    Result |= (ADC1->DRH << 8); 
    ADC1->CSR = 0;                                  /* Clear EOC Flag and Select Channel 0 */
    return Result;
}

void TIM1_Config_for (uint8_t Mode){
    TIM1->CR1   = 0x00;                 /* Disable Timer Counter */
    TIM1->IER   = 0x00;                 /* Disable Timer1 Interrupts */
    TIM1->CCER1 = 0x00;                 /* Disable Capture */
    TIM1->ARRH  = 0xFF;
    TIM1->ARRL  = 0xFF;                 /* Set Auto Reload Value to 65535 */
    TIM1->PSCRH = 0;
    if (Mode == MODE_Frequency){
        TIM1->CCMR1 = 0x01;             /* Timer1 Channel 1 Selected as Capture Input */
        TIM1->CCER1 = TIM1_CCER1_CC1E;  /* Timer1 Capture Enabled, Sens = Rising Edge */
        TIM1->SMCR  = 0x57;             /* Timer1 in External Clock Mode with Channel 1 */
        TIM1->PSCRL = 0x00;             /* Timer1 Clock Prescaler = 1 */
    }
    else if (Mode == MODE_Inductance){
        TIM1->CCMR1 = 0x02;             /* Timer1 Channel 2 Selected as Capture Input */
        TIM1->CCER1 = TIM1_CCER1_CC1E;  /* Timer1 Capture Enabled, Sens = Rising Edge */
        TIM1->SMCR  = 0x67;             /* Timer1 in External Clock Mode with Channel 2 */
        TIM1->PSCRL = 0x00;             /* Timer1 Clock Prescaler = 1 */
    }
    else if (Mode == MODE_Capacitance){
        TIM1->CCMR1 = 0x00;
        TIM1->CCER1 = 0x00;
        TIM1->SMCR  = 0x00;             /* Timer1 Set to Normal Mode (Internal Clock) */
        TIM1->PSCRL = 16;               /* Timer1 Clock = Fmaster / 16 */
    }
    TIM1->EGR   = TIM1_EGR_UG;          /* Update Registers for Take Prescaler */
    TIM1->IER   = TIM1_IER_UIE;         /* Enable Overflow Interrupt */
}

void TIM2_Config_for_Delay (void){
    TIM2->PSCR  = 4;                    /* Timer2 Clock = Fmaster / (2^4) */
    TIM2->ARRH  = (999 >> 8);           
    TIM2->ARRL  = (999 & 0xFF);         /* Set Overflow Value to 999 */
    TIM2->EGR   = TIM2_EGR_UG;          /* Generate Update Event for Take Prescaler */
    TIM2->IER   = TIM2_IER_UIE;         /* Enable Overflow Interrupt */
}

void Delay_us (uint16_t us){
    uint16_t tick = 0;
    TIM2->CNTRH = 0x00;
    TIM2->CNTRL = 0x00;                 /* Reset Counter Register */
    TIM2->CR1   = TIM2_CR1_CEN;         /* Enable Timer Counter */
    while (tick < us){
        tick  = (TIM2->CNTRH << 8);
        tick |= TIM2->CNTRL;
    }
    TIM2->CR1   = 0x00;                 /* Disable Timer Counter */
}

void Delay_ms (uint16_t ms){
    TIM2_OVF_Count = 0;
    TIM2->CNTRH = 0x00;
    TIM2->CNTRL = 0x00;                 /* Clear Counter Register */
    TIM2->CR1   = TIM2_CR1_CEN;         /* Enable Timer Counter */
    while (TIM2_OVF_Count < ms);
    TIM2->CR1   = 0x00;                 /* Disable Timer Counter */
}

float EEPROM_Read_Float (uint8_t Address){
    return (*(volatile float*)(0x4000 + Address));
}

void EEPROM_Save_Float (float Value, uint8_t Address){
    FLASH->DUKR  = 0xAE;
    FLASH->DUKR  = 0x56;                 /* Disable EEPROM Write Protect */
    (*(volatile float*)(0x4000 + Address)) = Value;
    FLASH->IAPSR = 0x00;                 /* Enable EEPROM Write Protect */
}

void TIM1_ResetCounter (void){
    TIM1->CR1   = 0;
    TIM1->CNTRH = 0;
    TIM1->CNTRL = 0;
    TIM1_OVF_Count = 0;
}

float TIM1_ReadCount (void){
    uint16_t tmp = 0;
    float Pulses;
    tmp = (TIM1->CNTRH << 8);
    tmp |= TIM1->CNTRL;
    Pulses = tmp + ((float)TIM1_OVF_Count * 65536);
    return Pulses;
}

float TIM1_PulseCounter (uint16_t GateTime_ms){
    TIM1_ResetCounter ();
    TIM1->CR1 |= TIM1_CR1_CEN;
    Delay_ms (GateTime_ms);
    TIM1->CR1 &= ~TIM1_CR1_CEN;
    return TIM1_ReadCount();
}

/*===================================== Misc Functions =====================================*/

void Print_Number (uint32_t l_Number){
    uint8_t i;
    uint8_t Buffer[5];
    Buffer [0] = l_Number / 10000;
    Buffer [1] = (l_Number / 1000) % 10;
    Buffer [2] = (l_Number / 100) % 10;
    Buffer [3] = (l_Number / 10) % 10;
    Buffer [4] = l_Number % 10;
    for (i=0; i<4; i++){
        if (Buffer [i] != 0)break;
    }
    while (i < 4){
        LCD_PutChar (Buffer[i] + '0');
        i++;
    }
    LCD_PutChar (Buffer[4] + '0');
}

void Print_Float (float fl_Number){
    uint8_t Fraction;
    uint32_t Part_int;
    if (fl_Number < 0.0){
        fl_Number = 0;
    }
    Part_int = fl_Number;
    Fraction = (fl_Number - Part_int) * 100;
    Print_Number (Part_int);
    LCD_PutChar ('.');
    LCD_PutChar ((Fraction / 10) + '0');
    LCD_PutChar ((Fraction % 10) + '0');
}