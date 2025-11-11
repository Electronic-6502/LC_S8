/**
 * @file    main.c
 * @author  Electronic6502
 * @version V1.0
 * @date    8 Oct 2025
 * @brief   LCR1 Source File
 */
/* Toolchain : SDCC */
#include <string.h>
#include "3rd_Lib/stm8s.h"
#include "periph_and_misc.h"
#include "LCD_1602.h"

#define VERSION         "V1.0"

#define C9_CAP_VAL      0.0000000047f          /* Value of C9 on PCB in Farad */ // 4.45nf mog

#define R19_L_VAL       1000000.0f              /* Capacitor Charge Resistors */
#define R12_M_VAL       10000.0f
#define R13_H_VAL       100.0f

uint8_t Crystal_Error_Flag = 0;
uint8_t Unit_String [4];
const uint8_t Empty[] = {"          "};         /* 10x Empty Character */
volatile uint8_t GotoNextMode = 0;
volatile uint8_t Button_Hold_Cnt = 0;
volatile uint16_t TIM1_OVF_Count = 0;
volatile uint16_t TIM2_OVF_Count = 0;

void Boot_Messages (void){
    static float VBAT;
    LCD_GotoXY (5,0);
    LCD_PutString ("LC-S8\n");
    LCD_GotoXY (5,1);
    LCD_PutString (VERSION);
    Delay_ms (2000);
    LCD_Clear ();
    LCD_GotoXY (6,0);
    LCD_PutString ("by:\n Electronic-6502");
    Delay_ms (2000);
    LCD_Clear ();
    LCD_PutString ("Battery Voltage\n");
    VBAT = (ADC_Read(VBAT_ADC) / 1023.0f) * 5.0f;       /* Read Battery Voltage */
    Print_Float (VBAT);
    LCD_PutChar ('V');
    Delay_ms (1800);
    if (Crystal_Error_Flag){
        LCD_Clear ();
        LCD_GotoXY (4,0);
        LCD_PutString ("Warning! \n Crystal Error!");
        Delay_ms (5000);
    }
    LCD_Clear ();
}

void ModeConfig (uint8_t Mode){
    Delay_ms (150);                                             /* Delay Needed to Debouncing Button */
    GotoNextMode = 0;
    LCD_Clear ();
    if (Mode == MODE_Frequency){   
        PinLow (REL_PORT, REL1_PIN);                            /* Switch Relay1 to NC */
        PinLow (REL_PORT, REL2_PIN);                            /* Switch Relay2 to NC */
        LCD_PutString ("Frequency:");
    }
    else if (Mode == MODE_Capacitance){
        PinHigh (REL_PORT, REL2_PIN);                           /* Switch Relay2 to NO */
        LCD_PutString ("CAP:"); 
        LCD_GotoXY (0,1);                                       /* Goto Next Line */
        LCD_PutString ("ESR:");                                   
    }
    else {                                                      /* Inductance Meter */
        PinLow (REL_PORT, REL2_PIN);
        PinHigh (REL_PORT, REL1_PIN);                           /* Switch Relay1 to NO */
        LCD_PutString ("Inductance:");
    }
    TIM1_Config_for (Mode);
}

void FrequencyMeter (void){
    float Freq;
    ModeConfig (MODE_Frequency);
    while (GotoNextMode == 0){
        Freq = TIM1_PulseCounter (1000);
        LCD_GotoXY (0,1); 
        LCD_PutString (Empty);
        LCD_GotoXY (0,1);
        if (Freq < 10000.0f){
            strcpy (Unit_String, "Hz");
            Print_Number (Freq);
        } 
        else{
            strcpy (Unit_String, "kHz");
            Freq /= 1000.0f;
            Print_Float (Freq);
        }
        LCD_PutString (Unit_String);
    }
}

void Capacitor_Discharge (void){
    PinHigh (CAP_PORT, CAP_DISCHG_PIN);
    while (ADC_Read(CAP_ADC) > 1);                         /* Wait for Discharge Capacitor */
    Delay_ms (10);
    PinLow (CAP_PORT, CAP_DISCHG_PIN);    
}

uint8_t Calibrate = 0;
    
float Measure_ESR (void){
    uint16_t ADC_Val, i;
    float ESR, ESR_ZERO, Volt, DurPulse, AfterPulse;        /* ESR Meter Variables */
    ESR_ZERO = EEPROM_Read_Float (EEP_ESR_ZERO_ADDR);    
    AfterPulse = 0.0f;
    DurPulse   = 0.0f;
    PinOutput (CAP_PORT, CAP_CHG_LOW_PIN);
    PinLow (CAP_PORT, CAP_CHG_LOW_PIN);                 /* Pull Down ADC with 1MΩ Resistor */
    for (i=0; i<256; i++){
        PinHigh (CAP_PORT, CAP_DISCHG_PIN);             /* Discharge Capacitor */
        Delay_us (600);
        PinLow (CAP_PORT, CAP_DISCHG_PIN);              /* Stop Discharging */        
        PinLow (CAP_HIGH_PORT, CAP_CHG_HIGH_PIN);       /* Start 50mA Pulse */
        //Delay_us (2);
        ADC_Val = ADC_Read (CAP_ADC);                   /* Read ADC During Pulse */
        PinHigh (CAP_HIGH_PORT, CAP_CHG_HIGH_PIN);      /* Stop 50mA Pulse */
        DurPulse += ADC_Val;
        Delay_us (50);                                  /* wait for Overhead Voltage drop */
        AfterPulse += ADC_Read (CAP_ADC);
    }
    PinInput (CAP_PORT, CAP_CHG_LOW_PIN);               /* Disable Pull Down */
    DurPulse = DurPulse - AfterPulse;
    Volt = (VREF * DurPulse) / 261888.0f;               /* Measure Voltage */
    ESR = R13_H_VAL / ((VREF / Volt) - 1.0f);
    if (Calibrate){
        ESR_ZERO = ESR;
        EEPROM_Save_Float (ESR_ZERO, EEP_ESR_ZERO_ADDR);
        LCD_Clear ();
        LCD_PutString ("Open Probes and\n Press Button");
        while (ReadPin (BUTTON_PORT, BUTTON_PIN));
        Delay_ms (500);
    }
    ESR = (ESR - ESR_ZERO) - 0.05f;       /* ESR minus Offset Resistance and Capacitor Charge Voltage */
    return ESR;
}

void CapacitorMeter (void){
    static float CAP, CAP_ZERO, ChargeTime, ESR;
    CAP_ZERO = EEPROM_Read_Float (EEP_CAP_ZERO_ADDR);
    RESTART_CAP_METER :
    ModeConfig (MODE_Capacitance);    
    while (GotoNextMode == 0){
        Capacitor_Discharge ();
        if (ReadPin(BUTTON_PORT, BUTTON_PIN) == 0){
            Button_Hold_Cnt++;
            if(Button_Hold_Cnt > 5) Calibrate = 1;
        }
        if (Calibrate){
            LCD_Clear ();
            while (ReadPin (BUTTON_PORT, BUTTON_PIN) == 0); /* Wait for Button Release */
            LCD_PutString ("Short Probes and\n Press Button");
            Delay_ms (500);
            while (ReadPin (BUTTON_PORT, BUTTON_PIN));      /* Wait for Press Button */
            Delay_ms (500);
        }
        CAP = 0.0f;
        ADC_Config ();                                      /* ADC in Single Conversion Mode */
        ESR = Measure_ESR ();
        ADC1->CR1 |= ADC1_CR1_CONT;                         /* ADC in Continuous Conversion Mode */
        ADC1->CR1 |= ADC1_CR1_ADON;                         /* Start ADC Conversion */
        TIM1_ResetCounter ();
        PinLow (CAP_HIGH_PORT, CAP_CHG_HIGH_PIN);     /* Charge Capacitor with ~50mA (for Short Check) */
        Delay_ms (50);
        PinHigh (CAP_HIGH_PORT, CAP_CHG_HIGH_PIN);          /* Stop Charging */
        Delay_ms (1);
        if (ADC_Read_CONT() > 1){                        /* Capacitor have Voltage, Not Shorted */
            Capacitor_Discharge ();
            PinLow (CAP_HIGH_PORT, CAP_CHG_HIGH_PIN);
            TIM1->CR1 |= TIM1_CR1_CEN;                          /* Enable Counter */
            while (ADC_Read_CONT() < 646);
            TIM1->CR1 &= ~TIM1_CR1_CEN;                         /* Disable Counter */
            PinHigh (CAP_HIGH_PORT, CAP_CHG_HIGH_PIN);
            ChargeTime = TIM1_ReadCount();                      /* Read Capacitor RC Time in uS */
            CAP = (ChargeTime / R13_H_VAL);                     /* Measure Capacitance in uF */               

            if (ChargeTime < 3000.0f){                          /* Capacitor between 300nF and 30uF */
                TIM1_ResetCounter ();
                Capacitor_Discharge ();
                PinLow (CAP_PORT, CAP_CHG_MED_PIN);             /* Charge Capacitor with ~500uA */
                TIM1->CR1 |= TIM1_CR1_CEN;
                while (ADC_Read_CONT() < 646);
                TIM1->CR1 &= ~TIM1_CR1_CEN;
                PinHigh (CAP_PORT, CAP_CHG_MED_PIN);    
                ChargeTime = TIM1_ReadCount(); 
                CAP = (ChargeTime / R12_M_VAL);                 /* Measure Capacitance in uF */           
            }           
            if (ChargeTime < 3000.0f){                          /* Capacitor Smaller than 300nF */
                TIM1_ResetCounter ();
                Capacitor_Discharge ();
                PinOutput (CAP_PORT, CAP_CHG_LOW_PIN);
                PinHigh (CAP_PORT, CAP_CHG_LOW_PIN);
                TIM1->CR1 |= TIM1_CR1_CEN;
                while (ADC_Read_CONT() < 646);
                TIM1->CR1 &= ~TIM1_CR1_CEN;
                PinInput (CAP_PORT, CAP_CHG_LOW_PIN);
                PinLow (CAP_PORT, CAP_CHG_LOW_PIN);    
                ChargeTime = TIM1_ReadCount(); 
                CAP = (ChargeTime / R19_L_VAL);                 /* Measure Capacitance in uF */                 
            }                                           
            LCD_GotoXY (4,0);
            LCD_PutString (Empty);
            LCD_GotoXY (4,0);
            if (CAP < 0.001f){                                  /* Capacitor Smaller than 1nF */
                Print_Number ((CAP - CAP_ZERO) * 1000000.0f);
                strcpy (Unit_String, "pF");
            }
            else if ((CAP >= 0.001f) && (CAP < 1.0f)){          /* Capacitor between 1nF and 1uF */
                Print_Float ((CAP - CAP_ZERO) * 1000.0f);       /* CAP_ZERO = Parasitic Capacitor */
                strcpy (Unit_String, "nF");
            }
            else if ((CAP >= 1.0f) && (CAP < 10000.0f)){        /* Capacitor between 1uF and 10mF */
                Print_Float (CAP);
                strcpy (Unit_String, "uF");
            }
            else {                                              /* Capacitor Larger than 10mF */
                Print_Float (CAP / 1000.0f);
                strcpy (Unit_String, "mF");
            }
        }
        else {
            LCD_GotoXY (4,0);
            LCD_PutString (Empty);
            LCD_GotoXY (4,0);
            strcpy (Unit_String, "---");                 /* Shorted Capacitor or Discreate Resistor */
            ESR += 0.05f;                                       
        }
        LCD_PutString (Unit_String);
        LCD_GotoXY (4,1);
        LCD_PutString (Empty);
        LCD_GotoXY (4,1);
        if ((ESR < 100.0f) && ((CAP == 0.0f) || (CAP > 1.0f))){ /* show ESR if Lower than 100 Ω */
            Print_Float (ESR);
            LCD_PutChar (0xF4);                                 /* Display ohm (Ω) Symbol */              
        }
        else {
            LCD_PutString ("---");
        }
/*=========================== Capacitance Meter Calibration ===========================*/
        if (Calibrate){
            CAP_ZERO = CAP;
            EEPROM_Save_Float (CAP_ZERO, EEP_CAP_ZERO_ADDR);
            LCD_Clear ();
            LCD_PutString ("  Calibration \n   Success !!  ");
            Delay_ms (3000);
            Calibrate = 0;
            goto RESTART_CAP_METER;
        }

    }
}

void InductanceMeter (void){
    float L1, Lx, CurrentFreq, Freq0;
    L1 = EEPROM_Read_Float (EEP_L1_ADDR);
    Freq0 = EEPROM_Read_Float (EEP_FREQ0_ADDR);
    RESTART_IND_METER :
    ModeConfig (MODE_Inductance);
    while (GotoNextMode == 0){
        CurrentFreq = TIM1_PulseCounter (50);
        if ((CurrentFreq > 3.0f) && (CurrentFreq < 150.0f)){    /* Choice Best Gate Time */
            CurrentFreq = TIM1_PulseCounter (1000);
        }
        else{
            CurrentFreq = TIM1_PulseCounter (400);
            CurrentFreq *= 2.5f;
        }
        Lx = (((Freq0 * Freq0)/(CurrentFreq * CurrentFreq))-1.0f ) * L1;
        LCD_GotoXY (0,1);
        LCD_PutString (Empty);
        LCD_GotoXY (0,1);        
        if (Lx < 0.001){
            strcpy (Unit_String, "uH");
            Print_Float (Lx * 1000000.0f);
        }
        else if ((Lx >= 0.001f) && (Lx < 1.0f)){
            strcpy (Unit_String, "mH");
            Print_Float (Lx * 1000.0f);
        }
        else if ((Lx >= 1.0f) && (Lx < 100.0f)){
            strcpy (Unit_String, "H");
            Print_Float (Lx);
        }
        else {
            strcpy (Unit_String, "---");
        }
        LCD_PutString (Unit_String);
/*======================================== Calibration ========================================*/
        if (ReadPin (BUTTON_PORT, BUTTON_PIN) == 0){
            if (Button_Hold_Cnt > 5){                 
                L1 = 1.0f / (PI_POW2_4x * (CurrentFreq * CurrentFreq) * C9_CAP_VAL);
                Freq0 = CurrentFreq;
                EEPROM_Save_Float (L1, EEP_L1_ADDR);
                EEPROM_Save_Float (Freq0, EEP_FREQ0_ADDR);
                LCD_GotoXY (0,1);
                LCD_PutString ("Calibrated!!");
                Delay_ms (3000);
                goto RESTART_IND_METER;
            }
            Button_Hold_Cnt++;
        }
    }
}

void GPIOC_ISR (void)__interrupt(5){            /* GPIOC Interrupt Vector (Button Detection) */
    if (Button_Hold_Cnt < 2){
        GotoNextMode = 1;
    }
    Button_Hold_Cnt = 0;
}

void Timer1_Overflow (void)__interrupt(11){     /* Timer1 Overflow Vector */
    TIM1_OVF_Count++;
    TIM1->SR1   = 0x00;
}

void Timer2_Overflow (void)__interrupt(13){
    TIM2_OVF_Count++;
    TIM2->SR1   = 0x00;                     /* Clear Update Flag */
}

void main (void){
    disableInterrupts();                    /* Global Disable Interrupts */
    GPIO_Config ();
    TIM2_Config_for_Delay ();
    enableInterrupts();                     /* Global Enable Interrupts */
    Crystal_Error_Flag = Clock_Config ();
    ADC_Config ();
    LCD_Init ();
    Boot_Messages ();
    while(1){
        FrequencyMeter ();
        CapacitorMeter ();
        InductanceMeter ();
    }
}