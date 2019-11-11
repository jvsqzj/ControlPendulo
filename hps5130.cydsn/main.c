/* =============================================
 * PI motor control
 * Copyright Eduardo Interiano, 2017
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Eduardo Interiano.
 * 
 * Remember to set the heap to 0x200 minimun in
 * SYSTEM tab for floating point number printing
 * =============================================
*/
#include "project.h"
#include "stdbool.h"
#include "stdio.h"
#include "usbuartio.h"    

/* Global constants */
#define PWM_MAX         499   // PWM 100% (499) with Tpwm = 500us @ 1MHz (500 counts)
#define PWM_FACTOR      100   // 100 counts/volt with Tpwm = 500us @ 1MHz
#define THETA_CHANNEL   0     // Mux input for speed feedback
#define ANGLE_FACTOR    360 / 5 //  72 deg / volt
#define X_CHANNEL       1     // Mux input for PI constants setting
#define VDAC_FACTOR     62.5  // 1bit/0.016mV
#define VDAC_MAX        300   // 4.8V max. with DVDAC 9 bits and PGA x2
#define POT_CONST       3100  // Max. counts to 100%

/* PI algorithm constants */
 #define FORCE_FACTOR   2  // Newtons / Volt
 #define REFERENCE      1.7805    // Posición vertical 
 #define MAXINTEGRAL    4.7  // Limits the integral part to 4.7V
 #define DEB_COUNTS     2    // 5ms counts for debounce
 #define TS_FACTOR      1    // 5ms counts for Ts = 5ms

/* Global variables */
char displayStr[20] = {'\0'};

/* isr global variables */

 volatile bool select_1 = false; // Memory variables for DISPLAY_SELECT button debouncing
 volatile bool select_2 = false; 
 volatile bool Select = false;   // Debounced DISPLAY_SELECT button state

 volatile bool step_1 = false;   // Memory variables for START/SAVE button debouncing
 volatile bool step_2 = false; 
 volatile bool step = false;     // Debounced START/SAVE button state

 volatile int8 debounce = DEB_COUNTS; // Counter for debouncing when Timer runs faster than 10ms
 volatile int8 factor = TS_FACTOR;    // Counter for Ts = 5ms when Timer runs faster than 5ms

/* Buttons variables */
 bool idle = true;  // Signals when step input can be applied
 bool run = false;  // Clock to advance Display state machine
 bool save = false; // flag to preserve PI constants adjustment

 /* Button sync enumerated type */
 enum  DebounceState {WAIT, RUN, PRESS};

/* PI algorithm global variables */
 float KP = 221;       // Proportional default constant
 float KI = 5;      // Integral default constant @ 5ms
 volatile float ik = 0; // Integral action and memory
 float yk = 0; 	 // theta feedback
 float ek = 0;   // theta error
 float mk = 0;   // total control action
 int16 vdac = 0; // For VDAC output
 int16 data = 0; // For ADC reading

/* Interrupt prototype */
CY_ISR_PROTO(isr_Timer_Handler);

// Interrupt handler declaration
CY_ISR(isr_Timer_Handler)
{

/* ISR PI algoritm local variables */
 yk = 0; 	 // theta feedback
 ek = 0;   // theta error
 mk = 0;   // total control action
 int16 vdac = 0; // For VDAC output
 int16 data = 0; // For ADC reading

#if (PWM_2_Resolution)    
 int16 pwm = 1;  // For PWM output
#endif
    
    /* Debounces the switches */
    if (--debounce <= 0) {
        select_2 = select_1;
        select_1 = !(CyPins_ReadPin(DISPLAY_SELECT_0));
        Select = (select_1 && select_2);

        step_2 = step_1;
        step_1 = !(CyPins_ReadPin(START_0));
        step = (step_1 && step_2);
    
        /* Signals step edge for oscilloscope sync */
        if ( (step && idle) == true) {
            CyPins_SetPin(STEP_LED_0);
            CyPins_SetPin(Pin_ENA_0);
        }
        else {
            CyPins_ClearPin(Pin_ENA_0);
            CyPins_ClearPin(STEP_LED_0);
        }
        debounce = DEB_COUNTS;
    }

    /* Performs the PID algorithm every 5ms when in idle state */
	if (--factor <= 0 && idle == true) {
		factor = TS_FACTOR;
        
        // Signals the start of the PI algorithm for time compliance measurements
        CyPins_SetPin(ISR_LED_0);

        // Reads angle value
        AMux_Select(THETA_CHANNEL);
        data = ADC_Read16();
        
        /* Converts the counts to volts 1V = 1krpm */
	    yk = ADC_CountsTo_Volts((int32) data); 
        
        /* Set the reference step */
        if (step == true) { 
            ek = REFERENCE - yk; // Follow the reference
        } 
        else {
            ek = 0; // Stop the motor
            ik = 0;
        }
    
        /* PI control algorithm calculation */        
        /* Integral part, the rightmost term is also ik_1 */
        ik = KI*ek + ik;       
        
        /* Total PI control action */ 
        mk = KP*ek + ik;

        /* PWM conditional use */
#if defined CY_PWM_PWM_2_H
        /* Scales mk to PWM range */
        if (mk >= 0)
            pwm = (int16) (mk*PWM_FACTOR);
        else
            pwm = (int16) (-1*mk*PWM_FACTOR);
        /* Saturates the PWM value */
        if (pwm > PWM_MAX) { 
            pwm = PWM_MAX;
        }
        else if (pwm < 0) {
            pwm = 0; 
        }
        
        vdac = (int16) (mk*VDAC_FACTOR);
    
        // saturates the DVDAC value 
        if (vdac > VDAC_MAX) { 
            vdac = VDAC_MAX;
        }
        else if (vdac < 0) {
            vdac = 0; 
        }
        DVDAC_SetValue(vdac); // Outputs DVDAC saturated value
        if(mk >= 0){    
            PWM_2_WriteCompare(pwm);
            PWM_1_WriteCompare(0);
        } else {
            PWM_1_WriteCompare(pwm);
            PWM_2_WriteCompare(0);
        }
#endif

        // Indicates the end of the PI algorithm
        CyPins_ClearPin(ISR_LED_0);
            
        /* Saturates the integral term for the next period */
        if (ik > MAXINTEGRAL)
            ik = MAXINTEGRAL;
        else
        if (ik < -MAXINTEGRAL) ik = -MAXINTEGRAL;
	
    }
    
} // CY_ISR

/* State machine for single step button action */
bool ButtonSync(enum DebounceState* DebounceButton, bool button) { 
    
 /* Local variable */   
 bool buttonstate = false;
    
    /* Button sync */
    switch (*DebounceButton) {
        case WAIT:   
            *DebounceButton = button? RUN:WAIT;    
        break;
       
        case RUN: 
            *DebounceButton = PRESS;
            buttonstate = true;
            
        break;
        
        case PRESS:    
            *DebounceButton = button? PRESS:WAIT;
        break;
    
        default: 
            *DebounceButton = WAIT;
            

    } // switch
        
    return buttonstate;

} // ButtonSync

int main(void)
{    

/* USBUART conditional use */
#if defined CY_USBFS_USBUART_H
    
/* USB variable definitions */
uint16 usbcount;
uint8  usbbuffer[USBUART_BUFFER_SIZE];
uint8  *usbp; // pointer for usbbuffer
char   *usbc; // pointer for displayStr

/* Pointers to usbbuffer and displayStr for USB communication */
usbp = &usbbuffer[0];
usbc = &displayStr[0];

#endif

/* PI constants adjustment variables */
int16   pot;    // For ADC reading
float   potf;   // P and I adjust temp variable

/* Definitions of state machines */
enum  AdjustState {IDLE, ADJUST_P, ADJUST_I} AdjustMachine = IDLE;
enum DebounceState ButtonSave = WAIT;
enum DebounceState ButtonSelect = WAIT;

    
    /* Startup code for all blocks */
    AMux_Start();
    ADC_Start();
    Timer_Start();
    LCD_Start();
    
    /* Correction for ADC_CountsTo_Volts */
    ADC_SetOffset(-1); // -1mV offset correction
    ADC_SetGain(673);  // 0.9895 gain correction

    /* QuadDec conditional use */
#if defined CY_QUADRATURE_DECODER_QuadDec_H
    QuadDec_Start();
#endif

    /* PWM conditional use */
#if defined CY_PWM_PWM_2_H    
    PWM_2_Start();
#endif

#if defined CY_PWM_PWM_1_H    
    PWM_1_Start();
#endif    

    /* DVDAC conditional use */
#if defined CY_DVDAC_DVDAC_H
    DVDAC_Start();
    PGA_Start();
    Opamp_Start();
#endif    

    /* UART conditional use */
#if defined CY_UART_UART_H 
    UART_Start();
    UART_PutString("PI Control v1.0\n\r");
#endif

    /* USBUART conditional use */
#if defined CY_USBFS_USBUART_H
    /* Start USBFS operation with 5-V operation. */
    USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
#endif

    /* Interrrup process init with StartEx not simple Start */
    isr_Timer_StartEx(isr_Timer_Handler);
    
    /* Enable global interrupts. */
    CyGlobalIntEnable; 

    /* Infinite loop */
    for(;;)
    {  
        /* Sync action buttons for single step */ 
        save = ButtonSync(&ButtonSave,step);
        run = ButtonSync(&ButtonSelect,Select);
        
        /* Display state machine uses run and save flags */   
        switch (AdjustMachine) {
    
            case IDLE:
    
                idle = true;
            
                LCD_Position(0,0);
                LCD_PrintString("PI control v1.0 ");
                int16 theta = ANGLE_FACTOR*(yk - REFERENCE);
                // Show the KP and KI constants in the LCD
                sprintf(displayStr,"yk=%.3dmk=%.3f ",theta,mk);
                LCD_Position(1,0);
                LCD_PrintString(displayStr);
    
                /* Advance the machine state */
                AdjustMachine = run? ADJUST_P:IDLE;
                
                /* Blink LED4 every 100ms to signal IDLE state */
//                CyPins_SetPin(LED4_0);
//                CyDelay(25);
//
//                CyPins_ClearPin(LED4_0);
//                CyDelay(75);                
                
            break;
    
            case ADJUST_P:
    
                idle = false;
                
                LCD_Position(0,0);
                LCD_PrintString("Proporcional    ");

                /* Advance the machine state */
                AdjustMachine = run? ADJUST_I:ADJUST_P;

                /* Process the proportional constant of the PI */
                AMux_Select(X_CHANNEL);
                pot = ADC_Read16();
                if (pot < 0) pot = 0; else if (pot > POT_CONST) pot = POT_CONST;
                potf =  (float) pot/POT_CONST;
                
                /* Display the KP constant new value -> old value */
                sprintf(displayStr,"%.4f -> %.4f ",potf,KP);
                LCD_Position(1,0);
                LCD_PrintString(displayStr);
    
                /* Save the KP value if the START/SAVE button was pressed */
                if (save == true) {
                    KP = potf;
                }
                
            break;
      
            case ADJUST_I:

                idle = false;
                
                LCD_Position(0,0);
                LCD_PrintString("Integral        ");

                /* Advance the machine state */
                AdjustMachine = run? IDLE:ADJUST_I;

                /* Process the integral constant of the PI */
                AMux_Select(X_CHANNEL);
                pot = ADC_Read16();
                if (pot < 0) pot = 0; else if (pot > POT_CONST) pot = POT_CONST;
              
                potf = (0.2*pot)/POT_CONST;
                
                /* Display the KI constant new value -> old value */
                sprintf(displayStr,"%.4f -> %.4f ",potf,KI);
                LCD_Position(1,0);
                LCD_PrintString(displayStr);                

                /* Save the KI value if the START/SAVE button was pressed */
                if (save == true) {
                    KI = potf;
                }
                
            break;
      	
            default: AdjustMachine = IDLE;
                
        } // switch
        
    /* USBUART conditional use */
#if defined CY_USBFS_USBUART_H                
        // Read data from USB_Uart
        USB_Uart_Init();
        usbcount = USB_Uart_Read(&usbp);
    
        // Echo received data to USB
        if (usbcount > 0) {
            USB_Uart_Write(usbcount,&usbp);
            LCD_Position(1,0);
            sprintf(displayStr,"%d chars",usbcount); // Does not work, Overwritten in static IDLE
            LCD_PrintString(displayStr);
            if (usbcount > 1) USB_Uart_PrintLn(&usbc);
        }
#endif
    } // for
} // main

/* [] END OF FILE */
