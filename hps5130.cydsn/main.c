#include "project.h"
#include "stdbool.h"
#include "stdio.h"
#include "usbuartio.h"    

/* Global constants */
#define PWM_MAX         499   // PWM 100% (499) with Tpwm = 500us @ 1MHz (500 counts)
#define PWM_FACTOR      100   // 100 counts/volt with Tpwm = 500us @ 1MHz
#define THETA_CHANNEL   0     // Mux input for angle feedback
#define ANGLE_FACTOR    360 / 5 //  72 deg / volt

/* PI algorithm constants */
 #define FORCE_FACTOR   2  // Newtons / Volt
 #define REFERENCE      1.64225    // Posición vertical 
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

/* PI algorithm global variables */
 float KP = 221;       // Proportional default constant
 float KI = 3;      // Integral default constant @ 5ms
 volatile float ik = 0; // Integral action and memory
 float yk = 0; 	 // theta feedback
 float ek = 0;   // theta error
 float mk = 0;   // total control action
 int16 data = 0; // For ADC reading

/* Interrupt prototype */
CY_ISR_PROTO(isr_Timer_Handler);

// Interrupt handler declaration
CY_ISR(isr_Timer_Handler) {
    /* ISR PI algoritm local variables */
    yk = 0; 	 // theta feedback
    ek = 0;   // theta error
    mk = 0;   // total control action
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
    
        /* Enables motor output so control action is nonzero */
        if ( (step) == true) {
            CyPins_SetPin(Pin_ENA_0);
        }
        else { /* disables motor output so control action is zero */
            CyPins_ClearPin(Pin_ENA_0);
        }
        debounce = DEB_COUNTS;
    }

    /* Performs the PID algorithm every 5ms when in idle state */
    if (--factor <= 0) {
        factor = TS_FACTOR;

        // Reads angle value
        AMux_Select(THETA_CHANNEL);
        data = ADC_Read16();
        
        /* Converts the counts to volts */
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
        /* Controls H-Bridge */
        if(mk >= 0){    
            PWM_2_WriteCompare(pwm);
            PWM_1_WriteCompare(0);
        } else {
            PWM_1_WriteCompare(pwm);
            PWM_2_WriteCompare(0);
        }
    #endif
                
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
    /* Definitions of state machines */
    enum DebounceState ButtonSave = WAIT;
    enum DebounceState ButtonSelect = WAIT;

    /* Startup code for all blocks */
    AMux_Start();
    ADC_Start();
    Timer_Start();
    LCD_Start();

/* PWM conditional use */
    #if defined CY_PWM_PWM_2_H    
    PWM_2_Start();
    #endif

    #if defined CY_PWM_PWM_1_H    
    PWM_1_Start();
    #endif    
    
    /* Correction for ADC_CountsTo_Volts */
    ADC_SetOffset(-1); // -1mV offset correction
    ADC_SetGain(673);  // 0.9895 gain correction

    /* Interrrup process init with StartEx not simple Start */
    isr_Timer_StartEx(isr_Timer_Handler);
    
    /* Enable global interrupts. */
    CyGlobalIntEnable; 

    /* Infinite loop */
    for(;;)
    {  
        /* Sync action buttons for single step */ 
        save = ButtonSync(&ButtonSave,step); //Sets step for use in interruption handler
            
        LCD_Position(0,0);
        LCD_PrintString("PI control v1.0 ");
        int16 theta = ANGLE_FACTOR*(yk - REFERENCE);
               
        sprintf(displayStr,"yk=%.3dmk=%.3f ",theta,mk);
        LCD_Position(1,0);
        LCD_PrintString(displayStr);
    } // for
} // main

/* [] END OF FILE */
