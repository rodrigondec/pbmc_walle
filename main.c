/* 
 * File:   timer0.c
 * Author: Aluno
 *
 * Created on 8 de Setembro de 2016, 10:33
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "p18f45k20.h"

// PIC18F45K20 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H

#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 18        // Brown Out Reset Voltage bits (VBOR set to 1.8 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config HFOFST = OFF      // HFINTOSC Fast Start-up (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
#pragma config MCLRE = OFF       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 16000000

#define led0 LATD0
#define led1 LATD1
#define led2 LATD2
#define led3 LATD3
#define led4 LATD4
#define led5 LATD5
#define led6 LATD6
#define led7 LATD7

/*
 * 
 */
void delayX(int n)
{
    for(int i = 0; i < n; i++)
        __delay_ms(1);
}

int off_duty = 1;
int velocidade = 0;
int max_cicle = 20;

int main(int argc, char** argv)
{
    // timer configuration
    T0CONbits.TMR0ON = 1;
    T0CONbits.T0CS = 0;               // Timer increments on instruction clock
    T0CONbits.PSA = 0;               // Prescale enable
    
    T0CONbits.T0PS0 = 0;
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS2 = 0;
    
    T0CONbits.T08BIT = 1;
    T0CONbits.T0SE = 0;
    
    INTCONbits.TMR0IE = 1;               // Enable interrupt on TMR0 overflow
 
    INTCONbits.TMR0IF = 0;              //clear interrupt flag
    INTCONbits.GIE = 1;   
    
    TRISD7 = 0; // RD7 to RD0 set to output for led.
    TRISD6 = 0;
    TRISD5 = 0;
    TRISD4 = 0;
    TRISD3 = 0;
    TRISD2 = 0;
    TRISD1 = 0;
    TRISD0 = 0; // END LEDS
    
    TRISC5 = 1; // input velocidade
    
    TRISC7 = 0; // Saída do PWM
    
    TRISB1 = 0; // motores 1
    TRISB2 = 0; // motores 1
    
    TRISB3 = 0; // motores 2
    TRISB4 = 0; // motores 2
    
    PORTBbits.RB1 = 1; // motores 1
    PORTBbits.RB2 = 0; // motores 1
    
    PORTBbits.RB3 = 0; // motores 2
    PORTBbits.RB4 = 1; // motores 2
    
    while(1){
        if(PORTCbits.RC5 == 1){
            velocidade++;
            velocidade = velocidade%3;
            if(velocidade == 0){
                off_duty = 1;
            }
            else if(velocidade == 1){
                off_duty = 7;
            }
            else if(velocidade == 2){
                off_duty = 15;
            }
            delayX(10);
        }
    }

    return (EXIT_SUCCESS);
}

int count = 0;


void interrupt tc_int(void){
    
    /* timer interrupt */
 
    if(INTCONbits.TMR0IF && INTCONbits.TMR0IE) // if timer flag is set & interrupt enabled
    {                                     
        INTCONbits.TMR0IF = 0; // clear the interrupt flag 
        
        if(count == off_duty)
        {
            RC7 = 1;
            led4 = 1; // toggle a bit to say we're alive
        }
        
        if(count == max_cicle)
        {
            RC7 = 0;
            led4 = 0; // toggle a bit to say we're alive
        }
        
        count = count%max_cicle;
        count++;
    }
}