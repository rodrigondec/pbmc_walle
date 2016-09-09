/* 
 * File:   Interrupts.c
 * Author: Julio
 *
 * Created on 8. September 2016, 09:36
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>


// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1H
#pragma config FOSC = INTIO67     // Oscillator Selection bits (External RC oscillator, port function on RA6)
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
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up (The system clock is held off until the HFINTOSC is stable.)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
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

#define _XTAL_FREQ 16000000
/*
 * 
 */
int main(int argc, char** argv) {

    TRISB0 = 1; /*input mode on RB0*/
    TRISD4 = 0;          
//    TRISD0 = 0;
    
    
    
    
    /* tiimer configuration
    
    
    T0CONbits.TMR0ON = 1; // enable timer 0
    T0CONbits.T08BIT = 1; //8 bit timer
    T0CONbits.T0CS = 0; //internal clock
    T0CONbits.PSA = 0; //internal clock
    T0CONbits.T0PS2 = 1; //prescale setup 1:256
    T0CONbits.T0PS1 = 1; //prescale setup 1:256
    T0CONbits.T0PS0 = 1; //prescale setup 1:256
   
    INTCONbits.TMR0IE = 1;               // Enable interrupt on TMR0 overflow
    INTCON2bits.TMR0IP = 0;               // low priority interrupt on timer 0
    INTCONbits.GIEL = 1;    //enable low priority interrupts
    

    */
    
    RCONbits.IPEN = 1;    // enable interrupt priority
    
    /*global interrupt enable*/
    INTCONbits.GIE = 1;    //enable all interrupts
    INTCONbits.INT0IE = 1;    // enable int0 

    INTCON2bits.INTEDG0 = 0;  // falling edge trigger the int0
    
    LATD4 = 0;
    LATD0 = 0;
    /*needs workarround*/
    __delay_ms(10);

    
    return (EXIT_SUCCESS);
}


void interrupt ISR(void){
    
    /*IO interrupt*/
    /*int 0 is aways high priority*/
    if(INTCONbits.INT0F){
        INTCONbits.INT0F = 0;
        LATD4 = !LATD4;
    }
}

void interrupt low_priority lowISR(){

    /*low priority interrupt write to timer 0*/
    
}

