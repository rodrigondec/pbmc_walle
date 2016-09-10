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

enum Direcao{ frente, tras, esquerda, direita };

int count = 0, on = 0;

// @Param n: Tempo de delay
void delayX(int n)
{
    for(int i = 0; i < n; i++)
        __delay_ms(1);
}


// @Param status: Define se o pwm vai estar ligado ou desligado.
// @Param velocidade: Define em que velocidade (1, 2 ou 3) o PWM será setado.
void SetPWM(int status, int velocidade)
{
    if(status == 1)
    {
        T0CONbits.TMR0ON = 1;
    
        switch(velocidade)
        {
            case 1:
                on = 15;
                led0 = 1;
                led1 = 0;
            break;
        
            case 2:
                on = 8;
                led0 = 0;
                led1 = 1;
            break;
        
            case 3:
                on = 1;
                led0 = 1;
                led1 = 1;
            break;
            
            default:
                on = 1;
                led0 = 1;
                led1 = 0;
            break;
        }
    }
    else
    {
        RC6 = 0; // Zera pino PWM
        T0CONbits.TMR0ON = 0; // disable Timer0
        INTCONbits.TMR0IF = 0; // clear the interrupt flag 
        led0 = 0;
        led1 = 0;
        led2 = 0;
    }
}

// @Param dir: Define a direção em que os motores giram. Frente (0), tras(1), esquerda(2) e direita(3).
void Direction(int dir)
{
    switch(dir)
    {
        case frente:
            PORTBbits.RB2 = 1;  // motores 1
            PORTBbits.RB3 = 0;  // motores 1
    
            PORTBbits.RB1 = 1; // motores 2
            PORTCbits.RC7 = 0; // motores 2
            
            led4 = 0;
            led5 = 0;
            led6 = 1;
            led7 = 0;
        break;
        
        
        case tras:
            PORTBbits.RB2 = 0;  // motores 1
            PORTBbits.RB3 = 1;  // motores 1
    
            PORTBbits.RB1 = 0; // motores 2
            PORTCbits.RC7 = 1; // motores 2
            
            led4 = 0;
            led5 = 0;
            led6 = 0;
            led7 = 1;
        break;
        
        
        case esquerda:
            PORTBbits.RB2 = 1;  // motores 1
            PORTBbits.RB3 = 0;  // motores 1
    
            PORTBbits.RB1 = 0; // motores 2
            PORTCbits.RC7 = 1; // motores 2
            
            led4 = 1;
            led5 = 0;
            led6 = 0;
            led7 = 0;
        break;
        
        
        case direita:
            PORTBbits.RB2 = 0;  // motores 1
            PORTBbits.RB3 = 1;  // motores 1
    
            PORTBbits.RB1 = 1; // motores 2
            PORTCbits.RC7 = 0; // motores 2
            
            led4 = 0;
            led5 = 1;
            led6 = 0;
            led7 = 0;
        break;
    }
}

void setup()    // Configura as Interrupções, pinos de entrada e saída, timer...
{
    // timer configuration
    T0CONbits.T0CS = 0;             // Timer increments on instruction clock
    T0CONbits.PSA = 0;              // Prescaler enable
    T0CONbits.T0PS0 = 0;            // Prescaler Configuration
    T0CONbits.T0PS1 = 0;            // Prescaler Configuration
    T0CONbits.T0PS2 = 0;            // Prescaler Configuration
    T0CONbits.T08BIT = 1;           // Timer de 8 (1) bits ou 16 bits (0)
    T0CONbits.T0SE = 0;             //   
    
    INTCONbits.TMR0IE = 1;          // Enable interrupt on TMR0 overflow
    INTCONbits.TMR0IF = 0;          //clear interrupt flag
    INTCONbits.GIE = 1;   
    
    TRISD7 = 0; // RD7 to RD0 set to output for led.
    TRISD6 = 0;
    TRISD5 = 0;
    TRISD4 = 0;
    TRISD3 = 0;
    TRISD2 = 0;
    TRISD1 = 0;
    TRISD0 = 0; // END LEDS
    
    TRISD0 = 0;     // Pino usado para o PWM
    TRISB0 = 0;     // Pino usado para o PWM
    
    TRISB2 = 0;     // Pino usado para controlar a direção dos motores 1
    TRISB3 = 0;     // Pino usado para controlar a direção dos motores 1
    
    TRISB1 = 0;     // Pino usado para controlar a direção dos motores 2
    TRISC7 = 0;     // Pino usado para controlar a direção dos motores 2
}

int main(int argc, char** argv)
{
    setup();
    
    while(1)
    {
        Direction(frente);  // Seleciona a direção dos motores
        SetPWM(1,1);        // Liga PWM na velocidade 3
        delayX(150);        // Delay 3s
        
        SetPWM(0,3);        // Desliga PWM
        delayX(50);         // Delay 1sn
        
        Direction(tras);  // Seleciona a direção dos motores
        SetPWM(1,3);        // Liga PWM na velocidade 3
        delayX(150);        // Delay 3s
        
        SetPWM(0,3);        // Desliga PWM
        delayX(50);         // Delay 1sn
    
        Direction(esquerda); // Seleciona a direção dos motores
        SetPWM(1,1);        // Liga PWM na velocidade 3
        delayX(100);        // Delay 2s
        
        SetPWM(0,3);        // Desliga PWM
        delayX(50);         // Delay 1s
        
        Direction(direita); // Seleciona a direção dos motores
        SetPWM(1,3);        // Liga PWM na velocidade 3
        delayX(100);        // Delay 2s
        
        SetPWM(0,3);        // Desliga PWM
        delayX(50);         // Delay 1s
    }

    return (EXIT_SUCCESS);
}


void interrupt tc_int(void){
    
    /* timer interrupt */
 
    if(INTCONbits.TMR0IF && INTCONbits.TMR0IE) // if timer flag is set & interrupt enabled
    {                                     
        INTCONbits.TMR0IF = 0;                  // clear the interrupt flag 
        
        if(count == on)     
        {
            PORTBbits.RB0 = 1;  
            led2 = 1;
            
        }
        
        if(count == 20)
        {
            PORTBbits.RB0 = 0;
            led2 = 0;
            count = 0;
        }
        
        count = (count + 1);
    }
}