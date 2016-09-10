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

enum Direcao{ frente, tras, esquerda, direita };

// Variaveis referentes ao PWM
int off_duty = 1;
int max_cicle = 20;
int count = 0;

// Variaveis de Controle
int modo = 0;       // Controla o modo em que está se movimentando
int velocidade = 1; // Controla a velocidade em que está se movimentando
int direcao = 0;
int start = 0;

void setup()    // Configura as Interrupções, pinos de entrada e saída, timer...
{
    // timer configuration
    T0CONbits.TMR0ON = 0;           // disable Timer0
    T0CONbits.T0CS = 0;             // Timer increments on instruction clock
    T0CONbits.PSA = 0;              // Prescaler enable
    T0CONbits.T0PS0 = 0;            // Prescaler Configuration
    T0CONbits.T0PS1 = 0;            // Prescaler Configuration
    T0CONbits.T0PS2 = 0;            // Prescaler Configuration
    T0CONbits.T08BIT = 1;           // Timer de 8 (1) bits ou 16 bits (0)
    T0CONbits.T0SE = 0;             //   
    
    // Timer Interruptions cofiguration
    INTCONbits.TMR0IE = 1;          // Enable interrupt on TMR0 overflow
    INTCONbits.TMR0IF = 0;          // clear interrupt flag
    INTCONbits.GIE = 1;             // global interrupt enable bit
    
    /*global interrupt enable*/
    INTCONbits.GIE = 1;       // enable all interrupts
    RCONbits.IPEN = 1;        // enable interrupt priority
    
    
    INTCONbits.INT0IE = 1;    // enable int0 
    INTCON2bits.INTEDG0 = 0;  // falling edge trigger the int0
    INTCONbits.INT0IF = 0;    // clear interrupt flag 
    
    INTCON3bits.INT1IE = 1;    // enable int0 
    INTCON2bits.INTEDG1 = 0;  // falling edge trigger the int0
    INTCON3bits.INT1IF = 0;    // clear interrupt flag 
    
    INTCON3bits.INT2IE = 1;    // enable int0 
    INTCON2bits.INTEDG2 = 0;  // falling edge trigger the int0
    INTCON3bits.INT2IF = 0;    // clear interrupt flag 
    
	TRISD7 = 0; // RD7 to RD0 set to output for led.
    TRISD6 = 0;
    TRISD5 = 0;
    TRISD4 = 0;
    TRISD3 = 0;
    TRISD2 = 0;
    TRISD1 = 0;
    TRISD0 = 0; // END LEDS
    
    TRISC5 = 1; // input botao velocidade
    
    TRISB0 = 1; // input botao modo
    
    TRISB1 = 1; // input botao direcao
    TRISB2 = 1; // input botao start
    
    TRISC7 = 0; // Saída do PWM
    
    TRISB3 = 0; // motores 1
    TRISB4 = 0; // motores 1
    
    TRISC3 = 0; // motores 2
    TRISC4 = 0; // motores 2
}


// @Param n: Tempo de delay
void delayX(int n)
{
    for(int i = 0; i < n; i++)
        __delay_ms(1);
}

// @Param status: Define se o pwm vai estar ligado ou desligado.
// @Param velocidade: Define em que velocidade (1, 2 ou 3) o PWM será setado.
void SetPWM(int status, int veloc)
{
    if(status == 1)
    {
        T0CONbits.TMR0ON = 1;   // enable Timer0
    
        switch(veloc)
        {
            case 1:
                off_duty = 15;
            break;
        
            case 2:
                off_duty = 7;
            break;
        
            case 3:
                off_duty = 1;
            break;
            
            default:
                off_duty = 1;
            break;
        }
    }
    else
    {
        PORTCbits.RC7 = 0;                      // zera pino PWM
        T0CONbits.TMR0ON = 0;                   // disable Timer0
        INTCONbits.TMR0IF = 0;                  // clear the interrupt flag 
    }
}

// @Param dir: Define a direção em que os motores giram. Frente (0), tras(1), esquerda(2) e direita(3).
void Direction(int dir)
{
    switch(dir)
    {
        case frente:
            PORTBbits.RB3 = 1;  // motores 1
            PORTBbits.RB4 = 0;  // motores 1
    
            PORTCbits.RC3 = 1; // motores 2
            PORTCbits.RC4 = 0; // motores 2
        break;
        
        
        case tras:
            PORTBbits.RB3 = 0;  // motores 1
            PORTBbits.RB4 = 1;  // motores 1
    
            PORTCbits.RC3 = 0; // motores 2
            PORTCbits.RC4 = 1; // motores 2
        break;
        
        
        case esquerda:
            PORTBbits.RB3 = 1;  // motores 1
            PORTBbits.RB4 = 0;  // motores 1
    
            PORTCbits.RC3 = 0; // motores 2
            PORTCbits.RC4 = 1; // motores 2
        break;
        
        
        case direita:
            PORTBbits.RB3 = 0;  // motores 1
            PORTBbits.RB4 = 1;  // motores 1
    
            PORTCbits.RC3 = 1; // motores 2
            PORTCbits.RC4 = 0; // motores 2
        break;
    }
}

// @Param M : Modo que vai ser executado (1, 2, ou 3)
// @Param dir : Direcao em que irá se movimentar (modo 1: 0 frente, 1 tras; modo 2/3: 0 direita, 1 esquerda)
void ativaModo(int M, int dir)   // Função responsavel por gerar o comportamento de cada modo
{
    switch(M)
    {
        case 1:
            if(dir == 0)
            {
                Direction(frente);      // Configura motores para frente
                SetPWM(1,velocidade);   // Liga PWM na velocidade escolhida no main
                delayX(100);            // Espera um tempo com o PWM ligado
                SetPWM(0,velocidade);   // Desliga o PWM
            }
            else
            {
                Direction(tras);        // Configura motores para tras
                SetPWM(1,velocidade);   // Liga PWM na velocidade escolhida no main
                delayX(100);            // Espera um tempo com o PWM ligado
                SetPWM(0,velocidade);   // Desliga o PWM
            }
        break;
        
        case 2:
            if(dir == 0)        // dir = 0 : Quadrado virando a direita
            {
                for(int i = 0; i < 4; i++)      // Repete os passos abaixo para formar o quadrado
                {
                    Direction(frente);          // Configura motores para frente
                    SetPWM(1,velocidade);       // Liga PWM na velocidade escolhida no main
                    delayX(100);                // Espera um tempo com o PWM ligado
                    
                    SetPWM(0,velocidade);       // Desliga o PWM
                    delayX(10);                 // Espera um tempo com o PWM desligado
                    
                    Direction(direita);         // Configura motores para a direita
                    SetPWM(1,velocidade);       // Liga PWM na velocidade escolhida no main
                    delayX(50);                 // Espera um tempo com o PWM ligado
                    
                    SetPWM(0,velocidade);       // Desliga o PWM
                    delayX(10);                 // Espera um tempo com o PWM desligado
                }
            }
            else                // dir != 0 : Quadrado virando a esquerda
            {
                for(int i = 0; i < 4; i++)
                {
                    Direction(frente);          // Configura motores para frente
                    SetPWM(1,velocidade);       // Liga PWM na velocidade escolhida no main
                    delayX(100);                // Espera um tempo com o PWM ligado
                    
                    SetPWM(0,velocidade);       // Desliga o PWM
                    delayX(10);                 // Espera um tempo com o PWM desligado
                    
                    Direction(esquerda);        // Configura motores para a esquerda
                    SetPWM(1,velocidade);       // Liga PWM na velocidade escolhida no main
                    delayX(50);                 // Espera um tempo com o PWM ligado
                    
                    SetPWM(0,velocidade);       // Desliga o PWM
                    delayX(10);                 // Espera um tempo com o PWM desligado
                }
            }
        break;
        
        case 3:
            if(dir == 0)        // dir = 0 : Girar em torno do proprio eixo para a direita
            {
                Direction(direita);             // Configura motores para a direita
                SetPWM(1,velocidade);           // Configura motores para a esquerda
                delayX(200);                    // Espera um tempo com o PWM ligado
                SetPWM(0,velocidade);           // Desliga o PWM
            }
            else                // dir != 0 : Girar em torno do proprio eixo para a esquerda
            {
                Direction(esquerda);            // Configura motores para a esquerda
                SetPWM(1,velocidade);           // Configura motores para a esquerda
                delayX(200);                    // Espera um tempo com o PWM ligado
                SetPWM(0,velocidade);           // Desliga o PWM
            }
        break;
    }
}

int main(int argc, char** argv)
{    
    setup();
    led3 = 1;
            
    while(1)
	{
        if(PORTCbits.RC5 == 1)          // Pino do botao que controla a velocidade
        {
            delayX(20); 
            
            if(velocidade != 3)
                velocidade++;
            else
                velocidade = 1;
            
            switch(velocidade)            // Liga os leds correspondentes à velocidade
            {            
                case 1:
                    led3 = 1;
                    led4 = 0;
                    led5 = 0;
                break;
            
            
                case 2:
                    led3 = 1;
                    led4 = 1;
                    led5 = 0;
                break;
            
            
                case 3:
                    led3 = 1;
                    led4 = 1;
                    led5 = 1;
                break;
            }
        }
        
        if(start != 0)
        {
            ativaModo(modo, direcao);
            start = 0;
            led7 = 0;
        }
    }

    return (EXIT_SUCCESS);
}

void interrupt tc_int(void)
{
    
    /* timer interrupt */
 
    if(INTCONbits.TMR0IF && INTCONbits.TMR0IE) // if timer flag is set & interrupt enabled
    {                                     
        INTCONbits.TMR0IF = 0; // clear the interrupt flag 
        
        if(count == off_duty)
        {
            PORTCbits.RC7 = 1;
            //led4 = 1; // toggle a bit to say we're alive
        }
        
        if(count == max_cicle)
        {
            PORTCbits.RC7 = 0;
            //led4 = 0; // toggle a bit to say we're alive
        }
        
        count = count%max_cicle;
        count++;
    }
    
    if(INTCONbits.INT0F)        // Interrupção0 entrada RB0
    {
        SetPWM(0,1);            // Desliga o PWM caso ainda esteja ligado
        
        if(modo != 3)           
            modo++;
        else
            modo = 0;
        
        switch(modo)            // Liga os leds correspondentes ao modo
        {
            case 0:
                led0 = 0;
                led1 = 0;
                led2 = 0;
            break;
            
            case 1:
                led0 = 1;
                led1 = 0;
                led2 = 0;
            break;
            
            
            case 2:
                led0 = 1;
                led1 = 1;
                led2 = 0;
            break;
            
            
            case 3:
                led0 = 1;
                led1 = 1;
                led2 = 1;
            break;
        }
        
        INTCONbits.INT0F = 0;   // Limpa flag da Interrupção0
    }
    
    
    if(INTCON3bits.INT1F)       // Interrupção1 entrada RB1
    {
        led6 = !led6;           // Liga um led correspondente a direção
        direcao = !direcao;     // Inverte a direção
        INTCON3bits.INT1F = 0;  // Limpa flag da Interrupção1
    }
    
    if(INTCON3bits.INT2F)       // Interrupção2 entrada RB2
    {
       led7 = 1;                // Liga um led para informar inicio do modo
       start = 1;               // Ativa a flag start para iniciar o modo
       INTCON3bits.INT2F = 0;   // Limpa flag da Interrupção2
    }
}