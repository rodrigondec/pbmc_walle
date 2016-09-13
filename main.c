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

#define motores1a RC1
#define motores1b RC2

#define motores2a RC4
#define motores2b RC5

#define pwm RC7

#define bt_velo PORTCbits.RC6

enum Direcao{ frente, tras, esquerda, direita };

/* Variaveis referentes ao PWM */
int off_duty = 1;   // Tempo de off duty do per�odo
int max_cicle = 20; // Tempo m�ximo do per�odo
int count = 0;      // Contador de ciclo para o pwm

/* Variaveis de Controle  */
int modo = 0;       // Controla o modo em que esta se movimentando
int velocidade = 0; // Controla a velocidade em que esta se movimentando
int direcao = 0;    // Controla a dire��o do modo
int running = 0;    // Flag para ver se o robo est� executando algum modo

void setup()    // Configura as Interrup�oes, pinos de entrada/saida, timer...
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
    
    INTCON3bits.INT1IE = 1;    // enable int1 
    INTCON2bits.INTEDG1 = 0;  // falling edge trigger the int1
    INTCON3bits.INT1IF = 0;    // clear interrupt flag 
    
    INTCON3bits.INT2IE = 1;    // enable int2 
    INTCON2bits.INTEDG2 = 0;  // falling edge trigger the int2
    INTCON3bits.INT2IF = 0;    // clear interrupt flag 
    
	/* RD0 to RD7 set to output for led */
    TRISD0 = 0; // led indicadora de velocidade juntamente com led1 representando os valores binarios 01 ou 10 ou 11 (velocidades 1, 2 e 3)
	TRISD1 = 0; // led indicadora de velocidade juntamente com led0 representando os valores binarios 01 ou 10 ou 11 (velocidades 1, 2 e 3)
    
    TRISD2 = 0; // led indicadora da potencia do PWM
    
    TRISD3 = 0; // led sem uso
    
    TRISD4 = 0; // led indicadora do modo curva para esquerda
    TRISD5 = 0; // led indicadora do modo curva para direita
    
    TRISD6 = 0; // led indicadora do modo andar para frente
    TRISD7 = 0; // led indicadora do modo andar para tras
    /* END LEDS */
    
    TRISC6 = 1; // input botao velocidade
    
    TRISB0 = 1; // input botao modo
    
    TRISB1 = 1; // input botao direcao
    TRISB2 = 1; // input botao start
    
    TRISC7 = 0; // Saida do PWM
    
    /* Motores 1 */
    TRISC1 = 0; // lado 'a' dos motores
    TRISC2 = 0; // lado 'b' dos motores
    /* END Motores 1 */
    
    /* Motores 2 */
    TRISC4 = 0; // lado 'a' dos motores
    TRISC5 = 0; // lado 'b' dos motores
    /* END Motores 2 */
}

void attLeds(){
    if(velocidade == 0){
        led0 = 1;
        led1 = 0;
    }
    else if(velocidade == 1){
        led0 = 0;
        led1 = 1;
    }
    else if(velocidade == 2){
        led0 = 1;
        led1 = 1;
    }
    
    if(running){
        led2 = 1;
    }
    else{
        led2 = 0;
    }
    
    led4 = direcao;
    
    if(modo == 0){
        led6 = 1;
        led7 = 0;
    }
    else if(modo == 1){
        led6 = 0;
        led7 = 1;
    }
    else if(modo == 2){
        led6 = 1;
        led7 = 1;
    }
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
            case 0:
                off_duty = 15; // velocidade 1 tem o off duty de 15 e on duty de 5
            break;
        
            case 1:
                off_duty = 8; // velocidade 2 tem o off duty de 8 e on duty de 12
            break;
        
            case 2:
                off_duty = 1; // velocidade 3 tem o off duty de 1 e on duty de 19
            break;
            
            default:
                off_duty = 1; // velocidade 3 tem o off duty de 1 e on duty de 19
            break;
        }
    }
    else
    {
        pwm = 0; // zera pino PWM
        T0CONbits.TMR0ON = 0; // disable Timer0
        INTCONbits.TMR0IF = 0; // clear the interrupt flag 
    }
}

// @Param dir: Define a direção em que os motores giram. Frente (0), tras(1), esquerda(2) e direita(3).
void Direction(int dir)
{
    switch(dir)
    {
        case frente:
            motores1a = 1;
            motores1b = 0;
    
            motores2a = 1;
            motores2b = 0;
        break;
        
        
        case tras:
            motores1a = 0;
            motores1b = 1;
    
            motores2a = 0;
            motores2b = 1;
        break;
        
        
        case esquerda:
            motores1a = 1;
            motores1b = 0;
    
            motores2a = 0;
            motores2b = 1;
        break;
        
        
        case direita:
            motores1a = 0;
            motores1b = 1;
    
            motores2a = 1;
            motores2b = 0;
        break;
    }
}

// @Param M : Modo que vai ser executado (1, 2, ou 3)
// @Param dir : Direcao em que irá se movimentar (modo 1: 0 frente, 1 tras; modo 2/3: 0 direita, 1 esquerda)
void ativaModo(int Modo, int dir)   // Função responsavel por gerar o comportamento de cada modo
{
    switch(Modo)
    {
        case 0:
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
        
        case 1:
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
        
        case 2:
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
    attLeds();
            
    while(1)
	{
        if(bt_velo == 1)          // Pino do botao que controla a velocidade
        {
            delayX(20); 
            
            velocidade++;
            velocidade = velocidade%3;
        }
        if(running){
            ativaModo(modo, direcao);
            running = 0;
        }
        attLeds();
    }

    return (EXIT_SUCCESS);
}

void interrupt tc_int(void)
{
    
    /* timer interrupt */
 
    if(INTCONbits.TMR0IF && INTCONbits.TMR0IE) // if timer flag is set & interrupt enabled
    {                                     
        INTCONbits.TMR0IF = 0; // clear the interrupt flag 
        if(running){
            if(count == off_duty)
            {
                pwm = 1;
                led2 = 1; // toggle a bit to say we're alive
            }

            if(count == max_cicle)
            {
                pwm = 0;
                led2 = 0; // toggle a bit to say we're alive
            }

            count = count%max_cicle;
            count++;
        }
    }
    
    if(INTCONbits.INT0F)        // Interrupção0 entrada RB0
    {
        if(!running){
            modo++;
            modo = modo%3;
        }
        
        INTCONbits.INT0F = 0;   // Limpa flag da Interrupção0
    }
    
    
    if(INTCON3bits.INT1F)       // Interrupção1 entrada RB1
    {
        if(!running){
            direcao = !direcao;     // Inverte a direção
        }
//        
        INTCON3bits.INT1F = 0;  // Limpa flag da Interrupção1
    }
    
    if(INTCON3bits.INT2F)       // Interrupção2 entrada RB2
    {
        if(!running){
            running = 1; // Ativa a flag running  
        }
        
        INTCON3bits.INT2F = 0;   // Limpa flag da Interrupção2
    }
}