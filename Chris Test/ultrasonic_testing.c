// PIN PTQ = Trigger
// PIN PTE23 = echo
#include <stdio.h>
#include "board.h"
// #include "peripherals.h"
// #include "pin_mux.h"
// #include "clock_config.h"
#include "MKL25Z4.h"
// #include "fsl_debug_console.h"

void Trigger_Timer_init(void);
void Capture_Timer_init(void);
void UART0_init(void);
void UART0Tx(char c);
void UART0_puts(char* s);


volatile uint32_t pulse_width;
volatile float distance;
// https://stackoverflow.com/questions/905928/using-floats-with-sprintf-in-embedded-c
int tmpInt1;
float tmpFrac;
int tmpInt2;

volatile uint32_t cont[2];
short int result;
char buffer[30];

int i = 0;
#define mod 44999  // 14999

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootClocks();

    SIM->SCGC5 |= 0x400;        /* enable clock to Port B */
    PORTB->PCR[18] = 0x100;     /* make PTB18 pin as GPIO */
    PTB->PSOR |= 0x40000;       /* Set PTB18 to turn off red LED */
    PTB->PDDR |= 0x40000;       /* make PTB18 as output pin */

    UART0_init();                               /* initialize UART0 for output */
    sprintf(buffer, "\r\nUltrasonic sensor Testing");   /* convert to string */
    UART0_puts(buffer);

    __disable_irq();                            /* global disable IRQs */
    Trigger_Timer_init();                      /* Configure PWM */
    Capture_Timer_init();
    __enable_irq();                             /* global enable IRQs */

    while (1) {
    }
}

void Trigger_Timer_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA(1);       /* enable clock to Port A*/
    SIM->SCGC6 |= SIM_SCGC6_TPM1(1);   /* enable clock to TPM1 */
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);   /* use MCGFLLCLK as timer counter clock */
    PORTA->PCR[12] |= PORT_PCR_MUX(3);     /* PTA12 used by TPM1_CH0 */

    TPM1->SC = 0;               /* disable timer */
    TPM1->CONTROLS[0].CnSC  = 0x80;       /* clear CHF  for Channel 1*/
    TPM1->CONTROLS[0].CnSC |= 0x20|0x08;  /* edge-aligned, pulse high MSB:MSA=10, ELSB:ELSA=10*/
    TPM1->CONTROLS[0].CnV   = 8;          /* Set up channel value for >10 us*/
    TPM1->SC |= 0x06;                     /* set timer with prescaler /64 */
    TPM1->MOD = mod;                      /* Set up modulo register = 44999 */
//*************************PRE-Scaler settings **************************
    TPM1->SC |= 0x08;                     /* enable timer */
//***********************************************************************
}


void Capture_Timer_init(void)  // Also enables the TPM2_CH1 interrupt
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA(1);       /* enable clock to Port A*/
    SIM->SCGC6 |= SIM_SCGC6_TPM1(1);   /* enable clock to TPM1 */
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
    PORTA->PCR[13] |= PORT_PCR_MUX(3);    /* PTE23 used by TPM2_CH1 */
    TPM1->SC = 0;               /* disable timer */

    TPM1->CONTROLS[1].CnSC = 0x80;      /* clear CHF  for Channel 1*/
//  MSB:MSA=00, ELSB:ELSA=11 and set interrupt*/
/*  capture on both edges, MSB:MSA=00, ELSB:ELSA=11 and set interrupt*/
    TPM1->CONTROLS[1].CnSC |= TPM_CnSC_CHIE_MASK|TPM_CnSC_ELSB_MASK|TPM_CnSC_ELSA_MASK;
    TPM1->MOD = mod;                      /* Set up modulo register = 44999*/
    TPM1->CONTROLS[1].CnV = (mod+1)/2 -1; /* Set up 50% dutycycle */

    TPM1->SC |= 0x80;           /* clear TOF */
    TPM1->SC |= 0x06;           /* enable timer with prescaler /2^6 = 64 */
//*************************PRE-Scaler settings *********************************************
    TPM1->SC |= 0x08;           /* enable timer             */
//******************************************************************************************
    NVIC_EnableIRQ(TPM1_IRQn);  /* enable Timer2 interrupt in NVIC */
}


void TPM1_IRQHandler(void) {
    while(!(TPM1->CONTROLS[1].CnSC & 0x80)) { } /* wait until the CHF is set */
    cont[i%2] = TPM1->CONTROLS[1].CnV;
    if(i%2 == 1){

        if(cont[1] > cont[0] ){
            pulse_width = cont[1] - cont[0];
        }
            else {
            pulse_width = cont[1] - cont[0] + mod + 1;
        }

        sprintf(buffer, "Pulse width %d \r\n", pulse_width); /* convert to string */
        UART0_puts(buffer);
        distance = (float)(pulse_width*2/3)*0.0343;

        tmpInt1 = distance;
        tmpFrac = distance - tmpInt1;
        tmpInt2 = (tmpFrac * 10000);

        sprintf(buffer, "Distance %d.%04d cm\r\n", tmpInt1, tmpInt2); /* convert to string */
        UART0_puts(buffer);

        distance = (float)(distance/2.54);

        tmpInt1 = distance;
        tmpFrac = distance - tmpInt1;
        tmpInt2 = (tmpFrac * 10000);

        sprintf(buffer, "Distance %d.%04d inches\r\n", tmpInt1, tmpInt2); /* convert to string */
        UART0_puts(buffer);

        if(distance < (float)6.0)
            PTB->PCOR |= 0x40000;       /* Clear PTB18 to turn on red LED */
        else
            PTB->PSOR |= 0x40000;       /* Set PTB18 to turn off red LED */
    }
    i++;
/*---------------------------------------------------------------------*/
    TPM1->CONTROLS[1].CnSC |= 0x80;    /* clear CHF */
}


/* initialize UART0 to transmit at 115200 Baud */
void UART0_init(void) {
    SIM->SCGC4 |= 0x0400;    /* enable clock for UART0 */
    SIM->SOPT2 |= 0x04000000;    /* use FLL output for UART Baud rate generator */
    UART0->C2   = 0;          /* turn off UART0 while changing configurations */
    UART0->BDH  = 0x00;
    UART0->BDL  = 0x1A;      /* 115200 Baud with 48 MHz*/
    UART0->C4   = 0x0F;       /* Over Sampling Ratio 16 */
    UART0->C1   = 0x00;       /* 8-bit data */
    UART0->C2   = 0x08;       /* enable transmit */

    SIM->SCGC5   |= 0x0200;    /* enable clock for PORTA */
    PORTA->PCR[2] = 0x0200; /* make PTA2 UART0_Tx pin */
}


void UART0Tx(char c) {
    while(!(UART0->S1 & 0x80)) {
    }   /* wait for transmit buffer empty */
    UART0->D = c; /* send a char */
}

void UART0_puts(char* s) {
    while (*s != 0)         /* if not end of string */
        UART0Tx(*s++);      /* send the character through UART0 */
}
