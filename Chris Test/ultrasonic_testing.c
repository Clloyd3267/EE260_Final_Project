// PIN PTA12 = Trigger
// PIN PTA13 = Echo

#include <stdio.h>
#include "board.h"
#include "MKL25Z4.h"

void LED_init(void);

void US_TriggerTimerInit(void);
void US_CaptureTimerInit(void);
volatile uint32_t US_pulseWidth;
volatile float US_distance;
// https://stackoverflow.com/questions/905928/using-floats-with-sprintf-in-embedded-c
int tmpInt1;
float tmpFrac;
int tmpInt2;
volatile uint32_t US_cont[2];
short int US_result;
char US_buffer[30];
int US_index = 0;
#define US_mod 44999  // 14999

void UART0_init(void);
void UART0Tx(char c);
void UART0_puts(char* s);

/*
 * This function is the main function of the project.
 *
 * Arguments: None
 *
 * Return:
 * - (int): Integer status of function exit. (0 -> good, anything else -> error)
 */
int main(void) 
{
    BOARD_InitBootClocks();  // Set system clock to 48 MHz internal clock

    // Initialize Interfaces
    LED_init();
    UART0_init();
    __disable_irq();
    US_TriggerTimerInit();
    US_CaptureTimerInit();
    __enable_irq();

    sprintf(US_buffer, "\r\nUltrasonic Sensor Testing");
    UART0_puts(US_buffer);

    while (1) {}
}

void LED_init(void)
{
    // Setup Blue LED
    SIM->SCGC5    |=  SIM_SCGC5_PORTD(1);       // Enable clock to Port D
    PORTD->PCR[1] |=  PORT_PCR_MUX(1);          // Make PTD1 pin as GPIO
    PTD->PDDR     |=  GPIO_PDDR_PDD(0x02);      // Make PTD1 an output pin

    // Setup Red LED
    SIM->SCGC5     |=  SIM_SCGC5_PORTB(1);      // Enable clock to Port B
    PORTB->PCR[18] |=  PORT_PCR_MUX(1);         // Make PTB18 pin as GPIO
    PTB->PDDR      |=  GPIO_PDDR_PDD(0x40000);  // Make PTB18 an output pin
}

void US_TriggerTimerInit(void)
{
    SIM->SCGC5             |= SIM_SCGC5_PORTA(1);   // Enable clock to Port A
    SIM->SCGC6             |= SIM_SCGC6_TPM1(1);    // Enable clock to TPM1
    SIM->SOPT2             |= SIM_SOPT2_TPMSRC(1);  // Use MCGFLLCLK clock
    PORTA->PCR[12]         |= PORT_PCR_MUX(3);      // PTA12 used by TPM1_CH0

    TPM1->SC                = 0;                    // Disable timer
    TPM1->CONTROLS[0].CnSC  = TPM_CnSC_CHF(1);      // Clear CHF for Channel 0

    // Edge-aligned, pulse high MSB:MSA=10, ELSB:ELSA=10
    TPM1->CONTROLS[0].CnSC |= TPM_CnSC_MSB_MASK |
                              TPM_CnSC_ELSB_MASK;

    TPM1->CONTROLS[0].CnV   = 8;                    // Channel value for >10 us
    TPM1->SC               |= TPM_SC_PS(6);         // Prescaler of /2^6 = /64
    TPM1->MOD               = US_mod;               // Set up modulo register
    TPM1->SC               |= TPM_SC_TOF(1);        // Clear TOF
    TPM1->SC               |= TPM_SC_CMOD(1);       // Enable timer
}


void US_CaptureTimerInit(void)
{
    SIM->SCGC5             |= SIM_SCGC5_PORTA(1);   // Enable clock to Port A
    SIM->SCGC6             |= SIM_SCGC6_TPM1(1);    // Enable clock to TPM1
    SIM->SOPT2             |= SIM_SOPT2_TPMSRC(1);  // Use MCGFLLCLK clock
    PORTA->PCR[13]         |= PORT_PCR_MUX(3);      // PTA13 used by TPM1_CH1

    TPM1->SC                = 0;                    // Disable timer
    TPM1->CONTROLS[1].CnSC  = TPM_CnSC_CHF(1);      // Clear CHF for Channel 1

    // Capture on both edges, MSB:MSA=00, ELSB:ELSA=11 and set interrupt
    TPM1->CONTROLS[1].CnSC |= TPM_CnSC_CHIE_MASK |
                              TPM_CnSC_ELSB_MASK |
                              TPM_CnSC_ELSA_MASK;

    TPM1->CONTROLS[1].CnV   = ((US_mod+1)/2) - 1;   // Set up 50% dutycycle
    TPM1->SC               |= TPM_SC_PS(6);         // Prescaler of /2^6 = /64
    TPM1->MOD               = US_mod;               // Set up modulo register
    TPM1->SC               |= TPM_SC_TOF(1);        // Clear TOF
    TPM1->SC               |= TPM_SC_CMOD(1);       // Enable timer
    NVIC_EnableIRQ(TPM1_IRQn);                      // Enable IRQ18 (TPM1)
}


void TPM1_IRQHandler(void)  // CDL=> Comment this function
{
    // CDL=> does this count as a polling loop?
    while (!(TPM1->CONTROLS[1].CnSC & TPM_CnSC_CHF_MASK)) {} // Wait for CHF flag

    US_cont[US_index % 2] = TPM1->CONTROLS[1].CnV;

    if ((US_index % 2) == 1)
    {
        if (US_cont[1] > US_cont[0])
        {
            US_pulseWidth = US_cont[1] - US_cont[0];
        }
        else
        {
            US_pulseWidth = US_cont[1] - US_cont[0] + US_mod + 1;
        }

        // sprintf(US_buffer, "Pulse width %d \r\n", US_pulseWidth);
        // UART0_puts(US_buffer);

        US_distance = (float)(US_pulseWidth * 2/3) * 0.0343;  // Convert to cm

        // tmpInt1 = US_distance;
        // tmpFrac = US_distance - tmpInt1;
        // tmpInt2 = (tmpFrac * 10000);

        // sprintf(US_buffer, "Distance %d.%04d cm\r\n", tmpInt1, tmpInt2);
        // UART0_puts(US_buffer);

        US_distance = (float)(US_distance / 2.54);  // Convert to inches

        // tmpInt1 = US_distance;
        // tmpFrac = US_distance - tmpInt1;
        // tmpInt2 = (tmpFrac * 10000);

        // sprintf(US_buffer, "Distance %d.%04d inches\r\n", tmpInt1, tmpInt2);
        // UART0_puts(US_buffer);

        US_distance = (float)(US_distance / 12);  // Convert to feet

        if (US_distance < (float)1.0)
        {
            // Red LED on, Blue LED off, DC Motor off, Buzzer on
            PTB->PCOR = GPIO_PCOR_PTCO(0x40000);  // Turn on Red LED
            PTD->PSOR = GPIO_PSOR_PTSO(0x02);     // Turn off Blue LED
        }
        else if (US_distance < (float)2.0)
        {
            // Blue LED on, Red LED off, Buzzer on
            PTD->PCOR = GPIO_PCOR_PTCO(0x02);     // Turn on Blue LED
            PTB->PSOR = GPIO_PSOR_PTSO(0x40000);  // Turn off Red LED
        }
        else
        {
            // All LED's off
            PTD->PSOR = GPIO_PSOR_PTSO(0x02);     // Turn off Blue LED
            PTB->PSOR = GPIO_PSOR_PTSO(0x40000);  // Turn off Red LED
        }
    }
    US_index++;
    TPM1->CONTROLS[1].CnSC |= TPM_CnSC_CHF(1);    // Clear CHF
}

// Old UART0
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
    }   /* wait for transmit US_buffer empty */
    UART0->D = c; /* send a char */
}

void UART0_puts(char* s) {
    while (*s != 0)         /* if not end of string */
        UART0Tx(*s++);      /* send the character through UART0 */
}
