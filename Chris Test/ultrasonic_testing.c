// PIN PTA12 = Trigger
// PIN PTA13 = Echo

// Includes
#include <stdio.h>
#include "board.h"
#include "MKL25Z4.h"

void LED_init(void);

void US_TriggerTimerInit(void);
void US_CaptureTimerInit(void);
#define US_mod 44999  // 14999 // CDL=> What does this mean?
int US_index = 0;

// CDL=> Why do these have to be volatile?
volatile uint32_t US_cont[2];
volatile uint32_t US_pulseWidth;
volatile float US_distance;

void UART0_init(void);
void UART0Tx(char c);
void UART0_puts(char* s);
char UART0_buffer[30];

// https://stackoverflow.com/questions/905928/using-floats-with-sprintf-in-embedded-c
// CDL=> Debug code for UART print
int UART0_tmpInt1;
float UART0_tmpFrac;
int UART0_tmpInt2;

void Buzzer_PWMTimerInit(void);
void Buzzer_update(void);
int Buzzer_index = 0;

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
    LED_init();             // Init Blue and Red LEDs
    UART0_init();           // Init UART0 interface
    __disable_irq();        // Global disable IRQs (during setup)
    Buzzer_PWMTimerInit();  // Init buzzer TPM timer
    US_TriggerTimerInit();  // Init trigger pin of ultrasonic sensor
    US_CaptureTimerInit();  // Init echo pin of ultrasonic sensor
    __enable_irq();         // Global enable IRQs (after setup)

    sprintf(UART0_buffer, "\r\nUltrasonic Sensor Testing");
    UART0_puts(UART0_buffer);

    while (1) {}  // Main program execution loop
}

/*
 * This function initializes the onboard Red (PTB18) and Blue (PTD1) LEDs.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
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

/*
 * This function initializes the Ultrasonic Sensor Trigger pin using the
 * TPM1_CH0 timer to create a PWM signal with a period of ~60 ms and a pulse
 * width of ~10 us. Outputs to sensor using PTA12.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
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

/*
 * This function initializes the Ultrasonic Sensor Echo pin using the
 * TPM1_CH1 timer set in input compare mode using PTA13. CDL=> Explain more here.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
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

    TPM1->MOD               = US_mod;               // Set up modulo register
    TPM1->CONTROLS[1].CnV   = ((US_mod+1)/2) - 1;   // Set up 50% duty cycle    // CDL=> Explain this?
    TPM1->SC               |= TPM_SC_PS(6);         // Prescaler of /2^6 = /64
    TPM1->SC               |= TPM_SC_TOF(1);        // Clear TOF
    TPM1->SC               |= TPM_SC_CMOD(1);       // Enable timer
    NVIC_EnableIRQ(TPM1_IRQn);                      // Enable IRQ18 (TPM1)
}

/*
 * This function initializes the buzzer interface using the TPM2_CH0 timer to
 * create a PWM signal to PTE22 with a frequency which varies from 500 Hz to
 * 3 KHz.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void Buzzer_PWMTimerInit(void)
{
    SIM->SCGC5             |= SIM_SCGC5_PORTE(1);   // Enable clock to Port E
    SIM->SCGC6             |= SIM_SCGC6_TPM2(1);    // Enable clock to TPM2
    SIM->SOPT2             |= SIM_SOPT2_TPMSRC(1);  // Use MCGFLLCLK clock
    PORTE->PCR[22]         |= PORT_PCR_MUX(3);      // PTE22 used by TPM2_CH0

    TPM2->SC                = 0;                    // Disable timer
    TPM2->CONTROLS[0].CnSC  = TPM_CnSC_CHF(1);      // Clear CHF for Channel 0

    // Enable TPM2_CH0 as edge-aligned PWM
    TPM2->CONTROLS[0].CnSC |= TPM_CnSC_MSB_MASK  |
                              TPM_CnSC_ELSB_MASK;

    TPM2->MOD               = 6000;                 // Set up modulo register
    TPM2->CONTROLS[0].CnV   = TPM2->MOD / 2;        // 50% duty cycle
    TPM2->SC               |= TPM_SC_PS(4);         // Prescaler of /2^4 = /16
    TPM2->SC               |= TPM_SC_TOF(1);        // Clear TOF
    // TPM2->SC               |= TPM_SC_CMOD(1);       // Enable timer
}

/*
 * This function updates the frequency of the buzzer interface.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void Buzzer_update(void)
{
    TPM2->SC             |= TPM_SC_CMOD(1);  // Enable timer
    TPM2->MOD             = Buzzer_index;           // Set up modulo register
    TPM2->CONTROLS[0].CnV = TPM2->MOD / 2;   // 50% duty cycle
    
    // Increment PWM max overflow value (period)
    if (Buzzer_index == 6000)
        Buzzer_index = 1000;
    else
        Buzzer_index += 200;  // CDL=> What value?
}

/*
 * This interrupt function is used to calculate the distance measured using an
 * ultrasonic sensor and turn on LEDs and a buzzer based on the distance
 * measured.
 *
 * Gets called (interrupts) when the TPM1_CH1 has a TOF (timer overflow flag).
 *
 * Arguments: None (void)
 *
 * Return: None (void)
 */
void TPM1_IRQHandler(void)
{
    // CDL=> does this count as a polling loop?
     // Wait for CHF flag to occur
    while (!(TPM1->CONTROLS[1].CnSC & TPM_CnSC_CHF_MASK)) {}

    // CDL=> Explain this block of code
    // ******************************************
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
    // ******************************************

        // Convert distance to cm
        US_distance = (float)(US_pulseWidth * 2/3) * 0.0343;

        // Convert distance to inches
        US_distance = (float)(US_distance / 2.54);

        // CDL=> Debug code for UART print
        // UART0_tmpInt1 = US_distance;
        // UART0_tmpFrac = US_distance - UART0_tmpInt1;
        // UART0_tmpInt2 = (UART0_tmpFrac * 10000);
        // sprintf(UART0_buffer, "Distance %d.%04d inches\r\n", UART0_tmpInt1, UART0_tmpInt2);
        // UART0_puts(UART0_buffer);

        // Convert distance to feet
        US_distance = (float)(US_distance / 12);

        if (US_distance < (float)1.0)
        {
            // Red LED on, Blue LED off, DC Motor off, Buzzer on
            PTB->PCOR = GPIO_PCOR_PTCO(0x40000);  // Turn on Red LED
            PTD->PSOR = GPIO_PSOR_PTSO(0x02);     // Turn off Blue LED
            TPM2->SC |= TPM_SC_CMOD(1);           // Enable buzzer
            Buzzer_update();
        }
        else if (US_distance < (float)2.0)
        {
            // Blue LED on, Red LED off, Buzzer on
            PTD->PCOR = GPIO_PCOR_PTCO(0x02);     // Turn on Blue LED
            PTB->PSOR = GPIO_PSOR_PTSO(0x40000);  // Turn off Red LED
            TPM2->SC |= TPM_SC_CMOD(1);           // Enable buzzer
            Buzzer_update();
        }
        else
        {
            // All LED's off
            PTD->PSOR = GPIO_PSOR_PTSO(0x02);     // Turn off Blue LED
            PTB->PSOR = GPIO_PSOR_PTSO(0x40000);  // Turn off Red LED
            TPM2->SC &= ~TPM_SC_CMOD(1);          // Disable buzzer
            Buzzer_index == 6000;                 // Reset buzzer frequency
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
    }   /* wait for transmit UART0_buffer empty */
    UART0->D = c; /* send a char */
}

void UART0_puts(char* s) {
    while (*s != 0)         /* if not end of string */
        UART0Tx(*s++);      /* send the character through UART0 */
}
