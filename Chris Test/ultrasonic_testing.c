// PIN PTA12 = Trigger
// PIN PTA13 = Echo

// Includes
#include "board.h"
#include "MKL25Z4.h"

void LED_init(void);

void US_TriggerTimerInit(void);
void US_CaptureTimerInit(void);
#define US_mod 44999
int US_index = 0;
uint32_t US_cont[2];
uint32_t US_pulseWidth;
float US_distanceCM;
float US_distanceInches;
float US_distanceFeet;

void Buzzer_PWMTimerInit(void);

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
    __disable_irq();        // Global disable IRQs (during setup)
    Buzzer_PWMTimerInit();  // Init buzzer TPM timer
    US_TriggerTimerInit();  // Init trigger pin of ultrasonic sensor
    US_CaptureTimerInit();  // Init echo pin of ultrasonic sensor
    __enable_irq();         // Global enable IRQs (after setup)

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
 * TPM1_CH1 timer set in input compare mode using PTA13.
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
    TPM1->CONTROLS[1].CnV   = ((US_mod+1)/2) - 1;   // Set up 50% duty cycle
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
}

/*
 * This interrupt function is used to calculate the distance measured using an
 * ultrasonic sensor and turn on LEDs and a buzzer based on the distance
 * measured.
 *
 * Gets called (interrupts) when the TPM1_CH1 has a CHF (count hit flag).
 *
 * Arguments: None (void)
 *
 * Return: None (void)
 */
void TPM1_IRQHandler(void)
{
    // Calculate pulse width
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

        // Convert distance to cm
        US_distanceCM = (float)(US_pulseWidth * 2/3) * 0.0343;

        // Convert distance to inches
        US_distanceInches = (float)(US_distanceCM / 2.54);

        // Convert distance to feet
        US_distanceFeet = (float)(US_distanceInches / 12);

        if (US_distanceFeet < (float)1.0)
        {
            // Red LED on, Blue LED off, DC Motor off, Buzzer on
            PTB->PCOR = GPIO_PCOR_PTCO(0x40000);  // Turn on Red LED
            PTD->PSOR = GPIO_PSOR_PTSO(0x02);     // Turn off Blue LED
            
            // Setup 50% duty cycle with a specific period 1000-6000
            TPM2->MOD = 1000 + 208 * US_distanceInches;
            TPM2->CONTROLS[0].CnV = TPM2->MOD / 2;
            TPM2->SC |= TPM_SC_CMOD(1);           // Enable buzzer
        }
        else if (US_distanceFeet < (float)2.0)
        {
            // Blue LED on, Red LED off, Buzzer on
            PTD->PCOR = GPIO_PCOR_PTCO(0x02);     // Turn on Blue LED
            PTB->PSOR = GPIO_PSOR_PTSO(0x40000);  // Turn off Red LED

            // Setup 50% duty cycle with a specific period 1000-6000
            TPM2->MOD = 1000 + 208 * US_distanceInches;
            TPM2->CONTROLS[0].CnV = TPM2->MOD / 2;
            TPM2->SC |= TPM_SC_CMOD(1);           // Enable buzzer
        }
        else
        {
            // All LED's off
            PTD->PSOR = GPIO_PSOR_PTSO(0x02);     // Turn off Blue LED
            PTB->PSOR = GPIO_PSOR_PTSO(0x40000);  // Turn off Red LED
            TPM2->SC &= ~TPM_SC_CMOD(1);          // Disable buzzer
        }
    }
    US_index++;
    TPM1->CONTROLS[1].CnSC |= TPM_CnSC_CHF(1);    // Clear CHF
}