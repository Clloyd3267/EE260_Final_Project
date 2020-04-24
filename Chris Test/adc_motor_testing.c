// Includes
#include "MKL25Z4.h"
#include "board.h"

void ADC0_init(void);  // Init function for the ADC0 interface
void PWM_init(void);   // Init function for the TPM0_CH0 interface
void LED_init(void);   // Init function for the onboard LED interface
void LED_set(int s);   // Set function for the LED interface
void delayUs(int n);   // Delay function (us)

// Used to store the digital value from the ADC0 (PTE20) after conversion.
short int result;

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
    LED_init();              // Configure LEDs
    __disable_irq();         // Global disable IRQs (during setup)
    ADC0_init();             // Configure ADC0
    PWM_init();              // Configure PWM
    __enable_irq();          // Global enable IRQs (after setup)

    while (1) {}              // Empty main control loop to demonstrate that the
                              // program can operate with just interrupts
}

/*
 * This function initializes the TPM0_CH1 PWM (PTC2) output interface.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void PWM_init(void)
{
    SIM->SCGC5   |= 0x800;          // Enable clock to Port 
    PORTC->PCR[2] = 0x0400;         // PTC2 used by TPM0
    SIM->SCGC6   |= 0x01000000;     // Enable clock to TPM0
    SIM->SOPT2   |= 0x01000000;     // Use MCGFLLCLK as timer counter clock
    TPM0->SC = 0;                   // Disable timer

    // Enable TPM0_CH1 as edge-aligned PWM with an interrupt enabled
    TPM0->CONTROLS[1].CnSC |= TPM_CnSC_MSB_MASK | 
                              TPM_CnSC_ELSB_MASK | 
                              TPM_CnSC_CHIE_MASK;

    TPM0->MOD             = 60000;  // Set up modulo for 50 Hz-48.00 MHz
    TPM0->CONTROLS[1].CnV = 1500;   // Set up channel value for 2.5% duty-cycle
    TPM0->SC             |= 0x0C;   // enable TPM0 with pre-scaler /16
    NVIC_EnableIRQ(TPM0_IRQn);      // Enable IRQ17 (bit 17 of ISER[0] -> TPM0)
}

/*
 * This function initializes the ADC0 (PTE20) analog input interface.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void ADC0_init(void)
{
    uint16_t calibration;

    SIM->SCGC5    |= 0x2000;     // Clock to PORTE
    PORTE->PCR[20] = 0;          // PTE20 analog input

    SIM->SCGC6    |= 0x8000000;  // Clock to ADC0
    ADC0->SC2     &= ~0x40;      // Software trigger

    // Clock div by 4, long sample time, single ended 12 bit, bus clock
    ADC0->CFG1     = 0x40 | 0x10 | 0x04 | 0x00;

    // Start Calibration
    ADC0->SC3 |= ADC_SC3_CAL_MASK;
    while (ADC0->SC3 & ADC_SC3_CAL_MASK) {}  // Wait for calibration to complete
    
    // Initialize a 16-bit variable in RAM
    calibration = 0x0;

    // Add the plus-side calibration results to the variable
    calibration += ADC0->CLP0;
    calibration += ADC0->CLP1;
    calibration += ADC0->CLP2;
    calibration += ADC0->CLP3;
    calibration += ADC0->CLP4;
    calibration += ADC0->CLPS;

    // Divide by two
    calibration /= 2;

    // Set the MSB of the variable
    calibration |= 0x8000;

    // Store the value in the plus-side gain calibration register
    ADC0->PG = calibration;

    // Repeat the procedure for the minus-side calibration value
    calibration  = 0x0000;
    calibration += ADC0->CLM0;
    calibration += ADC0->CLM1;
    calibration += ADC0->CLM2;
    calibration += ADC0->CLM3;
    calibration += ADC0->CLM4;
    calibration += ADC0->CLMS;
    calibration /= 2;
    calibration |= 0x8000;
    ADC0->MG     = calibration;
    
    // Calibration done

    // Reconfigure ADC0
    // Clock div by 4, long sample time, single ended 12 bit, bus clock
    ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;

    NVIC_EnableIRQ(ADC0_IRQn);         // Enable ADC0 interrupt
    ADC0->SC1[0] |= ADC_SC1_AIEN_MASK; // Set up ADC0 interface to use interrupt
}

/*
 * This function initializes the onboard LEDs.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void LED_init(void) 
{
    SIM->SCGC5    |= 0x400;    // Enable clock to Port B
    SIM->SCGC5    |= 0x1000;   // Enable clock to Port D
    PORTB->PCR[18] = 0x100;    // Make PTB18 pin as GPIO
    PTB->PDDR     |= 0x40000;  // Make PTB18 as output pin
    PORTB->PCR[19] = 0x100;    // Make PTB19 pin as GPIO
    PTB->PDDR     |= 0x80000;  // Make PTB19 as output pin
    PORTD->PCR[1]  = 0x100;    // Make PTD1 pin as GPIO
    PTD->PDDR     |= 0x02;     // Make PTD1 as output pin
}

/*
 * This function sets the onboard LEDs based on the lower three bits of (s).
 *
 * Arguments: 
 * - s (int): The input number to set the LEDs to.
 *
 * Return: None (void)
 */
void LED_set(int s) 
{
    if (s & 1)                // Use bit 0 of s to control red LED */
        PTB->PCOR = 0x40000;  // Turn on red LED
    else
        PTB->PSOR = 0x40000;  // Turn off red LED

    if (s & 2)                // Use bit 1 of s to control green LED
        PTB->PCOR = 0x80000;  // Turn on green LED
    else
        PTB->PSOR = 0x80000;  // Turn off green LED

    if (s & 4)                // Use bit 2 of s to control blue LED
        PTD->PCOR = 0x02;     // Turn on blue LED
    else
        PTD->PSOR = 0x02;     // Turn off blue LED
}

/*
 * This interrupt function takes analog input from the ADC0 (PTE20), and puts 
 * out a PWM output (PTC2) with a duty cycle between 2.5% and 12.5% based on 
 * the analog input.
 *
 * Gets called (interrupts) around every 20ms ie. when the TPM0_CH0 timer has a 
 * TOF (timer overflow flag).
 *
 * Arguments: None (void)
 *
 * Return: None (void)
 */
void TPM0_IRQHandler(void)
{
    ADC0->SC1[0] &= ~0x1F;  // Start conversion on channel 0
    
    delayUs(10);            // Delay to allow for conversion to happen

    // For LED
    // Display result on LED with bits 2, 1, & 0 after shift
    LED_set(result >> 7);

    // For Servo
    // Duty cycle between 2.5% and 12.5%
    TPM0->CONTROLS[1].CnV = 1500 + result * 3/2;

    // For the Buzzer
    // Frequencies from 500 Hz to 3000 Hz
    // TPM0->MOD = 6000 - 5 * (result/4);
    // Set up channel value between 50%
    // TPM0->CONTROLS[1].CnV = TPM0->MOD/2;

    // For DC motor
    // Set up channel value between 0% - 93%
    // TPM0->CONTROLS[1].CnV = result * 14;
}

/*
 * This interrupt function reads the value of the ADC0 data register when the 
 * ADC0 finishes.
 *
 * Gets called (interrupts) around 10 ms after starting the ADC0 ie. when the 
 * ADC0 triggers a COCO (Conversion Complete) flag.
 *
 * Arguments: None (void)
 *
 * Return: None (void)
 */
void ADC0_IRQHandler(void)
{
    result = ADC0->R[0];  // Read conversion result and clear COCO flag
}

/*
 * This function delays (blocking) by approximately (n) microseconds.
 *
 * Note: The CPU clock is set to 48 MHz.
 *
 * Arguments:
 * - n: The number of microseconds to delay by.
 *
 * Return: None (void)
 */
void delayUs(int n)
{
    for (int i = 0 ; i < n; i++)
        for (int j = 0; j < 30; j++) {}
}