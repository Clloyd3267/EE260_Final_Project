/*
 * Name         : EE260_Final_Project.c
 * Author(s)    : John Bourikas, Wayne Welles, Chris Lloyd
 * Class        : EE260 (Final Project)
 * Due Date     : 2020-05-04
 * Target Board : FRDM_KL25Z
 * Description  : CDL=> Here Later
 *
 */

// Includes
#include "board.h"
#include "MKL25Z4.h"

// An enum to define the operating modes (states) of the program // CDL=> Explain each mode here and in description
typedef enum
{
    MODE_1_ANALOG_SERVO_POS = 0x1,
    MODE_2_ANALOG_MTR_SPD   = 0x2,
    MODE_3_SER_SERVO_SCAN   = 0x3,
    MODE_4_SER_SERVO_POS    = 0x4,
    MODE_5_SER_MOTOR_SPD    = 0x5
} modes;
#define DEFAULT_MODE MODE_1_ANALOG_SERVO_POS
modes currentMode;
char* modeToString(modes mode);
char* getModeName(modes mode);

// Keypad (KP_ prefix) related functionality
void KP_init(void);
char KP_getkey(void);
char KP_KeyChars[] = {'0', '1', '2', '3', '4', '5'};

// UART0 (UART0_ prefix) related functionality
#define UART0_BAUD_RATE 115200
#define UART0_OVER_SAMPLE 15
void UART0_init(void);
void UART0_TransmitPoll(char data);
void UART0_print(char* string);
void UART0_println(char* string);

// UART0 Receive buffer
#define UART0_RECEIVE_BUFFER_MAX_LENGTH 100
char UART0_receiveBuffer [UART0_RECEIVE_BUFFER_MAX_LENGTH];
uint8_t UART0_receiveCounter = 0;

// LCD Display (LCD_ prefix) related functionality
void LCD_init(void);
void LCD_nibble_write(unsigned char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_print(char* string);
#define LCD_EN 8  // BIT2 mask
#define LCD_RS 4  // BIT0 mask
#define LCD_RS_SETTINGS 0
#define LCD_RS_DATA 1

// Ultrasonic Sensor (US_ prefix) related functionality
void US_TriggerTimerInit(void);
void US_CaptureTimerInit(void);
#define US_mod 44999
int US_index = 0;
uint32_t US_cont[2];
uint32_t US_pulseWidth;
float US_distanceCM;
float US_distanceInches;
float US_distanceFeet;

// Buzzer (BUZZER_ prefix) related functionality
void Buzzer_PWMTimerInit(void);

// Onboard LEDs (LED_ prefix) related functionality
void LED_init(void);

// Other Functions
#define SYSTEM_CLOCK 48000000  // 48 MHz system clock
void delayTicks(int ticks);
void delayMs(int ms);
void delayUs(int us);

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
    LED_init();              // Init Blue and Red LEDs
    LCD_init();              // Init LCD interface

    __disable_irq();         // Global disable IRQs (during setup)
    KP_init();               // Initialize the keypad interface
    UART0_init();            // Initialize the UART0 interface
    Buzzer_PWMTimerInit();   // Init buzzer TPM timer
    US_TriggerTimerInit();   // Init trigger pin of ultrasonic sensor
    US_CaptureTimerInit();   // Init echo pin of ultrasonic sensor
    __enable_irq();          // Global enable IRQs (after setup)

    // Set the starting mode
    currentMode = DEFAULT_MODE;
    LCD_print(getModeName(currentMode));
    UART0_print(getModeName(currentMode));

    // Empty main control loop to demonstrate that the program can operate
    // with just interrupts
    while (1) {}
}

// char* modeToString(modes mode) // CDL=> Remove later. Debug only!
// {
//     switch (mode)
//     {
//         case MODE_1_ANALOG_SERVO_POS:
//             return "MODE_1_ANALOG_SERVO_POS";
//         case MODE_2_ANALOG_MTR_SPD:
//             return "MODE_2_ANALOG_MTR_SPD";
//         case MODE_3_SER_SERVO_SCAN:
//             return "MODE_3_SER_SERVO_SCAN";
//         case MODE_4_SER_SERVO_POS:
//             return "MODE_4_SER_SERVO_POS";
//         case MODE_5_SER_MOTOR_SPD:
//             return "MODE_5_SER_MOTOR_SPD";
//         default:
//             return "ERROR: Invalid Mode!!!";
//     }
// }

/*
 * This function returns a character array (cstring) representation of a (mode).
 *
 * Arguments:
 * - mode (enum modes): The mode to convert to string.
 *
 * Return:
 *
 * - (char*): A character array (cstring) representation of a (mode).
 */
char* getModeName(modes mode)
{
    switch (mode)
    {
        case MODE_1_ANALOG_SERVO_POS:
            return "ANALOG_SERVO_POS";
        case MODE_2_ANALOG_MTR_SPD:
            return "ANALOG_MTR_SPD";
        case MODE_3_SER_SERVO_SCAN:
            return "SER_SERVO_SCAN";
        case MODE_4_SER_SERVO_POS:
            return "SER_SERVO_POS";
        case MODE_5_SER_MOTOR_SPD:
            return "SER_MOTOR_SPD";
        default:
            return "Invalid Mode!!!";
    }
}

/*
 * This function initializes PortC for the keypad.
 * All pins are configured as GPIO input pin with pull-up enabled. CDL=> label cols and rows
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void KP_init(void)
{
    SIM->SCGC5    |= SIM_SCGC5_PORTD(1);   // Enable clock to Port D

    // Row 1 and 2 as GPIO with enabled pullups
    PORTD->PCR[0] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1); // Row 1
    PORTD->PCR[2] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1); // Row 2

    // Col 1 as GPIO with enabled pullups and falling edge interrupt
    PORTD->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[3] &= ~PORT_PCR_IRQC(0xF);  // Clear interrupt selection
    PORTD->PCR[3] |= PORT_PCR_IRQC(0xA);   // Enable falling edge interrupt

    // Col 2 as GPIO with enabled pullups and falling edge interrupt
    PORTD->PCR[4] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[4] &= ~PORT_PCR_IRQC(0xF);  // Clear interrupt selection
    PORTD->PCR[4] |= PORT_PCR_IRQC(0xA);   // Enable falling edge interrupt

    // Col 3 as GPIO with enabled pullups and falling edge interrupt
    PORTD->PCR[5] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[5] &= ~PORT_PCR_IRQC(0xF);  // Clear interrupt selection
    PORTD->PCR[5] |= PORT_PCR_IRQC(0xA);   // Enable falling edge interrupt

    PTD->PDDR     |= GPIO_PDDR_PDD(0x05);  // Make rows outputs, cols inputs
    PTD->PCOR     |= GPIO_PCOR_PTCO(0x05); // Enable all rows

    NVIC_EnableIRQ(PORTD_IRQn);            // Enable IRQ31 (PTD interrupts)
}

/*
 * This interrupt function takes finds the key pressed on the matrix keypad when
 * an interrupt occurs.
 *
 * Gets called (interrupts) when any of the keypad cols have a falling edge.
 *
 * Arguments: None (void)
 *
 * Return: None (void)
 */
void PORTD_IRQHandler(void)
{
    unsigned char key;  // Actual value of key press
//    char keyChar;       // Character representation of value of key press

    key = KP_getkey();  // Get pressed key from keypad

    // Ensure key was not released too soon
    if ((key != 0x0) && (key != currentMode))
    {
        // Change to state described by key
        currentMode = key;

        // Get character representation of key // CDL=> Remove later?
        // keyChar = KP_KeyChars[key];

        LCD_print(getModeName(currentMode));
        UART0_println(getModeName(currentMode));

        switch (currentMode)  // CDL=> Here!
        {
            case MODE_1_ANALOG_SERVO_POS:

                break;

            case MODE_2_ANALOG_MTR_SPD:

                break;

            case MODE_3_SER_SERVO_SCAN:
                UART0_println("Enter a scan speed for the servo (0 to 100):");
                break;

            case MODE_4_SER_SERVO_POS:
                UART0_println("Enter an angular position for the servo (-90 to +90):");
                break;

            case MODE_5_SER_MOTOR_SPD:
                UART0_println("Enter a speed for the DC Motor (0 to 100):");
                break;
        }

    }
    PORTD->ISFR |= PORT_ISFR_ISF(0x38);  // Clear interrupt flags
}

/*
 * This non-blocking function checks what key was pressed assuming one was.
 *
 * Arguments: None
 *
 * Return:
 * - (char): The key that was pressed.
 */
char KP_getkey(void)
{
    int row, cols;

    // Only one row is active at a time (logic 0)
    const char row_select[] = {0x01, 0x04};

    // Activate one row at a time and read the input to see which column is active
    for (row = 0; row < 2; row++)
    {
        PTD->PSOR = GPIO_PSOR_PTSO(0x05);             // Drive all rows high
        PTD->PCOR = GPIO_PCOR_PTCO(row_select[row]);  // Drive the active row low

        delayUs(2);               // Wait for signal to settle
        cols = PTD->PDIR & 0x38;  // Read all columns
        if (cols != 0x38) break;  // Check if a column is low, meaning key pressed
    }

    PTD->PCOR = GPIO_PCOR_PTCO(0x05);  // Enable all rows

    if (row == 2) return 0;  // If we get here, no key is pressed

    // Get the key value by checking the column and row combination
    if ((cols == 0x30) && (row == 0)) return 1;
    if ((cols == 0x28) && (row == 0)) return 2;
    if ((cols == 0x18) && (row == 0)) return 3;
    if ((cols == 0x30) && (row == 1)) return 4;
    if ((cols == 0x28) && (row == 1)) return 5;

    return 0;  // Return 0 to be safe (should not get here)
}

/*
 * This function initializes Port A for the UART0 and sets up the interface.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void UART0_init(void)
{
    // Setup UART0 Interface
    SIM->SCGC4     |= SIM_SCGC4_UART0(1);     // Enable clock for UART0
    SIM->SOPT2     |= SIM_SOPT2_UART0SRC(1);  // Use system clock
    UART0->C2       = 0;                      // Turn off UART0 during setup
    UART0->C1       = UART0_C1_M(0);          // Use 8-bit data format

    // Calculate the baud rate using the SBR and OSR from the following equation
    //
    // SBR = SYS_CLOCK / ((OSR + 1) * BAUD_RATE)
    //
    uint16_t tmpSBR = SYSTEM_CLOCK / (UART0_BAUD_RATE * (UART0_OVER_SAMPLE + 1));

    // Set the OSR and the SBR (split into BDL and BDH)
    UART0->C4       = UARTLP_C4_OSR(UART0_OVER_SAMPLE);
    UART0->BDH      = UART0_BDH_SBR(tmpSBR >> 8) & UARTLP_BDH_SBR_MASK;
    UART0->BDL      = UART0_BDL_SBR(tmpSBR & UARTLP_BDL_SBR_MASK);

    // Enable receive, transmit and receive interrupt
    UART0->C2       = UART0_C2_TE(1) | UART0_C2_RE(1) | UART0_C2_RIE(1);

    // Setup UART0_Tx and UART0_Rx on PTA1 and PTA2
    SIM->SCGC5     |= SIM_SCGC5_PORTA(1);     // Enable clock for PORTA
    PORTA->PCR[2]   = PORT_PCR_MUX(2);        // Make PTA2 UART0_Tx pin
    PORTA->PCR[1]   = PORT_PCR_MUX(2);        // Make PTA1 UART0_Rx pin

    NVIC_EnableIRQ(UART0_IRQn);               // Enable IRQ12 (UART0_IRQn)
}

/*
 * This function sends a character (char) of data.
 *
 * Arguments:
 * - data: The character (char) of data.
 *
 * Return: None (void)
 */
void UART0_TransmitPoll(char data)
{
    // Wait until transmit register is empty  // CDL=> Use interrupts?
    while (!(UART0->S1 & UART_S1_TDRE_MASK));

    // Send data
    UART0->D = data;
}

/*
 * This function sends a cstring (with '\0' at end) to the UART0.
 *
 * Note: This function does not send a linebreak "\r\n"!
 *
 * Arguments:
 * - s: The cstring (char*) to send.
 *
 * Return: None (void)
 */
void UART0_print(char* string)
{
    while (*string != 0)                // While not at end of cstring
    {
        UART0_TransmitPoll(*string++);  // Send the character through UART0
    }
}

/*
 * This function sends a cstring (with '\0' at end) to the UART0.
 *
 * Note: This function does send a linebreak "\r\n"!
 *
 * Arguments:
 * - s: The cstring (char*) to send.
 *
 * Return: None (void)
 */
void UART0_println(char* string)
{
    UART0_print(string);

    // Add Linebreak
    UART0_TransmitPoll('\r');
    UART0_TransmitPoll('\n');
}

/*
 * This interrupt function gets a character from the UART0 receive buffer.
 *
 * Gets called (interrupts) when a new character is received indicated by the
 * Receive Data Register Full (RDRF) flag is set.
 *
 * Arguments: None (void)
 *
 * Return: None (void)
 */
void UART0_IRQHandler(void)
{
    char character;

    if (UART0->S1 & UART_S1_RDRF_MASK)  // Check for RDRF flag (data available)
    {
        // Receive a character and clear the RDRF flag
        character = UART0->D;

        if ((currentMode == MODE_3_SER_SERVO_SCAN) ||
            (currentMode == MODE_4_SER_SERVO_POS)  ||
            (currentMode == MODE_5_SER_MOTOR_SPD))
        {
            if (character == '\r')  // Enter is pressed
            {
                // Add Linebreak
                UART0_TransmitPoll('\r');
                UART0_TransmitPoll('\n');

                // End string
                UART0_receiveBuffer[UART0_receiveCounter++] = '\0';

                // CDL=> Do something with string
                UART0_println(UART0_receiveBuffer);

                UART0_receiveCounter = 0;  // Reset buffer
            }
            else
            {
                UART0_TransmitPoll(character);

                // Add character to receive buffer
                UART0_receiveBuffer[UART0_receiveCounter++] = character;
            }

            // Overflow case (simply reset buffer counter)
            if (UART0_receiveCounter == UART0_RECEIVE_BUFFER_MAX_LENGTH)
            {
                UART0_receiveCounter = 0;
            }
        }
    }
    // CDL=> Add transmit interrupts?
}

/*
 * This function initializes PortB for the LCD and sets up the interface.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void LCD_init(void)
{
    // Setup Interface pins
    SIM->SCGC5    |= SIM_SCGC5_PORTB(1);    // Enable clock to Port B
    PORTB->PCR[2]  = PORT_PCR_MUX(1);       // Make PTB2 pin as GPIO  (RS)
    PORTB->PCR[3]  = PORT_PCR_MUX(1);       // Make PTB3 pin as GPIO  (EN)
    PORTB->PCR[8]  = PORT_PCR_MUX(1);       // Make PTB8 pin as GPIO  (DB4)
    PORTB->PCR[9]  = PORT_PCR_MUX(1);       // Make PTB9 pin as GPIO  (DB5)
    PORTB->PCR[10] = PORT_PCR_MUX(1);       // Make PTB10 pin as GPIO (DB6)
    PORTB->PCR[11] = PORT_PCR_MUX(1);       // Make PTB11 pin as GPIO (DB7)
    PTB->PDDR     |= GPIO_PDDR_PDD(0xF0C);  // Make PTB11-8, 2, 3 as output pins

    // Setup LCD device
    delayMs(30);                            // Initialization sequence
    LCD_nibble_write(0x03, LCD_RS_SETTINGS);
    delayMs(10);
    LCD_nibble_write(0x03, LCD_RS_SETTINGS);
    delayMs(1);
    LCD_nibble_write(0x03, LCD_RS_SETTINGS);
    delayMs(1);
    LCD_nibble_write(0x02, LCD_RS_SETTINGS);            // Use 4-bit data mode
    delayMs(1);

    LCD_command(0x28);                     // Set 4-bit data, 2-line, 5x7 font
    LCD_command(0x06);                     // Move cursor right
    LCD_command(0x01);                     // Clear screen, move cursor to home
    LCD_command(0x0F);                     // Turn on display, cursor blinking
}

/*
 * This function writes a nibble of data to the LCD.
 *
 * Arguments:
 * - data: The data nibble to send.
 * - control: The control digits to send.
 *
 * Return: None (void)
 */
void LCD_nibble_write(unsigned char data, unsigned char control)
{
    data       &= 0x0F;                            // Validate input data
    control     = (control << 2) & LCD_RS;         // Validate input control
    PTB->PDOR  |= (data << 8) | control;           // Write data without enable bit
    PTB->PDOR  |= (data << 8) | control | LCD_EN;  // Write data with enable bit
    delayUs(1);
    PTB->PDOR  &= ~LCD_EN;                         // Write data without enable bit
    PTB->PDOR  &= ~((data << 8) | control);        // Remove nibble of data
}

/*
 * This function writes a command to the LCD. (RS=0)
 *
 * Arguments:
 * - command: The command to send.
 *
 * Return: None (void)
 */
void LCD_command(unsigned char command)
{
    LCD_nibble_write((command >> 4) & 0x0F, LCD_RS_SETTINGS);  // Upper nibble first
    LCD_nibble_write(command & 0x0F, LCD_RS_SETTINGS);    // Then lower nibble

    if (command < 4)
        delayMs(4);  // Commands 1 and 2 need up to 1.64 ms
    else
        delayMs(1);  // All others take only 40 us
}

/*
 * This function writes data to the LCD. (RS=1)
 *
 * Arguments:
 * - command: The command to send.
 *
 * Return: None (void)
 */
void LCD_data(unsigned char data)
{
    LCD_nibble_write((data >> 4) & 0x0F, LCD_RS_DATA);  // Upper nibble first
    LCD_nibble_write(data & 0x0F, LCD_RS_DATA);    // Then lower nibble
    delayMs(1);
}

/*
 * This function sends a cstring (with '\0' at end) to the LCD.
 *
 * Note: This function does not send a linebreak "\r\n"!
 *
 * Arguments:
 * - s: The cstring (char*) to send.
 *
 * Return: None (void)
 */
void LCD_print(char* string)
{
    LCD_command(0x01);        // Clear display and set cursor to first line

    while (*string != 0)      // While not at end of cstring
    {
        LCD_data(*string++);  // Print Character
    }
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
    if ((currentMode == MODE_2_ANALOG_MTR_SPD) || (currentMode == MODE_5_SER_MOTOR_SPD))
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
                PTB->PCOR |= GPIO_PCOR_PTCO(0x40000);  // Turn on Red LED
                PTD->PSOR |= GPIO_PSOR_PTSO(0x02);     // Turn off Blue LED

                // Setup 50% duty cycle with a specific period 1000-6000
                TPM2->MOD = 1000 + 208 * US_distanceInches;
                TPM2->CONTROLS[0].CnV = TPM2->MOD / 2;
                TPM2->SC |= TPM_SC_CMOD(1);            // Enable buzzer
            }
            else if (US_distanceFeet < (float)2.0)
            {
                // Blue LED on, Red LED off, Buzzer on
                PTD->PCOR |= GPIO_PCOR_PTCO(0x02);     // Turn on Blue LED
                PTB->PSOR |= GPIO_PSOR_PTSO(0x40000);  // Turn off Red LED

                // Setup 50% duty cycle with a specific period 1000-6000
                TPM2->MOD = 1000 + 208 * US_distanceInches;
                TPM2->CONTROLS[0].CnV = TPM2->MOD / 2;
                TPM2->SC |= TPM_SC_CMOD(1);            // Enable buzzer
            }
            else
            {
                // All LED's off
                PTD->PSOR |= GPIO_PSOR_PTSO(0x02);     // Turn off Blue LED
                PTB->PSOR |= GPIO_PSOR_PTSO(0x40000);  // Turn off Red LED
                TPM2->SC &= ~TPM_SC_CMOD(1);           // Disable buzzer
            }
        }
        US_index++;
    }
    else
    {
        // All LED's off
        PTD->PSOR |= GPIO_PSOR_PTSO(0x02);             // Turn off Blue LED
        PTB->PSOR |= GPIO_PSOR_PTSO(0x40000);          // Turn off Red LED
        TPM2->SC &= ~TPM_SC_CMOD(1);                   // Disable buzzer
    }

    // Clear CHF flag
    TPM1->CONTROLS[1].CnSC |= TPM_CnSC_CHF(1);
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
 * This function delays (blocking) by approximately (ticks) ticks.
 *
 * Note: The CPU clock is set to 48 MHz.
 *
 * Arguments:
 * - ticks: The number of ticks to count down from.
 *
 * Return: None (void)
 */
void delayTicks(int ticks)
{
    SysTick->LOAD = ticks;  // Set timer for (ticks) ticks
    SysTick->VAL = 0;       // Ensure timer is reset

    // Use 48 Mhz / 16 clock and enable timer
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;

    // Wait for timer to reach 0 (count reached)
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {}

    SysTick->CTRL = 0;  // Disable timer
}

/*
 * This function delays (blocking) by approximately (ms) milliseconds.
 *
 * Note: The CPU clock is set to 48 MHz.
 *
 * Arguments:
 * - ms: The number of milliseconds to delay by.
 *
 * Return: None (void)
 */
void delayMs(int ms)
{
    // Set timer for (ms) ms
    delayTicks((ms * (SYSTEM_CLOCK / 16)) / 1000);
}

/*
 * This function delays (blocking) by approximately (us) microseconds.
 *
 * Note: The CPU clock is set to 48 MHz.
 *
 * Arguments:
 * - us: The number of milliseconds to delay by.
 *
 * Return: None (void)
 */
void delayUs(int us)
{
    // Set timer for (us) us
    delayTicks((us * (SYSTEM_CLOCK / 16)) / 1000000);
}
