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
#include <stdlib.h>
#include <ctype.h>

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
char* getModeName(modes mode);
void setCurrentMode(modes mode);

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

// Servo and DC Motor (MOTOR_ prefix) related functionality
int MTR_servoScanSpeed;
float MTR_servoAngularPos;
int MTR_dcSpeed;
int countUp;
#define MTR_SCAN_INCR 5
void MTR_PWMTimerInit(void);

// Analog to Digital 0 (ADC0_ prefix) related functionality
void ADC0_init(void);

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
    US_TriggerTimerInit();   // Init trigger pin of ultrasonic sensor
    US_CaptureTimerInit();   // Init echo pin of ultrasonic sensor
    Buzzer_PWMTimerInit();   // Init buzzer TPM timer
    MTR_PWMTimerInit();      // Init DC and Servo Motor interface
    ADC0_init();             // Init ADC0 interface
    __enable_irq();          // Global enable IRQs (after setup)

    // Set the starting mode
    setCurrentMode(DEFAULT_MODE);

    // Empty main control loop to demonstrate that the program can operate
    // with just interrupts
    while (1) {}
}

/*
 * This function returns a character array (cstring) representation of a (mode).
 *
 * Arguments:
 * - mode (enum modes): The mode to convert to string.
 *
 * Return:
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
 * This function sets the current mode and enables/disables devices based on the
 * mode.
 *
 * Arguments:
 * - mode (enum modes): The mode to set.
 *
 * Return: None (void)
 */
void setCurrentMode(modes mode)
{
    // Set the current mode
    currentMode = mode;

    // Print the current mode to the terminal and LCD
    LCD_print(getModeName(currentMode));
    UART0_println(getModeName(currentMode));

    // Print mode specific information
    switch (currentMode)
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

    // Setup servo scan mode
    if (currentMode == MODE_3_SER_SERVO_SCAN)
    {
        SysTick->LOAD = 150000;  // Set slowest speed
        SysTick->VAL = 0;        // Ensure timer is reset

        __disable_irq();         // Global disable IRQs
        // Use 48 Mhz / 16 clock, interrupt, and enable timer
        SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
        __enable_irq();          // Global enable IRQs
    }
    else
    {
        SysTick->CTRL = 0;       // Disable timer
    }

    // Enable/Disable motor PWM timer channels
    switch (currentMode)
    {
        case MODE_1_ANALOG_SERVO_POS:
        case MODE_3_SER_SERVO_SCAN:
        case MODE_4_SER_SERVO_POS:
            TPM0->CONTROLS[2].CnSC |= TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
            TPM0->CONTROLS[3].CnSC = 0;
            break;

        case MODE_2_ANALOG_MTR_SPD:
        case MODE_5_SER_MOTOR_SPD:
            TPM0->CONTROLS[2].CnSC = 0;
            TPM0->CONTROLS[3].CnSC |= TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
            break;
        default:
            break;

    }
}

/*
 * This function initializes PortC for the keypad.
 * All pins are configured as GPIO input pin with pull-up enabled. Only the
 * first two rows and the first three cols were uses for space purposes.
 *
 * PTD0 -> Row 1 (Output)
 * PTD2 -> Row 2 (Output)
 * PTD3 -> Col 1 (Input)
 * PTD4 -> Col 2 (Input)
 * PTD5 -> Col 3 (Input)
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

    NVIC_SetPriority(PORTD_IRQn, 0);
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
    char key;

    key = KP_getkey();  // Get pressed key from keypad

    // Ensure key was not released too soon and mode is not the current one
    if ((key != 0x0) && (key != currentMode))
    {
        // Change to state described by key
        setCurrentMode(key);
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

    NVIC_SetPriority(PORTD_IRQn, 1);
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
    // Wait until transmit register is empty
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

        // Ensure only in a mode which needs the serial terminal
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

                // Get a float representation of the string
                float val = atof(UART0_receiveBuffer);

                // Parse and validate number based on mode
                switch (currentMode)
                {
                    case MODE_3_SER_SERVO_SCAN:
                        MTR_servoScanSpeed = (int)val;
                        if ((MTR_servoScanSpeed < 0) ||
                            (MTR_servoScanSpeed > 100))
                        {
                            UART0_println("Error: Invalid number!");
                        }
                        else
                        {
                            // Set the Servo Motor scan speed to a certain value
                            SysTick->LOAD = 150000 - MTR_servoScanSpeed * 1200;
                        }
                        break;

                    case MODE_4_SER_SERVO_POS:
                        MTR_servoAngularPos = val;
                        if ((MTR_servoAngularPos < -90) ||
                            (MTR_servoAngularPos > 90))
                        {
                            UART0_println("Error: Invalid number!");
                        }
                        else
                        {
                            // Set servo position to a certain angular position
                            TPM0->CONTROLS[2].CnV = 1659 +
                                                (MTR_servoAngularPos + 90) * 33;
                        }
                        break;

                    case MODE_5_SER_MOTOR_SPD:
                        MTR_dcSpeed = (int)val;
                        if ((MTR_dcSpeed < 0) ||
                            (MTR_dcSpeed > 100))
                        {
                            UART0_println("Error: Invalid number!");
                        }
                        else
                        {
                            // Set the DC Motor speed to a certain speed
                            TPM0->CONTROLS[3].CnV = 18000 + (MTR_dcSpeed * 420);
                        }
                        break;

                    default:
                        break;
                }

                UART0_receiveCounter = 0;  // Reset buffer
            }
            else if (isalnum(character) || ispunct(character))
            {
                UART0_TransmitPoll(character);

                // Add character to receive buffer
                UART0_receiveBuffer[UART0_receiveCounter++] = character;
            }
            else if (character == 0x7F)
            {
                UART0_TransmitPoll(character);

                // Remove previous character receive buffer
                UART0_receiveCounter--;
            }

            // Overflow case (simply reset buffer counter)
            if (UART0_receiveCounter == UART0_RECEIVE_BUFFER_MAX_LENGTH)
            {
                UART0_receiveCounter = 0;
            }
        }
    }
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
    LCD_nibble_write(0x02, LCD_RS_SETTINGS);
    delayMs(1);

    LCD_command(0x28);                      // Set 4-bit data, 2-line, 5x7 font
    LCD_command(0x06);                      // Move cursor right
    LCD_command(0x01);                      // Clear screen, move cursor to home
    LCD_command(0x0F);                      // Turn on display, cursor blinking
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

    NVIC_SetPriority(PORTD_IRQn, 1);
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
    if ((currentMode == MODE_2_ANALOG_MTR_SPD) ||
        (currentMode == MODE_5_SER_MOTOR_SPD))
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
                TPM0->SC &= ~TPM_SC_CMOD(3);           // Disable Motor PWM
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
                TPM0->SC &= ~TPM_SC_CMOD(3);           // Disable Motor PWM
            }
            else
            {
                // All LED's off
                PTD->PSOR |= GPIO_PSOR_PTSO(0x02);     // Turn off Blue LED
                PTB->PSOR |= GPIO_PSOR_PTSO(0x40000);  // Turn off Red LED
                TPM2->SC &= ~TPM_SC_CMOD(3);           // Disable buzzer
                TPM0->SC |= TPM_SC_CMOD(1);            // Enable Motor PWM
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
 * This function initializes the TPM0_CH1 and TPM0_CH2 PWM interfaces.
 *
 * PTA5  -> Servo Motor -> TPM0_CH2
 * PTE30 -> DC Motor    -> TPM0_CH3
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void MTR_PWMTimerInit(void)
{
    // Initialize motor variables
    MTR_servoScanSpeed = 0;
    MTR_servoAngularPos = -90;
    MTR_dcSpeed = 0;
    countUp = 1;

    SIM->SCGC5    |= SIM_SCGC5_PORTA(1);       // Enable clock to Port A
    SIM->SCGC5    |= SIM_SCGC5_PORTE(1);       // Enable clock to Port E
    PORTA->PCR[5]  = PORT_PCR_MUX(3);          // PTA5 used by TPM0_CH2
    PORTE->PCR[30] = PORT_PCR_MUX(3);          // PTE30 used by TPM0_CH3
    SIM->SCGC6    |= SIM_SCGC6_TPM0(1);        // Enable clock to TPM0
    SIM->SOPT2    |= SIM_SOPT2_TPMSRC(1);      // Use MCGFLLCLK as timer clock
    TPM0->SC       = 0;                        // Disable timer

    // Enable TPM0_CH2 as edge-aligned PWM
    TPM0->CONTROLS[2].CnSC |= TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM0->CONTROLS[2].CnV   = 1659;

    // Enable TPM0_CH1 as edge-aligned PWM
    TPM0->CONTROLS[3].CnSC |= TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM0->CONTROLS[3].CnV   = 0;

    TPM0->MOD               = 60000;           // Set up modulo for 20ms period
    TPM0->SC               |= TPM_SC_PS(4);    // Use prescaler of /2^4=16
    TPM0->SC               |= TPM_SC_CMOD(1);  // Enable counter for PWM
    TPM0->SC               |= TPM_SC_TOIE(1);  // Enable Timer overflow interrupt

    NVIC_SetPriority(PORTD_IRQn, 1);
    NVIC_EnableIRQ(TPM0_IRQn);      // Enable IRQ17 (TPM0)
}

/*
 * This function initializes the ADC0 analog input interface.
 *
 * PTE20 -> Potentiometer   -> ADC0_CH0
 * PTE21 -> Photo Resistors -> ADC0_CH4
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void ADC0_init(void)
{
    uint16_t calibration;

    SIM->SCGC5    |= SIM_SCGC5_PORTE(1);  // Clock to PORTE
    PORTE->PCR[20] = PORT_PCR_MUX(0);     // PTE20 analog input
    PORTE->PCR[21] = PORT_PCR_MUX(0);     // PTE21 analog input

    SIM->SCGC6    |= SIM_SCGC6_ADC0(1);   // Clock to ADC0
    ADC0->SC2     &= ~ADC_SC2_ADTRG(1);   // Software trigger

    // Clock div by 4, long sample time, single ended 12 bit, bus clock
    ADC0->CFG1     = ADC_CFG1_ADIV(2)   |
                     ADC_CFG1_ADLSMP(1) |
                     ADC_CFG1_MODE(1)   |
                     ADC_CFG1_ADICLK(0);

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
    ADC0->CFG1     = ADC_CFG1_ADIV(2)   |
                     ADC_CFG1_ADLSMP(1) |
                     ADC_CFG1_MODE(1)   |
                     ADC_CFG1_ADICLK(0);

    NVIC_SetPriority(PORTD_IRQn, 1);
    NVIC_EnableIRQ(ADC0_IRQn);         // Enable ADC0 interrupt

    ADC0->SC1[0] |= ADC_SC1_AIEN_MASK; // Set up ADC0 interface to use interrupt
}

/*
 * This interrupt function is used to start an analog to digital conversion
 * based on the current mode.
 *
 * Gets called (interrupts) around every 20ms ie. when the TPM0 timer has a
 * TOF (timer overflow flag).
 *
 * Arguments: None (void)
 *
 * Return: None (void)
 */
void TPM0_IRQHandler(void)
{
    if (currentMode == MODE_1_ANALOG_SERVO_POS)
    {
        ADC0->SC1[0] &= ~0x1B;  // Start conversion on channel 4
    }
    else if (currentMode == MODE_2_ANALOG_MTR_SPD)
    {
        ADC0->SC1[0] |= 0x04;   // Start conversion on channel 0
    }
    TPM0->SC |= TPM_SC_TOF(1);  // Clear the TOF
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
    int result = ADC0->R[0];  // Read conversion result and clear COCO flag

    if (currentMode == MODE_1_ANALOG_SERVO_POS)
    {
        // Range where Photo Resistors operate:
        // 1.65V +-0.35V (1.3V - 2V)
        // Step Size = VREF / NUM_STEPS = 3.3V/4096 = 805.66 uV
        float voltage = result / 1241;

        MTR_servoAngularPos = -90 + (voltage * 55);

        if (ADC0->SC1[0] & 0x04)
        {
            if ((1.3 < voltage) && (voltage < 3.3))
            {
                // Set servo position to a certain angular position
                TPM0->CONTROLS[2].CnV = 1659 +
                                    (MTR_servoAngularPos + 90) * 33;
            }
            else
            {
                ADC0->SC1[0] &= ~0x1F;  // Start conversion on channel 0
            }
        }
        else
        {
            // Set servo position to a certain angular position
            TPM0->CONTROLS[2].CnV = 1659 +
                                (MTR_servoAngularPos + 90) * 33;
        }
    }
    else if (currentMode == MODE_2_ANALOG_MTR_SPD)
    {
        MTR_dcSpeed = result;

        // Set the DC Motor speed to a certain speed
        TPM0->CONTROLS[3].CnV = 18000 + (MTR_dcSpeed * 10);
    }
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

/*
 * This interrupt function is used change the angular position of the servo
 * motor to make it scan back and forth. The speed of the scan is controlled by
 * the count of the SysTick Counter. Setup for a counter value between 50ms and
 * 10ms.
 *
 * Gets called (interrupts) when the SysTick timer reaches 0 or the count
 * flag is enabled.
 *
 * Arguments: None (void)
 *
 * Return: None (void)
 */
void SysTick_Handler(void)
{
    if (currentMode == MODE_3_SER_SERVO_SCAN)
    {
        // Set servo position to a certain angular position
        TPM0->CONTROLS[2].CnV = 1659 + ((MTR_servoAngularPos) + 90) * 33;

        // Increment degrees
        if (countUp)
        {
            MTR_servoAngularPos += MTR_SCAN_INCR;
        }
        else
        {
            MTR_servoAngularPos -= MTR_SCAN_INCR;
        }

        // Change direction when reaching -90 or 90 degrees
        if (MTR_servoAngularPos > 90)
        {
            countUp = 0;
        }
        if (-90 > MTR_servoAngularPos)
        {
            countUp = 1;
        }
    }
}
