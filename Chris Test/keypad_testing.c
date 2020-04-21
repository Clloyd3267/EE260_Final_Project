#include "MKL25Z4.h"
#include "board.h"

typedef uint8_t boolean;  // Since Embedded C does not have a standard type for
                          // booleans, using uint8_t as boolean. MUST CHECK WITH
                          // IF STATEMENT. 0 -> False, Anything else -> True.

// Keypad (KP_ prefix) related functionality
void KP_init(void);
boolean KP_keyPressed(void);
char KP_getkey(void);
char KP_KeyChars[] = {'0', '1', '2', '3', '4', '5'};

// UART0 Serial (UART0_ prefix) related functionality
void UART0_init(void);
void UART0_Transmit_Poll(uint8_t);

// Other Functions
void delayMs(int n);
void delayUs(int n);

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
    KP_init();
    UART0_init();

    unsigned char key;  // Actual value of key press
    char keyChar;       // Character representation of value of key press

    while (1) // Main program execution loop
    {
        if (KP_keyPressed())    // Wait for key press
        {
            key = KP_getkey();  // Get pressed key from keypad
            if (key != 0x0)     // Ensure key was not released too soon
            {
                // Get character representation of key
                keyChar = KP_KeyChars[key];

                // Output the most recent key press char to UART0
                UART0_Transmit_Poll(keyChar);
                UART0_Transmit_Poll('\r');
                UART0_Transmit_Poll('\n');

                /*
                 * Blocking Delay to allow for time between key presses.
                 * Approximately 100 MS of delay. 
                 */
                delayMs(100);
            }
        }
    }
}

/*
 * This function initializes PortC for the keypad.
 * All pins are configured as GPIO input pin with pull-up enabled.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void KP_init(void)
{
    SIM->SCGC5   |= SIM_SCGC5_PORTD(1);  // Enable clock to Port D

    // PTC0-PTC7 as GPIO and enable pullups
    PORTD->PCR[0] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);  // Row 1
    PORTD->PCR[2] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);  // Row 2
    PORTD->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);  // Col 1
    PORTD->PCR[4] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);  // Col 2
    PORTD->PCR[5] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);  // Col 3

    PTD->PDDR     = GPIO_PDDR_PDD(0x05);   // Make PTD0 and PTD2 output pins, PTD5-3 as inputs
    PTD->PCOR     = GPIO_PCOR_PTCO(0x05);  // Enable all rows
}

/*
 * This function checks if a key is pressed on the keypad. Assumes rows are set
 * to logic 0.
 *
 * Note: see type definition of boolean at the start of this file.
 *
 * Arguments: None
 *
 * Return:
 * - (boolean): True -> Key is pressed, False -> No key is pressed.
 */
boolean KP_keyPressed(void)
{
    uint8_t cols;
    cols = PTD->PDIR & 0x38;  // Read all columns
    return (cols != 0x38);    // Check if any of the columns are 0 (key pressed)
}

/*
 * This non-blocking function checks what key was pressed assuming one was.
 * To check if a key was pressed, use "boolean KP_keyPressed()".
 *
 * The upper nibble of Port C is used as input. Pull-ups are enabled such that
 * when the keys are not pressed, these pins are pull up high.
 * The lower nibble of Port C is used as output that drives the keypad rows.
 * If any key is pressed, the program drives one row low at a time and
 * leave the rest of the rows inactive (float) then read the input pins.
 * Knowing which row is active and which column is active, the program
 * can decide which key is pressed.
 *
 * Note: To get the actual hex value of pressed key, use "uint8_t KP_hexKey[]".
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

    // if (cols == 0x30) return row * 4 + 1;    // Key in column 0
    // if (cols == 0x28) return row * 4 + 2;    // Key in column 1
    // if (cols == 0x18) return row * 4 + 3;    // Key in column 2

    return 0;  // Return 0 to be safe (should not get here)
}

/*
 * This function initializes PortA for the UART0 and sets up the interface.
 *
 * Arguments: None
 *
 * Return: None (void)
 */
void UART0_init(void)
{
    // Setup UART0 Interface
    SIM->SCGC4 |= SIM_SCGC4_UART0(1);     // Enable clock for UART0
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);  // Use FLL output for UART0 Baud rate generator
    UART0->C2   = 0;                      // Turn off UART0 while changing configurations
    UART0->C4   = UART0_C4_OSR(15);       // Over Sampling Ratio (15+1)
    UART0->BDH  = UART0_BDH_SBR(0);       // SBR12:SBR8 = 0x0  (115200 Baud, 48 MHz clock)
    UART0->BDL  = UART0_BDL_SBR(26);     // SBR7:SBR0   = 0x1A (115200 Baud, 48 MHz clock)
    UART0->C1   = UART0_C1_M(0);          // 8-bit data
    UART0->C2   = UART0_C2_TE(1) |        // Enable transmit & receive
                  UART0_C2_RE(1);

    // Setup UART0_Tx and UART0_Rx on PortA
    SIM->SCGC5   |= SIM_SCGC5_PORTA(1);   // Enable clock for PORTA
    PORTA->PCR[2] = PORT_PCR_MUX(2);      // Make PTA2 UART0_Tx pin
    PORTA->PCR[1] = PORT_PCR_MUX(2);      // Make PTA1 UART0_Rx pin
}

/*
 * This function sends a character (uint8_t) of data.
 *
 * Arguments:
 * - data: The character (uint8_t) of data.
 *
 * Return: None (void)
 */
void UART0_Transmit_Poll(uint8_t data)
{
    while (!(UART0->S1 & UART_S1_TDRE_MASK));  // Wait until transmit register is empty
    UART0->D = data;                           // Send data
}

/*
 * This function delays (blocking) by approximately (n) milliseconds.
 *
 * Note: The CPU clock is set to 48 MHz.
 *
 * Arguments:
 * - n: The number of milliseconds to delay by.
 *
 * Return: None (void)
 */
void delayMs(int n)
{
    for (int i = 0 ; i < n; i++)
        for (int j = 0; j < 3500; j++) {}
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
        for (int j = 0; j < 5; j++) {}
}
