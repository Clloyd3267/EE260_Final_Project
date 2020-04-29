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
    MODE_1_ANALOG_SERVO_POS   = 0x1,
    MODE_2_ANALOG_MOTOR_SPEED = 0x2,
    MODE_3_SER_SERVO_SCAN     = 0x3,
    MODE_4_SER_SERVO_POS      = 0x4,
    MODE_5_SER_MOTOR_SPEED    = 0x5
} modes;
#define DEFAULT_MODE MODE_1_ANALOG_SERVO_POS
modes currentMode = DEFAULT_MODE;
char* modeToString(modes mode);

// Keypad (KP_ prefix) related functionality
void KP_init(void);
char KP_getkey(void);
char KP_KeyChars[] = {'0', '1', '2', '3', '4', '5'};

// UART0 (UART0_ prefix) related functionality
#define UART0_BAUD_RATE 115200
#define UART0_OVER_SAMPLE 15
void UART0_init(void);
void UART0_TransmitPoll(char data);
void UART0_putString(char* string);

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
    __disable_irq();        // Global disable IRQs (during setup)
    KP_init();              // Initialize the keypad interface
    UART0_init();           // Initialize the UART0 interface
    __enable_irq();         // Global enable IRQs (after setup)

    UART0_putString("Hello World!\r\n");

    while (1) {}            // Empty main control loop to demonstrate that the
                            // program can operate with just interrupts
}

char* modeToString(modes mode)
{
    switch (mode)
    {
        case MODE_1_ANALOG_SERVO_POS:
            return "MODE_1_ANALOG_SERVO_POS";
        case MODE_2_ANALOG_MOTOR_SPEED:
            return "MODE_2_ANALOG_MOTOR_SPEED";
        case MODE_3_SER_SERVO_SCAN:
            return "MODE_3_SER_SERVO_SCAN";
        case MODE_4_SER_SERVO_POS:
            return "MODE_4_SER_SERVO_POS";
        case MODE_5_SER_MOTOR_SPEED:
            return "MODE_5_SER_MOTOR_SPEED";
        default:
            return "ERROR: Invalid Mode!!!";
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
    PORTD->PCR[0] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);  // Row 1
    PORTD->PCR[2] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);  // Row 2

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

    PTD->PDDR      = GPIO_PDDR_PDD(0x05);  // Make rows outputs, cols inputs
    PTD->PCOR      = GPIO_PCOR_PTCO(0x05); // Enable all rows

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
    char keyChar;       // Character representation of value of key press

    key = KP_getkey();  // Get pressed key from keypad
    if ((key != 0x0) && (key != currentMode))     // Ensure key was not released too soon
    {
        // Change to state described by key
        currentMode = key;

        // Get character representation of key
        keyChar = KP_KeyChars[key];

        UART0_putString(modeToString(currentMode));
        UART0_TransmitPoll('\r');
        UART0_TransmitPoll('\n');

        // Output the most recent key press char to UART0
        // UART0_TransmitPoll(keyChar);
        // UART0_TransmitPoll('\r');
        // UART0_TransmitPoll('\n');
    }
    PORTD->ISFR = PORT_ISFR_ISF(0x38);  // Clear interrupt flags
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
    // Wait until transmit register is empty
    while (!(UART0->S1 & UART_S1_TDRE_MASK));

    // Send data
    UART0->D = data;
}

/*
 * This function sends a cstring (with '\0' at end).
 *
 * Arguments:
 * - s: The cstring (char*) to send.
 *
 * Return: None (void)
 */
void UART0_putString(char* string)
{
    while (*string != 0)                // While not at end of cstring
    {
        UART0_TransmitPoll(*string++);  // Send the character through UART0
    }
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
    // CDL=> Come back here later
    char c;
    c = UART0->D;
    UART0_TransmitPoll(c);
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
