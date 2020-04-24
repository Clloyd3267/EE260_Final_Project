/* p6_4.c UART0 Receive using interrupt (modified) */
#include <MKL25Z4.H>
void UART0_init(void);

int main (void) {
    __disable_irq();        /* global disable IRQs */
    UART0_init();
    __enable_irq();         /* global enable IRQs */

    while (1) {
        /* UART0 receive is moved to UART0 interrupt handler*/
//    	UART0->D = 'A';
//    	for(int l=0; l<1750000; l++){}  //delay of 1/2 second (approximately)
    }
}

/* UART0 interrupt handler */
void UART0_IRQHandler(void) {
    char c;
    c = UART0->D;           /* read the char received */
    UART0->D = c;           /* transmit the char back */
}

/* initialize UART0 to receive at 115200 Baud */
void UART0_init(void) {
    SIM->SCGC4 |= SIM_SCGC4_UART0(1);    					/* enable clock for UART0 */
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);   					/* use FLL output for UART Baud rate generator */
    UART0->C2 = 0;          			//0x00				/* turn off UART0 while changing configurations */

    UART0->BDH = UART0_BDH_SBR(0);		//0x00
    UART0->BDL = UART0_BDL_SBR(12);		//0x0C      		/* 115200 Baud - Using 20.9715 MHz clock*/
    //UART0->BDL  = UART0_BDL_SBR(26); 	//0x1A;      		/* SBR7:SBR0  = 0b00011010 - 115200 Baud - Clock = 48.00 MHz */
    //UART0->BDL  = UART0_BDL_SBR(137);	//0x89;      		/* SBR7:SBR0  = 0b10001001 -   9600 Baud - Clock = 20.97 MHz*/

    UART0->C4 = UART0_C4_OSR(15);		//0x0F       		/* Over Sampling Ratio 16 */
    UART0->C1 = UART0_C1_M(0);			//0x00;       		/* 8-bit data */
    UART0->C2 = UART0_C2_TE(1)|UART0_C2_RE(1)|UART0_C2_TIE(1);	//0x2C;       		/* enable receive, transmit and receive interrupt*/

    NVIC->ISER[0] |= 0x00001000;    /* enable INT12 (bit 12 of ISER[0]) */

    SIM->SCGC5 |= SIM_SCGC5_PORTA(1);	//0x0200;    		/* enable clock for PORTA */
    PORTA->PCR[1] = PORT_PCR_MUX(2);	//0x0200; 			/* make PTA1 UART0_Rx pin */
    PORTA->PCR[2] = PORT_PCR_MUX(2);	//0x0200; 			/* make PTA2 UART0_Tx pin */
}
