#include <msp.h>

#define LEFT_POSITION 5000
#define RIGHT_POSITION 10000

/* ADC results buffer */
static uint16_t resultsBuffer;

// Function Prototypes
void uart_config();
void port_config(void);
void PCM_config(void);
void FLCT_config();
void CS_config_(void);
void ADC14_config(void);
void sendToSerial(unsigned char value);

/*
 * Main function
 */
int main(void)
{
    port_config();  // Evaluate port configuration function                                               function
    uart_config();  // Evaluate UART configuration function
    PCM_config();   // Evaluate PCM configuration function
    FLCT_config();  // Evaluate FLCT onfiguration function
    CS_config_();   // Evaluate Clock configuration function
    ADC14_config(); // // Evaluate ADC14 configuration function

    /* Enabling Interrupts */
    // Enable ADC and UAR interrupt in NVIC module
    // Enable global interrupt
    NVIC->ISER[0] = 1 << ((ADC14_IRQn)&31);
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn)&31);
    NVIC->ISER[1] |= BIT5;

    //NVIC->IP[37] = 10; // Priority for S2
    //NVIC->IP[24] = 2; // Priority for ADC14
    //NVIC->IP[16] = 12;

    __enable_irq();

    /* Triggering the start of the sample */
    // Enable and start conversion
    ADC14->CTL0 |= ADC14_CTL0_ENC;
    ADC14->CTL0 |= ADC14_CTL0_SC;

    while (1)
    {
        __sleep(); // Sleep
    }
}

/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    if (ADC14->IFGR0 & ADC14_IFGR0_IFG0)
    {
        /* Store ADC14 conversion results */
        //__delay_cycles(120000);
        resultsBuffer = ADC14->MEM[0];
        //resultsBuffer[1] = ADC14->MEM[1];

        int joysticPos = 0;
        int left;
        int right;

        /*read x and y positions from Joy-stick*/
        joysticPos = resultsBuffer;
        //yPos = resultsBuffer[1];

        if (joysticPos > RIGHT_POSITION)
        { //detect x position against threshold
            right = 1;
        }
        else
        {
            right = 0;
        }
        if (joysticPos < LEFT_POSITION)
        {
            left = 1;
        }
        else
        {
            left = 0;
        }
        //send serial commands for any motion detected from joy-stick
        if (left)
        {
            __delay_cycles(120000);
            sendToSerial('L'); // Transmit L is JoyStick is from to Left side(X-axis)
        }
        if (right)
        {
            __delay_cycles(120000);
            sendToSerial('R'); // Transmit R is JoyStick is from to Right side(X-axis)
        }
    }
}

void sendToSerial(unsigned char value)
{
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG))
        ;                    // Check if the TX buffer is empty first
    EUSCI_A0->TXBUF = value; // Transmit "R" to indicate a right button press
}

void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG) // Check if  a character is received
    {
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG))
            ; // Check if the TX buffer is empty first
    }
}

void uart_config()
{
    P1->SEL0 |= BIT2 | BIT3; // select the eUSCI funtions of P1.2 and P1.3
    P1->DIR &= ~BIT2;        // P1.2 is the receive data (RXD) input pin
    P1->DIR |= BIT3;
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;       // Put eUSCI in reset
    EUSCI_A0->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK; // Remain eUSCI in reset

    // EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_PEN | EUSCI_A_CTLW0_PAR;      // Configure eUSCI clock source for SMCLK
    // Baud Rate Register calculations (see Section 24.3.10 in Reference Manual):
    // 12000000/(16*9600) = 78.125
    // Fractional portion = 0.250
    // Reference Manual Table 24-4: UCBRSx = 0x10
    // UCBRFx = int ( (78.112-78)*16) = 2
    EUSCI_A0->BRW = 78; // 12000000/16* 4800
    EUSCI_A0->MCTLW = (0x10 << EUSCI_A_MCTLW_BRS_OFS) | (2 << EUSCI_A_MCTLW_BRF_OFS) |
                      EUSCI_A_MCTLW_OS16;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;     // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;         // Enable USCI_A0 RX interrupt
}

void port_config(void)
{
    /* Configures Pin 6.0 and 4.4 as ADC input */
    P4->DIR &= ~BIT4;
    P4->SEL1 |= BIT4;
    P4->SEL0 |= BIT4;

    // Joystick (X = J1.2, P6.0) Table 6-72. as ADC  input Channel A15
    P6->DIR &= ~BIT0;
    P6->SEL1 |= BIT0;
    P6->SEL0 |= BIT0;

    // Confugre Red LED as ouput to be controlled when D is sent from PC
    P2->DIR |= BIT6;

    // Configure S2 on booterpack as inut
    P3->DIR &= ~BIT5;
    P3->REN |= BIT5;
    P3->OUT |= BIT5;

    P3->IES = BIT5; // Interrupt on high-to-low transition (Falling edged trigger on P3.5)
    P3->IFG = 0;    // Clear all P3 interrupt flags
    P3->IE = BIT5;  // Enable interrupt for P3.5 (Pin interrupt enable)
}

void PCM_config(void)
{
    /* Set the core voltage level to VCORE1 */
    // Transition to VCORE Level 1 from current power state properly
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY))
        ;
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY))
        ;
}

void FLCT_config()
{
    // Configure Flash wait-state to 1 for both banks 0 & 1
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) |
                         FLCTL_BANK0_RDCTL_WAIT_1;
    FLCTL->BANK1_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) |
                         FLCTL_BANK1_RDCTL_WAIT_1;
}
void CS_config_(void)
{
    /* Initializes Clock System */
    CS->KEY = CS_KEY_VAL;         // Unlock CS module for register access
    CS->CTL0 = 0;                 // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3; // Set DCO to 48MHz

    // Select MCLK = DCO, with divider 1
    // Select SMCLK = DCO with a divider of 1
    // Select HSMCLK = DCO with a divider of 1
    // Select ACLK = REFO with a divider of 1
    CS->CTL1 = (CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK |
                             CS_CTL1_SELS_MASK | CS_CTL1_DIVS_MASK |
                             CS_CTL1_SELS_MASK | CS_CTL1_DIVHS_MASK |
                             CS_CTL1_SELA_MASK | CS_CTL1_DIVM_MASK)) |
               CS_CTL1_SELM_3 |
               CS_CTL1_SELS_2 |
               CS_CTL1_SELS_3 |
               CS_CTL1_SELA_3 |
               CS_CTL1_DIVM__1 |
               CS_CTL1_DIVS__1 |
               CS_CTL1_DIVHS_1 |
               CS_CTL1_DIVA__1;
    CS->KEY = 0;
}

void ADC14_config(void)
{
    // Reset ENC bit for configuration
    ADC14->CTL0 &= ~ADC14_CTL0_ENC;

    //Wait for BUSY to be zero
    while (ADC14->CTL0 & ADC14_CTL0_BUSY)
        ;

    // Source of sampling is Sampling Timer
    //Multiple channel repeated conversion mode
    // Sample and Hold timer is 96 ADC clock cycles
    // ADC ON
    ADC14->CTL0 = ADC14_CTL0_ON |
                  ADC14_CTL0_MSC |
                  ADC14_CTL0_SHT0_0 |
                  ADC14_CTL0_SHP |
                  ADC14_CTL0_CONSEQ_2; //Single channel repeated conversions
    //ADC14_CTL0_MSC

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15)) */

    //Vcc- Vss references
    // Input channel A15
    // End of Sequence
    ADC14->MCTL[0] = ADC14_MCTLN_VRSEL_0 | ADC14_MCTLN_INCH_15;

    // Enable interrupt for ADC14->MEM[1]
    ADC14->IER0 |= ADC14_IER0_IE0;
}

void PORT3_IRQHandler(void)
{
    if (!(P3->IN & BIT5))
        __delay_cycles(120000);
    sendToSerial('S'); // Transmit F is JoyStick to Fire
}
