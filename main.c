#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"
#include <math.h>

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

char str2[20];
uint32_t table1[4096];
uint32_t accum = 0;
uint32_t phase = 0;
char str[81];
uint32_t out = 0;
uint8_t mode = 5;
uint32_t op2;
int8_t neg = 1;
uint32_t freq1 = 0;
uint32_t freq2 = 0;
float storefreq[50] = { 0 };
uint16_t storecnt = 0;
float storeinstant[50] = { 0 };
int8_t checkdc = 0;
float inpamp = 0.0;
int8_t inpampcheck = 0;
uint8_t gainaccess = 0;
uint8_t sweepaccess = 0;
uint8_t errcndf = 0;
uint8_t errcndv = 0;
uint8_t errcndo = 0;
uint8_t errcndf2 = 0;
uint8_t errcndd = 0;
uint16_t rr;
//char strdown[20];

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #7");
// 1
    __asm("WMS_LOOP1:   SUB  R1, #1");
// 7
    __asm("             CBZ  R1, WMS_DONE1");
// 6+1*3
    __asm("             NOP");
// 6

    __asm("             B    WMS_LOOP1");
// 6*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");
// 1
    __asm("             CBZ  R0, WMS_DONE0");
// 1
    __asm("             NOP");
// 1
    __asm("             B    WMS_LOOP0");
// 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");
// ---
// 40 clocks/us + error
}
//----------------------------------------------------------------------------------------------------------------
// Blocking function that returns only when SW1 is pressed
void waitPbPress()
{
    while (PUSH_BUTTON)
        ;
}
//----------------------------------------------------------------------------------------------------------------
// Initialize Hardware
void initHw()
{
// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

// Set GPIO ports to use APB (not needed since default configuration -- for clarity)
// Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

// Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF
            | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOD;

// Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

// Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                         // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                       // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

// Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0xB0;                   // make bits 4 , 5 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0xB0;                      // set drive strength to 2mA
    GPIO_PORTB_AFSEL_R |= 0xB0; // select alternative functions for MOSI, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK
            | GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0xB0;        // enable digital operation on TX, CLK pins
    GPIO_PORTB_PUR_R |= 0x10;                      // must be enabled when SPO=1

// Configure the SSI2 as a SPI master, mode 3, 16bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;       // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                  // select master mode
    SSI2_CC_R = 0;                    // select system clock as the clock source
    SSI2_CPSR_R = 20;                  // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 16-bit
    SSI2_CR1_R |= SSI_CR1_SSE;      // turn on SSI2              // turn on SSI2

// Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER1_TAILR_R = 400;      // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);     // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    SYSCTL_RCGCADC_R |= 1;

}
//----------------------------------------------------------------------------------------------------------------
void output_slow()
{
// Configure AN0 as an analog input
// turn on ADC module 0 clocking
    GPIO_PORTE_AFSEL_R |= 0x08;    // select alternative functions for AN0 (PE3)
    GPIO_PORTE_DEN_R &= ~0x08;          // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= 0x08;           // turn on analog operation on pin PE3
    ADC0_CC_R = ADC_CC_CS_SYSPLL; // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3; // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR; // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 0;                               // set first sample to AN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;             // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;

}

//----------------------------------------------------------------------------------------------------------------
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF)
        ;
    UART0_DR_R = c;
}
//----------------------------------------------------------------------------------------------------------------
void putnUart0(uint8_t n)
{
    while (UART0_FR_R & UART_FR_TXFF)
        ;
    UART0_DR_R = n;
}
//----------------------------------------------------------------------------------------------------------------
// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}
//----------------------------------------------------------------------------------------------------------------
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
        ;
    return UART0_DR_R & 0xFF;
}
//----------------------------------------------------------------------------------------------------------------
void reset(void)
{
//
// Jump to the CCS C initialization routine.  This will enable the
// floating-point unit as well, so that does not need to be done here.
//
    __asm("    .global _c_int00\n"
            "    b.w     _c_int00");
}
//----------------------------------------------------------------------------------------------------------------
void sendGraphicsLcdData(uint32_t data)
{
// CS_NOT = 0;                        // assert chip select
    __asm (" NOP");
    // allow line to settle
    __asm (" NOP");
    __asm (" NOP");
    __asm (" NOP");
// A0 = 1;                            // set A0 for data
    SSI2_DR_R = data;                  // write data
    while (SSI2_SR_R & SSI_SR_BSY)
        ;    // wait for transmission to stop
// CS_NOT = 1;                        // de-assert chip select
}
//----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------
void output_fast()
{
// Configure AN1 as an analog input
// turn on ADC module 0 clocking
    GPIO_PORTE_AFSEL_R |= 0x04;    // select alternative functions for AN1 (PE2)
    GPIO_PORTE_DEN_R &= ~0x04;          // turn off digital operation on pin PE2
    GPIO_PORTE_AMSEL_R |= 0x04;           // turn on analog operation on pin PE2
    ADC0_CC_R = ADC_CC_CS_SYSPLL; // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3; // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR; // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 1;                               // set first sample to AN1
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;             // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;
}
//----------------------------------------------------------------------------------------------------------------
int32_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                    // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY)
        ;          // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                    // get single result from the FIFO
}
//----------------------------------------------------------------------------------------------------------------
void voltage_test()
{
    uint32_t raw;
    waitMicrosecond(300000);
    output_slow();
    float instantvalue;
    char str1[30];
    raw = readAdc0Ss3();
    instantvalue = (3.3 * (raw + 0.5) / 4096);
    storeinstant[storecnt++] = instantvalue;
    //putsUart0("inside voltage test");
    sprintf(str1, "Amplitude is= %3.6f", instantvalue);
    putsUart0(str1);
    putsUart0("\r\n");
}

//----------------------------------------------------------------------------------------------------------------
void gain()
{
    char str1[30];
    char str2[30];
    uint8_t i;
    float gain1;
    for (i = 0; i < storecnt; i++)
    {
        sprintf(str2, "frequency is = %3.3f", storefreq[i]);
        putsUart0(str2);
        putsUart0(" ");

        gain1 = -20 * log(storeinstant[i] / inpamp);
        sprintf(str1, "gain is= %3.3f dB", gain1);
        putsUart0(str1);
        putsUart0("\r\n");
    }
}

//----------------------------------------------------------------------------------------------------------------
void dcTest()
{
    uint32_t raw = 0;
    output_slow();
    waitMicrosecond(600000);
    float instantvalue;
    char str1[10];
    raw = readAdc0Ss3();
    instantvalue = (3.5 * (raw + 0.5) / 4096);
    putsUart0("DC voltage test");
    sprintf(str1, "%3.3f", instantvalue);
    putsUart0(str1);
    putsUart0("\r\n");

}

//----------------------------------------------------------------------------------------------------------------
void sineTest()
{
    uint32_t raw;
    waitMicrosecond(300000);
    output_slow();
    float instantvalue;
    char str1[10];
    raw = readAdc0Ss3();
    instantvalue = (5.3 * (raw + 0.5) / 4096);
    putsUart0("Sine voltage test");
    sprintf(str1, "%3.3f", instantvalue);
    putsUart0(str1);
    putsUart0("\r\n");

}

//----------------------------------------------------------------------------------------------------------------
void Timer1Isr()
{
    if (mode == 0)
    {
        GREEN_LED ^= 1;
        accum = accum + phase;

        out = table1[accum >> 20];
        SSI2_DR_R = out;
    }
    if (mode == 1)
    {
        RED_LED ^= 1;
        SSI2_DR_R = op2;
    }

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
// clear interrupt flag

}

void output_dc(char* frequency)
{

    float amplitude_dc = neg * atof(frequency);
    uint16_t op_dc;
    mode = 0;
    if (amplitude_dc >= -5 && amplitude_dc <= 5)
    {
        putsUart0("dc function is invoked");
        putsUart0("\r\n");
        op_dc = 1998 + (amplitude_dc * -377);
        op2 = 0x3000 | op_dc;
        mode = 1;
        neg = 1;
    }
    else
    {
        putsUart0(" amplitude out of limit ");
        putsUart0("\r\n");

    }

}
//----------------------------------------------------------------------------------------------------------------------------------------------

void output_sinewave(char* frequency, char* voltage, char* offset)
{
    if (errcndf == 1 || errcndv == 1 || errcndo == 1)
    {
        putsUart0("Invoking sine fucntion failed\n");
    }
    else
    {
        uint16_t freq = atoi(frequency);
        float off = atof(offset);
        float volt = atof(voltage);
        phase = freq * pow(2, 32) / 100000;
        uint16_t i = 0;
        mode = 0;
        //{
        if (volt >= -5 && volt <= 5)
        {
            if (off >= -5 && off <= 5)
            {
                putsUart0("sine function is invoked");
                putsUart0("\r\n");

                for (i = 0; i < 4096; i++)
                {
                    rr =
                            (1998
                                    - (377
                                            * ((volt) * sin(i * 2 * 3.14 / 4096)
                                                    + off)));
                    table1[i] = 0x3000 + rr;
                }
            }
            else
            {
                putsUart0("invalid offset limit");
                putsUart0("\r\n");
            }
            //Timer1Isr();
            // TIMER1_CTL_R |= TIMER_CTL_TAEN;
        }
        // turn-on time
        else
        {
            putsUart0("invalid amplitude limit");
            putsUart0("\r\n");
        }
    }

}

//----------------------------------------------------------------------------------------------------------------------------------------------

void output_sawtooth(char* frequency, char* voltage, char* offset)
{
    if (errcndf == 1 || errcndv == 1 || errcndo == 1)
    {
        putsUart0("Invoking sawtooth fucntion failed\n");
    }
    else
    {
        uint16_t freq = atoi(frequency);
        float off = atof(offset);
//uint32_t gop;
        float volt = atof(voltage);
        phase = freq * pow(2, 32) / 100000;
        uint16_t i = 0;
        mode = 0;
//{
        if (volt >= -5 && volt <= 5)
        {
            if (off >= -5 && off <= 5)
            {
                putsUart0("sawtooth function is invoked");
                putsUart0("\r\n");
                for (i = 0; i < 4096; i++)
                {

                    table1[i] = 0x3000 - 380 * off + (380 * volt + 1998)
                            - ((2 * 380 * volt * i) / 4096);

                }
//Timer1Isr();
// TIMER1_CTL_R |= TIMER_CTL_TAEN;
            }
            else
            {
                putsUart0("invalid offset limit");
                putsUart0("\r\n");
            }
        }
// turn-on timer
        else
        {
            putsUart0("invalid amplitude limit");
            putsUart0("\r\n");
        }
    }
}
//----------------------------------------------------------------------------------------------------------------
void output_triangle(char* frequency, char* voltage, char* offset)
{
    if (errcndf == 1 || errcndv == 1 || errcndo == 1)
    {
        putsUart0("Invoking triangle fucntion failed\n");
    }
    else
    {
        uint16_t freq = atoi(frequency);
        float off = atof(offset);
//uint32_t gop;
        float volt = atof(voltage);
        phase = freq * pow(2, 32) / 100000;
        uint16_t i = 0;
        mode = 0;

//{
        if (volt >= -5 && volt <= 5)
        {
            if (off >= -5 && off <= 5)
            {
                putsUart0("triangle function is invoked");
                putsUart0("\r\n");
                for (i = 0; i < 1024; i++)
                {

                    table1[i] = 0x3000 - 380 * off + 1998
                            - (380 * ((volt * i / 1024)));

                }
                for (i = 1024; i < 3072; i++)
                {

                    table1[i] = 0x3000 - 380 * off + 1998
                            - (380 * ((volt * (2048 - i) / 1024)));

                }
                for (i = 3072; i < 4096; i++)
                {

                    table1[i] = 0x3000 - 380 * off + 1998
                            + (380 * ((volt * (4096 - i) / 1024)));

                }
            }
            else
            {
                putsUart0("invalid offset limit");
                putsUart0("\r\n");
            }

//Timer1Isr();
// TIMER1_CTL_R |= TIMER_CTL_TAEN;
        }
// turn-on timer
        else
        {
            putsUart0("invalid amplitude limit");
            putsUart0("\r\n");
        }
    }
}

//----------------------------------------------------------------------------------------------------------------
void output_square(char* frequency, char* voltage, char* offset)
{
    if (errcndf == 1 || errcndv == 1 || errcndo == 1)
    {
        putsUart0("Invoking square fucntion failed\n");
    }
    else
    {
        uint16_t freq = atoi(frequency);
        float off = atof(offset);
//uint32_t gop;
        float volt = atof(voltage);
        phase = freq * pow(2, 32) / 100000;
        uint16_t i = 0;
        mode = 0;
//{
        if (volt >= -5 && volt <= 5)
        {
            if (off >= -5 && off <= 5)
            {
                putsUart0("square function is invoked");
                putsUart0("\r\n");
                for (i = 0; i < 2048; i++)
                {

                    table1[i] = 0x3000 - 380 * off + 1998 + (370 * volt * 1);

                }
                for (i = 2048; i < 4096; i++)
                {

                    table1[i] = 0x3000 - 380 * off + 1998 + (370 * volt * (-1));

                }
            }
            else
            {
                putsUart0("invalid offset limit");
                putsUart0("\r\n");
            }
//Timer1Isr();
// TIMER1_CTL_R |= TIMER_CTL_TAEN;
        }
// turn-on timer
        else
        {
            putsUart0(" invalid amplitude limit");
            putsUart0("\r\n");
        }
    }
}

//----------------------------------------------------------------------------------------------------------------
void output_squareduty(char* frequency, char* voltage, char* offset, char* duty)
{
    if (errcndf == 1 || errcndv == 1 || errcndo == 1 || errcndd == 1)
    {
        putsUart0("Invoking squareduty fucntion failed\n");
    }
    else
    {
        uint16_t freq = atoi(frequency);
        float off = atof(offset);
        float dut = atof(duty);
        float volt = atof(voltage);
        phase = freq * pow(2, 32) / 100000;
        uint16_t i = 0;
        mode = 0;
        float duty;
        duty = 4096 - (4096 * dut / 100);
        /*for(i=0;i<4096;i++)
         {
         table1[i]=0;
         }*/
        memset(&table1[0], 0, sizeof(table1));
//{
        if (volt >= -5 && volt <= 5)
        {
            if (off >= -5 && off <= 5)
            {
                if (dut >= 0 && dut <= 100)
                {
                    putsUart0("square duty function is invoked");
                    putsUart0("\r\n");
                    for (i = 0; i < duty; i++)
                    {

                        table1[i] = 0x3000 - 380 * off + 1998
                                + (377 * volt * 1);

                    }
                    for (i = duty; i < 4096; i++)
                    {

                        table1[i] = 0x3000 - 380 * off + 1998
                                + (377 * volt * (-1));

                    }
                }
                else
                {
                    putsUart0("invalid duty limit");
                    putsUart0("\r\n");
                }
            }
            else
            {
                putsUart0("invalid offset limit");
                putsUart0("\r\n");
            }
//Timer1Isr();
// TIMER1_CTL_R |= TIMER_CTL_TAEN;
        }
// turn-on timer
        else
        {
            putsUart0("invalid amplitude limit");
            putsUart0("\r\n");
        }
    }
}
//Ref:stack overflow(but did not use)---------------------------------------------------------------------------------------------------------------------------------------------

char* mystrtok(char *str, char *tok)
{
    static char buffer[100];
    int pos = 0;
    char *trav;
    int i = 0;
    if (str == NULL)
    {
        return NULL;
    }
    if (pos != 0)
    {
        pos = pos + 1;
    }
    trav = str + pos;
    while (*trav)
    {
        if (*trav != *tok)
        {
            buffer[i++] = *trav++;
            pos++;
        }
        else
        {
            break;
        }
    }
    buffer[i] = '\0';
    return (&buffer[0]);
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
char ch;
uint8_t MaxChar = 81;
uint8_t count = 0;

//----------------------------------------------------------------------------------------------------------------
void errorf(char* frequency)
{
// checking frequency error
    uint16_t g, f2 = 0;
    for (g = 0; g < strlen(frequency); g++)
    {
        if (frequency[g] == '1' || frequency[g] == '2' || frequency[g] == '3'
                || frequency[g] == '4' || frequency[g] == '5'
                || frequency[g] == '6' || frequency[g] == '7'
                || frequency[g] == '8' || frequency[g] == '9'
                || frequency[g] == '0')
        {
//putcUart0(frequency[g]);
        }
        else
        {
//putcUart0(frequency[g]);
            f2++;
        }

    }
    if (f2 == 0)
    {
        putsUart0("frequency valid");
        putsUart0("\r\n");
        errcndf = 0;
    }

    else
    {
        putsUart0("frequency invalid");
        putsUart0("\r\n");
        errcndf = 1;
    }
}
//----------------------------------------------------------------------------------------------------------------
void errord(char* duty)
{
// checking frequency error
    uint16_t g, f2 = 0;
    for (g = 0; g < strlen(duty); g++)
    {
        if (duty[g] == '1' || duty[g] == '2' || duty[g] == '3' || duty[g] == '4'
                || duty[g] == '5' || duty[g] == '6' || duty[g] == '7'
                || duty[g] == '8' || duty[g] == '9' || duty[g] == '0')
        {
//putcUart0(duty[g]);
        }
        else
        {
//putcUart0(duty[g]);
            f2++;
        }

    }
    if (f2 == 0)
    {
        putsUart0("duty valid");
        putsUart0("\r\n");
        errcndd = 0;
    }

    else
    {
        putsUart0("duty invalid");
        putsUart0("\r\n");
        errcndd = 1;
    }
}

//----------------------------------------------------------------------------------------------------------------
void errorv(char* voltage)
{                              // checking voltage error
    uint16_t h, f3 = 0;
    for (h = 0; h < strlen(voltage); h++)
    {
        if (voltage[h] == '1' || voltage[h] == '2' || voltage[h] == '3'
                || voltage[h] == '4' || voltage[h] == '5' || voltage[h] == '6'
                || voltage[h] == '7' || voltage[h] == '8' || voltage[h] == '9'
                || voltage[h] == '0' || voltage[h] == '-' || voltage[h] == '.')
        {
//putcUart0(voltage[h]);
        }
        else
        {
//putcUart0(voltage[h]);
            f3++;
        }

    }
    if (f3 == 0)
    {
        putsUart0("voltage valid");
        putsUart0("\r\n");
        errcndv = 0;
    }

    else
    {
        putsUart0("voltage invalid");
        putsUart0("\r\n");
        errcndv = 1;
    }
}
//----------------------------------------------------------------------------------------------------------------
void erroro(char* offset)
{
// checking offset error
    uint16_t p = 0, f4 = 0;
    for (p = 0; p < strlen(offset); p++)
    {
        if (offset[p] == '1' || offset[p] == '2' || offset[p] == '3'
                || offset[p] == '4' || offset[p] == '5' || offset[p] == '6'
                || offset[p] == '7' || offset[p] == '8' || offset[p] == '9'
                || offset[p] == '0' || offset[p] == '-' || offset[p] == '.')
        {
//putcUart0(voltage[h]);
        }
        else
        {
//putcUart0(voltage[h]);
            f4++;
        }

    }
    if (f4 == 0)
    {
        putsUart0("offset valid");
        putsUart0("\r\n");
        errcndo = 0;
    }

    else
    {
        putsUart0("offset invalid");
        putsUart0("\r\n");
        errcndo = 1;
    }

}
//----------------------------------------------------------------------------------------------------------------
void errordc(char* frequency)
{
    uint16_t g = 0, f2 = 0;
    for (g = 0; g < strlen(frequency); g++)
    {
        if (frequency[g] == '.' || frequency[g] == '1' || frequency[g] == '2'
                || frequency[g] == '3' || frequency[g] == '4'
                || frequency[g] == '5' || frequency[g] == '6'
                || frequency[g] == '7' || frequency[g] == '8'
                || frequency[g] == '9' || frequency[g] == '0'
                || frequency[g] == '-')
        {
//putcUart0(frequency[g]);
        }
        else
        {
//putcUart0(frequency[g]);
            f2++;
        }

    }
    if (f2 == 0)
    {
        putsUart0("dc voltage valid");
        putsUart0("\r\n");
    }

    else
    {
        putsUart0("dc voltage invalid");
        putsUart0("\r\n");
    }
}
//----------------------------------------------------------------------------------------------------------------
void errorsweep(char* voltage)
{                              // checking voltage error
    uint16_t h, f3 = 0;
    for (h = 0; h < strlen(voltage); h++)
    {
        if (voltage[h] == '1' || voltage[h] == '2' || voltage[h] == '3'
                || voltage[h] == '4' || voltage[h] == '5' || voltage[h] == '6'
                || voltage[h] == '7' || voltage[h] == '8' || voltage[h] == '9'
                || voltage[h] == '0')
        {
//putcUart0(voltage[h]);
        }
        else
        {
//putcUart0(voltage[h]);
            f3++;
        }

    }
    if (f3 == 0)
    {
        putsUart0("frequency2 valid");
        putsUart0("\r\n");
        errcndf2 = 0;
    }

    else
    {
        putsUart0("frequency2 invalid");
        putsUart0("\r\n");
        errcndf2 = 1;
    }
}
//----------------------------------------------------------------------------------------------------------------
int main(void)
{

// Initialize hardware
    initHw();

    GREEN_LED ^= 1;
    waitMicrosecond(1000000);
    GREEN_LED = 0;
// Step 5 code and discard---------------------------------------------------------------------------------------------
//  while(1){
//   uint32_t r=0x27FF;
// sendGraphicsLcdData(r);
//}
    while (1)
    {
//STEP2-----------------------------------------------------------------------------------------------------------------------------------------
        putsUart0(
                "\r\n--------------------------------------------------------------------------------");
        putsUart0("\r\n YETISH KRISHNA REDDY\r");
        putsUart0("\r\n UTA ID: 1001574820 \r");
        putsUart0(
                "\r\n Direct Digital Synthesis Signal Generator and Scalar Network Analyzer\r\n");
        putsUart0("\r\n Command Help:");
        putsUart0("\r\n 1:Sine FREQ, AMP, [OFS]");
        putsUart0("\r\n 2:Square FREQ, AMP, [OFS]");
        putsUart0("\r\n 3:Sawtooth FREQ, AMP, [OFS]");
        putsUart0("\r\n 4:Triangle FREQ, AMP, [OFS]");
        putsUart0("\r\n 5:DC AMP");
        putsUart0("\r\n 6:Sqduty FREQ, AMP, [OFS], [DC]");
        putsUart0("\r\n 7:voltage (supports dc and ac )");
        putsUart0("\r\n 8:Sweep FREQ1, FREQ2");
        putsUart0("\r\n 9:Gain");
        putsUart0("\r\n 10:Reset");

        putsUart0(
                "\r\n--------------------------------------------------------------------------------\r\n");
        putsUart0("Enter characters for string\r\n");

        while (1)
        {

            //count =0;
            count = 0;
//if (count==0){
            Jump: ch = getcUart0();
            ch = tolower(ch);
//}
            if (ch == 8)
            {
                if (count > 0)
                {
                    count--;
                    goto Jump;
                }
                else
                {
                    goto Jump;
                }
            }
            if ((ch == 13) || (ch == 20))
            {
                str[count] = 0;
                break;
            }
            else
            {
                if (ch < ' ')
                {
                    goto Jump;
                }
                else
                {
                    str[count++] = ch;
                    if ((count > MaxChar) || (count == MaxChar))
                    {
                        str[count] = '\0';

                    }
                    else
                    {
                        goto Jump;
                    }
                }
            }
        }
        putsUart0("\r\nEntered string is:\r\n");
        putsUart0(str);
        if (*str == '\0')
        {
            *str = '0';
            //putsUart0(str);
        }
        putsUart0("\r\n");
//----------------------------------------------------------------------------------------------------------------
        /*                       int v;
         int count3 = 0;
         for (v = 0; v < strlen(str); v++) {

         if (str[v] != ' ' && str[v] != '\0'
         && str[v] != ',' && str[v] != ';'
         && str[v] != ':' && str[v] != '!'
         && str[v] != '@' && str[v]!= '-'
         && str[v] != '#' && str[v]!= '$'
         && str[v] != '*' && str[v]!= '_'
         && str[v] != '&' && str[v]!= '^'){
         str2[count3++] = str[v];
         }
         }
         putsUart0("Input string without Space is \r\n");
         putsUart0(str2);
         putsUart0("\r\n");
         */
//STEP3------------------------------------------------------------------------------------------------
        char str1[5][10];
        uint16_t count = 0;
        uint16_t count1 = 0;
        uint16_t i = 0;
        for (i = 0; i < strlen(str) - 1; i++)
        {
            if ((str[i] != ' ' && str[i] != ',' && str[i] != ';'
                    && str[i] != ':' && str[i] != '!' && str[i] != '_'
                    && str[i] != '#' && str[i] != '@' && str[i] != '$'
                    && str[i] != '*' && str[i] != '/' && str[i] != ','
                    && str[i] != '&' && str[i] != '^')
                    && (str[i + 1] == ' ' || str[i + 1] == ','
                            || str[i + 1] == ';' || str[i + 1] == ':'
                            || str[i + 1] == '!' || str[i + 1] == '_'
                            || str[i + 1] == '#' || str[i + 1] == '@'
                            || str[i + 1] == '$' || str[i + 1] == '*'
                            || str[i + 1] == '/' || str[i + 1] == ','
                            || str[i + 1] == '&' || str[i + 1] == '^'))
            {
                str1[count][count1] = str[i];
                str1[count][++count1] = '\0';
                count++;
                count1 = 0;
            }
            else if (str[i] != ' ' && str[i] != ',' && str[i] != ';'
                    && str[i] != ':' && str[i] != '!' && str[i] != '_'
                    && str[i] != '#' && str[i] != '@' && str[i] != '$'
                    && str[i] != '*' && str[i] != '/' && str[i] != ','
                    && str[i] != '&' && str[i] != '^')
            {
                str1[count][count1] = str[i];
                count1++;
            }
            if (i == strlen(str) - 2)
            {
                if (((str[i] == ' ' || str[i] == ',' || str[i] == ';'
                        || str[i] == ':' || str[i] == '!' || str[i] == '_'
                        || str[i] == '#' || str[i] == '@' || str[i] == '$'
                        || str[i] == '*' || str[i] == '/' || str[i] == ','
                        || str[i] == '&' || str[i] == '^')
                        && (str[i + 1] != ' ' && str[i + 1] != ','
                                && str[i + 1] != ';' && str[i + 1] != ':'
                                && str[i + 1] != '!' && str[i + 1] != '_'
                                && str[i + 1] != '#' && str[i + 1] != '@'
                                && str[i + 1] != '$' && str[i + 1] != '*'
                                && str[i + 1] != '/' && str[i + 1] != ','
                                && str[i + 1] != '&' && str[i + 1] != '^')))
                {
                    str1[count++][count1] = str[i + 1];
                    str1[count][++count1] = '\0';
                }
                else if ((str[i] != ' ' && str[i] != ',' && str[i] != ';'
                        && str[i] != ':' && str[i] != '!' && str[i] != '_'
                        && str[i] != '#' && str[i] != '@' && str[i] != '$'
                        && str[i] != '*' && str[i] != '/' && str[i] != ','
                        && str[i] != '&' && str[i] != '^')
                        && (str[i + 1] != ' ' && str[i + 1] != ','
                                && str[i + 1] != ';' && str[i + 1] != ':'
                                && str[i + 1] != '!' && str[i + 1] != '_'
                                && str[i + 1] != '#' && str[i + 1] != '@'
                                && str[i + 1] != '$' && str[i + 1] != '*'
                                && str[i + 1] != '/' && str[i + 1] != ','
                                && str[i + 1] != '&' && str[i + 1] != '^'))
                {
                    str1[count][count1] = str[i + 1];
                    str1[count++][++count1] = '\0';
                }
            }
        }
//for(i=0;i<count;i++)
//printf("%s****\n",str1[i]);
        char* signaltype = str1[0];
        char* frequency = str1[1];
        char* voltage = str1[2];
        char* offset = str1[3];
        char* duty = str1[4];
//when nothing is entered---------------------------------------------------------------------------------------------------------------
        uint8_t os = 0, sgtp = 0, fcy = 0, dty = 0, vtg = 0;
        os = strcmp(*offset, '\0');
        if (os == 0)
        {
            *offset = '0';
        }
        sgtp = strcmp(*signaltype, '\0');
        if (sgtp == 0)
        {
            *signaltype = '0';
        }
        fcy = strcmp(*frequency, '\0');
        if (fcy == 0)
        {
            *frequency = '0';
        }
        vtg = strcmp(*voltage, '\0');
        if (vtg == 0)
        {
            *voltage = '0';
        }
        dty = strcmp(*duty, '\0');
        if (dty == 0)
        {
            *duty = '0';
        }

//checking string error--------------------------------------------------------------------------------------------

        uint16_t f, f1 = 0;
        for (f = 0; f < strlen(signaltype); f++)
        {
            if (signaltype[f] < 48 || signaltype[f] > 57)
            {
//putcUart0(signaltype[f]);
            }
            else
            {
                f1++;

            }
        }
        if (f1 == 0)
        {
            putsUart0("possible valid command");
            putsUart0("\r\n");
        }

        else
        {
            putsUart0("invalid input");
            putsUart0("\r\n");
        }

//sine operation--------------------------------------------------------------------------------------------------------------------------------
        char test[] = "sine";
        int8_t ret = 0;
        ret = strcmp(signaltype, test);
        if (ret == 0)
        {
            sweepaccess = 1;
            checkdc = 0;
            inpamp = atof(voltage);

            //printf("%s\n", voltage);
            putsUart0("the voltage is=");
            putsUart0(voltage);
            putsUart0("\r\n");
            //printf("%d\n", frequency);
            putsUart0("the frequency is=");
            putsUart0(frequency);
            putsUart0("\r\n");
            //printf("%d\n", offset);
            putsUart0("the offset is=");
            putsUart0(offset);
            putsUart0("\r\n");
            //printf("%d\n", signaltype);
            putsUart0("the signal type is=");
            putsUart0(signaltype);
            putsUart0("\r\n");
            errorf(frequency);
            errorv(voltage);
            erroro(offset);
            output_sinewave(frequency, voltage, offset);

        }

// DC operation-------------------------------------------------------------------------------------------------------------------------------
        char test1[] = "dc";
        int8_t ret1 = 0;
        ret1 = strcmp(signaltype, test1);
        if (ret1 == 0)
        {
            checkdc = 1;
            errordc(frequency);
            output_dc(frequency);
        }

//sawtooth operation---------------------------------------------------------------------------------------------------------------------------
        char test2[] = "sawtooth";
        int8_t ret2 = 0;
        ret2 = strcmp(signaltype, test2);
        if (ret2 == 0)
        {
            sweepaccess = 0;

            //printf("%s\n", voltage);
            putsUart0("the voltage is=");
            putsUart0(voltage);
            putsUart0("\r\n");
            //printf("%d\n", frequency);
            putsUart0("the frequency is=");
            putsUart0(frequency);
            putsUart0("\r\n");
            //printf("%d\n", offset);
            putsUart0("the offset is=");
            putsUart0(offset);
            putsUart0("\r\n");
            //printf("%d\n", signaltype);
            putsUart0("the signal type is=");
            putsUart0(signaltype);
            putsUart0("\r\n");
            errorf(frequency);
            errorv(voltage);
            erroro(offset);
            output_sawtooth(frequency, voltage, offset);
        }

//triangle operation-------------------------------------------------------------------------------------------------------------------------------
        char test3[] = "triangle";
        int8_t ret3 = 0;
        ret3 = strcmp(signaltype, test3);
        if (ret3 == 0)
        {
            sweepaccess = 0;

            //printf("%s\n", voltage);
            putsUart0("the voltage is=");
            putsUart0(voltage);
            putsUart0("\r\n");
            //printf("%d\n", frequency);
            putsUart0("the frequency is=");
            putsUart0(frequency);
            putsUart0("\r\n");
            //printf("%d\n", offset);
            putsUart0("the offset is=");
            putsUart0(offset);
            putsUart0("\r\n");
            //printf("%d\n", signaltype);
            putsUart0("the signal type is=");
            putsUart0(signaltype);
            putsUart0("\r\n");
            errorf(frequency);
            errorv(voltage);
            erroro(offset);

            output_triangle(frequency, voltage, offset);
        }

//square operation-----------------------------------------------------------------------------------------------------------------------------
        char test4[] = "square";
        int8_t ret4 = 0;
        ret4 = strcmp(signaltype, test4);
        if (ret4 == 0)
        {
            sweepaccess = 0;

            //printf("%s\n", voltage);
            putsUart0("the voltage is=");
            putsUart0(voltage);
            putsUart0("\r\n");
            //printf("%d\n", frequency);
            putsUart0("the frequency is=");
            putsUart0(frequency);
            putsUart0("\r\n");
            //printf("%d\n", offset);
            putsUart0("the offset is=");
            putsUart0(offset);
            putsUart0("\r\n");
            //printf("%d\n", signaltype);
            putsUart0("the signal type is=");
            putsUart0(signaltype);
            putsUart0("\r\n");
            errorf(frequency);
            errorv(voltage);
            erroro(offset);
            output_square(frequency, voltage, offset);
        }

        //square duty operation-----------------------------------------------------------------------------------------------------------------------------
        char test10[] = "sqduty";
        int8_t ret10 = 0;
        ret10 = strcmp(signaltype, test10);
        if (ret10 == 0)
        {
            sweepaccess = 0;

            //printf("%s\n", voltage);
            putsUart0("the voltage is=");
            putsUart0(voltage);
            putsUart0("\r\n");
            //printf("%d\n", frequency);
            putsUart0("the frequency is=");
            putsUart0(frequency);
            putsUart0("\r\n");
            //printf("%d\n", offset);
            putsUart0("the offset is=");
            putsUart0(offset);
            putsUart0("\r\n");
            putsUart0("the duty is=");
            putsUart0(duty);
            putsUart0("\r\n");
            //printf("%d\n", signaltype);
            putsUart0("the signal type is=");
            putsUart0(signaltype);
            putsUart0("\r\n");
            errorf(frequency);
            errorv(voltage);
            erroro(offset);
            errord(duty);
            output_squareduty(frequency, voltage, offset, duty);
        }
//reset operation---------------------------------------------------------------------------------------------------------------------------
        char test5[] = "reset";
        int8_t ret5 = 0;
        ret5 = strcmp(signaltype, test5);
        if (ret5 == 0)
        {
            putsUart0("reset function is invoked");
            putsUart0("\r\n");
            reset();
        }
        //gain operation---------------------------------------------------------------------------------------------------------------------------
        char test9[] = "gain";
        int8_t ret9 = 0;
        ret9 = strcmp(signaltype, test9);
        if (ret9 == 0 && gainaccess == 1)
        {
            putsUart0("gain function is invoked");
            putsUart0("\r\n");
            gain();
        }
        else if (ret9 == 0 && gainaccess == 0)
            putsUart0("gain cannot be calculated without computing for sweep");
        putsUart0("\r\n");
//sweep-------------------------------------------------------------------------------------------------------------------------------------------
        char test6[] = "sweep";
        int8_t ret6 = 0;
        ret6 = strcmp(signaltype, test6);
        if (ret6 == 0 && checkdc == 0 && sweepaccess == 1)
        {
            gainaccess = 1;
            storecnt = 0;
            putsUart0("sweep function is invoked");
            putsUart0("\r\n");
            //printf("%s\n", voltage);
            putsUart0("the frequency 2 is=");
            putsUart0(voltage);
            putsUart0("\r\n");
            //printf("%d\n", frequency);
            putsUart0("the frequency 1 is=");
            putsUart0(frequency);
            putsUart0("\r\n");
            //printf("%d\n", signaltype);
            putsUart0("the signal type is=");
            putsUart0(signaltype);
            putsUart0("\r\n");
            errorf(frequency);
            errorsweep(voltage);
            //erroro(offset);
            uint16_t new_freq = 0;
            uint32_t q = 0;
            freq1 = atoi(frequency);
            freq2 = atoi(voltage);
            uint32_t step = 0;
            step = ((freq2 - freq1) / 10);
            char str4[81];
            if (freq1 < freq2)
            {
                new_freq = freq1;
                int8_t countstep = 0;
                sprintf(str4, "Frequency = %d  ", new_freq);
                putsUart0(str4);
                storefreq[countstep++] = new_freq;
                voltage_test();
                for (q = 0; q < (freq2 - freq1); q += step)
                {
                    new_freq = new_freq + step;
                    phase = new_freq * pow(2, 32) / 100000;
                    waitMicrosecond(300000);
                    sprintf(str4, "Frequency = %d  ", new_freq);
                    putsUart0(str4);
                    //TIMER1_CTL_R |= TIMER_CTL_TAEN;
                    voltage_test();
                    //sprintf(str4, "Frequency = %d  ", new_freq);
                    //putsUart0(str4);
                    //gain();
                    storefreq[countstep++] = new_freq;

                }
                if (new_freq < 1000)
                {
                    output_slow();
                    waitMicrosecond(300000);
                    sprintf(str4, "Frequency = %d  ", new_freq);
                    putsUart0(str4);
                    voltage_test();
                    storefreq[countstep++] = new_freq;
                }
                if (new_freq > 1000)
                {
                    output_fast();
                    waitMicrosecond(100000);
                    sprintf(str4, "Frequency(outputfast) = %d  ", new_freq);
                    putsUart0(str4);
                    voltage_test();
                    storefreq[countstep++] = new_freq;
                }
            }
            if (freq1 > freq2)
            {

                putsUart0(" error \r \n");

            }

        }
        else if (ret6 == 0 && checkdc == 0 && sweepaccess == 0)
        {
            putsUart0("Sweep cannot be computed without generating sine wave");
            putsUart0("\r\n");
        }
        else if (ret6 == 0 && checkdc == 1)
        {
            putsUart0("Cannot find sweep for a dc signal");
            putsUart0("\r\n");
            checkdc = 0;
            gainaccess = 0;
        }

//dcvoltagetest operation---------------------------------------------------------------------------------------------------------------------------
        char test7[] = "voltage";
        int8_t ret7 = 0;
        ret7 = strcmp(signaltype, test7);
        if (ret7 == 0)
        {
            putsUart0("voltage function is invoked");
            putsUart0("\r\n");
            voltage_test();
        }

//dcsinetest operation---------------------------------------------------------------------------------------------------------------------------
        char test8[] = "sinetest";
        int8_t ret8 = 0;
        ret8 = strcmp(signaltype, test8);
        if (ret8 == 0)
        {
            putsUart0("sineTest function is invoked");
            putsUart0("\r\n");
            sineTest();
        }

//error condition-----------------------------------------------------------------------------------------------
        if ((ret && ret1 && ret2 && ret3 && ret4 && ret5 && ret6 && ret7 && ret8
                && ret9 && ret10) != 0)
        {
            putsUart0("function is not found or possible error");
            putsUart0("\r\n");
        }

//Clearing the strings---------------------------------------------------------------------------------------------------------------
        uint32_t x, z;
        uint32_t y;
        for (x = 0; x < count; x++)
        {
            for (y = 0; y < 20; y++)
            {
                str1[x][y] = '\0';
            }
        }
        for (z = 0; z < count; z++)
        {
            str[z] = '\0';
        }
//------------------------------------------------------------------------------------------------------------------

    }
}

