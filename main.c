#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"



uint8_t receivedByte;
bool dataReceivedFlag = true;

void UART5_send(void);
void UART5_Transmit(uint8_t data);

void init_portf(void){

    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;   // Enable clock for Port F
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;      // Unlock Port F
    GPIO_PORTF_CR_R = 0x1f;                 // Commit changes,1-enable (PF7-PF0 = 00011111)
    GPIO_PORTF_DEN_R = 0x1f;                // Digital function enable, 1-enable (PF7-PF0 = 00011111)
    GPIO_PORTF_DIR_R = 0x0e;                // Set output/input, 1-output (PF7-PF0 = 00001110)
    GPIO_PORTF_PUR_R = 0x11;                // Enable pull-up resistor, 1-enable (PF7-PF0 = 00010001)
    GPIO_PORTF_DATA_R = 0x00;               // Reset the data register (PF7-PF0 = 00000000)

}

void init_porte(void) {


    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOE;     // Enable GPIO Port E clock
    SYSCTL_RCGCUART_R |= (1<<5);     // Enable UART5 clock


    GPIO_PORTE_LOCK_R = GPIO_LOCK_KEY;      // Unlock Port E
    GPIO_PORTE_CR_R = 0xff;                 // Commit changes,1-enable (PE6-PD0 = 11000000)
    GPIO_PORTE_DEN_R = 0x30;                // Digital enable PD6 and PE6
    GPIO_PORTE_AFSEL_R = 0x30;              // Enable alternate function for PD6 and PE6
    GPIO_PORTE_AMSEL_R = 0x00;              // Turnoff analog function
    GPIO_PORTE_PCTL_R &= ~0x00FF0000;
    GPIO_PORTE_PCTL_R |= 0x00110000;
}

void init_uart5(void) {


    UART5_CTL_R = 0x00;                     // Disable UART before configuration
    UART5_IBRD_R = 104;                     // Integer part of BRD = 16MHz / (16 * 9600) = 104
    UART5_FBRD_R = 11;                      // Fractional part of BRD = 0.16 * 64 + 0.5 = 11
    UART5_CC_R = 0x00;
    UART5_LCRH_R = 0x72;                    // 8 bits, odd parity
    UART5_CTL_R = 0x301;                    // Enable UART

}

uint8_t UART5_ReceiveByte(void) {

    while ((UART5_FR_R & 0x10) != 0) // Wait until RXFE is 0
    {
        UART5_send();
    }
    return UART5_DR_R; // Read data

}

void UART5_Read(void){

    receivedByte =  UART5_ReceiveByte();


        if (receivedByte == 0xAA)
        {
            GPIO_PORTF_DATA_R |= 0x08;  // Turn on green LED
            GPIO_PORTF_DATA_R &= ~0x04;   // Turn off blue LED
            GPIO_PORTF_DATA_R &= ~0x02;     // Turn off red LED (error

            }
        else if (receivedByte == 0xF0)
        {
            GPIO_PORTF_DATA_R &= ~0x08;  // Turn off green LED
             GPIO_PORTF_DATA_R |= 0x04;   // Turn on blue LED
             GPIO_PORTF_DATA_R &= ~0x02;     // Turn off red LED (error

        }
        else
        {
            GPIO_PORTF_DATA_R &= ~0x08;  // Turn off green LED
            GPIO_PORTF_DATA_R &= ~0x04;   // Turn off blue LED
            GPIO_PORTF_DATA_R |= 0x02;     // Turn on red LED (error)

        }

}

void UART5_send(void){

    if (!(GPIO_PORTF_DATA_R & 0X01)) {
        UART5_Transmit(0xAA);
        while (!(GPIO_PORTF_DATA_R & 0X01));
    }
    if (!(GPIO_PORTF_DATA_R & 0X10)) {
        UART5_Transmit(0xF0);
        while (!(GPIO_PORTF_DATA_R & 0X10));
    }
}

void UART5_Transmit(uint8_t data) {
    while (UART5_FR_R & UART_FR_TXFF);
    UART5_DR_R = data;
}

int main(void) {
    init_portf();
    init_porte();
    init_uart5();


    while (1) {

        UART5_Read();
    }
}
