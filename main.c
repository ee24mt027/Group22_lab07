#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

void uart_init();
void uart_send(uint8_t data);
void GPIO_PORT_F_init(void);


uint8_t uart_receive();

void GPIO_PORT_F_init(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;            // ENABLE CLOCK TO GPIOF
    GPIO_PORTF_LOCK_R = 0x4C4F434B;          // UNLOCK COMMIT REGISTER
    GPIO_PORTF_CR_R   = 0x1F;                // MAKE PORTF0 CONFIGURABLE
    GPIO_PORTF_DEN_R  = 0x1F;                // SET PORTF DIGITAL ENABLE
    GPIO_PORTF_DIR_R  = 0x0E;                // SET PF0, PF4 as input and PF1, PF2 and PF3 as output
    GPIO_PORTF_PUR_R  = 0x11;                // PORTF PF0 and PF4 IS PULLED UP

    NVIC_EN0_R |= 1 << 30;
    GPIO_PORTF_IS_R  = 0x00;                 // INTERRUPT SENSE EDGE SENSITIVE
    GPIO_PORTF_IBE_R = 0x00;                 // INTERRUPT GENERATION IS CONTROLLED BY THE GPIO INTERRUPT EVENT
    GPIO_PORTF_IEV_R = 0x00;                 // INTERRUPT EVENT FALLING EDGE
    GPIO_PORTF_IM_R  |= 0x11;                // INTERRUPT MASK ENABLE FOR SWITCH1 AND SWITCH2
}

void uart_init()
{

    SYSCTL_RCGCUART_R |= (1<<5);
    SYSCTL_RCGCGPIO_R |= (1<<4);

    GPIO_PORTE_LOCK_R = 0x4C4F434B;
    GPIO_PORTE_CR_R = 0xFF;
    GPIO_PORTE_AFSEL_R |= (1<<4) | (1<<5);
    GPIO_PORTE_PCTL_R &= ~0x00FF0000;
    GPIO_PORTE_PCTL_R |= 0x00110000;
    GPIO_PORTE_DEN_R |= (1<<4) | (1<<5);
    GPIO_PORTE_DIR_R |= (1<<5);
    GPIO_PORTE_DIR_R &= ~(1<<4);


    UART5_CTL_R &= ~(1<<0);
    UART5_IBRD_R = 104;
    UART5_FBRD_R = 11;
    UART5_LCRH_R = 0x60;
    UART5_CC_R = 0x0;
    UART5_CTL_R |= (1<<7);
    UART5_CTL_R |= 0x301;
}

void uart_send(uint8_t data)
{
    while((UART5_FR_R & 0x20) != 0);
    UART5_DR_R = data;
}

uint8_t uart_receive()
{
    while((UART5_FR_R & 0x10) != 0);
    return (uint8_t)(UART5_DR_R & 0xFF);
}

int main(void)
{

    GPIO_PORT_F_init();
    uart_init();

    while(1)
    {



    }
}


void Portf_interrupt_handler(void)            // Interrupt handler for GPIO Port F
{
    uint8_t received_data;
      int i;


    if (GPIO_PORTF_RIS_R & 0x10)             // CHECK IF SWITCH 1 CAUSED INTERRUPT
    {
    uart_send(0xAA);
    received_data = uart_receive();
    GPIO_PORTF_ICR_R = 0x10;                 // CLEAR INTERRUPT FLAG FOR SWITCH 1


    }

    if (GPIO_PORTF_RIS_R & 0x01)             // CHECK IF SWITCH 2 CAUSED INTERRUPT
    {
    uart_send(0x12);
    received_data = uart_receive();
    //GPIO_PORTF_DATA_R |= (1<<3) | (0<<1);               // TOGGLE RED LED
    GPIO_PORTF_ICR_R = 0x01;                 // CLEAR INTERRUPT FLAG FOR SWITCH 1
    }
    for(i = 0; i < 1000000; i++);
}
