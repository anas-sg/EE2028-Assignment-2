/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/
#include "lpc17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_rit.h"
#include <stdio.h>
#include <stdlib.h>

#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "temp.h"
#include "light.h"
#include "uart2.h"

#define LIGHT_LOW_WARNING 50
#define TEMP_HIGH_WARNING 306
#define NUM_HALF_PERIODS 340
#define MOVEMENT 5

volatile uint32_t msTicks = 0, value = 0, temp_count = 0, t1, t_temp;
volatile uint32_t n = 0, fire = 0, dark = 0, light, temp;
volatile int8_t x, y, z, monitor = 0, blink = 0;
int32_t xoff, yoff, zoff;
char num[] = "0123456789ABCDEF";
char x_str[10], y_str[10], z_str[10], temp_str[7], light_str[7], msg_str[45];

static void init_ssp(void) {
    SSP_CFG_Type SSP_ConfigStruct;
    PINSEL_CFG_Type PinCfg;

    /*
     * Initialize SPI pin connect
     * P0.7 - SCK;
     * P0.8 - MISO
     * P0.9 - MOSI
     * P2.2 - SSEL - used as GPIO
     */
    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 7;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 8;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 9;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Funcnum = 0;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 2;
    PINSEL_ConfigPin(&PinCfg);

    SSP_ConfigStructInit(&SSP_ConfigStruct);

    // Initialize SSP peripheral with parameter given in structure above
    SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

    // Enable SSP peripheral
    SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void) {
    PINSEL_CFG_Type PinCfg;

    /* Initialize I2C2 pin connect */
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11;
    PINSEL_ConfigPin(&PinCfg);

    // Initialize I2C peripheral
    I2C_Init(LPC_I2C2, 100000);

    /* Enable I2C1 operation */
    I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void) {
    /*Initialize button SW3 EINT0*/
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PINSEL_ConfigPin(&PinCfg);
    /*Mask red & blue LEDs*/
    FIO_SetMask(2, 1, 1);
    FIO_SetMask(0, 1<<26, 1);
}

static void init_uart(void) {
    PINSEL_CFG_Type PinCfg;
    UART_CFG_Type uartCfg;

    /* Initialize UART3 pin connect */
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);

    uartCfg.Baud_rate = 115200;
    uartCfg.Databits = UART_DATABIT_8;
    uartCfg.Parity = UART_PARITY_NONE;
    uartCfg.Stopbits = UART_STOPBIT_1;

    UART_Init(LPC_UART3, &uartCfg);
    UART_TxCmd(LPC_UART3, ENABLE);
}

static void init_rit(void) {
    LPC_SC->PCONP |= 1<<16;                 // Power up RIT because default off pg 63
    LPC_SC->PCLKSEL1 |= 1<<26;              // RIT clock = CCLK = 100 MHz (pg 57)
    LPC_RIT->RICOUNTER = 0;                 // Set counter = 0
    LPC_RIT->RICOMPVAL = SystemCoreClock/2; // Max count = 100M/2 = 50M
    LPC_RIT->RICTRL |= 1<<1;                // Clear timer when counter reaches value
    LPC_RIT->RICTRL |= 1<<3;                // Enable timer
}

static void init_timer1(void) {
    LPC_SC->PCLKSEL0 |= 1<<4;               // Clock for timer = CCLK = 100 MHz
    LPC_TIM1->MR0 = SystemCoreClock;        // Max count = 100M (T = 1s)
    LPC_TIM1->MCR |= 1<<0 | 1<<1;           // When count == MR0, reset Timer Counter and generate interrupt
    LPC_TIM1->TCR |= 1<<1;                  // Reset Timer1
}

void SysTick_Handler(void) {
    msTicks++;
}

void EINT3_IRQHandler(void) {
    // Determine if GPIO Interrupt P2.5 (shorted to P1.31 SW4) has occurred
    if ((LPC_GPIOINT->IO2IntStatF>>5) & 0x1) {
        monitor = 1;
        led7seg_setChar('0', 0);
        UART_SendString(LPC_UART3, (uint8_t*)"Entering MONITOR mode\r\n");
        oled_putString(1, 1, (uint8_t*)"    MONITOR", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        oled_putString(1, 9, (uint8_t*)"x:", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        oled_putString(1, 17, (uint8_t*)"y:", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        oled_putString(1, 25, (uint8_t*)"z:", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        oled_putString(1, 33, (uint8_t*)"temp:         C", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        oled_fillRect(79, 33, 80, 34, OLED_COLOR_WHITE);
        oled_putString(1, 41, (uint8_t*)"light:       lx", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        NVIC_EnableIRQ(EINT0_IRQn);
        NVIC_EnableIRQ(RIT_IRQn);
        NVIC_EnableIRQ(TIMER1_IRQn);
        LPC_TIM1->TCR = 1;            // Start timer 1s
        /*Enable GPIO Interrupt P0.2 (temperature sensor)*/
        LPC_GPIOINT->IO0IntEnF |= 1<<2;
        LPC_GPIOINT->IO0IntEnR |= 1<<2;
        // Clear GPIO Interrupt P2.5 (shorted to P1.31 SW4)
        LPC_GPIOINT->IO2IntClr |= 1<<5;
        /*Disable GPIO Interrupt P2.5 (shorted to P1.31 SW4)*/
        LPC_GPIOINT->IO2IntEnF &= ~(1<<5);
    }

    // if (((LPC_GPIOINT->IO0IntStatF >> 2) & 0x1) || ((LPC_GPIOINT->IO0IntStatR >> 2) & 0x1))
    else {
        if (++temp_count == NUM_HALF_PERIODS) {
            t_temp = msTicks - t_temp;
            temp = ((2*1000*t_temp) / (NUM_HALF_PERIODS) - 2731);
            temp_count = 0;
            t_temp = msTicks;
        }
        LPC_GPIOINT ->IO0IntClr |= (1<<2);
    }
}

void EINT0_IRQHandler(void) {
    uint32_t t2 = msTicks;
    if (t2 - t1 < 1000){
        UART_SendString(LPC_UART3, (uint8_t*)"Leaving MONITOR mode\r\n");
        UART_SendString(LPC_UART3, (uint8_t*)"Entering CARETAKER mode\r\n");
        value = fire = dark = monitor = 0;
        oled_clearScreen(OLED_COLOR_BLACK);
        led7seg_setChar('Z', 0);            // Turn off 7-seg
        LPC_GPIOINT->IO2IntEnF |= 1<<5;     // Enable GPIO Interrupt P2.5 (shorted to P1.31 SW4)
        /*Disable GPIO Interrupt P0.2 (temperature sensor)*/
        LPC_GPIOINT->IO0IntEnF &= ~(1<<2);
        LPC_GPIOINT->IO0IntEnR &= ~(1<<2);
        /*Disable RIT & TIMER1; off & mask LEDs*/
        NVIC_DisableIRQ(EINT0_IRQn);
        NVIC_DisableIRQ(RIT_IRQn);
        NVIC_ClearPendingIRQ(RIT_IRQn);
        NVIC_DisableIRQ(TIMER1_IRQn);
        NVIC_ClearPendingIRQ(TIMER1_IRQn);
        LPC_TIM1->TCR |= 1<<1;              // Reset Timer1 and keep it reset
        GPIO_ClearValue(2, 1);              // Turn off red LED
        GPIO_ClearValue(0, 1<<26);          // Turn off blue LED
        FIO_SetMask(2, 1, 1);               // Mask red LED pin
        FIO_SetMask(0, 1<<26, 1);           // Mask blue LED pin
    }
    t1 = t2;
    LPC_SC->EXTINT = 1<<0;                  // Clear Interrupt Flag
}

void RIT_IRQHandler(void) {
    if ((blink = !blink)) {
        GPIO_SetValue(2, 1);        // on red
        GPIO_ClearValue(0, 1<<26);  // off blue
    }
    else {
        GPIO_ClearValue(2, 1);      // off red
        GPIO_SetValue(0, 1<<26);    // on blue
    }
    LPC_RIT->RICTRL |= 1;           // Clear Interrupt Flag
    NVIC_ClearPendingIRQ(RIT_IRQn);
}

void TIMER1_IRQHandler(void) {
    if (value == 15) {
        value = 0;
    }
    else {
        value++;
    }
    led7seg_setChar(num[value], 0);
    if (value == 5 || value == 10 || value == 15) {
        acc_read(&x, &y, &z);
        x = x + xoff;
        y = y + yoff;
        z = z + zoff;
        /*Unmask LEDs if fire and/or motion in darkness occurs*/
        if (light_read() < LIGHT_LOW_WARNING && (abs(x)>=MOVEMENT || abs(y)>=MOVEMENT || abs(z)>=MOVEMENT)) {
            dark = 1;
            FIO_SetMask(0, 1<<26, 0);
        }
        if (temp > TEMP_HIGH_WARNING) {
            fire = 1;
            FIO_SetMask(2, 1, 0);
        }
        sprintf(x_str, "%4d", x);
        oled_putString(49, 9, (uint8_t*)x_str, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        sprintf(y_str, "%4d", y);
        oled_putString(49, 17, (uint8_t*)y_str, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        sprintf(z_str, "%4d", z);
        oled_putString(49, 25, (uint8_t*)z_str, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        sprintf(temp_str, "%5.1f", temp/10.0);
        oled_putString(43, 33, (uint8_t*)temp_str, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        sprintf(light_str, "%5u", light_read());
        oled_putString(43, 41, (uint8_t*)light_str, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        if (value == 15) {
            if (fire) {
                UART_SendString(LPC_UART3, (uint8_t*)"Fire detected\r\n");
            }
            if (dark) {
                UART_SendString(LPC_UART3, (uint8_t*)"Movement in darkness detected\r\n");
            }
            sprintf(msg_str, "%03u_-_T%2.1f_L%u_AX%d_AY%d_AZ%d\r\n", n++, temp/10.0, light_read(), x, y, z);
            UART_SendString(LPC_UART3, (uint8_t*)msg_str);
        }
    }
    LPC_TIM1->IR |= 1;
}
int main(void) {
    init_i2c();
    init_uart();
    init_ssp();
    init_GPIO();

    led7seg_init();
    led7seg_setChar('z', 0);
    acc_init();
    oled_init();
    light_enable();
    light_setRange(LIGHT_RANGE_1000);
    rgb_init();
    init_rit();
    init_timer1();

    /* Assume base board in zero-g position when reading first value.*/
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 0-z;
    oled_clearScreen(OLED_COLOR_BLACK);

    /*Enable EINT0 Interrupt P2.10 (SW3)*/
    LPC_SC->EXTMODE = 1<<0;     //EINT0 is edge sensitive (falling edge)
    LPC_SC->EXTINT = 1;         //Clear EINT0

    /*Enable GPIO Interrupt P2.5 (shorted to P1.31)*/
    LPC_GPIOINT->IO2IntEnF |= 1<<5;

    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);
    NVIC_SetPriority(EINT0_IRQn, 0);
    NVIC_SetPriority(TIMER1_IRQn, 1);
    NVIC_SetPriority(RIT_IRQn, 2);
    NVIC_SetPriority(EINT3_IRQn, 3);
    NVIC_SetPriority(SysTick_IRQn, 4);
    SysTick_Config(SystemCoreClock/1000);
    t1 = t_temp = msTicks;
    UART_SendString(LPC_UART3, (uint8_t*)"Entering CARETAKER mode\r\n");
}

void check_failed(uint8_t *file, uint32_t line) {
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while(1);
}
