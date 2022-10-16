/*
 * SIN WAVE CONTROL FOR BRUSHLESS SERVO
 * Copyright (C) 2022 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */

// The PWM input must be soldered to the UART pin P0.5.  Then, the
// PWM lowpass filtering capacitor must be removed.

// Write a 4 byte packet to control the MOSFETs.
// byte 0: 0xff
// byte 1: 0x88
// byte 2: power 0 - somewhat less than 255
//      Can't be full scale because time is needed for the interrupt handlers
//      If the motor shorts for certain phases, POWER needs to be smaller.
// byte 3: phase code
//      0 - SIN_TOTAL or 
//      COAST_MOTOR to float the MOSFETS
//      GROUND_MOTOR to connect the MOSFETS to ground


#include <c8051f330.h>                    // SFR declarations
#include <stdint.h>

#include "motor.h"

#define BAUD 115200

// mosfet pins
sbit PFET0 = P1^5;
sbit PFET1 = P1^4;
sbit PFET2 = P1^6;
sbit NFET0 = P1^1;
sbit NFET1 = P1^0;
sbit NFET2 = P1^2;
// PWM = P0.7
// UART RX = P0.5
// UART TX = P0.4

// debug
sbit LED = P0^3;                          // LED='1' means ON

// from BLHeli/blob/master/SiLabs/Turnigy_Plush_6A.inc
#define NFETON_DELAY 1  // Wait delay from pfets off to nfets on


#include "sin_table.h"

int8_t *sin_table_start = sin_table;
int8_t *sin_table_end = sin_table + SIN_TOTAL;
uint8_t counter;

#define UART_SYNC1 0
#define UART_SYNC2 1
#define UART_POWER 2 
#define UART_PHASE 3
uint8_t uart_state = UART_SYNC1;
uint8_t uart_power;
uint8_t uart_phase;
uint8_t need_pwm = 0;
uint8_t need_update = 0;
uint16_t duty0 = PCA_RELOAD + MAX_PWM / 2;
uint16_t duty1 = PCA_RELOAD + MAX_PWM / 2;
uint16_t duty2 = PCA_RELOAD + MAX_PWM / 2;

void update_motor()
{
    if(uart_phase == GROUND_MOTOR)
    {
        need_pwm = 0;
// turn on all N
// turn off all P
        P1 = 0x07;
//         PFET0 = 0;
//         PFET1 = 0; 
//         PFET2 = 0; 
//         NFET0 = 1; 
//         NFET1 = 1; 
//         NFET2 = 1; 
        duty0 = PCA_RELOAD + MAX_PWM / 2;
        duty1 = PCA_RELOAD + MAX_PWM / 2;
        duty2 = PCA_RELOAD + MAX_PWM / 2;
    }
    else
    if(uart_phase >= SIN_TOTAL)
    {
// MOSFETs off
        need_pwm = 0;
        P1 = 0x00;
//         PFET0 = 0;
//         PFET1 = 0; 
//         PFET2 = 0; 
//         NFET0 = 0; 
//         NFET1 = 0; 
//         NFET2 = 0;
        duty0 = PCA_RELOAD + MAX_PWM / 2;
        duty1 = PCA_RELOAD + MAX_PWM / 2;
        duty2 = PCA_RELOAD + MAX_PWM / 2;
    }
    else
    {
// compute phases
        int8_t *ptr;
        uint16_t duty0_;
        uint16_t duty1_;
        uint16_t duty2_;
        ptr = sin_table_start + uart_phase;
#define COMPUTE_PHASE \
    ((int32_t)PCA_RELOAD + MAX_PWM / 2 + MAX_PWM * *ptr / 0xff * uart_power / 0xff)

        duty0_ = COMPUTE_PHASE;
        ptr += SIN_TOTAL / 3;
        if(ptr >= sin_table_end) ptr -= SIN_TOTAL;
        duty1_ = COMPUTE_PHASE;
        ptr += SIN_TOTAL / 3;
        if(ptr >= sin_table_end) ptr -= SIN_TOTAL;
        duty2_ = COMPUTE_PHASE;


        EA = 0;
        duty0 = duty0_;
        duty1 = duty1_;
        duty2 = duty2_;
        EA = 1;

        need_pwm = 1;
    }


    need_update = 0;
}

void main (void) 
{

// disable watchdog timer
    PCA0MD = 0;  // Disable watchdog timer


// Initialize system clock to
// 24.5MHz
    OSCICN = 0x83;                     // Configure internal oscillator for
                                       // its maximum frequency
    RSTSRC = 0x04;                         // enable missing clock detector

// crossbar page 128
    XBR0     = 0x01; // enable UART pins
// page 129
    XBR1     = 0xc0; // Enable crossbar and disable pull-ups

// UART page 158
    SCON0 = 0x10; // SCON0: 8-bit variable bit rate
                  //        level of STOP bit is ignored
                  //        RX enabled
                  //        ninth bits are zeros
                  //        clear RI0 and TI0 bits

// baud rate is created by timer 1 in 8 bit mode.  page 154
// baud = SYSTEMCLOCK / TH1 / 2
    TH1 = -(SYSTEMCLOCK/BAUD/2);
    TL1 = TH1; // init Timer1
// page 183
    CKCON = 0x0c; // No prescaling for timers
// page 182
// Timer0 in 16-bit mode
// Timer1 in 8-bit autoreload
    TMOD = 0x21; 
    TR1 = 1; // Timer1 on

    IP |= 0x10; // Make UART high priority
    ES0 = 1; // enable UART0 interrupt page 89

    LED = 0;

// Timer 0
//    TH0 = MAX_PWM >> 8;    // Init Timer0 High register
//    TL0 = MAX_PWM & 0xff ; // Init Timer0 Low register
//    ET0 = 1;               // Timer0 interrupt enabled
//    TR0 = 1; // Timer0 ON


// PCA page 205
    PCA0CN = 0x00; // Stop counter; clear all flags
    PCA0MD = 0x09; // Use SYSCLK as time base.  Enable overflow interrupt.
    PCA0CPM0 = 0xC9; // Module 0 = 16 bit Match and Interrupt
    PCA0CPM1 = 0xC9; // Module 1 = 16 bit Match and Interrupt
    PCA0CPM2 = 0xC9; // Module 2 = 16 bit Match and Interrupt
// set the period
    PCA0L = PCA_RELOAD & 0xff;
    PCA0H = PCA_RELOAD >> 8;

    update_motor();
    EIE1 |= 0x10; // enable PCA interrupts
    CR = 1; // start PCA counter


    P1MDOUT |= 0x77; // enable MOSFET pins as a push-pull outputs
    P0MDOUT |= 0x08; // enable LED as a push-pull output
    EA = 1; // enable global interrupts

    while (1) { // spin forever
        if(need_update)
            update_motor();
    }
}

void UART0_Interrupt (void) interrupt 4
{
    if (RI0 == 1)
    {
        unsigned char c;
        c = SBUF0; // Read a character from UART
        RI0 = 0; // Clear interrupt flag
        
        switch(uart_state)
        {
            case UART_SYNC1:
                if(c == 0xff)
                    uart_state = UART_SYNC2;
                break;
            case UART_SYNC2:
                if(c == 0x88)
                    uart_state = UART_POWER;
                else
                    uart_state = UART_SYNC1;
                break;
            case UART_POWER:
                uart_power = c;
                uart_state = UART_PHASE;
                break;
            case UART_PHASE:
                uart_phase = c;
                uart_state = UART_SYNC1;
                need_update = 1;
                break;
        }

//LED = ~LED; // debug
    }
}

void PCA0_ISR (void) interrupt 11
{
// overflow
    if(CF)
    {
// turn on all the P's
        if(need_pwm)
        {
//            P1 = 0x70;
            NFET0 = 0;
            NFET1 = 0;
            NFET2 = 0;

// copy in the precalculated duty cycles & delay before PFETs
            PCA0L = PCA_RELOAD & 0xff;
            PCA0H = PCA_RELOAD >> 8;
            PCA0CPL0 = duty0 & 0xff;
            PCA0CPH0 = duty0 >> 8;
            PCA0CPL1 = duty1 & 0xff;
            PCA0CPH1 = duty1 >> 8;
            PCA0CPL2 = duty2 & 0xff;
            PCA0CPH2 = duty2 >> 8;

            PFET0 = 1;
            PFET1 = 1;
            PFET2 = 1;
        }
        else
        {
// copy in the precalculated duty cycles
            PCA0L = PCA_RELOAD & 0xff;
            PCA0H = PCA_RELOAD >> 8;
            PCA0CPL0 = duty0 & 0xff;
            PCA0CPH0 = duty0 >> 8;
            PCA0CPL1 = duty1 & 0xff;
            PCA0CPH1 = duty1 >> 8;
            PCA0CPL2 = duty2 & 0xff;
            PCA0CPH2 = duty2 >> 8;
        }

        CF = 0;

    }

    if(CCF0)
    {
        if(need_pwm)
        {
            PFET0 = 0;
            for(counter = NFETON_DELAY; counter > 0; counter--)
                ;
            NFET0 = 1;
        }
        CCF0 = 0;                           // Clear module 0 interrupt flag.
//LED = 0; // debug
    }

    if(CCF1)
    {
        if(need_pwm)
        {
            PFET1 = 0;
            for(counter = NFETON_DELAY; counter > 0; counter--)
                ;
            NFET1 = 1;
        }
        CCF1 = 0;                           // Clear module 1 interrupt flag.
    }

    if(CCF2)
    {
        if(need_pwm)
        {
            PFET2 = 0;
            for(counter = NFETON_DELAY; counter > 0; counter--)
                ;
            NFET2 = 1;
        }
        CCF2 = 0;                           // Clear module 2 interrupt flag.
    }
}



// void Timer0_ISR (void) interrupt 1
// {
//    TH0 = MAX_PWM >> 8;           // Reinit Timer0 High register
//    TL0 = MAX_PWM & 0xff;            // Reinit Timer0 Low register
//    LED = ~LED;  // Toggle the LED
// }





