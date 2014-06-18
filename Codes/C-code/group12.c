/********************************************************************************
 Project Title - Autonomous Defensive Robot
 @authors : 
  
  Group 12
   Harshvardhan Mandad 08005022, 
   Akhil Tak 08005029, 
   Vivek Surana 08005030, 
   Vinay Surana 08005031
   
   
 AVR Studio Version 4.17, Build 666

 Date: 25 March 2011

 This is project is about making which can detect enemies inside arena and shoot them.
 
 Note:

 1. Make sure that in the configuration options following settings are
        done for proper operation of the code

        Microcontroller: atmega2560
        Frequency: 11059200
        Optimization: -O0 (For more information read section: Selecting proper optimization options
                                                below figure 4.22 in the hardware manual)

 2. Difference between the codes for RS232 serial, USB and wireless communication is only in the serial port number.
        Rest of the things are the same.

 3. For USB communication check the Jumper 1 position on the ATMEGA2560 microcontroller adaptor board

 *********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, ERTS Lab IIT Bombay erts@cse.iitb.ac.in                     -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

 * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

 * Source code can be used for academic purpose.
         For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to:
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

 ********************************************************************************/


#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include "servo_motor.h"
#include "lcd.h"

/*
initialization of globals used in the program
 */
unsigned char data; //to store received data from UDR1
int count = 0;      // to maintain a count to avoid extra signal sent by matlab(Matlab sends an extra garbage byte along with every useful byte sent).
int degree = 0;     // the amount of angle in degrees that the servo motors should rotate from it's fixed initial reference position.


/*
Setting the congfigrations for the buzzer.
*/
void buzzer_pin_config(void) {
    DDRC = DDRC | 0x08; //Setting PORTC 3 as outpt
    PORTC = PORTC & 0xF7; //Setting PORTC 3 logic low to turnoff buzzer
}

/*
Function to initialize ports
 */
void port_init() {
    buzzer_pin_config();
}

/*
 Function that starts the buzzer.
 */
void buzzer_on(void) {
    unsigned char port_restore = 0;
    port_restore = PINC;
    port_restore = port_restore | 0x08;
    PORTC = port_restore;
}

/*
function to stop the buzzer.
 */
void buzzer_off(void) {
    unsigned char port_restore = 0;
    port_restore = PINC;
    port_restore = port_restore & 0xF7;
    PORTC = port_restore;
}

/*
Function To Initialize UART0
desired baud rate:9600
actual baud rate:9600 (error 0.0%)
char size: 8 bit
parity: Disabled
 */
void uart0_init(void) {
    UCSR0B = 0x00; //disable while setting baud rate
    UCSR0A = 0x00;
    UCSR0C = 0x06;
    UBRR0L = 0x47; //set baud rate lo
    UBRR0H = 0x00; //set baud rate hi
    UCSR0B = 0x98;
}

/*
 Function that actually receives the signals(commands) from the computer over the zigbee communication channel.
 It takes in the degree values and rotates the servo motors on the bot accordingly.
 */
SIGNAL(SIG_USART0_RECV) // ISR for receive complete interrupt
{
    data = UDR0; //making copy of data from UDR0 in 'data' variable

    count++;
    if (count % 2 == 1) {
        degree = (int) data;
        lcd_cursor(1, 3 + count);
        if (degree == 250) { //a special degree value for switching the buzzer on. This value is sent when the front camera detects laser beam on the ball(i.e. A HIT)
            buzzer_on();
            _delay_ms(500);
            buzzer_off();
        } else {
            if (degree > 207) {
                degree = degree - 80;
            }
            lcd_print((char) 1, (char) 12, (int) degree, 3);
            /*
            the degree sent from the computer is half the actual angle of rotation so that it can be sent in one byte.
             Hence, the actual degree is 2 times the received degree value.
            */
            degree = (degree * 2) % 360;

            lcd_print((char) 1, (char) 8, (int) degree, 3);

            //changing the degree value to shoot in a range of 10 degrees(degree-8,degree+2) continuously(in steps of 2 degrees).
            degree = (degree >= 8) ? (degree - 8) : 0;
            int p = 0;

            for (p = 0; p < 6 && degree < 360; p++) {
                
                if (degree <= 180) {    //degree value < 180 thus, only one servo motor needs to move(the bottom one).
                    servo_1(degree);
                    servo_2(180);
                } else {                //degree valu > 180 thus, both motors need to move.
                    servo_1(180);
                    servo_2(360 - degree);
                }
                degree += 2;
                if(p!=0) buzzer_on();          
                _delay_ms(200);
                buzzer_off();
            }
        }
        count = 1;      //a pair of signals has been received now waiting for new pair of signals(rotation angle).
    }
}

/*
 Function To Initialize all The Devices.
 */
void init_devices() {
    cli(); //Clears the global interrupts
    port_init(); //Initializes all the ports
    uart0_init(); //Initailize UART1 for serial communiaction
    sei(); //Enables the global interrupts
}

/*
 Main function that does the initializations and brings the motors to initial position.
 */
int main(void) {
    init_devices();
    init_servo();
    init_lcd();

    lcd_set_4bit();
    lcd_init();
    lcd_cursor(2, 1);
    lcd_string("FIRING BOT");
    servo_1(0);
    _delay_ms(750);
    servo_2(180);
    count = 0;
    degree = 0;
    _delay_ms(1000);

    //servo_1_free();
    //servo_2_free();
    //servo_3_free();
    while (1);
}

