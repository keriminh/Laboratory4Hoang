/* 
File: lcd_display_driver.c
Author: Minh Hoang
Created on August 29, 2025, 10:45 AM
Description: C source file to define each function in the C header file lcd_display_driver.h
*/

#include <stdio.h>
#include <stdlib.h>

#include <xc.h>
#include "lcd_display_driver.h"

void lcd_display_driver_delay(int steps){
    int i = 0;
    for(i = 0; i < steps; i++){};
    return;
}

void lcd_display_driver_enable(){
    // Raise E from 0 to 1
    LATDbits.LATD4 = 1;
    lcd_display_driver_delay(2000);
    // Lower E from 1 to 0
    LATDbits.LATD4 = 0;
    lcd_display_driver_delay(2000);
    return;
}

void lcd_display_driver_initialize(){ //performs the sequence of initialization operations to bring up the display so that it operates in a mode with 5x7 dots, dual line, and no cursor
//    lcd_display_driver_enable();
    TRISDbits.TRISD4 = 0;   // E
    TRISDbits.TRISD5 = 0;   // RS
    TRISBbits.TRISB15 = 0;  // RW
    TRISE = 0xFF00;			// Used for display driver
    
    // Function set:
    LATDbits.LATD5 = 0;
    LATBbits.LATB15 = 0;
    LATE = 0b00111000;    // 2-line mode, 5x7 dots
    lcd_display_driver_enable();

    // Display ON/OFF control
    LATDbits.LATD5 = 0;
    LATBbits.LATB15 = 0;
    LATE = 0b00001100;    // turn display ON, cursor OFF, blinking OFF
    lcd_display_driver_enable();

    // Clear Display
    LATE = 0b00000001;
    lcd_display_driver_enable();

    //Entry mode set
    LATDbits.LATD5 = 0;
    LATBbits.LATB15 = 0;
    LATE = 0b00000110;    // increment mode, entire shift mode
    lcd_display_driver_enable();
    
    return;
}

void lcd_display_driver_clear(){ //clears the display of all characters
    LATBbits.LATB15 = 0;
    LATDbits.LATD5 = 0;
    LATE = 0b00000001;    // Set DB0 to 1 to clear display
    lcd_display_driver_enable();
    return;
}

void lcd_display_driver_write(char* data, int length){ //writes the entire string defined by ?data? of length ?length? to the display
    // CG RAM data write
    LATBbits.LATB15 = 1;
    LATDbits.LATD5 = 0;
    int i = 0;
    for(i = 0; i < length; i++){
        LATE = data[i]; // assign ASCII code of each char to the 8-bit display
        lcd_display_driver_enable();
    }
    return;
}

void display_driver_use_first_line(void){ //set the configures the display to write to the first line of the LCD display
    LATBbits.LATB15 = 0;
    LATDbits.LATD5 = 0;
    LATE = 0b10000000;
    lcd_display_driver_enable();
    return;
}

void display_driver_use_second_line(void){ //set the configures the display to write to the second line of the LCD display
    LATBbits.LATB15 = 0;
    LATDbits.LATD5 = 0;
    LATE = 0b11000000;
    lcd_display_driver_enable();
    return;
}
