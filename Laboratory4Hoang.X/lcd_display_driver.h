/* 
File: lcd_display_driver.h
Author: Minh Hoang
Created on August 29, 2025, 10:45 AM
Description: C header file to initialize all functions needed for the LCD display on the board
*/

#ifndef LCD_DISPLAY_DRIVER_H
#define	LCD_DISPLAY_DRIVER_H


void lcd_display_driver_delay(int);

void lcd_display_driver_enable(); //sets the toggles the enable bit on the LCD controller/driver long enough to perform a display operation

void lcd_display_driver_initialize(); //performs the sequence of initialization operations to bring up the display so that it operates in a mode with 5x7 dots, dual line, and no cursor

void lcd_display_driver_clear(); //clears the display of all characters

void lcd_display_driver_write(char*, int); //writes the entire string defined by ?data? of length ?length? to the display

void display_driver_use_first_line(void); //set the configures the display to write to the first line of the LCD display

void display_driver_use_second_line(void); //set the configures the display to write to the second line of the LCD display
#endif	/* LCD_DISPLAY_DRIVER_H */

