/* 
File:   main.c
Author: Minh Hoang
Date: November 4th, 2025
Description: 

*/

#pragma config POSCMOD = HS
#pragma config FNOSC = PRIPLL
#pragma config FPLLMUL = MUL_20
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLODIV = DIV_1
#pragma config FPBDIV = DIV_1
#pragma config FWDTEN = OFF

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <sys/attribs.h>
#include <math.h>
#include "lcd_display_driver.h"

#define ENCODER_CPR 48              // Counts per revolution of the encoder
#define ENCODER_GEAR_RATIO 98.78    // Gear ratio of the encoder
#define FPB 80000000                // Peripheral Bus clock set at 80MHz
#define BAUD 230400                 // Desired baud rate is 230400

/* Global variables for encoders */
static volatile unsigned int prev_encoderA = 0;
static volatile unsigned int prev_encoderB = 0;
static volatile int count = 0;

static volatile float referenceAngle = 0;   // Reference angle
static volatile float angle = 0;            // Current angle

static volatile float e_p = 0.0;       // Error for proportional (exercise E3)
const float g_p = 4.0;                 // Proportional gain
static volatile float e_i = 0.0;       // Error for proportional-integral (exercise E4)
const float g_i = 2.0;

/* Initialize functions */
void readUART2(char* message, int maxLength);
void writeUART2(const char* string);
int read_potentiometer(void);

/* Control mode flag */
static volatile int controlMode = 0;


void main(void){
    DDPCON = 0;
//    TRISAbits.TRISA0 = 0;	// LED D3
//    TRISAbits.TRISA1 = 0;	// LED D4
//    TRISAbits.TRISA3 = 0;	// LEd D6
    TRISA = 0xFF00;
    
    TRISFbits.TRISF0 = 0;   // Status pin 87
    TRISFbits.TRISF1 = 0;   // Status pin 88
    
    /* Initialize ADC */
    AD1CON3bits.ADCS = 2;
    AD1CON1bits.ON = 1;

    /* Configure interrupts */
    INTCONbits.MVEC = 1;    // Enable multiple-vector mode
    __builtin_disable_interrupts();

    // Change notification interrupt
    CNCONbits.ON = 1;       // Enable CN interrupt
    CNENbits.CNEN19 = 1;    // Enable change notification interrupt CN19 (S4)
    CNENbits.CNEN15 = 1;    // Enable change notification interrupt CN15 (S3)
    CNENbits.CNEN8 = 1;     // Enable change notification interrupt CN8 (Encoder A)
    CNENbits.CNEN9 = 1;     // Enable change notification interrupt CN8 (Encoder B)
    
    IPC6bits.CNIP = 5;      // Priority 5 for CN
    IPC6bits.CNIS = 0;      // Sub-priority 0 for CN
    IFS1bits.CNIF = 0;      // Clear status bit for CN
    IEC1bits.CNIE = 1;      // Enable CN ISR
    
    // UART2 interrupt
    U2STAbits.URXISEL = 0x0; 
    IPC8bits.U2IP = 6;     // Priority 6 for U2RX
    IPC8bits.U2IS = 4;     // Sub-priority 4 for U2RX
    IFS1bits.U2RXIF = 0;    // Clear flag
    IEC1bits.U2RXIE = 1;    // Enable U2RX interrupt
    
    // External interrupt 2
    INTCONbits.INT2EP = 0;  // INT2 triggers on falling edge
    IPC2bits.INT2IP = 4;    // Priority 5 for INT2
    IPC2bits.INT2IS = 0;    // Sub-priority 0 for INT2
    IFS0bits.INT2IF = 0;    // Clear status bit for INT2
    IEC0bits.INT2IE = 1;    // Enable interrupts INT2
    
    // Timer interrupt
    // Timer 5
    TMR5 = 0;
    PR5 = 31249;            // 10Hz
    T5CONbits.TCKPS = 0b111;// 1:256 prescale value
    T5CONbits.TGATE = 0;
    T5CONbits.TCS = 0;
    IPC5bits.T5IP = 5;      // Priority 5 for Timer 5
    IPC5bits.T5IS = 0;      // Sub-priority 0 for Timer 5
    IFS0bits.T5IF = 0;      // Clear flag
    
    // Timer 2
    TMR2 = 0;
    PR2 = 6249;             // 50Hz
    T2CONbits.TCKPS = 0b111;// 1:256 prescalar value
    T2CONbits.TGATE = 0;
    T2CONbits.TCS = 0;
    IPC2bits.T2IP = 5;      // Sub-priority 0 for Timer 2
    IPC2bits.T2IS = 0;      // Sub-priority 0 for Timer 2
    T2CONbits.ON = 1;       // Turn on Timer 2
    IEC0bits.T2IE = 1;      // Enable Timer 2
    IFS0bits.T2IF = 0;      // CLear flag
    
    // Timer 3 and OC4
    TMR3 = 0;
    PR3 = 1022;
    T3CONbits.TCKPS = 0b011;// 1:8 prescalar
    OC4CONbits.OCM = 0b110; // PWM mode on, fault pin disabled
    OC4CONbits.OCTSEL = 1;  // Use Timer 3 for timing
    OC4RS = 1023;
    OC4R = 1023;
    T3CONbits.ON = 1;
    OC4CONbits.ON = 1;      // Turn on OC4

    __builtin_enable_interrupts();
    
    /* Configure UART Communication*/
    U2MODEbits.BRGH = 0;            // Set M=16
    U2BRG = FPB/(16*BAUD) - 1;      // Set baud rate to 230400
    U2MODEbits.PDSEL = 0b00;        // 8-bit data, no parity 
    U2MODEbits.STSEL = 0;           // 1 stop bit
    U2STAbits.URXEN = 1;            // Receiver bit RX enabled
    U2STAbits.UTXEN = 1;            // Transmitter bit TX enabled
    U2MODEbits.UEN = 0b00;          // Configure hardware flow control using RTS and CTS
    U2MODEbits.ON = 1;              // UART 2 enabled
    
    // Angle display
    char refAngle[10];				// Store reference angle char output
    char encoderAngle[10];			// Store current angle char output
    
    // Initialize display
    lcd_display_driver_initialize();
    
    while(1){
        angle = count * 360 / (ENCODER_CPR * ENCODER_GEAR_RATIO);
        // Write reference angle on display
        int refLength = sprintf(refAngle, "REF:%11.2f%c", referenceAngle, 0xDF);
        display_driver_use_first_line();
        lcd_display_driver_write(refAngle, refLength);
        // Write current angle on display
        int angleLength = sprintf(encoderAngle, "ANGLE:%9.2f%c", angle, 0xDF);
        display_driver_use_second_line();
        lcd_display_driver_write(encoderAngle, angleLength);
    }
    return;
}

void __ISR(_CHANGE_NOTICE_VECTOR, IPL5SOFT) CNINT(void){
    
    LATAbits.LATA0 = prev_encoderA; // LED D3 shows encoder A bit
    LATAbits.LATA1 = prev_encoderB; // LED D4 shows encoder B bit

	unsigned int encoderA = PORTGbits.RG6;	// Read encoder bit A
	unsigned int encoderB = PORTGbits.RG7;	// Read encoder bit B

	// A = 0; B = 0
	if( (encoderA == 0) && (encoderB == 0)){
		if( (prev_encoderA == 1) && (prev_encoderB == 0) ){
			count++;
		}
		else if( (prev_encoderA == 0) && (prev_encoderB == 1) ){
			count--;
		} 
	}
    // A = 0; B = 1
    if( (encoderA == 0) && (encoderB == 1)){
		if( (prev_encoderA == 0) && (prev_encoderB == 0) ){
			count++;
		}
		else if( (prev_encoderA == 1) && (prev_encoderB == 1) ){
			count--;
		} 

	}
    // A = 1; B = 1
    if( (encoderA == 1) && (encoderB == 1)){
		if( (prev_encoderA == 0) && (prev_encoderB == 1) ){
			count++;
		}
		else if( (prev_encoderA == 1) && (prev_encoderB == 0) ){
			count--;
		} 
	}
    // A = 1; B = 0
    if( (encoderA == 1) && (encoderB == 0)){
		if( (prev_encoderA == 1) && (prev_encoderB == 1) ){
			count++;
		}
		else if( (prev_encoderA == 0) && (prev_encoderB == 0) ){
			count--;
		} 
	}
	// Update previous state
	prev_encoderA = encoderA;
	prev_encoderB = encoderB;
    
    // Update controlMode when pressing S3 or S4
    if(PORTDbits.RD6 == 0){         // S3 pressed, controlMode = 0
        controlMode = 0;
    } else if(PORTDbits.RD13 == 0){ // S4 pressed, controlMode = 1
        controlMode = 1;
    }
    IFS1bits.CNIF = 0;	// Clear CN flag
}

void __ISR(_EXTERNAL_2_VECTOR, IPL5SOFT) S5(void){ // External interrupt 2 for S5
    controlMode = 2;                // S5 pressed, controlMode = 2
    IFS0bits.INT2IF = 0;
}

void __ISR(_UART_2_VECTOR, IPL6SOFT) U2RX(void){
    // Read angle from UART2
    char messageBuffer[10];
    readUART2(messageBuffer, 10);
    referenceAngle = atof(messageBuffer);
    
    // Reset and start timer 5
    TMR5 = 0;
    T5CONbits.ON = 1;
    IEC0bits.T5IE = 1;
    
    _CP0_SET_COUNT(0);
    
    IFS1bits.U2RXIF = 0;	 // Clear flag
}

void __ISR(_TIMER_5_VECTOR, IPL5SOFT) T5(void){
    float currentTime = _CP0_GET_COUNT() * (25.0e-9);
    angle = count * 360 / (ENCODER_CPR * ENCODER_GEAR_RATIO);
    char messageBuffer[10];
    
    // Send current time
    sprintf(messageBuffer, "%0.4f\r\n", currentTime);
    writeUART2(messageBuffer);
    
    // Send reference angle
    sprintf(messageBuffer, "%0.4f\r\n", referenceAngle);
    writeUART2(messageBuffer);
    
    // Send current angle
    sprintf(messageBuffer, "%0.4f\r\n", angle);
    writeUART2(messageBuffer);
    
    // Disable Timer 5 ISR after 5 seconds
    if(currentTime >= 5.0){
        T5CONbits.ON = 0;   // Turn off T5
        IEC0bits.T5IE = 0;  // Disable T5
    }
    IFS0bits.T5IF = 0;      // Clear flag
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) T2(void){
    angle = count * 360 / (ENCODER_CPR * ENCODER_GEAR_RATIO);
    float angleDiff = referenceAngle - angle; // Angle difference between reference angle and current angle
    OC4RS = read_potentiometer();           // Assign potentiometer value to OC4RS
    if(controlMode == 0){			// Potentiometer control
        LATAbits.LATA5 = 1;         // Turn on LED D8 to check
        if(angleDiff < -7.5){       // If ref angle < current angle 
            // F1 = 1; F0 = 0 to decrease count
            LATFbits.LATF1 = 1;
            LATFbits.LATF0 = 0;
        } else if(angleDiff > 7.5){ // If ref angle > current angle
            // F1 = 0; F0 = 1 to increase count
            LATFbits.LATF1 = 0;
            LATFbits.LATF0 = 1;
        } else{                     // If angleDiff is within -7.5 and 7.5 then stop changing the count                          
            // F1 = 0; F0 = 0 to stop changing count
            LATFbits.LATF1 = 0;
            LATFbits.LATF0 = 0;
        }
    } else if(controlMode == 1){	// Proportional control
        LATAbits.LATA6 = 1;         // Turn on LED D9 to check
        e_p = angleDiff;
        if(fabs(g_p * e_p) > 1023){
            OC4RS = 1023;
        } else{
            OC4RS = fabs(g_p * e_p);
        }

        if((g_p * e_p) < 0.0){       // If ref angle < current angle 
            // F1 = 1; F0 = 0 to decrease count
            LATFbits.LATF1 = 1;
            LATFbits.LATF0 = 0;
        } else if((g_p * e_p) > 0.0){ // If ref angle > current angle
            // F1 = 0; F0 = 1 to increase count
            LATFbits.LATF1 = 0;
            LATFbits.LATF0 = 1;
        } else if((g_p * e_p) == 0.0){  // If angleDiff = 0 then stop changing the count                          
            // F1 = 0; F0 = 0 to stop changing count
            LATFbits.LATF1 = 0;
            LATFbits.LATF0 = 0;
        }
        
    } else if(controlMode == 2){	// Proportional-integral control
        LATAbits.LATA7 = 1;         // Turn on LED D10 to check
        e_p = angleDiff;
        e_i = e_i + 0.02 * e_p;

        if(fabs((g_p * e_p) + (g_i * e_i)) >= 1023){
            OC4RS = 1023;
        } else{
            OC4RS = fabs((g_p * e_p) + (g_i * e_i));
        }

        if(((g_p * e_p) + (g_i * e_i)) < 0.0){       // If ref angle < current angle 
            // F1 = 1; F0 = 0 to decrease count
            LATFbits.LATF1 = 1;
            LATFbits.LATF0 = 0;
        } else if(((g_p * e_p) + (g_i * e_i)) > 0.0){ // If ref angle > current angle
            // F1 = 0; F0 = 1 to increase count
            LATFbits.LATF1 = 0;
            LATFbits.LATF0 = 1;
        } else if(((g_p * e_p) + (g_i * e_i)) == 0.0){  // If angleDiff = 0 then stop changing the count                          
            // F1 = 0; F0 = 0 to stop changing count
            LATFbits.LATF1 = 0;
            LATFbits.LATF0 = 0;
        }   
    }
    IFS0bits.T2IF = 0;              // Clear flag
}

void readUART2(char* message, int maxLength){
    char data = 0;
    int num_bytes = 0;
    while(1){
        if(U2STAbits.URXDA){    // If data is available
            data = U2RXREG;     // Receive data
            if(data == '\n' || data == '\r'){
                break;
            } else{
                message[num_bytes] = data;
                num_bytes++;
            }
        }
    }
    message[num_bytes]='\0';
    return;
}

void writeUART2(const char *message){
    while(*message != '\0'){
        while(U2STAbits.UTXBF){;}	// Check for transmit buffer full status bit
        U2TXREG = *message;
        ++message;
    }
    return;
}

int read_potentiometer(void){
    AD1CHSbits.CH0SA = 2;   // Analog input 2
    AD1CON1bits.SAMP = 1;   // Start sampling
    // Wait for 100 core timer ticks so capacitor can reach steady state
    unsigned int start = _CP0_GET_COUNT();
    unsigned int current = _CP0_GET_COUNT();
    while( (current - start) < 100 ){
        current = _CP0_GET_COUNT();
    }
    AD1CON1bits.SAMP = 0;   // Hold sampling, start conversion
    while(!AD1CON1bits.DONE){;}   // Wait until conversion is done

    return ADC1BUF0;   // Return result in 10 bits
}




