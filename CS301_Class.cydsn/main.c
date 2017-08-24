/* ========================================
 * Fully working code: 
 * PWM      : 
 * Encoder  : 
 * ADC      :
 * USB      : port displays speed and position.
 * CMD: "PW xx"
 * Copyright Univ of Auckland, 2016
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Univ of Auckland.
 *
 * ========================================
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <project.h>
#include <ADC_SAR.h>
#include <CONTROL.h>

#define TOGGLE_LED LED_Write(~LED_Read())
#define PACKETSIZE 32
#define RXSTRINGSIZE 255

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 64 // USBUART fixed buffer size

#define CHAR_NULL '0'
#define CHAR_BACKSP 0x08
#define CHAR_DEL 0x7F
#define CHAR_ENTER 0x0D
#define LOW_DIGIT '0'
#define HIGH_DIGIT '9'

#define SOP 0xaa
//* ========================================
char rf_string[RXSTRINGSIZE];
char displaystring[BUF_SIZE] = "UART Lab Exercise 4\n";
char line[BUF_SIZE], entry[BUF_SIZE];
uint8 usbBuffer[BUF_SIZE];

void usbPutString(char *s);
void usbPutChar(char c);
void handle_rx_binary();
void handle_rx_ascii();
void handle_usb();
//* ========================================
#include "defines.h"
#include "vars.h"

int main()
{
// --------------------------------    
// ----- INITIALIZATIONS ----------
    CYGlobalIntEnable;

// ------USB SETUP ----------------    
#ifdef USE_USB    
    USBUART_Start(0,USBUART_5V_OPERATION);
#endif   

    int usbOutput = 0;
    RF_BT_SELECT_Write(0);
    
// ------MOTOR SETUP --------------      
    int motorSpeed = 20;
    
    CONTROL_Write(0b00000000); // enable motor
    Clock_PWM_Start(); // Start clock for PWM
    PWM_1_Start();
    PWM_2_Start();
    //PWM_1_WriteCompare(motorSpeed);
    //PWM_2_WriteCompare(motorSpeed*0.96);
    PWM_1_WriteCompare(1);
    PWM_2_WriteCompare(0);
    
    
    //CONTROL_Write(0b00000011); // disable motor
    
// ------ADC SETUP ----------------      
    ADC_Start();
    uint16 ADCValue = 0;
    
// ------UART_RF Setup------------- 
    USBUART_Start(0,USBUART_5V_OPERATION);
    UART_Start();
    isrRF_RX_Start();
    usbPutString(displaystring);
    
    
    while(1) {
        /*
        ADC_StartConvert(); // start conversion
        int i=0;
        uint16 max = 0;
        for(i=0;i < 70;i++){ // 70 data points, to cover one period of the waveform            
            while(1) {
                if (ADC_IsEndConversion(ADC_RETURN_STATUS) != 0) { // when conversion completes
                    ADCValue = ADC_GetResult16(4u); // get result from channel 4
                    if (ADCValue > max){ // take max over that period
                        max = ADCValue;
                    }    
                    break;
                }
            }
        }    
        if (max < 2400){
            LED_Write(1);  
        } else {
            LED_Write(0);   
        }
        
        if (usbOutput == 1) {
            itoa(max, line, 10);
            usbPutString(line);
            usbPutString("\n");
        }   
        */
        if (flag_rx == 1){
            char data;
            data = UART_GetChar();
            usbPutChar(data);
            flag_rx = 0;
        }  
    }
}
//* ========================================
void usbPutString(char *s) {
// !! Assumes that *s is a string with allocated space >=64 chars     
//  Since USB implementation retricts data packets to 64 chars, this function truncates the
//  length to 62 char (63rd char is a '!')

#ifdef USE_USB     
    while (USBUART_CDCIsReady() == 0);
    s[63]='\0';
    s[62]='!';
    USBUART_PutData((uint8*)s,strlen(s));
#endif
}
//* ========================================
void usbPutChar(char c) {
#ifdef USE_USB     
    while (USBUART_CDCIsReady() == 0);
    USBUART_PutChar(c);
#endif    
}
//* ========================================
void handle_usb() {
    // handles input at terminal, echos it back to the terminal
    // turn echo OFF, key emulation: only CR
    // entered string is made available in 'line' and 'flag_KB_string' is set
    
    static uint8 usbStarted = FALSE;
    static uint16 usbBufCount = 0;
    uint8 c; 
    

    if (!usbStarted) {
        if (USBUART_GetConfiguration()) {
            USBUART_CDC_Init();
            usbStarted = TRUE;
        }
    }
    else {
        if (USBUART_DataIsReady() != 0) {  
            c = USBUART_GetChar();

            if ((c == 13) || (c == 10)) {
                if (usbBufCount > 0) {
                    entry[usbBufCount]= '\0';
                    strcpy(line,entry);
                    usbBufCount = 0;
                    flag_KB_string = 1;
                }
            }
            else {
                if (((c == CHAR_BACKSP) || (c == CHAR_DEL) ) && (usbBufCount > 0) )
                    usbBufCount--;
                else {
                    if (usbBufCount > (BUF_SIZE-2) ) {
                       USBUART_PutChar('!');        
                    } else {
                        entry[usbBufCount++] = c;
                    }
                }  
            }
        }
    }    
}

/* [] END OF FILE */
