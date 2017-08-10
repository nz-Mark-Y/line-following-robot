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
//* ========================================
#include "defines.h"
#include "vars.h"
//* ========================================
void usbPutString(char *s);
void usbPutChar(char c);
void handle_usb();
//* ========================================


int main()
{
    

// --------------------------------    
// ----- INITIALIZATIONS ----------
    CYGlobalIntEnable;

// ------USB SETUP ----------------    
#ifdef USE_USB    
    USBUART_Start(0,USBUART_5V_OPERATION);
#endif        
    usbPutString(displaystring);
    RF_BT_SELECT_Write(0);

    M1_D1_Write(0);
    M1_D2_Write(1);
    M1_enable_Write(1);
    M1_IN1_Write(0);
    M1_IN2_Write(1);
    
    M2_D1_Write(0);
    M2_D2_Write(1);
    M2_enable_Write(1);
    M2_IN1_Write(1);
    M2_IN2_Write(0);

    uint16 ADCValue = 0;
    ADC_Start();
    int LED_on = 0;
    while(1) {
        ADC_StartConvert();
        int i=0;
        uint16 max = 0;
        for(i=0;i < 70;i++){
            
            while(1) {
                if (ADC_IsEndConversion(ADC_RETURN_STATUS) != 0) {
                    ADCValue = ADC_GetResult16(4u);
                    if (ADCValue > max){
                        max = ADCValue;
                    }    
                    break;
                }
            }
        }
        
        if (max < 2500 && LED_on == 0){
            LED_Write(1);  
            LED_on = 1;
        }
        if(max > 2700 && LED_on == 1){
            LED_Write(0);
            LED_on = 0;
        }
        itoa(max, line, 10);
        usbPutString(line);
        usbPutString("\n");
    }
    
  /* 
    usbPutString(displaystring);
    for(;;)
    {
        handle_usb();    
        if (flag_KB_string == 1)
        {
            usbPutString(line);
            flag_KB_string = 0;
        }        
    }  
   */
}
//* ========================================
void usbPutString(char *s)
{
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
void usbPutChar(char c)
{
#ifdef USE_USB     
    while (USBUART_CDCIsReady() == 0);
    USBUART_PutChar(c);
#endif    
}
//* ========================================
void handle_usb()
{
    // handles input at terminal, echos it back to the terminal
    // turn echo OFF, key emulation: only CR
    // entered string is made available in 'line' and 'flag_KB_string' is set
    
    static uint8 usbStarted = FALSE;
    static uint16 usbBufCount = 0;
    uint8 c; 
    

    if (!usbStarted)
    {
        if (USBUART_GetConfiguration())
        {
            USBUART_CDC_Init();
            usbStarted = TRUE;
        }
    }
    else
    {
        if (USBUART_DataIsReady() != 0)
        {  
            c = USBUART_GetChar();

            if ((c == 13) || (c == 10))
            {
//                if (usbBufCount > 0)
                {
                    entry[usbBufCount]= '\0';
                    strcpy(line,entry);
                    usbBufCount = 0;
                    flag_KB_string = 1;
                }
            }
            else 
            {
                if (((c == CHAR_BACKSP) || (c == CHAR_DEL) ) && (usbBufCount > 0) )
                    usbBufCount--;
                else
                {
                    if (usbBufCount > (BUF_SIZE-2) ) // one less else strtok triggers a crash
                    {
                       USBUART_PutChar('!');        
                    }
                    else
                        entry[usbBufCount++] = c;  
                }  
            }
        }
    }    
}


/* [] END OF FILE */
