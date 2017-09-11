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
char rf_string_complete[RXSTRINGSIZE];
char displaystring[BUF_SIZE] = "UART Lab Exercise 4\n";
char testString[BUF_SIZE];
char line[BUF_SIZE], entry[BUF_SIZE], test[BUF_SIZE];
uint8 usbBuffer[BUF_SIZE];

int count = 0;
int startCount = 0;
int usbOutput = 1;

int motor1Distance = 0; //global variables that keep track of distance travelled by robot's two wheels
int motor2Distance = 0; //distances are in millimetres

//Arrays for light sensors, index value 0 is not used (makes it easier to link index number to sensor number)
int maxValue[] = {0,0,0,0,0,0,0};             //max value sensed by each light sensor (ignore index value 0)
int ADCValue[] = {0,0,0,0,0,0,0};             //current ADC value from each light sensor (ignore index value 0)
int isUnderLine[] = {0,0,0,0,0,0,0};       //boolean value for determining whether or not a light sensor is under the line or not (ignore index value 0)

//Initialising motor speed values
int motor1Speed = 20;
int motor2Speed = 20*0.96; 

//Flags for turns
int turningLeft = 0;
int turningRight = 0;


void sensorIsUnderLine(int sensorNum);
int getBatteryVoltage();
int handleRadioData();
void usbPutString(char *s);
void usbPutChar(char c);
void handle_rx_binary();
void handle_rx_ascii();
void handle_usb();
void calculateDistanceTravelled();
//* ========================================
#include "defines.h"
#include "vars.h"

int main()
{
// --------------------------------    
// ----- INITIALIZATIONS ----------
    CYGlobalIntEnable;

// ------USB SETUP ----------------    
    USBUART_Start(0,USBUART_5V_OPERATION);  
    RF_BT_SELECT_Write(0);
    
// ------MOTOR SETUP --------------       
    
    CONTROL_Write(0b00000000); // enable motor
    Clock_PWM_Start(); // Start clock for PWM
    PWM_1_Start();
    PWM_2_Start();
    PWM_1_WriteCompare(motor1Speed);
    PWM_2_WriteCompare(motor2Speed);
    
    //CONTROL_Write(0b00000011); // disable motor
    
// ------ADC SETUP ----------------      
    ADC_Start();
    
// ------UART_RF Setup------------- 
    USBUART_Start(0,USBUART_5V_OPERATION);
    UART_Start();
    isrRF_RX_Start();
    usbPutString(displaystring);   
    
    while(1) {
        //Update each sensor values of maxValue, ADCValue and isUnderLine
        int m;
        for (m = 1; m < 7; m++){
            sensorIsUnderLine(m);
        }
        
        //Print each sensor values of maxValues
        for(m = 1; m < 7; m++){
            if (usbOutput == 1) {
                itoa(maxValue[m], line, 10);
                itoa(m, testString, 10); 
                usbPutString("Light Sensor ");
                usbPutString(testString);
                usbPutString(" : ");
                usbPutString(line);
                usbPutString("\n");
            }
        }
        
        int voltage = getBatteryVoltage();
        int completeStructure = handleRadioData();
        if (completeStructure == 1) {
            int8 strength = system_state.rssi;
            int16 xpos = system_state.robot_xpos;
            int16 ypos = system_state.robot_ypos;
            int16 orient = system_state.robot_orientation;
                
            if (orient > 0) {                   
                itoa(strength, line, 10);
                usbPutString("RSSI: ");
                usbPutString(line);
                usbPutString("\n\r");
                
                itoa(xpos, line, 10);
                usbPutString("XPOS: ");
                usbPutString(line);
                usbPutString("\n\r");
                
                itoa(ypos, line, 10);
                usbPutString("YPOS: ");
                usbPutString(line);
                usbPutString("\n\r");
                
                itoa(orient, test, 10);
                usbPutString("Orientation: ");
                usbPutString(test);
                usbPutString("\n\r");
            }
        }
    calculateDistanceTravelled();
    }
}

//* ========================================
void sensorIsUnderLine(int sensorNum) {
    ADC_StartConvert(); // start conversion
    int i = 0;
    ADCValue[sensorNum] = 0;
    maxValue[sensorNum] = 0;
    for(i=0;i < 70;i++){ // 70 data points, to cover one period of the waveform            
        while(1) {
            if (ADC_IsEndConversion(ADC_RETURN_STATUS) != 0) { // when conversion completes
                ADCValue[sensorNum] = ADC_GetResult16(sensorNum); // get result from channel 4
                if (ADCValue[sensorNum] > maxValue[sensorNum]){ // take max over that period
                    maxValue[sensorNum] = ADCValue[sensorNum];
                }    
                break;
            }
        }               
    }
  
    if (maxValue[sensorNum] < 2400){
        LED_Write(1);
        isUnderLine[sensorNum] = 1;     //Under the line
    } else {
        LED_Write(0);   
        isUnderLine[sensorNum] = 0;     //Not under the line
    }
    return;
}
//* ========================================
int getBatteryVoltage() {
   ADC_StartConvert();
        int ADCValue = 0;
        while (1) {
            if (ADC_IsEndConversion(ADC_RETURN_STATUS) != 0) {
                ADCValue = ADC_GetResult16(4u);
                break;
            }
        } 
    int voltage = (ADCValue*8*1000)/4096;
    return voltage;
}
//* ========================================
int handleRadioData() {
    int rxReceived = 0;
    if (flag_rx == 1) {
        char data;           
        if (count > PACKETSIZE) {
            memcpy(&system_state, rf_string, PACKETSIZE);
            count = 0;
                
        rxReceived = 1;    
        }
        
        data = UART_GetChar();
        if (data == SOP) {
            startCount++;
            if (startCount == 2) {
                count = -1;
                startCount = 0;
            }                  
        } 
        if (count >= 0) {
            rf_string[count] = data;
        }
        count++;
        flag_rx = 0;
    }
    return rxReceived;
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

//function to turn the robot to the left
//change to reasonable turning speed for both motors***
void turn_left(){
        PWM_1_WriteCompare(255); //move motor1 backward
        PWM_2_WriteCompare(0); //move motor2 forward  
}

//function to turn the robot to the right
//change to reasonable turning speed for both motors***
void turn_right(){
        PWM_1_WriteCompare(0); //move motor1 forward
        PWM_2_WriteCompare(255); //move motor2 backward 
}

//function to handle turns
//for now, all the intersections will result in a left turn (unless its not posssible then it'll turn right)
void handle_turns() {

    //CASE 1
    //not sure if this type of intersection exists in the map or not so it is commented out for now
    /*if (isUnderLine[1] == 1 && isUnderLine[2] == 1 && isUnderLine[3] == 1 && isUnderLine[4] == 1 && isUnderLine[5] == 1 && isUnderLine[6] == 1) {
        turningLeft = 1;
        while (turningLeft) {
            turn_left();
            if (
        }
        //turn left
    }*/
    
    //CASE 2
    if (isUnderLine[1] == 0 && isUnderLine[2] == 0 && isUnderLine[3] == 1 && isUnderLine[4] == 1 && isUnderLine[5] == 1 && isUnderLine[6] == 1) {
        turningLeft = 1;
        while (turningLeft) {
            turn_left();
            if (isUnderLine[1] == 1 && isUnderLine[2] == 1 && isUnderLine[3] == 1 && isUnderLine[4] == 1 && isUnderLine[5] == 0 && isUnderLine[6] == 1) {
                turningLeft = 0;
            }
        }
    }
    //CASE 3
    else if (isUnderLine[1] == 0 && isUnderLine[2] == 0 && isUnderLine[3] == 1 && isUnderLine[4] == 1 && isUnderLine[5] == 0 && isUnderLine[6] == 1) {
        turningLeft = 1;
        while (turningLeft) {
            turn_left();
            if (isUnderLine[1] == 1 && isUnderLine[2] == 1 && isUnderLine[3] == 1 && isUnderLine[4] == 1 && isUnderLine[5] == 0 && isUnderLine[6] == 0) {
                turningLeft = 0;
            }
        }
    } 
    //CASE 4
    else if (isUnderLine[1] == 0 && isUnderLine[2] == 0 && isUnderLine[3] == 0 && isUnderLine[4] == 1 && isUnderLine[5] == 1 && isUnderLine[6] == 1) {
        turningRight = 1;
        while (turningRight) {
            turn_right();
            if (isUnderLine[1] == 1 && isUnderLine[2] == 1 && isUnderLine[3] == 0 && isUnderLine[4] == 1 && isUnderLine[5] == 1 && isUnderLine[6] == 0) {
                turningRight = 0;
            }
        }
    }
    //CASE 5
    else if (isUnderLine[1] == 1 && isUnderLine[2] == 1 && isUnderLine[3] == 0 && isUnderLine[4] == 1 && isUnderLine[5] == 1 && isUnderLine[6] == 1) {
        turningRight = 1;
        while (turningRight) {
               turn_right();
            if (isUnderLine[1] == 1 && isUnderLine[2] == 1 && isUnderLine[3] == 1 && isUnderLine[4] == 1 && isUnderLine[5] == 1 && isUnderLine[6] == 0) {
                turningRight = 0;
            }
        }
    } 
    //CASE 6
    else if (isUnderLine[1] == 1 && isUnderLine[2] == 1 && isUnderLine[3] == 1 && isUnderLine[4] == 1 && isUnderLine[5] == 0 && isUnderLine[6] == 1) {
        turningLeft = 1;
        while (turningLeft) {
            turn_left();
            if (isUnderLine[1] == 1 && isUnderLine[2] == 1 && isUnderLine[3] == 1 && isUnderLine[4] == 1 && isUnderLine[5] == 1 && isUnderLine[6] == 0) {
                turningLeft = 0;
            }
        }
    }    
}

//function havent been added to main-while loop
//checks the value in the light_sensor array and move th robot accordingly
void check_array() {  
    //both sensors are on the line
    //CASE 7
    if (isUnderLine[1] == 1 && isUnderLine[2] == 1 && isUnderLine[3] == 0 && isUnderLine[4] == 1 && isUnderLine[5] == 0 && isUnderLine[6] == 1) {
        //move forward as per usual
        turningLeft = 0; //done turning (if it was turning previously) so unflag
        PWM_1_WriteCompare(motor1Speed);
        PWM_2_WriteCompare(motor2Speed);
    }
    //sensor 1 is on the line 
    //sensor 2 off the line
    //CASE 10
    else if (isUnderLine[1] == 1 && isUnderLine[2] == 0 && isUnderLine[3] == 0 && isUnderLine[4] == 1 && isUnderLine[5] == 0) {
        //decrease motor1 speed
        //increase motor2 speed
        motor1Speed = motor1Speed + 1;
        motor2Speed = motor2Speed - 1;        
        PWM_1_WriteCompare(motor1Speed);
        PWM_2_WriteCompare(motor2Speed);
    }
    //sensor 1 off the line
    //sensor 2 on the line
    //CASE 9
    else if (isUnderLine[1] == 0 && isUnderLine[2] == 1 && isUnderLine[3] == 0 && isUnderLine[4] == 1 && isUnderLine[5] == 0) {
        //increase motor1 speed
        //decrease motor2 speed
        motor1Speed = motor1Speed - 1;
        motor2Speed = motor2Speed + 1;  
        PWM_1_WriteCompare(motor1Speed);
        PWM_2_WriteCompare(motor2Speed);
    }
    //both front sensors off the line
    //CASE 8
    else if (isUnderLine[1] == 0 && isUnderLine[2] == 0 && isUnderLine[4] == 1 && isUnderLine[6] == 1) {
        //stop both motors
        PWM_1_WriteCompare(127);
        PWM_2_WriteCompare(127);        
    }
    else {
        //sensors are not aligned for straight line algorithm
        handle_turns();
    }
}

//uses quadrature reading to determine the distance travelled by robot's two wheels
void calculateDistanceTravelled(){
    
    //append distance value to global distance variable
    motor1Distance = motor1Distance + (QuadDec_M1_GetCounter()*0.8887);//*202.6327/4/3/19
    motor2Distance = motor2Distance + (QuadDec_M2_GetCounter()*0.8887);
    
    QuadDec_M1_SetCounter(0);//reset counter after reading from it
    QuadDec_M2_SetCounter(0);
}    
/* [] END OF FILE */
