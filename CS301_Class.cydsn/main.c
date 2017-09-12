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

#define RXSTRINGSIZE 255
#define BUF_SIZE 64 // USBUART fixed buffer size

#define SOP 0xaa
//* ========================================
char rf_string[RXSTRINGSIZE];
char line[BUF_SIZE], battery_voltage_string[BUF_SIZE], test[BUF_SIZE], entry[BUF_SIZE], motor_line_1[BUF_SIZE], motor_line_2[BUF_SIZE];
uint8 usb_buffer[BUF_SIZE];
//* ========================================
int mode = 0;
int count = 0;
int start_count = 0;
int usb_output = 1;
//* ========================================
int motor_1_distance = 0; //global variables that keep track of distance travelled by robot's two wheels
int motor_2_distance = 0; //distances are in millimetres
//* ========================================
//Arrays for light sensors, index value 0 is not used (makes it easier to link index number to sensor number)
int max_value[] = {0,0,0,0,0,0,0};             //max value sensed by each light sensor (ignore index value 0)
int min_value[] = {0,0,0,0,0,0,0};             //min value sensed by each light sensor (ignore index value 0)
int ADC_value[] = {0,0,0,0,0,0,0};             //current ADC value from each light sensor (ignore index value 0)
int is_under_line[] = {0,0,0,0,0,0,0};       //boolean value for determining whether or not a light sensor is under the line or not (ignore index value 0)
//* ========================================
//Initialising motor speed values
int motor_1_default_speed = 80;
int motor_2_default_speed = 80;
int motor_1_speed = 80;
int motor_2_speed = 80;
//* ========================================
//Flags for turns
int turning_left = 0;
int turning_right = 0;
//* ========================================
void print_light_sensor_values();
void sensor_is_under_line(int sensorNum);
int get_battery_voltage();
int handle_radio_data();
void usb_put_string(char *s);
void usb_put_char(char c);
void handle_usb();
void turn_left();
void turn_right();
void go_straight();
void handle_turns();
void curves_mode();
void turns_mode();
void calculate_distance_travelled();
void check_mode();
//* ========================================
#include "defines.h"
#include "vars.h"
//* ========================================
int main() {
// --------------------------------    
// ----- INITIALIZATIONS ----------
    CYGlobalIntEnable;

// ------USB SETUP ----------------    
    USBUART_Start(0,USBUART_5V_OPERATION);  
    RF_BT_SELECT_Write(0);
    
// ------MOTOR SETUP --------------     
    CONTROL_Write(0b00000011); // disable motor    
    CyDelay(2000);
    
    CONTROL_Write(0b00000000); // enable motor
    Clock_PWM_Start(); // Start clock for PWM
    PWM_1_Start();
    PWM_2_Start();
    PWM_1_WriteCompare(motor_1_speed);
    PWM_2_WriteCompare(motor_2_speed);

// ------ADC SETUP ----------------      
    ADC_Start();
    QuadDec_M1_Start();
    QuadDec_M2_Start();
    
// ------UART_RF Setup------------- 
    USBUART_Start(0,USBUART_5V_OPERATION);
    UART_Start();
    isrRF_RX_Start();
    
    while(1) {
        //Update each sensor values of max_value, ADC_value and is_under_line
        int m;
        for (m = 1; m < 7; m++){
            sensor_is_under_line(m);
        }
        
        check_mode();
        
        if (mode == 0) {
            curves_mode();
        } else if (mode == 1) {
            turns_mode();   
        }   
        /*
        int voltage = get_battery_voltage();
        itoa(voltage, battery_voltage_string, 10);
        usb_put_string("Battery Voltage: ");
        usb_put_string(battery_voltage_string);
        usb_put_string("\n\r");
    
        int complete_structure = handle_radio_data();
        if (complete_structure == 1) {
            int8 strength = system_state.rssi;
            int16 xpos = system_state.robot_xpos;
            int16 ypos = system_state.robot_ypos;
            int16 orient = system_state.robot_orientation;
                
            if (orient > 0) {                   
                itoa(strength, line, 10);
                usb_put_string("RSSI: ");
                usb_put_string(line);
                usb_put_string("\n\r");
                
                itoa(xpos, line, 10);
                usb_put_string("XPOS: ");
                usb_put_string(line);
                usb_put_string("\n\r");
                
                itoa(ypos, line, 10);
                usb_put_string("YPOS: ");
                usb_put_string(line);
                usb_put_string("\n\r");
                
                itoa(orient, test, 10);
                usb_put_string("Orientation: ");
                usb_put_string(test);
                usb_put_string("\n\r");
            }
        }
    
    */
    calculate_distance_travelled();
    }
}

//* ========================================
void print_light_sensor_values() {
    // prints out light sensor values 
    
    char sensor_string_1[BUF_SIZE], sensor_string_2[BUF_SIZE], sensor_string_3[BUF_SIZE], sensor_string_4[BUF_SIZE], sensor_string_5[BUF_SIZE], sensor_string_6[BUF_SIZE];
    char line1[BUF_SIZE], line2[BUF_SIZE],  line3[BUF_SIZE],  line4[BUF_SIZE],  line5[BUF_SIZE],  line6[BUF_SIZE];
    
    //LS1
    itoa((max_value[1]-min_value[1]), line1, 10);
    itoa(1, sensor_string_1, 10); 
    usb_put_string("Light Sensor ");
    usb_put_string(sensor_string_1);
    usb_put_string(" : ");
    usb_put_string(line1);
    if(is_under_line[1]){
        usb_put_string(" , is under the line.");
    }
    usb_put_string("\n\r");
        
    //LS2
    itoa((max_value[2]-min_value[2]), line2, 10);
    itoa(2, sensor_string_2, 10); 
    usb_put_string("Light Sensor ");
    usb_put_string(sensor_string_2);
    usb_put_string(" : ");
    usb_put_string(line2);
    if(is_under_line[2]){
        usb_put_string(" , is under the line.");
    }
    usb_put_string("\n\r");
        
    //LS3
    itoa((max_value[3]-min_value[3]), line3, 10);
    itoa(3, sensor_string_3, 10); 
    usb_put_string("Light Sensor ");
    usb_put_string(sensor_string_3);
    usb_put_string(" : ");
    usb_put_string(line3);
    if(is_under_line[3]){
        usb_put_string(" , is under the line.");
    }
    usb_put_string("\n\r");
        
    //LS4
    itoa((max_value[4]-min_value[4]), line4, 10);
    itoa(4, sensor_string_4, 10); 
    usb_put_string("Light Sensor ");
    usb_put_string(sensor_string_4);
    usb_put_string(" : ");
    usb_put_string(line4);
    if(is_under_line[4]){
        usb_put_string(" , is under the line.");
    }
    usb_put_string("\n\r");
        
    //LS5
    itoa((max_value[5]-min_value[5]), line5, 10);
    itoa(5, sensor_string_5, 10); 
    usb_put_string("Light Sensor ");
    usb_put_string(sensor_string_5);
    usb_put_string(" : ");
    usb_put_string(line5);
    if(is_under_line[5]){
        usb_put_string(" , is under the line.");
    }
    usb_put_string("\n\r");
        
    //LS6
    itoa((max_value[6]-min_value[6]), line6, 10);
    itoa(6, sensor_string_6, 10); 
    usb_put_string("Light Sensor ");
    usb_put_string(sensor_string_6);
    usb_put_string(" : ");
    usb_put_string(line6);
    if(is_under_line[6]){
        usb_put_string(" , is under the line.");
    }
    usb_put_string("\n\r");
}
//* ========================================
void sensor_is_under_line(int sensorNum) {
    // checks if a light sensor is under a line
    
    ADC_StartConvert(); // start conversion
    int i = 0;
    ADC_value[sensorNum] = 0;
    max_value[sensorNum] = 0;
    min_value[sensorNum] = 4096;
    for(i=0;i < 70;i++){ // 70 data points, to cover one period of the waveform            
        while(1) {
            if (ADC_IsEndConversion(ADC_RETURN_STATUS) != 0) { // when conversion completes
                ADC_value[sensorNum] = ADC_GetResult16(sensorNum); // get result from channel
                if (ADC_value[sensorNum] > max_value[sensorNum]){ // take max over that period
                    max_value[sensorNum] = ADC_value[sensorNum];
                } else if(ADC_value[sensorNum] < min_value[sensorNum]){
                    min_value[sensorNum] = ADC_value[sensorNum];  // take min over that period
                }
                break;
            }
        }               
    }
  
    if(sensorNum == 1) {
        max_value[1] = ((max_value[1] - 1638) * 1.6667);
        min_value[1] = (min_value[1] - 1638) * 1.6667;      
    }
    
    if ((max_value[sensorNum]-min_value[sensorNum]) < 180){
        LED_Write(1);
        is_under_line[sensorNum] = 1;     //Under the line
    } else {
        LED_Write(0);   
        is_under_line[sensorNum] = 0;     //Not under the line
    }
    return;
}
//* ========================================
int get_battery_voltage() {
    // gets the battery voltage
    
    ADC_StartConvert();
        int ADC_value = 0;
        while (1) {
            if (ADC_IsEndConversion(ADC_RETURN_STATUS) != 0) {
                ADC_value = ADC_GetResult16(0);
                break;
            }
        } 
    int voltage = (ADC_value*8*1000)/4096;  
    return voltage;
}
//* ========================================
int handle_radio_data() {
    // handles received radio data, in binary
    
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
            start_count++;
            if (start_count == 2) {
                count = -1;
                start_count = 0;
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
void usb_put_string(char *s) {
    //  assumes that *s is a string with allocated space >=64 chars     
    //  since USB implementation retricts data packets to 64 chars, this function truncates the length to 62 char (63rd char is a '!')  

#ifdef USE_USB     
    while (USBUART_CDCIsReady() == 0);
    s[63]='\0';
    s[62]='!';
    USBUART_PutData((uint8*)s,strlen(s));
#endif
}
//* ========================================
void usb_put_char(char c) {
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
//* ========================================
void turn_left(){
    // function to turn the robot to the left
    // change to reasonable turning speed for both motors
    
    PWM_1_WriteCompare(180); //move motor1 backward
    PWM_2_WriteCompare(65); //move motor2 forward  
}
//* ========================================
void turn_right() {
    // function to turn the robot to the right
    // change to reasonable turning speed for both motors
    
    PWM_1_WriteCompare(65); //move motor1 forward
    PWM_2_WriteCompare(180); //move motor2 backward 
}
//* ========================================
void go_straight() {
    // function to make the robot go straight
    // change to default speed for both motors
    
    motor_1_speed = motor_1_default_speed;
    motor_2_speed = motor_2_default_speed;
    PWM_1_WriteCompare(motor_1_speed);
    PWM_2_WriteCompare(motor_2_speed);
}
//* ========================================
void turns_mode() { 
    // function to handle turns mode
    
    if (turning_left == 1) {
        if (is_under_line[1] == 1 && is_under_line[2] == 1 && is_under_line[3] == 1 && is_under_line[4] == 1) {
            turning_left = 0;
            go_straight();
        } else {
            turn_left(); 
        }
    }
    
    if (turning_right == 1) {
        if (is_under_line[1] == 1 && is_under_line[2] == 1 && is_under_line[3] == 1 && is_under_line[4] == 1) {
            turning_right = 0;
            go_straight();
        } else {   
            turn_right();
        }
    }

    // keep robot on straight line
    if (is_under_line[1] == 1 && is_under_line[2] == 1) {
        turning_left = 0;
        turning_right = 0;
        go_straight();
    }
    else if (is_under_line[1] == 1 && is_under_line[2] == 0) {
        motor_1_speed = 105;
        motor_2_speed = 75;       
        PWM_1_WriteCompare(motor_1_speed);
        PWM_2_WriteCompare(motor_2_speed);
    }
    else if (is_under_line[1] == 0 && is_under_line[2] == 1) {
        motor_1_speed = 75;
        motor_2_speed = 105;
        PWM_1_WriteCompare(motor_1_speed);
        PWM_2_WriteCompare(motor_2_speed);
    }
    
    // handle 90 degree turns
    //CASE 3
    else if (is_under_line[1] == 0 && is_under_line[2] == 0 && is_under_line[3] == 1 && is_under_line[4] == 1 && is_under_line[5] == 0 && is_under_line[6] == 1) {
        turning_left = 1;    
    } 
    //CASE 4
    else if (is_under_line[1] == 0 && is_under_line[2] == 0 && is_under_line[3] == 0 && is_under_line[4] == 1 && is_under_line[5] == 1 && is_under_line[6] == 1) {
        turning_right = 1;
    }
}
//* ========================================
void curves_mode() {  
    // function to handle curves mode
 
    //CASE 7
    if (is_under_line[1] == 1 && is_under_line[2] == 1) { // both sensors are on the line
        turning_left = 0;
        turning_right = 0;
        go_straight(); // move forward as per usual
    }
    //CASE 10
    else if (is_under_line[1] == 1 && is_under_line[2] == 0) { // sensor 1 is on the line, sensor 2 is off the line    
        motor_1_speed = 105; // decrease motor1 speed
        motor_2_speed = 75; // increase motor2 speed        
        PWM_1_WriteCompare(motor_1_speed);
        PWM_2_WriteCompare(motor_2_speed);
    }
    //CASE 9
    else if (is_under_line[1] == 0 && is_under_line[2] == 1) { // sensor 1 is off the line, sensor 2 is on the line
        motor_1_speed = 75; // increase motor1 speed
        motor_2_speed = 105; // decrease motor2 speed
        PWM_1_WriteCompare(motor_1_speed);
        PWM_2_WriteCompare(motor_2_speed);
    }
    //CASE 8
    else if (is_under_line[1] == 0 && is_under_line[2] == 0) { // both front sensors off the line
        motor_1_speed = 170; // slow reverse
        motor_2_speed = 170; // slow reverse
        PWM_1_WriteCompare(motor_1_speed);
        PWM_2_WriteCompare(motor_2_speed);
    }
}
//* ========================================
void calculate_distance_travelled(){
    // uses quadrature reading to determine the distance travelled by robot's two wheels
    
    //append distance value to global distance variable
    //motor_1_distance = motor_1_distance + (QuadDec_M1_GetCounter()*0.8887);//*202.6327/4/3/19
    //motor_2_distance = motor_2_distance + (QuadDec_M2_GetCounter()*0.8887);
    
    motor_1_distance = QuadDec_M1_GetCounter();//*202.6327/4/3/19
    motor_2_distance = QuadDec_M2_GetCounter();
    
    //QuadDec_M1_SetCounter(0);//reset counter after reading from it
    //QuadDec_M2_SetCounter(0);
    if (usb_output == 1) {
        itoa(motor_1_distance, motor_line_1, 10);
        usb_put_string("motor_1_distance: ");
        usb_put_string(motor_line_1);
        usb_put_string("\n\r");
        
        itoa(motor_2_distance, motor_line_2, 10);
        usb_put_string("motor_2_distance: ");
        usb_put_string(motor_line_2);
        usb_put_string("\n\r"); 
    }
}    
//* ========================================
void check_mode() {
    // checks mode switches to determine mode
    
    if ((mode_switch0_Read() == 0) && (mode_switch1_Read() == 0)) {
        mode = 0;
    } else if ((mode_switch0_Read() == 0) && (mode_switch1_Read() == 1)) {
        mode = 1;
    } else if ((mode_switch0_Read() == 1) && (mode_switch1_Read() == 0)) {
        mode = 2;
    } else if ((mode_switch0_Read() == 1) && (mode_switch1_Read() == 1)) {
        mode = 3;
    }
}
//* ========================================
/* [] END OF FILE */
