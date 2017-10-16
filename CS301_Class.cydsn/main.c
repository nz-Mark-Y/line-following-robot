/* ========================================
 * Group 6 COMPSYS 301 Project
 * Line Following Robot
 *
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
#include <math.h>
#include "defines.h"
#include "vars.h"
#include "Astar.h"
#include "DFS.h"
#include "map.h"
//* ========================================
char rf_string[RXSTRINGSIZE];
char line[BUF_SIZE], test[BUF_SIZE], entry[BUF_SIZE], motor_line_1[BUF_SIZE], motor_line_2[BUF_SIZE];
uint8 usb_buffer[BUF_SIZE];
//* ========================================
int mode = 0;
int count = 0;
int start_count = 0;
int usb_output = 0;
//* ========================================
// Max speed of 69 cm/s
int motor_1_distance = 0;                                                                       // distance travelled in mm
int motor_2_distance = 0;   
int average_distance = 0;
int current_speed = 0;
int target_speed = 48;
int target_distance = 2000;
int motor_1_temp_distance = 0;
int motor_2_temp_distance = 0;
int correction = 20;
//* ========================================
int last_on_line = 1;
// arrays for light sensors
int max_value[] = {0,0,0,0,0,0,0};                                                              // max value sensed by each light sensor 
int min_value[] = {0,0,0,0,0,0,0};                                                              // min value sensed by each light sensor 
int ADC_value[] = {0,0,0,0,0,0,0};                                                              // current ADC value from each light sensor 
int is_under_line[] = {0,0,0,0,0,0,0};                                                          // boolean value for holding if a light sensor is under the line
//* ========================================
// initialising motor speed values
int voltage = 0;
float initial_speed = 127;
int motor_1_default_speed = 127;
int motor_2_default_speed = 127;
int motor_1_speed = 127;
int motor_2_speed = 127;

int straightspeed1 = 70;
int straightspeed2 = 66;


int back_turn_speed = 52;                                                                       // for backwards turning wheel
int forward_turn_speed = 37;                                                                    // for forwards turning wheel
int motor_correction_speed = 25;                                                                // for correction
int motor_backwards_speed = 170;                                                                // for reversing
//* ========================================
// flags for turns
int turning_left = 0;
int turning_right = 0;
int is_reverse = 0;
int is_turning_left = 0;
int is_turning_right = 0;
int has_turned = 0;
int has_turned_left = 0;
int has_turned_right = 0;
int has_gone_straight = 0;
int to_turn_left = 0;
int to_turn_right = 0;

int timer_initial = 0;
int start_ypos = 0;
int counter = 10;
int state = 0;
int next_turn = 0;
int correcting_left = 0;
int correcting_right = 0;
int has_been_in_light = 0;
int timer_to_start = 0;
int turn_array[555];
int turn_max = 555;
int i = 0; //turn array index
//* ========================================
// function definitions
void maze_mode_1();
void maze_straight();
void print_light_sensor_values();
void sensor_is_under_line(int sensorNum);
int get_battery_voltage();
int handle_radio_data();
void usb_put_string(char *s);
void usb_put_char(char c);
void handle_usb();
void turn_left();
void turn_left_slow();
void turn_right();
void turn_right_slow();
void go_straight();
void handle_turns();
void curves_mode();
void turns_mode();
void calculate_distance_travelled();
void travel_straight();
void set_speeds();
void check_mode();
//* ========================================
int main() {
// ----- INITIALIZATIONS ----------
    CYGlobalIntEnable;

    isr_TS_Start();
    isr_TS_Enable();    
    Timer_TS_SetInterruptMode(3);
    /*
    isr_TS_1_Start();
    isr_TS_1_Enable();
    Timer_TS_1_Start();
    Timer_TS_1_Enable();
    Timer_TS_1_SetInterruptMode(3);
    */
    
// ------USB SETUP ----------------    
    USBUART_Start(0,USBUART_5V_OPERATION);  
    RF_BT_SELECT_Write(0);
    
// ------MOTOR SETUP --------------     
    //CONTROL_Write(0b00000011);                                                                  // disable motor    
    //CyDelay(2000);
    
    CONTROL_Write(0b00000000);                                                                  // enable motor
    Clock_PWM_Start();                                                                          // Start clock for PWM
    PWM_1_Start();
    PWM_2_Start();
    PWM_1_WriteCompare(127);                                                                    // move motor1 forward
    PWM_2_WriteCompare(127);
    
// ------ADC SETUP ----------------      
    ADC_Start();
    QuadDec_M1_Start();
    QuadDec_M2_Start();
    
// ------UART_RF Setup------------- 
    USBUART_Start(0,USBUART_5V_OPERATION);
    UART_Start();
    isrRF_RX_Start();
    
    if (mode_switch0_Read() == 1) {
        usb_output = 1;
    }
    
    /*
    int16_t retsteps[555] = { -1 }; //retsteps is the array of steps to reach the target
	int16_t numberOfSteps = astar(1, 13, 5, 5, retsteps);
    */
    
    int16_t retsteps[555] ; //retsteps is the array of steps needed to traverse the map
	int16_t numberOfSteps = dfs(17,13, retsteps);
    
    for (i=0;i<turn_max;i++) {
        turn_array[i] = -1;
    }
    i = 0;
    
    int x1, y1, x2, y2, x3, y3 = 0;
    int j = 0;
    while (retsteps[i+2] != -1) {    
        x1 = retsteps[i] % 19;
        y1 = retsteps[i] / 19;
        x2 = retsteps[i+1] % 19;
        y2 = retsteps[i+1] / 19;
        x3 = retsteps[i+2] % 19;
        y3 = retsteps[i+2] / 19;

        if ((x1 == x2) && (x2 == x3)) {
            if ((y1 == y3) && (y2 != y3)) {
                turn_array[j] = 3;
                j++;  
            } else {
                if ((map[y2][x2+1] == 0) || (map[y2][x2-1] == 0)) {
                    turn_array[j] = 0;
                    j++;
                }
            }
        } else if ((y1 == y2) && (y2 == y3)) {
            if ((x1 == x3) && (x2 != x3)) {
                turn_array[j] = 3;
                j++;      
            } else {
                if ((map[y2+1][x2] == 0) || (map[y2-1][x2] == 0)) {
                    turn_array[j] = 0;
                    j++;
                }   
            }
        } else if ((x1 == x2) && (x2 == x3 + 1)) {
            if (y1 == y2 + 1) {
                turn_array[j] = 1;
                j++;  
            } else if (y1 == y2 - 1) {
                turn_array[j] = 2;
                j++;
            }
        } else if ((x1 == x2) && (x2 == x3 - 1)) {
            if (y1 == y2 + 1) {
                turn_array[j] = 2;
                j++;
            } else if (y1 == y2 - 1) {
                turn_array[j] = 1;
                j++;
            }   
        } else if ((x2 == x3) && (x1 == x2 - 1)) {
            if (y2 == y3 + 1) {
                turn_array[j] = 1;
                j++;  
            } else if (y2 == y3 - 1) {
                turn_array[j] = 2;
                j++;
            }
        } else if ((x2 == x3) && (x1 == x2 + 1)) {
            if (y2 == y3 + 1) {
                turn_array[j] = 2;
                j++;
            } else if (y2 == y3 - 1) {
                turn_array[j] = 1;
                j++;
            }   
        }       
        i++;
    }
    i=0;
    
    while(1) { 
        //set_speeds();
        /*
        if (usb_output == 1) {
            if (turn_array[i] != -1) {
                if (i>-1) {
                    itoa(turn_array[i], line, 10);
                    usb_put_string(line);
                    usb_put_string("\n\r");
                    i++;
                }
                if (i>turn_max) {
                    i = 0;
                }
            }
        }
        */  
        /*    
        if (retsteps[i] != -1) {
            x1 = retsteps[i] % 19;
            y1 = retsteps[i] / 19;

            itoa(x1, line, 10);
            itoa(y1, test, 10);
            usb_put_string(line);
            usb_put_string(":");
            usb_put_string(test);
            usb_put_string("\n\r");  
            i++;
        }
        */
        
        // update each sensor values of max_value, ADC_value and is_under_line
        int m;   
        for (m = 1; m < 7; m++){
            sensor_is_under_line(m);
        }

        if ((is_under_line[1] == 1)  && (is_under_line[2] != 1)) {
            last_on_line = 1;
        } else if ((is_under_line[1] != 1)  && (is_under_line[2] == 1)) {
            last_on_line = 2;
        }
        
        check_mode();
        next_turn = turn_array[i];
        if (mode == 0) {
            maze_mode_1(); 
        } else if (mode == 1) {
            turns_mode();   
        } else if (mode == 2) {
            curves_mode();
        } else if (mode == 3) {
            PWM_1_WriteCompare(127);
            PWM_2_WriteCompare(127);
        }
        /*
        int complete_structure = handle_radio_data(); 
        if (usb_output == 1) {
            if (complete_structure == 1) {
                int8 strength = system_state.rssi;
                int16 xpos = system_state.robot_xpos;
                int16 ypos = system_state.robot_ypos;
                int16 orient = system_state.robot_orientation;     
                   
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
        calculate_distance_travelled();
         */
    }
}

//* ========================================
void maze_mode_1() { // 7.7V
    if (state == 0) { 
        maze_straight();
        
        if (next_turn == 1) { //turn left, only check sensor 3
            if ((is_under_line[3] == 1 && is_under_line[4] == 1) || (is_under_line[3] == 1 && is_under_line[6] == 1)) {
                if ((has_turned_left == 1) || (has_turned_right == 1) || (has_gone_straight == 1)) {
                    
                } else {
                    PWM_1_WriteCompare(127);
                    PWM_2_WriteCompare(127);
                    state = 1;   
                }
            }
        }
        else if (next_turn == 2) {
            if ((is_under_line[5] == 1 && is_under_line[4] == 1) || (is_under_line[5] == 1 && is_under_line[6] == 1)) {
                if ((has_turned_left == 1) || (has_turned_right == 1) || (has_gone_straight == 1)) {
                    
                } else {
                    PWM_1_WriteCompare(127);
                    PWM_2_WriteCompare(127);
                    state = 1;   
                }            
            }
        }
        
        /*
        if (is_under_line[3] || is_under_line[5]) {
            if ((has_turned_left == 1) || (has_turned_right == 1) || (has_gone_straight == 1)) {
                
            } else {
                PWM_1_WriteCompare(127);
                PWM_2_WriteCompare(127);
                state = 1;   
            }
        }
        */
        
        if (((is_under_line[1] != 1) && (is_under_line[2] != 1)) && ((is_under_line[3] != 1) && (is_under_line[5] != 1))) {
            PWM_1_WriteCompare(127);
            PWM_2_WriteCompare(127);
            state = 3;  
        }
        
    } else if (state == 1) {
        PWM_1_WriteCompare(180);
        PWM_2_WriteCompare(175);
        sensor_is_under_line(3);
        sensor_is_under_line(5);
        if (is_under_line[3] || is_under_line[5]) {
            if (is_under_line[3] == 1) {
                to_turn_left = 1;
            } else if (is_under_line[5] == 1) {
                to_turn_right = 1;
            }
            PWM_1_WriteCompare(127);
            PWM_2_WriteCompare(127);
            state = 2;
        }
    } else if (state == 2) {
        if (next_turn == 0) {
            maze_straight();
            has_gone_straight = 1;
            i++;
            if (i > turn_max) {
                i = 0;
            }
            Timer_TS_WritePeriod(40);
            Timer_TS_Start();
            state = 0;
        } else if (next_turn == 1) { // Left turn
            if (is_under_line[1] == 1 || is_under_line[2] == 1) {
                if (has_been_in_light == 1) {
                    has_been_in_light = 0;
                    state = 0;
                    has_turned_left = 1;
                    i++;
                    if (i > turn_max) {
                        i = 0;
                    }
                    Timer_TS_WritePeriod(150);
                    Timer_TS_Start();
                } else {
                    turn_left();
                }
            } else if (is_under_line[1] == 0 && is_under_line[2] == 0) {
                turn_left();
                has_been_in_light = 1;            
            }
        } else if (next_turn == 2) { // Right turn
            if (is_under_line[1] == 1 || is_under_line[2] == 1) {
                if (has_been_in_light == 1) {
                    has_been_in_light = 0;
                    state = 0;
                    has_turned_right = 1;
                    i++;
                    if (i > turn_max) {
                        i = 0;
                    }
                    Timer_TS_WritePeriod(150);
                    Timer_TS_Start();
                } else {
                    turn_right();
                }
            } else if (is_under_line[1] == 0 && is_under_line[2] == 0) {
                turn_right();
                has_been_in_light = 1;            
            }
        } else if (next_turn == 3) { // u turn
            if (is_under_line[1] == 1 || is_under_line[2] == 1) {
                if (has_been_in_light == 1) {
                    if (to_turn_left == 1) {
                        turn_left();
                    }
                    else if (to_turn_right == 1) {
                        turn_right();
                    }
                    has_been_in_light = 2;
                } else if (has_been_in_light == 3) {
                    
                    has_been_in_light = 0;
                    state = 0;
                    has_turned_right = 1;
                    i++;
                    if (i > turn_max) {
                        i = 0;
                    }
                    Timer_TS_WritePeriod(150);
                    Timer_TS_Start();
                } else {
                    if (to_turn_left == 1) {
                        turn_left();
                    } else if (to_turn_right == 1) {
                        turn_right();
                    }
                }
            } else if (is_under_line[1] == 0 && is_under_line[2] == 0) {
                    if (to_turn_left == 1) {
                        turn_left();
                    } else if (to_turn_right == 1) {
                        turn_right();
                    }
                if (has_been_in_light == 2) {
                    has_been_in_light = 3;
                } else if (has_been_in_light == 3) {
                    has_been_in_light = 3;
                } else {
                    has_been_in_light = 1;  
                }
            }
        }        
    } else if (state == 3) {
        if (is_under_line[1] == 1 || is_under_line[2] == 1) {
            state = 0;
        } else {
            sensor_is_under_line(3);
            sensor_is_under_line(5);
            if (is_under_line[3] || is_under_line[5]) {
                if ((has_turned_left == 1) || (has_turned_right == 1) || (has_gone_straight == 1)) {
                    
                } else {
                    PWM_1_WriteCompare(127);
                    PWM_2_WriteCompare(127);
                    state = 2;   
                    return;
                }
            }
            if (last_on_line == 1) {
                turn_left_slow();    
            } else {
                turn_right_slow();    
            }
        }
    } else {
       
    }
}
//* ========================================
void maze_straight() {
    if ((is_under_line[1] == 1) && (is_under_line[2] == 1)) {
        if (correcting_left == 1) {
            PWM_1_WriteCompare(straightspeed1);
            PWM_2_WriteCompare(straightspeed2 + 25);
            correcting_left = 0;
        } else if (correcting_right == 1) {
            PWM_1_WriteCompare(straightspeed1);
            PWM_2_WriteCompare(straightspeed2 - 25);
            correcting_right = 0;
        } else {
            PWM_1_WriteCompare(straightspeed1);
            PWM_2_WriteCompare(straightspeed2);
        }
    } else if (is_under_line[1] == 1 && is_under_line[2] == 0) {  
        PWM_1_WriteCompare(straightspeed1);
        PWM_2_WriteCompare(straightspeed2 - 15);
        correcting_left = 1;
    } else if (is_under_line[1] == 0 && is_under_line[2] == 1) {
        PWM_1_WriteCompare(straightspeed1);
        PWM_2_WriteCompare(straightspeed2 + 15);
        correcting_right = 1;
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
    
    ADC_StartConvert();                                                                         // start conversion
    int i = 0;
    ADC_value[sensorNum] = 0;
    max_value[sensorNum] = 0;
    min_value[sensorNum] = 4096;
    for (i = 0; i < 70; i++) {                                                                  // 70 data points, to cover one period of the waveform            
        while(1) {
            if (ADC_IsEndConversion(ADC_RETURN_STATUS) != 0) {                                  // when conversion completes
                ADC_value[sensorNum] = ADC_GetResult16(sensorNum);                              // get result from channel
                if (ADC_value[sensorNum] > max_value[sensorNum]) {                              // take max over that period
                    max_value[sensorNum] = ADC_value[sensorNum];
                } else if(ADC_value[sensorNum] < min_value[sensorNum]) {
                    min_value[sensorNum] = ADC_value[sensorNum];                                // take min over that period
                }
                break;
            }
        }               
    }
  
    if (sensorNum == 1) {
        max_value[1] = ((max_value[1] - 1638) * 1.6667);
        min_value[1] = (min_value[1] - 1638) * 1.6667;      
    }
    
    if ((max_value[sensorNum]-min_value[sensorNum]) < 400) {
        is_under_line[sensorNum] = 1;                                                           // under the line
    } else { 
        is_under_line[sensorNum] = 0;                                                           // not under the line
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
    int voltage = (ADC_value*8*1000) / 4096;  
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
                if (((c == CHAR_BACKSP) || (c == CHAR_DEL) ) && (usbBufCount > 0))
                    usbBufCount--;
                else {
                    if (usbBufCount > (BUF_SIZE-2)) {
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
    
    PWM_1_WriteCompare(127 + 40);                                               // move motor1 backward
    PWM_2_WriteCompare(127 - 45);                                               // move motor2 forward  
}
//* ========================================
void turn_left_slow(){
    // function to turn the robot to the left
    // change to reasonable turning speed for both motors
    
    PWM_1_WriteCompare(127 + 30);                                               // move motor1 backward
    PWM_2_WriteCompare(127 - 35);                                               // move motor2 forward  
}
//* ========================================
void turn_right() {
    // function to turn the robot to the right
    // change to reasonable turning speed for both motors

    PWM_1_WriteCompare(127 - 40);                                               // move motor 1 forward
    PWM_2_WriteCompare(127 + 45);                                              // move motor 2 backward 
}
//* ========================================
void turn_right_slow() {
    // function to turn the robot to the right
    // change to reasonable turning speed for both motors

    PWM_1_WriteCompare(127 - 30);                                               // move motor 1 forward
    PWM_2_WriteCompare(127 + 35);                                              // move motor 2 backward 
}
//* ========================================
void go_straight() {
    // function to make the robot go straight
    // change to default speed for both motors
    
    motor_1_speed = motor_1_default_speed;
    motor_2_speed = motor_2_default_speed;
    PWM_1_WriteCompare(motor_1_default_speed);
    PWM_2_WriteCompare(motor_2_default_speed);
}
//* ========================================
void turns_mode() { 
    // function to handle turns mode
    if (turning_left == 1) {
        if (is_under_line[1] == 1 || is_under_line[2] == 1) {
            has_turned = 1;
            Timer_TS_Start();
            Timer_TS_Enable();
            turning_left = 0;
            motor_1_speed = motor_1_default_speed;
            motor_2_speed = motor_2_default_speed + motor_correction_speed;
            PWM_1_WriteCompare(motor_1_speed);
            PWM_2_WriteCompare(motor_2_speed);
        } else {   
            turn_left();
        }
    }
    else if (turning_right == 1) {
        if (is_under_line[1] == 1 || is_under_line[2] == 1) {
            has_turned = 1;
            Timer_TS_Start();
            Timer_TS_Enable();
            turning_right = 0;
            motor_1_speed = motor_1_default_speed + motor_correction_speed;
            motor_2_speed = motor_2_default_speed;       
            PWM_1_WriteCompare(motor_1_speed);
            PWM_2_WriteCompare(motor_2_speed);
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
        motor_1_speed = motor_1_default_speed + motor_correction_speed;
        motor_2_speed = motor_2_default_speed;       
        PWM_1_WriteCompare(motor_1_speed);
        PWM_2_WriteCompare(motor_2_speed);
    }
    else if (is_under_line[1] == 0 && is_under_line[2] == 1) {
        motor_1_speed = motor_1_default_speed;
        motor_2_speed = motor_2_default_speed + motor_correction_speed;
        PWM_1_WriteCompare(motor_1_speed);
        PWM_2_WriteCompare(motor_2_speed);
    }
    
    // handle 90 degree turns
    //CASE 3
    if (is_under_line[1] == 0 && is_under_line[2] == 0 && is_under_line[3] == 1) {
        if (has_turned == 1) {
            motor_1_speed = motor_backwards_speed;                                              // slow reverse
            motor_2_speed = motor_backwards_speed;                                              // slow reverse
            PWM_1_WriteCompare(motor_1_speed);
            PWM_2_WriteCompare(motor_2_speed);       
        }
        turning_left = 1;    
    } 
    //CASE 4
    else if (is_under_line[1] == 0 && is_under_line[2] == 0 && is_under_line[5] == 1) {
        if (has_turned == 1) {
            motor_1_speed = motor_backwards_speed;                                              // slow reverse
            motor_2_speed = motor_backwards_speed;                                              // slow reverse
            PWM_1_WriteCompare(motor_1_speed);
            PWM_2_WriteCompare(motor_2_speed);       
        }        
        turning_right = 1;
    }
}
//* ========================================
void curves_mode() {  
    // function to handle curves mode
 
    //CASE 7
    if (is_under_line[1] == 1 && is_under_line[2] == 1) {                                       // both sensors are on the line
        turning_left = 0;
        turning_right = 0;
        go_straight();                                                                          // move forward as per usual
    }
    //CASE 10
    else if (is_under_line[1] == 1 && is_under_line[2] == 0) {                                  // sensor 1 is on the line, sensor 2 is off the line    
        motor_1_speed = motor_1_default_speed + motor_correction_speed;                         // decrease motor1 speed
        motor_2_speed = motor_2_default_speed;                                                  // increase motor2 speed        
        PWM_1_WriteCompare(motor_1_speed);
        PWM_2_WriteCompare(motor_2_speed);
    }
    //CASE 9
    else if (is_under_line[1] == 0 && is_under_line[2] == 1) {                                  // sensor 1 is off the line, sensor 2 is on the line
        motor_1_speed = motor_1_default_speed;                                                  // increase motor1 speed
        motor_2_speed = motor_2_default_speed + motor_correction_speed;                         // decrease motor2 speed
        PWM_1_WriteCompare(motor_1_speed);
        PWM_2_WriteCompare(motor_2_speed);
    }
    //CASE 8
    else if (is_under_line[1] == 0 && is_under_line[2] == 0) {                                  // both front sensors off the line
        motor_1_speed = motor_backwards_speed;                                                  // slow reverse
        motor_2_speed = motor_backwards_speed;                                                  // slow reverse
        PWM_1_WriteCompare(motor_1_speed);
        PWM_2_WriteCompare(motor_2_speed);
    }
}
//* ========================================
void calculate_distance_travelled(){
    // uses quadrature reading to determine the distance travelled by robot's two wheels
    motor_1_distance = motor_1_distance + fabs((float)(QuadDec_M1_GetCounter()*0.8887));        //append distance value to global distance variable
    motor_2_distance = motor_2_distance + fabs((float)(QuadDec_M2_GetCounter()*0.8887));        // *202.6327/4/3/19
    QuadDec_M1_SetCounter(0);                                                                   // reset counter after reading from it
    QuadDec_M2_SetCounter(0);
}
//* ========================================
void travel_straight() {
    // code for benchmark test 5
    int small_correction = 1;
    if (counter % 10 == 0 && counter < 31) {
        straightspeed2 = straightspeed2 + small_correction;
    }
    if (counter % 5 == 0 && counter > 31 && counter < 46) {
        straightspeed2 = straightspeed2 - small_correction;
    }
    if (current_speed != 0) {
        if (current_speed < target_speed) {
            straightspeed1 = straightspeed1 - correction;
            straightspeed2 = straightspeed2 - correction;
        } else if (current_speed > target_speed) {
            straightspeed1 = straightspeed1 + correction;
            straightspeed2 = straightspeed2 + correction;
        } 
        correction = correction / 2;
        if (correction < 1) {
            correction = 1;
        }
    }
    counter = counter + 1;
    
    if (mode_switch1_Read() == 1) {
        if (counter == 1) {
            start_ypos = system_state.robot_ypos;
        } else {
            if ((target_distance - 20) <= system_state.robot_ypos - start_ypos) {
                PWM_1_WriteCompare(127);                                                                // stop
                PWM_2_WriteCompare(127);  
            } else {
                PWM_1_WriteCompare(straightspeed1);                                                                
                PWM_2_WriteCompare(straightspeed2);
            }
        }
    } else {
        if ((target_distance - 20) <= average_distance) {
            PWM_1_WriteCompare(127);                                                                    // stop
            PWM_2_WriteCompare(127);  
        } else {
            PWM_1_WriteCompare(straightspeed1);                                                                
            PWM_2_WriteCompare(straightspeed2);
        }
    }
}
//* ========================================
void set_speeds() {
    // sets initial and turning speeds as functions of battery voltage
    
    voltage = get_battery_voltage();
    if (voltage < 7300) {
        initial_speed = 127;
    } else {
        initial_speed = (float)((float)((float)voltage / 160.0) + (float)(135.0 / 4.0));
        if (voltage < 7700) {
            motor_correction_speed = 26;
        } else if (voltage < 8100) {
            motor_correction_speed = 25;
        } else {
            motor_correction_speed = 24;
        }
        if (voltage < 7600) {
            forward_turn_speed = 39;
        } else if (voltage < 7900) {
            forward_turn_speed = 38;
        } else if (voltage < 8200) {
            forward_turn_speed = 37;
        } else {
            forward_turn_speed = 36;
        }
        if (voltage < 7540) {
            back_turn_speed = 55;
        } else if (voltage < 7780) {
            back_turn_speed = 54;
        } else if (voltage < 8020) {
            back_turn_speed = 53;
        } else if (voltage < 8260) {
            back_turn_speed = 52;
        } else {
            back_turn_speed = 51;
        }
    }
    motor_1_default_speed = (int) initial_speed;
    motor_2_default_speed = (int) initial_speed; 
    motor_backwards_speed = (float)((float)((float)voltage / -150.0) + (float)(656.0 / 3.0));
}
//* ========================================
void check_mode() {
    // checks mode switches to determine mode
    
    if(mode_switch2_Read() == 0){
        if(mode_switch3_Read() == 0){
            mode = 0;
        } else if(mode_switch3_Read() == 1) {
            mode = 1;
        }
    } else if(mode_switch2_Read() == 1) {
        if(mode_switch3_Read() == 0){
            mode = 2;
        } else if(mode_switch3_Read() == 1) {
            mode = 3;
        }
    }
}
//* ========================================
/* [] END OF FILE */
