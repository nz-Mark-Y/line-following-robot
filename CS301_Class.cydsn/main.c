/* ========================================
 * Group 6 COMPSYS 301 Project
 * Line Following Robot
 *
 * Copyright Univ of Auckland, 2016
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF University of Auckland.
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
char line[BUF_SIZE], test[BUF_SIZE], entry[BUF_SIZE];
uint8 usb_buffer[BUF_SIZE];
//* ========================================
int mode = 0;
int count = 0;
int start_count = 0;
//* ========================================
int motor_1_distance = 0;                                                                       // distance travelled in mm
int motor_2_distance = 0;   
int average_distance = 0;
int current_speed = 0;
int target_speed = 48;
int target_distance = 2000;
int motor_1_temp_distance = 0;
int motor_2_temp_distance = 0;
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
int straightspeed1 = 77;
int straightspeed2 = 74;
//* ========================================
// flags for turns
volatile int has_turned;
int to_turn_left = 0;
int to_turn_right = 0;
int dead_end = 0;

int state = 0;
int next_turn = 0;                                                                              // the next turn the robot will complete. 0 = straight, 1 = left, 2 = right, 3 = uturn
int correcting_left = 0;
int correcting_right = 0;
int has_been_in_light = 0;
int turn_max = 555;

// counters
int ab = 0;
int light_counter = 0;
int u_turn_counter = 0;
//* ========================================
// function definitions
void maze_mode_1();
void maze_straight();
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
void calculate_distance_travelled();
void check_mode();
//* ========================================
int main() {
// ----- INITIALIZATIONS ----------
    CYGlobalIntEnable;
    
// ------USB SETUP ----------------    
    USBUART_Start(0,USBUART_5V_OPERATION);  
    RF_BT_SELECT_Write(0);
    
// ------MOTOR SETUP --------------       
    CONTROL_Write(0b00000000);                                                                  // enable motor
    Clock_PWM_Start();                                                                          // Start clock for PWM
    PWM_1_Start();
    PWM_2_Start();
    PWM_1_WriteCompare(127);                                                                    // move motor1 forward
    PWM_2_WriteCompare(127);
    
// ------ADC SETUP ----------------      
    ADC_Start();                                                                                // start adc     
    QuadDec_M1_Start();                                                                         // start quadrature decoders
    QuadDec_M2_Start();
    
// ------UART_RF Setup------------- 
    UART_Start();
    isrRF_RX_Start();
    
    int i = 0; 
    has_turned = 0;
    int turn_array[555];
    for (i=0;i<turn_max;i++) {                                                                  // set up arrays for maze traversing algorithms
        turn_array[i] = -1;                                                                     // initialise these arrays to -1 
    }  
    
    int16_t retsteps[555];                                                                      //retsteps is the returned array of steps needed to traverse the map
    for (i=0;i<555;i++) {
        retsteps[i] = -1;
    }
    
    int16_t totalsteps[1000];                                                                   // totalsteps is the total array of steps needed to traverse the map
    for (i=0;i<1000;i++) {
        totalsteps[i] = -1;
    }
    i = 0;
    
    int x1, y1, x2, y2, x3, y3 = 0;                                                             // coordinate analysis. extracts required turns from coordinate list  
    int j = 0;
    int l,a;
    int m = 0;
    int16_t numberOfSteps = 0;
    check_mode();                                                                               // check mode switches
    if (mode == 0) {
	    dfs(start_X, start_Y, totalsteps);                                                      // run dfs from start position
        while (totalsteps[i+2] != -1) {                                                         // for every coordinate, in groups of three
            x1 = totalsteps[i] % 19;                                                            // pull out coordinate data
            y1 = totalsteps[i] / 19;
            x2 = totalsteps[i+1] % 19;
            y2 = totalsteps[i+1] / 19;
            x3 = totalsteps[i+2] % 19;
            y3 = totalsteps[i+2] / 19;

            if ((x1 == x2) && (x2 == x3)) {                                                     // same x values for all three coordinates
                if ((y1 == y3) && (y2 != y3)) {                                                 // if first y value = third y value then u turn
                    turn_array[j] = 3;  
                    j++;  
                } else {
                    if ((map[y2][x2+1] == 0) || (map[y2][x2-1] == 0)) {                         // if coordinates in a line then straight ahead
                        turn_array[j] = 0;
                        j++;
                    }
                }
            } else if ((y1 == y2) && (y2 == y3)) {                                              // same y values for all three coordinates
                if ((x1 == x3) && (x2 != x3)) {                                                 // if first x value = third x value then u turn
                    turn_array[j] = 3;
                    j++;      
                } else {
                    if ((map[y2+1][x2] == 0) || (map[y2-1][x2] == 0)) {                         // if coordinates in a line then straight ahead
                        turn_array[j] = 0;
                        j++;
                    }   
                }
            } else if ((x1 == x2) && (x2 == x3 + 1)) {                                          // if x coordinate decreases after two the same
                if (y1 == y2 + 1) {                                                             // check y coordinate
                    turn_array[j] = 1;                                                          // left turn required
                    j++;  
                } else if (y1 == y2 - 1) {
                    turn_array[j] = 2;                                                          // right turn required
                    j++;
                }
            } else if ((x1 == x2) && (x2 == x3 - 1)) {                                          // if x coordinate decreases after two the same    
                if (y1 == y2 + 1) {                                                             // check y coordinate
                    turn_array[j] = 2;                                                          // right turn required
                    j++;
                } else if (y1 == y2 - 1) {
                    turn_array[j] = 1;                                                          // left turn required
                    j++;
                }   
            } else if ((x2 == x3) && (x1 == x2 - 1)) {                                          // if x coordinate is two the same after increase
                if (y2 == y3 + 1) {
                    turn_array[j] = 1;                                                          // left turn required
                    j++;                    
                } else if (y2 == y3 - 1) {
                    turn_array[j] = 2;                                                          // right turn required
                    j++;
                }
            } else if ((x2 == x3) && (x1 == x2 + 1)) {                                          // if x coordinate is two the same after decrease
                if (y2 == y3 + 1) {
                    turn_array[j] = 2;                                                          // right turn required
                    j++;
                } else if (y2 == y3 - 1) {
                    turn_array[j] = 1;                                                          // left turn required
                    j++;
                }   
            }       
            i++;
        } 
    } else if (mode == 1) {
        int k = 0;
        int startx = start_X;                                                                   // pull start coordinate from map.h     
        int starty = start_Y;
        for (k=0; k<5; k++) {                                                                   // repeat for number of food pellets
            numberOfSteps = astar(startx, starty, food_list[k][0], food_list[k][1], retsteps);  // run astar given start coordinates and food coordinates
            startx = food_list[k][0];                                                           // set new start coordinates as previous food coordinate
            starty = food_list[k][1];

            for(a = 0; a < numberOfSteps; a++){                                                 // copy from returned array into total array
                totalsteps[m] = retsteps[a];
                m++;
            }   
            m -=1;                                                                              // truncate off last coordinate to prevent duplication
            
            for (l=0;l<555;l++) {                                                               // reinitialise returned array
                retsteps[l] = -1;
            }
        } 
        while (totalsteps[i+2] != -1) {                                                         // for every coordinate, in groups of three
            x1 = totalsteps[i] % 19;                                                            // pull out coordinate data
            y1 = totalsteps[i] / 19;
            x2 = totalsteps[i+1] % 19;
            y2 = totalsteps[i+1] / 19;
            x3 = totalsteps[i+2] % 19;
            y3 = totalsteps[i+2] / 19;

            if ((x1 == x2) && (x2 == x3)) {                                                     // same x values for all three coordinates
                if ((y1 == y3) && (y2 != y3)) {                                                 // if first y value = third y value then u turn
                    turn_array[j] = 3;  
                    j++;  
                } else {
                    if ((map[y2][x2+1] == 0) || (map[y2][x2-1] == 0)) {                         // if coordinates in a line then straight ahead
                        turn_array[j] = 0;
                        j++;
                    }
                }
            } else if ((y1 == y2) && (y2 == y3)) {                                              // same y values for all three coordinates
                if ((x1 == x3) && (x2 != x3)) {                                                 // if first x value = third x value then u turn
                    turn_array[j] = 3;
                    j++;      
                } else {
                    if ((map[y2+1][x2] == 0) || (map[y2-1][x2] == 0)) {                         // if coordinates in a line then straight ahead
                        turn_array[j] = 0;
                        j++;
                    }   
                }
            } else if ((x1 == x2) && (x2 == x3 + 1)) {                                          // if x coordinate decreases after two the same
                if (y1 == y2 + 1) {                                                             // check y coordinate
                    turn_array[j] = 1;                                                          // left turn required
                    j++;  
                } else if (y1 == y2 - 1) {
                    turn_array[j] = 2;                                                          // right turn required
                    j++;
                }
            } else if ((x1 == x2) && (x2 == x3 - 1)) {                                          // if x coordinate decreases after two the same    
                if (y1 == y2 + 1) {                                                             // check y coordinate
                    turn_array[j] = 2;                                                          // right turn required
                    j++;
                } else if (y1 == y2 - 1) {
                    turn_array[j] = 1;                                                          // left turn required
                    j++;
                }   
            } else if ((x2 == x3) && (x1 == x2 - 1)) {                                          // if x coordinate is two the same after increase
                if (y2 == y3 + 1) {
                    turn_array[j] = 1;                                                          // left turn required
                    j++;                    
                } else if (y2 == y3 - 1) {
                    turn_array[j] = 2;                                                          // right turn required
                    j++;
                }
            } else if ((x2 == x3) && (x1 == x2 + 1)) {                                          // if x coordinate is two the same after decrease
                if (y2 == y3 + 1) {
                    turn_array[j] = 2;                                                          // right turn required
                    j++;
                } else if (y2 == y3 - 1) {
                    turn_array[j] = 1;                                                          // left turn required
                    j++;
                }   
            }       
            i++;
        }
    }

    ab=0;                                                                                       // set turn counter to 0
    CyDelay(3000);                                                                              // have a 3 second delay to ensure calculations are complete
    
    while(1) {                                                                                  // main while loop of function
        if (dead_end != 1) {
            next_turn = turn_array[ab];                                                         // update the turn array with next turn if not a dead end
        } 

        /*if (totalsteps[ab] != -1) {
            if (ab>-1) {
                x1 = totalsteps[ab] % 19;
                y1 = totalsteps[ab] / 19;

                itoa(x1, line, 10);
                itoa(y1, test, 10);
                usb_put_string(line);
                usb_put_string(":");
                usb_put_string(test);
                usb_put_string("\n\r");  
            }
            ab++;  
        }*/
        
        
        sensor_is_under_line(6);                                                                // update each sensor values of max_value, ADC_value and is_under_line
        sensor_is_under_line(4);
        sensor_is_under_line(1);
        sensor_is_under_line(2);
        sensor_is_under_line(3);
        sensor_is_under_line(5);
        
        maze_mode_1();                                                                          // maze logic

        if ((is_under_line[1] == 1)  && (is_under_line[2] != 1)) {                              // check which of 1 or 2 was the last on the line
            last_on_line = 1;
        } else if ((is_under_line[1] != 1)  && (is_under_line[2] == 1)) {
            last_on_line = 2;
        }

    }
}
//* ========================================
void maze_mode_1() { // voltage range of 8.3 - 7.8V, ideal voltage is 8.1V
    // the maze algorithm to handle all intersections, dead ends, and corrections when both front light sensors are off the line
    // utilises maze_straight() for straight ahead sections

    if (state == 0) {                                                                           // straight ahead state 
        if (has_turned == 0) {                                                                  // if robot has turned within a certain time period, ignore side sensors
            if (next_turn == 1) {                                                               // if next turn is a left, only check sensor 3
                if ((is_under_line[3] == 1 && is_under_line[4] == 1) || (is_under_line[3] == 1 && is_under_line[6] == 1)) {                    
                    PWM_1_WriteCompare(127);
                    PWM_2_WriteCompare(127);
                    state = 1;                                                                  // stop and go to reverse state
                    u_turn_counter = 0;
                    return;                    
                }
            } else if (next_turn == 2) {                                                        // if next turn is a right, only check sensor 5
                if ((is_under_line[5] == 1 && is_under_line[4] == 1) || (is_under_line[5] == 1 && is_under_line[6] == 1)) {                   
                    PWM_1_WriteCompare(127);
                    PWM_2_WriteCompare(127);
                    state = 1;                                                                  // stop and go to reverse state
                    u_turn_counter = 0;
                    return;      
                }
            } else if (next_turn == 3) {                                                        // if next turn is a u turn, check either
                if ((is_under_line[5] == 1) || (is_under_line[3] == 1)) {                     
                    PWM_1_WriteCompare(127);
                    PWM_2_WriteCompare(127);
                    u_turn_counter = 0;
                    state = 1;                                                                  // stop and go to reverse state
                    u_turn_counter = 0;
                    return;
                }
            } else if (next_turn == 0) {                                                        // if next turn is a straight, check either
                if (is_under_line[5] == 1 || is_under_line[3] == 1) {  
                    state = 2;                                                                  // go to turning state
                    u_turn_counter = 0;
                    return;                  
                }
            } 
        }
        if (((is_under_line[1] != 1) && (is_under_line[2] != 1)) && ((is_under_line[3] != 1) && (is_under_line[5] != 1))) {     // correction or u turn required
            if ((has_turned == 0) && (next_turn == 3) && (is_under_line[6] == 1)) {                  // u turn criteria (for end of line)
                if (u_turn_counter > 2) {
                    PWM_1_WriteCompare(127);
                    PWM_2_WriteCompare(127);
                    next_turn = 2;                                                                                                  // stop and set flag for dead end, then go to reverse state
                    dead_end = 1;
                    state = 1;
                    u_turn_counter = 0;
                    return;
                } else {
                    u_turn_counter++;
                }
            } else {                                                                                                            // else correction required
                PWM_1_WriteCompare(127);
                PWM_2_WriteCompare(127);
                state = 3;                                                                                                      // stop and go to correction state
                return;
            }
        }
        
        maze_straight();                                                                        // straight ahead
        
        if ((is_under_line[3] == 1) || (is_under_line[5] == 1)) {                               // reset light counter if a side sensor is under a line
            light_counter = 0;
        }
        
        if ((is_under_line[3] == 0) && (is_under_line[5] == 0) && (has_turned == 1)) {          // if 6 cycles of 3 and 5 under light then reset turning counter
            light_counter++;
            if (light_counter > 4) {
                light_counter = 0;
                has_turned = 0;
            }
        }    
    } else if (state == 1) {                                                                    // stop and reverse State
        if (has_turned == 1) {                                                                  // if turn recently made, return to state 0
            state = 0;
            return;
        }
        PWM_1_WriteCompare(175);                                                                // reverse motors
        PWM_2_WriteCompare(170);
        if (is_under_line[3] || is_under_line[5]) {                                             // wait for side sensors to be under line, then go onto the turning state
            if (is_under_line[3] == 1) {
                to_turn_left = 1;
            } else {
                to_turn_left = 0;
            }
            PWM_1_WriteCompare(127);
            PWM_2_WriteCompare(127);
            state = 2;            
        }  
        if ((is_under_line[4] == 1) && (dead_end == 1)) {                                       // if at a dead end, then just wait for sensor 4 before going on to the turning state
            PWM_1_WriteCompare(127);
            PWM_2_WriteCompare(127);
            state = 2;    
        }
    } else if (state == 2) {                                                                    // turning state
        if (has_turned == 1) {                                                                  // if turn recently made, return to state 0
            state = 0;
            return;
        }
        if (next_turn == 0) {                                                                   // straight ahead            
            state = 0;                                                                          // reset state, counters, increment turn array
            light_counter = 0;
            has_turned = 1;
            ab++;
            if (ab > turn_max) {
                ab = 0;
            }       
        } else if (next_turn == 1) {                                                            // left turn
            if (is_under_line[1] == 1 || is_under_line[2] == 1) {                               // turn until front sensors in light, and flag set, then turn is finished
                if (has_been_in_light == 1) {
                    has_been_in_light = 0;
                    state = 0;                                                                  // reset state, counters, increment turn array
                    light_counter = 0;
                    has_turned = 1;
                    ab++;
                    if (ab > turn_max) {
                        ab = 0;
                    } 
                } else {
                    turn_left();
                }
            } else if (is_under_line[1] == 0 && is_under_line[2] == 0) {                        // turn until front light sensors in light, set flag
                turn_left();
                has_been_in_light = 1;            
            }
        } else if (next_turn == 2) {                                                            // right turn
            if (is_under_line[1] == 1 || is_under_line[2] == 1) {                               // turn until front sensors in light, and flag set, then turn is finished
                if (has_been_in_light == 1) {
                    dead_end = 0;
                    has_been_in_light = 0;
                    state = 0;                                                                  // reset state, counters, increment turn array
                    light_counter = 0;
                    has_turned = 1;
                    ab++;
                    if (ab > turn_max) {
                        ab = 0;
                    }
                } else {
                    turn_right();
                }
            } else if (is_under_line[1] == 0 && is_under_line[2] == 0) {                        // turn until front light sensors in light, set flag
                turn_right();
                has_been_in_light = 1;            
            }
        } else if (next_turn == 3) {                                                            // u turn
            if (is_under_line[1] == 1 || is_under_line[2] == 1) {                               // need to go from line, to light, to line, to light, to line again to complete turn
                if (has_been_in_light == 1) {
                    if (to_turn_left == 1) {                                                    // turn left or right based on which sensor triggered
                        turn_left();
                    } else {
                        turn_right();
                    }
                    has_been_in_light = 2;
                } else if (has_been_in_light == 3) {                                            // on line with final flag set = turn completed
                    has_been_in_light = 0;
                    to_turn_left = 0;
                    state = 0;                                                                  // reset state, counters, increment turn array
                    light_counter = 0;
                    has_turned = 1;                    
                    ab++;
                    if (ab > turn_max) {
                        ab = 0;
                    }
                } else {
                    if (to_turn_left == 1) {                                                    // turn left or right based on which sensor triggered
                        turn_left();
                    } else {
                        turn_right();
                    }
                }
            } else if (is_under_line[1] == 0 && is_under_line[2] == 0) {
                if (to_turn_left == 1) {                                                        // turn left or right based on which sensor triggered
                    turn_left();
                } else {
                    turn_right();
                }
                if (has_been_in_light == 2) {                                                   // counting times under the light
                    has_been_in_light = 3;
                } else if (has_been_in_light == 3) {
                    has_been_in_light = 3;
                } else {
                    has_been_in_light = 1;  
                }
            }  
        }
    } else if (state == 3) {                                                                    // correction state
        if (is_under_line[1] == 1 || is_under_line[2] == 1) {                                   // correction complete
            state = 0;
        } else {
            if (is_under_line[3] || is_under_line[5]) {                                         // check for intersections
                if (has_turned == 0) {
                    PWM_1_WriteCompare(127);
                    PWM_2_WriteCompare(127);
                    state = 2;   
                    return;
                }
            }
            if (last_on_line == 1) {                                                            // correct left or right based on which sensor was last under the line
                turn_left_slow();    
            } else {
                turn_right_slow();    
            }
        }
    }
}
//* ========================================
void maze_straight() {
    // function which handles the straight line motion for the robot travelling through the maze
    if ((is_under_line[1] == 1) && (is_under_line[2] == 1)) {
        if (correcting_left == 1) {                                                             // if a left correction has been made
            PWM_1_WriteCompare(straightspeed1);
            PWM_2_WriteCompare(straightspeed2 + 25);                                            // anti correct to remain on the line
            correcting_left = 0;
        } else if (correcting_right == 1) {                                                     // if a right correction has been made
            PWM_1_WriteCompare(straightspeed1);
            PWM_2_WriteCompare(straightspeed2 - 25);                                            // anti correct to remain on the line
            correcting_right = 0;   
        } else {                                                                                // otherwise just go straight
            PWM_1_WriteCompare(straightspeed1);
            PWM_2_WriteCompare(straightspeed2);
        }
    } else if (is_under_line[1] == 1 && is_under_line[2] == 0) {                                // check for if a correction is requried
        PWM_1_WriteCompare(straightspeed1);
        PWM_2_WriteCompare(straightspeed2 - 15);
        correcting_left = 1;
    } else if (is_under_line[1] == 0 && is_under_line[2] == 1) {                                // check for if a correction is required
        PWM_1_WriteCompare(straightspeed1);
        PWM_2_WriteCompare(straightspeed2 + 15);
        correcting_right = 1;
    }
}
//* ========================================
void sensor_is_under_line(int sensorNum) {
    // checks if a light sensor is under a line
    
    ADC_StartConvert();                                                                         // start conversion
    int p = 0;
    ADC_value[sensorNum] = 0;
    max_value[sensorNum] = 0;
    min_value[sensorNum] = 4096;
    for (p = 0; p < 70; p++) {                                                                  // 70 data points, to cover one period of the waveform            
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
    PWM_2_WriteCompare(127 - 43);                                               // move motor2 forward  
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
    PWM_2_WriteCompare(127 + 43);                                              // move motor 2 backward 
}
//* ========================================
void turn_right_slow() {
    // function to turn the robot to the right
    // change to reasonable turning speed for both motors

    PWM_1_WriteCompare(127 - 30);                                               // move motor 1 forward
    PWM_2_WriteCompare(127 + 35);                                              // move motor 2 backward 
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
