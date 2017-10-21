/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <stdint.h>
#include <stdio.h>

#define WIDTH 19
#define HEIGHT 15
#define SIZE 285

void addStep(int16_t *retsteps);
void recursiveDFSCall(int16_t* retsteps);
void dfs(int16_t startlocation_x, int16_t startlocation_y, int16_t *retsteps);
/* [] END OF FILE */
