/*
	Group 6 Compsys 301
*/
#include <stdint.h>
#include <stdio.h>
#include "DFS.h"
extern int16_t map[15][19];

int16_t numOfSteps;
int16_t currentX, currentY;
int16_t traversal_map[15][19];

void dfs(int16_t startlocation_x, int16_t startlocation_y, int16_t *retsteps){	
	
	int16_t i, j;
	for (i = 0; i < HEIGHT; i++){
		for (j = 0; j < WIDTH; j++){
			traversal_map[i][j] = map[i][j];	//copy map to traversal map
		}	
	}
		
	currentX = startlocation_x;
	currentY = startlocation_y;		
	
	numOfSteps = 0;
	recursiveDFSCall(retsteps);	
	
	return;
}

/*
This function adds the current node to the retsteps array
*/
void addStep(int16_t *retsteps){
	
	//check how many path nodes are left to visit
	int16_t i, j;
	int16_t nodesLeftToVisit = 0;
	for (i = 0; i < HEIGHT; i++){  
		for (j = 0; j < WIDTH; j++){  			
			if(traversal_map[i][j] == 0){
				nodesLeftToVisit++;
			}
		}
	}
	
	if (nodesLeftToVisit == 0){//Don't add any more steps if all path nodes have been visited
		return;
	}
	
	//printf("%d: %d,%d-- 	", numOfSteps, currentX, currentY);
	retsteps[numOfSteps] = currentY * WIDTH + currentX;// add new step to array of steps
	
	traversal_map[currentY][currentX] = 2;	//mark the corresponding coordinate on the traversal_map as having been visited
	numOfSteps++;
}

/*
This is a recursive function that calls itself for each path node adjacent to the current node
*/
void recursiveDFSCall(int16_t* retsteps){
	//The tempX and tempY variables store the coordinates of the current node, before the values of currentX and currentY are modified by subsequent recursive DFS calls
	//As we trace back from the recursive calls, the values stored in the temp variables are loaded back into the currentX and currentY variables
	int16_t tempX = currentX;
	int16_t tempY = currentY;

	addStep(retsteps);	//add the current node to the retsteps array	
	
	if ((currentY > 0)&&(traversal_map[currentY-1][currentX] == 0)){ // check if the node to the north of the current node is a path node and hasn't already been visited
		currentY -= 1;	//The north node becomes the new current node
		recursiveDFSCall(retsteps);
		
		//The coordinates stored in the temp variables are loaded into the currentX and currentY variables as we return from the recursive DFS calls
		currentX = tempX;
		currentY = tempY;
		addStep(retsteps);			
	}
	if ((currentY < (HEIGHT - 1))&&(traversal_map[currentY + 1][currentX] == 0)){ // check if the node to the south of the current node is a path node and hasn't already been visited
		currentY += 1;	//The south node becomes the new current node
		recursiveDFSCall(retsteps);
		currentX = tempX;
		currentY = tempY;
		addStep(retsteps);		
	}
	if ((currentX > 0)&&(traversal_map[currentY][currentX-1] == 0)){// check if the node to the west of the current node is a path node and hasn't already been visited
		currentX -= 1;	//The west node becomes the new current node
		recursiveDFSCall(retsteps);
		currentX = tempX;
		currentY = tempY;
		addStep(retsteps);			
	}
	if ((currentX < (WIDTH - 1))&&(traversal_map[currentY][currentX + 1] == 0)){ // check if the node to the east of the current node is a path node and hasn't already been visited
		currentX += 1;	//The east node becomes the new current node
		recursiveDFSCall(retsteps);
		currentX = tempX;
		currentY = tempY;
		addStep(retsteps);		
	}	
}