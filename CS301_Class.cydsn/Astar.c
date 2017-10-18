/*
	Group 6 Compsys 301
*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <limits.h>
#include "Astar.h"
extern int16_t map[15][19];

int16_t astar(int16_t startlocation_x, int16_t startlocation_y, int16_t targetlocation_x, int16_t targetlocation_y, int16_t *retsteps){
    
	int16_t traceBackSteps[555] = { -1 }; // all elements initialised to -1
	
	int16_t startlocation = startlocation_y * WIDTH + startlocation_x;	//the x and y co-ordinates for the startlocation location are combined to give a single number
	int16_t targetlocation = targetlocation_y * WIDTH + targetlocation_x;	//the x and y co-ordinates for the target location are combined to give a single number
	
	int16_t	stepsCount;
	int16_t current; 		
	
	int16_t openSet[SIZE] = {-1};	//openSet contains currently discovered nodes that have not been evaluated yet.
	int16_t closedSet[SIZE] = {-1};	//closedSet contains the set of nodes that have already been evaluated
	int16_t cameFromNode[SIZE] = {-1};	//cameFromNode contains the parent node from which a node can be reached most easily
	int16_t gScore[SIZE] = {-1};	//For each node, the cost of getting from the start node to that node.
	
	int16_t i;
	for (i = 0; i < SIZE; i++){	//initialise array elements to -1
		openSet[i] = -1;
		closedSet[i] = -1;
		cameFromNode[i] = -1;
		gScore[i] = -1;
	}	

	openSet[startlocation] = 1;	//start node is added to openSet
	gScore[startlocation] = 0;	// The cost of going from the start node to the start node is zero
	//While there are still nodes that are needed to be searched, keep looping
	while (!isOpenSetEmpty(openSet)){
		
		current = next_current(openSet, gScore, targetlocation);	//current is the node in openSet with lowest fScore
		
		closedSet[current] = 1;	//The new current node is added to closedSet
		
		addNeigbours(current, map, gScore, cameFromNode,  openSet, closedSet);	//get the current node's neighbour nodes

		if (current == targetlocation){	//If the targetlocation is found, exit the loop
			
			stepsCount = reconstruct_path(cameFromNode, traceBackSteps, startlocation, targetlocation);
			
			//invert traceBackSteps array to obtain steps from startlocation to targetlocation
			for (i = 0; i < stepsCount; i++){		
				retsteps[i] = traceBackSteps[stepsCount - i - 1];
			}
			break;
		}	
	}	
	
	for (i = 0; i < stepsCount; i++){		
		printf("Node %i : (%i, %i)\n", i,retsteps[i]%WIDTH, retsteps[i]/WIDTH);
	}
	return stepsCount;
}

/*
This function finds adjacent path nodes to the current node and adds them to openSet 
*/
static void addNeigbours(int16_t current, int16_t *map, int16_t *gScore, int16_t *cameFromNode,  int16_t *openSet, int16_t *closedSet){
	int16_t	adjacentNode = 0;
	 

	// if node to the West of current node is a neighbouring path node and hasn't yet been added to closedSet, then add it to openSet and update the gScore of that node
	if (map[current - 1] == 0 && (current - 1) >= 0 &&  closedSet[current - 1] == -1){				
		
		cameFromNode[current - 1] = current;	//The parent of this adjacent path node is set as the current node
		gScore[current - 1] = gScore[current] + 1;	//the gScore of this adjacent path node is 1 unit higher than the gScore of the current node
		openSet[current - 1] = 1;
	}	

	// if node to the East of current node is a neighbouring path node and hasn't yet been added to closedSet, then add it to openSet and update the gScore of that node
	if (map[current + 1] == 0 && (current + 1) < SIZE &&  closedSet[current + 1] == -1){		
		
		cameFromNode[current + 1] = current;
		gScore[current + 1] = gScore[current] + 1;
		openSet[current + 1] = 1;	
	}

	// if node to the North of current node is a neighbouring path node and hasn't yet been added to closedSet, then add it to openSet and update the gScore of that node
	if (map[current - WIDTH] == 0 && (current - WIDTH) >= 0 &&  closedSet[current - WIDTH] == -1){				
		
		cameFromNode[current - WIDTH] = current;
		gScore[current - WIDTH] = gScore[current] + 1;
		openSet[current - WIDTH] = 1;	
	}
	
	// if node to the South of current node is a neighbouring path node and hasn't yet been added to closedSet, then add it to openSet and update the gScore of that node
	if (map[current + WIDTH] == 0 && (current + WIDTH) < SIZE &&  closedSet[current + WIDTH] == -1){			
		
		cameFromNode[current + WIDTH] = current;
		gScore[current + WIDTH] = gScore[current] + 1;
		openSet[current + WIDTH] = 1;		
	}
	
}
/*
Returns the array of steps needed to go from start to target
*/
static int16_t reconstruct_path(int16_t *cameFromNode, int16_t *traceBackSteps, int16_t startlocation, int16_t targetlocation){
	
	int16_t current = targetlocation;
	int16_t stepsCount = 0;
	
	traceBackSteps[stepsCount] = current;
	stepsCount++;

	while (current != startlocation){	//once we've traced back to the start, break the loop		
		traceBackSteps[stepsCount] = cameFromNode[current];
		stepsCount++;
		current = cameFromNode[current];		
	}
	
	return stepsCount;
}

/*
	Returns the node in openSet with the lowest fScore
*/
static int16_t next_current(int16_t *openSet, int16_t *gScore,  int16_t targetlocation){
		
	int16_t h_score = 0;
	int16_t new_h_score = 0;	
	
	int16_t minimumfScore = 10000;	
	int16_t fScore = 0;
	int16_t newCurrent = 0;
	
	int16_t i;
	for (i = 0; i < SIZE; i++){		
		if (openSet[i] == 1){	//loop through openSet and find the node with the lowest fScore				
			new_h_score = heuristic_score_estimate(i % WIDTH, i / WIDTH, targetlocation % WIDTH, targetlocation / WIDTH);
			
			fScore = gScore[i] + new_h_score;	//fScore is the total cost of getting from the start node to the goal by passing by that node. f(n)= g(n) + heuristic(n)
			if (fScore < minimumfScore){
				newCurrent = i;
				minimumfScore = fScore;				
			} else if (fScore == minimumfScore){	//If the two fScores are the same then the one with the lower heuristic score is selected, as there is more certainty with that value
				h_score = heuristic_score_estimate(newCurrent % WIDTH, newCurrent / WIDTH, targetlocation % WIDTH, targetlocation / WIDTH);
				if (new_h_score <= h_score){
					newCurrent = i;
					minimumfScore = fScore;					
				}
			}
		}
	}
	
	openSet[newCurrent] = 0;
	
	return newCurrent;
}

/*The heuristic_cost_estimate() is a function that estimates the cost of the cheapest path from the current node to the goal node
Heuristic means that the function is not perfect but performs well enough for the requirements
*/
static int16_t heuristic_score_estimate(int16_t currNode_x, int16_t currNode_y, int16_t goal_x, int16_t goal_y){
	
	return round(sqrt(pow(goal_x - currNode_x, 2) + pow(goal_y - currNode_y, 2)) );	//this formula finds the straight line distance between the two nodes
}

/*
Checks if openSet array is empty
*/
static int16_t isOpenSetEmpty(int16_t *openSet){
	int16_t i;
	
	for (i = 0; i < SIZE; i++){
		if(openSet[i] == 1){
			return 0;
		}
	}	
	return 1;
}
/* [] END OF FILE */
