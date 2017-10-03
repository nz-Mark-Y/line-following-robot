#include <stdio.h>
#include <stdint.h>

#define WIDTH 19
#define HEIGHT 15
#define SIZE 285

static int16_t heuristic_score_estimate(int16_t currNode_x, int16_t currNode_y, int16_t goal_x, int16_t goal_y);
static int16_t isOpenSetEmpty(int16_t *openSet);
static void addNeigbours(int16_t current, int16_t *map, int16_t *gScore, int16_t *cameFromNode,  int16_t *openSet, int16_t *closedSet);
static int16_t reconstruct_path(int16_t *cameFromNode, int16_t *traceBackSteps, int16_t startlocation, int16_t targetlocation);
int16_t astar(int16_t startlocation_x, int16_t startlocation_y, int16_t targetlocation_x, int16_t targetlocation_y, int16_t *retsteps); 
static int16_t next_current(int16_t *openSet, int16_t *gScore, int16_t targetlocation);