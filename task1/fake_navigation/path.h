#include "utils.h"
#include <vector>
#include <queue>

void controlLoop(Robot &robot);
void followPath(std::vector<PathPoint> path, Robot &robot, std::queue<Msg>* messages);