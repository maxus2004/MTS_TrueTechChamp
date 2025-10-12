#include "utils.h"
#include <vector>
#include <queue>

void controlLoop(Robot &robot);
void followPath(std::vector<cv::Point2f> &path, Robot &robot, std::queue<Msg>* messages);