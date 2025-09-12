#include "utils.h"

cv::Point worldToGrid(float x, float y){
    return cv::Point(x/CELL_SIZE+GRID_W/2,y/CELL_SIZE+GRID_H/2);
}

cv::Point worldToGrid(ScanPoint p){
    return cv::Point(p.x/CELL_SIZE+GRID_W/2,p.y/CELL_SIZE+GRID_H/2);
}

float distance(float x1, float y1, float x2, float y2){
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}
float distance(ScanPoint p1, ScanPoint p2){
    return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
}