#pragma once
#include <opencv2/opencv.hpp>
#include "params.h"

struct MtsTelemetryPacket {
    char header[8];
    float x,y,a;
    float vx,vy,va;
    float gx,gy,gz;
    unsigned int lidar_count;
    float distances[360];
};

struct Telemetry {
    float ds;
    float gy;
    float v;
    float distances[360];
};

struct Robot {
    float x;
    float y;
    float v;
    float a;
};

struct ScanPoint{
    float a,d;
    float x,y;
};

struct PathPoint{
    float x,y,r;
};

enum State{
    ManualControl,
    PathFollowing
};

extern State state;

cv::Point worldToGrid(float x, float y);
cv::Point worldToGrid(ScanPoint p);
cv::Point worldToGrid(PathPoint p);
float distance(float x1, float y1, float x2, float y2);
float distance(ScanPoint p1, ScanPoint p2);
float fixAngleOverflow(float a);