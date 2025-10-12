#include "path.h"
#include <thread>
#include "movement.h"
#include "utils.h"
#include <iostream>

using namespace std;

float target_a = 0;
float target_v = 0;
float prev_a = 0;

extern bool telemetry_updated;

#define LINEAR_SPEED 1.0f
#define TURNING_SPEED 0.3f
#define TURNING_SLOWDOWN_DISTANCE 0.5f
#define LINEAR_PRECISION_METERS 0.1f
#define ANGULAR_PRECISION_RADIANS 0.1f

#define TURNING_K_P 12.0f
#define TURNING_MAX_P 3.0f
#define TURNING_K_D 6.0f

#define DRIVING_K_P 50.0f
#define DRIVING_MAX_P 1.0f

void updatePID(Robot &robot){
    float a_error = fixAngleOverflow(target_a-robot.a);
    float va = fixAngleOverflow(robot.a-prev_a)/DT;

    float turning_p = -a_error*TURNING_K_P;
    if(turning_p > TURNING_MAX_P) turning_p = TURNING_MAX_P;
    if(turning_p < -TURNING_MAX_P) turning_p = -TURNING_MAX_P;
    float turning_d = va*TURNING_K_D;

    float v_error = robot.v-target_v;

    float driving_p = -v_error*DRIVING_K_P;
    if(target_v == 0) driving_p = 0;
    if(driving_p > DRIVING_MAX_P) driving_p = DRIVING_MAX_P;
    if(driving_p < -DRIVING_MAX_P) driving_p = -DRIVING_MAX_P;

    send_move(driving_p, turning_p+turning_d);

    prev_a = robot.a;
}

void wait_for_telemetry(){
    while(!telemetry_updated){
        this_thread::yield();
    }
    telemetry_updated = false;
}

void followPath(vector<cv::Point2f> &path, Robot &robot, queue<Msg>* messages){
    state=State::PathFollowing;
    cout << "starting path following" << endl;
    
    while(true){
        vector<cv::Point2f> pathCopy = path;
        int i = 0;
        while(true){
            if(distance(pathCopy[i].x,pathCopy[i].y,robot.x,robot.y)>=0.15f){
                break;
            }
            i++;
        }
        cv::Point2f nextPoint = pathCopy[i];
        target_a = atan2(nextPoint.x-robot.x,-(nextPoint.y-robot.y));
        float abs_a_diff = abs(fixAngleOverflow(target_a-robot.a));
        float v = 0.2f;
        v -= abs_a_diff;
        if(v < 0) v = 0;
        target_v = v;
        wait_for_telemetry();
        updatePID(robot);
        if (!messages->empty() && messages->front() == Msg::STOPFOLOW) {
            messages->pop();
            cout << "aborted" << endl;
            break;
        }
    }

    target_v = 0;
    send_move(0, 0);
    state = State::ManualControl;
}