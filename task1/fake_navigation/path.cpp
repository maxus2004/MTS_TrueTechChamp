#include "path.h"
#include <thread>
#include "movement.h"
#include <iostream>

using namespace std;

float target_a = 0;
float target_v = 0;

void controlLoop(Robot &robot){
    asio::io_context io_context;
    asio::ip::udp::socket socket(io_context, asio::ip::udp::v4());
    while(state==State::PathFollowing){
        float a = fixAngleOverflow(target_a-robot.a);
        float cmd_a = 0;
        if(a>0){
            cmd_a = -0.3;
        }else{
            cmd_a = 0.3;
        }
        send_move(target_v, cmd_a, socket);
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void followPath(vector<PathPoint> path, Robot &robot){
    state=State::PathFollowing;
    thread control_thread(controlLoop, std::ref(robot));
    cout << "starting path following" << endl;
    for(int i = 1;i<path.size();i++){
        cout << "point " << i << "/" << path.size()-1 << endl;
        target_v = 0;
        target_a = atan2(path[i].x-robot.x,-(path[i].y-robot.y));
        cout << "turning" << endl;
        while(abs(fixAngleOverflow(robot.a-target_a))>0.01){
            this_thread::sleep_for(chrono::milliseconds(10));
        }
        target_v = 0.3;
        cout << "driving" << endl;
        while(distance(path[i].x,path[i].y,robot.x,robot.y)>0.1){
            target_a = atan2(path[i].x-robot.x,-(path[i].y-robot.y));
            this_thread::sleep_for(chrono::milliseconds(10));
        }
        target_v = 0;
    }
    cout << "done" << endl;
    state=State::ManualControl;
    control_thread.join();
}