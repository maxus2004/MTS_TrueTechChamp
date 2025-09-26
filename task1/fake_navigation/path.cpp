#include "path.h"
#include <thread>
#include "movement.h"
#include "utils.h"
#include <iostream>

using namespace std;

float target_a = 0;
float target_v = 0;

#define LINEAR_SPEED 0.3f
#define TURN_SPEED 0.6f;
#define LINEAR_PRECISION_METERS 0.1f
#define ANGULAR_PRECISION_RADIANS 0.01f
#define UPDATE_INTERVAL_SECONDS 0.01f

void controlLoop(Robot &robot){
    asio::io_context io_context;
    asio::ip::udp::socket socket(io_context, asio::ip::udp::v4());
    while(state==State::PathFollowing){
        float a = fixAngleOverflow(target_a-robot.a);
        float cmd_a = 0;
        if(a>0){
            cmd_a = -TURN_SPEED;
        }else{
            cmd_a = TURN_SPEED;
        }
        send_move(target_v, cmd_a, socket);
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void followPath(vector<PathPoint> path, Robot &robot, queue<Msg>* messages){
    state=State::PathFollowing;
    thread control_thread(controlLoop, std::ref(robot));
    cout << "starting path following" << endl;
    for(int i = 1;i<path.size();i++){
        cout << "point " << i << "/" << path.size()-1 << endl;

        float align_end_a = atan2(path[i].x-robot.x,-(path[i].y-robot.y));

        if(abs(fixAngleOverflow(robot.a-align_end_a))>ANGULAR_PRECISION_RADIANS){
            cout << "aligning" << endl;
            target_v = 0;
            target_a = align_end_a;
            while(abs(fixAngleOverflow(robot.a-align_end_a))>ANGULAR_PRECISION_RADIANS){
                if (!messages->empty() && messages->front() == Msg::STOPFOLOW) {
                    messages->pop();
                    cout << "aborted" << endl;
                    target_v = 0;
                    state = State::ManualControl;
                    control_thread.join();
                    return;;
                }
                this_thread::sleep_for(chrono::duration<float>(UPDATE_INTERVAL_SECONDS));
            }
        }
        target_v = LINEAR_SPEED;


        if(i+1<path.size()){
            float turn_start_a = align_end_a;
            float turn_end_a = atan2(path[i+1].x-path[i].x,-(path[i+1].y-path[i].y));
            float turn_delta_a = fixAngleOverflow(turn_end_a-turn_start_a);
            float turn_start_distance = abs(path[i].r * tan(turn_delta_a/2));
            float turn_arc_length = abs(path[i].r * turn_delta_a);

            cout << "driving..." << endl;
            while(distance(path[i].x,path[i].y,robot.x,robot.y)>LINEAR_PRECISION_METERS+turn_start_distance){
                if (!messages->empty() && messages->front() == Msg::STOPFOLOW) {
                    messages->pop();
                    cout << "aborted" << endl;
                    target_v = 0;
                    state = State::ManualControl;
                    control_thread.join();
                    return;;
                }
                this_thread::sleep_for(chrono::duration<float>(UPDATE_INTERVAL_SECONDS));
            }

            cout << "turning..." << endl;
            float turn_progress = 0;
            if(turn_arc_length != 0){
                while(turn_progress<1){
                    if (!messages->empty() && messages->front() == Msg::STOPFOLOW) {
                        messages->pop();
                        cout << "aborted" << endl;
                        target_v = 0;
                        state = State::ManualControl;
                        control_thread.join();
                        return;;
                    }
                    target_a = turn_start_a+turn_delta_a*turn_progress;
                    turn_progress += robot.v*UPDATE_INTERVAL_SECONDS/turn_arc_length;
                    this_thread::sleep_for(chrono::duration<float>(UPDATE_INTERVAL_SECONDS));
                }
            }
            target_a = turn_end_a;
        }else{
            cout << "driving..." << endl;
            while(distance(path[i].x,path[i].y,robot.x,robot.y)>LINEAR_PRECISION_METERS){
                if (!messages->empty() && messages->front() == Msg::STOPFOLOW) {
                    messages->pop();
                    cout << "aborted" << endl;
                    target_v = 0;
                    state = State::ManualControl;
                    control_thread.join();
                    return;;
                }
                this_thread::sleep_for(chrono::duration<float>(UPDATE_INTERVAL_SECONDS));
            }
        }
    }
    target_v = 0;
    cout << "done" << endl;

    state = State::ManualControl;
    control_thread.join();
}