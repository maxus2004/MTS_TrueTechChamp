#include <asio.hpp>
#include <asio/ip/udp.hpp>
#include <iostream>
#include <cmath>
#include <raylib.h>
//#include <opencv2/opencv.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "utils.h"
#include "params.h"
#include "movement.h"

using asio::ip::tcp;
using namespace std;

cv::Mat1b grid(cv::Size(GRID_W, GRID_H)); 

Robot robot{9,-1.25,0};

bool running = true;

float prev_mts_x = 0;
float prev_mts_y = 0;

void update_telemetry(Telemetry* telemetry, tcp::socket* telemetry_socket){
    //get telemetry data
    std::array<char, 8192> buf;
    size_t len = telemetry_socket->read_some(asio::buffer(buf));
    MtsTelemetryPacket packet;
    memcpy(&packet, buf.data(), sizeof(packet));

    //get actual odometry data from fucked up telemetry
    telemetry->gy = packet.gy;
    telemetry->ds = distance(packet.x,packet.y,prev_mts_x,prev_mts_y) * ENCODER_LINEAR_MULTIPLIER;
    //copy lidar data to telemetry variable
    memcpy(&(telemetry->distances),&(packet.distances),sizeof(telemetry->distances));

    //update prev values
    prev_mts_x = packet.x;
    prev_mts_y = packet.y;
}

void getScanPoints(ScanPoint *points, Telemetry &telemetry, Robot &robot){
    for(int i = 0;i<360;i++){
        float a = robot.a+(45.0f-i/4.0f)/57.2958f;
        float d = telemetry.distances[i];
        float x = d*sin(a)+robot.x;
        float y = -d*cos(a)+robot.y;
        points[i] = {a,d,x,y};
    }
}

void draw_loop() {
    asio::io_context io_context;
    asio::ip::udp::socket socket(io_context, asio::ip::udp::v4());

    InitWindow(GRID_W, GRID_H, "ArchBTW monitoring");

    while (!WindowShouldClose()) {
        handle_input(socket);
        BeginDrawing();

        //draw map cells
        for (int x = 0; x < GRID_W; x++) {
            for (int y = 0; y < GRID_H; y++) {
                switch (grid[y][x]) {
                    case 0:
                        DrawPixel(x, y, GRAY);
                        break;
                    case 1:
                        DrawPixel(x, y, BLACK);
                        break;
                    case 2:
                        DrawPixel(x, y, WHITE);
                        break;
                }
            }
        }

        //draw robot
        float screen_robot_x = robot.x/CELL_SIZE+GRID_W/2;
        float screen_robot_y = robot.y/CELL_SIZE+GRID_H/2;
        float dir_x = 10*sin(robot.a);
        float dir_y = -10*cos(robot.a);
        DrawCircle(screen_robot_x, screen_robot_y, 10, GREEN);
        DrawLineEx({screen_robot_x, screen_robot_y}, {screen_robot_x+dir_x, screen_robot_y+dir_y}, 3, BLACK);

        EndDrawing();
    }

    running = false;
}

int main() {
    //connect to simulation
    asio::io_context io_context;
    tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), 5600));
    tcp::socket telemetry_socket(io_context);
    acceptor.accept(telemetry_socket);

    thread draw_thread(draw_loop);

    Telemetry telemetry;

    while (running) {
        //receive telemetry from robot in simulation
        update_telemetry(&telemetry, &telemetry_socket);

        //dead reckoning (The missle knows where it is because....)
        robot.a -= telemetry.gy*DT;
        robot.x += telemetry.ds*sin(robot.a);
        robot.y -= telemetry.ds*cos(robot.a);

        //copy grid before modifying to fix flickering in render loop
        //maybe it's better to pause rendering when modifying grid instead of copying, but this works for now
        cv::Mat1b gridCopy = grid.clone();

        //obstacle mapping
        ScanPoint scanPoints[360];
        getScanPoints(scanPoints,telemetry,robot);
        
        //fill while cells where there are no obstacles
        for(int i = 1;i<360;i++){
            cv::Point trianglePoints[3] = {
                worldToGrid(robot.x, robot.y),
                worldToGrid(scanPoints[i-1]),
                worldToGrid(scanPoints[i])
            };
            cv::fillConvexPoly(gridCopy,trianglePoints,3,2);
        }

        //fill black cells where there are obstacles
        for(int i = 0;i<360;i++){
            if(scanPoints[i].d < 8){
                if(i != 0 && distance(scanPoints[i-1], scanPoints[i]) < 0.25 && scanPoints[i-1].d < 8){
                    //if points are close to eachother, connect them with a line
                    cv::line(gridCopy,worldToGrid(scanPoints[i-1]),worldToGrid(scanPoints[i]),1);
                }else{
                    //else draw a single point
                    cv::Point point = worldToGrid(scanPoints[i]);
                    gridCopy[point.y][point.x] = 1;
                }
            }
        }

        //update grid for visualization
        gridCopy.copyTo(grid);

        //TOOD: improve robot position by tracking horizontal/vertical walls

    }

    draw_thread.join();
}
