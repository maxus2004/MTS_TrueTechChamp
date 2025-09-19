#include <asio.hpp>
#include <asio/ip/udp.hpp>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <raylib.h>
#include <opencv4/opencv2/opencv.hpp>
#include "utils.h"
#include "params.h"
#include "movement.h"
#include "path.h"

using asio::ip::tcp;
using namespace std;

cv::Mat1b grid(cv::Size(GRID_W, GRID_H)); 
cv::Mat1b pathfind_grid(cv::Size(GRID_W, GRID_H)); 


Robot robot{12,-1.25,0,0};

bool running = true;
State state = State::ManualControl;

float prev_mts_x = 0;
float prev_mts_y = 0;

vector<PathPoint> path;

void update_telemetry(Telemetry* telemetry, tcp::socket* telemetry_socket){
    //get telemetry data
    std::array<char, 8192> buf;
    size_t len = telemetry_socket->read_some(asio::buffer(buf));
    MtsTelemetryPacket packet;
    memcpy(&packet, buf.data(), sizeof(packet));

    //get actual odometry data from fucked up telemetry
    telemetry->gy = packet.gy;
    telemetry->ds = distance(packet.x,packet.y,prev_mts_x,prev_mts_y) * ENCODER_LINEAR_MULTIPLIER;
    telemetry->v = telemetry->ds/DT;
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

void start_path(){
    path.push_back({12,-1.25,0});
    path.push_back({11.5,1,1});
    path.push_back({5.5,-3,1});
    path.push_back({3.5,0,1});
    path.push_back({-4,0,1});
    path.push_back({-5,-2,1});
    path.push_back({-7.5,-3,0});
    path.push_back({-7.5,0,0});
    followPath(path,robot);
}

thread path_thread;

void draw_loop() {
    asio::io_context io_context;
    asio::ip::udp::socket socket(io_context, asio::ip::udp::v4());

    InitWindow(GRID_W, GRID_H, "ArchBTW monitoring");

    while (!WindowShouldClose()) {
        if(state == State::ManualControl){
            handle_wasd(socket);
        }
        if(IsKeyPressed(KEY_P)){
            path_thread = thread(start_path);
        }
        BeginDrawing();

        //draw map cells
        for (int x = 0; x < GRID_W; x++) {
            for (int y = 0; y < GRID_H; y++) {
                switch (grid[y][x]) {
                    case 0:
                        if(pathfind_grid[y][x] == 255){
                            DrawPixel(x, y, Color{ 170, 130, 130, 255 });
                        }else{
                            DrawPixel(x, y, Color{ 130, 130, 130, 255 });
                        }
                        break;
                    case 1:
                        DrawPixel(x, y, Color{ 0, 0, 0, 255 });
                        break;
                    case 2:
                        if(pathfind_grid[y][x] == 255){
                            DrawPixel(x, y, Color{ 255, 170, 170, 255 });
                        }else{
                            DrawPixel(x, y, Color{ 255, 255, 255, 255 });
                        }
                        break;
                }
            }
        }

        //draw path
        for(int i = 1;i<path.size();i++){
            cv::Point2f p1 = worldToGrid(path[i-1]);
            cv::Point2f p2 = worldToGrid(path[i]);
            DrawLineEx({p1.x, p1.y}, {p2.x, p2.y}, 3, BLUE);
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
        robot.v = telemetry.v;

        //copy grid before modifying to fix flickering in render loop
        //maybe it's better to pause rendering when modifying grid instead of copying, but this works for now
        cv::Mat1b gridCopy = grid.clone();

        //obstacle mapping
        ScanPoint scanPoints[360];
        getScanPoints(scanPoints,telemetry,robot);
        
        //fill while cells where there are no obstacles
        for(int i = 3;i<358;i++){
            cv::Point trianglePoints[3] = {
                worldToGrid(robot.x, robot.y),
                worldToGrid(scanPoints[i-1]),
                worldToGrid(scanPoints[i])
            };
            cv::fillConvexPoly(gridCopy,trianglePoints,3,2);
        }

        //fill black cells where there are obstacles
        for(int i = 1;i<360;i++){
            if(scanPoints[i].d < 8 && scanPoints[i-1].d < 8 && distance(scanPoints[i-1], scanPoints[i]) < 0.25){
                cv::line(gridCopy,worldToGrid(scanPoints[i-1]),worldToGrid(scanPoints[i]),1);
            }
        }

        //update grid for visualization
        gridCopy.copyTo(grid);

        //TOOD: improve robot position by tracking horizontal/vertical walls

        //expand walls for pathfinding
        cv::compare(gridCopy,1,gridCopy,cv::CMP_EQ);
        int dilation_size = 10;
        int dilation_type = cv::MORPH_ELLIPSE;
        cv::Mat element = cv::getStructuringElement( dilation_type,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );
        cv::dilate(gridCopy,pathfind_grid,element);
    }

    draw_thread.join();
}
