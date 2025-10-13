#include <asio.hpp>
#include <asio/ip/udp.hpp>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <locale>
#include <queue>
#include <thread>
#include <raylib.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <set>
#include <numeric>
#include "utils.h"
#include "params.h"
#include "movement.h"
#include "path.h"

using asio::ip::tcp;
using namespace std;

cv::Mat1b grid(cv::Size(GRID_W, GRID_H)); 
cv::Mat1f cost_grid(cv::Size(GRID_W, GRID_H)); 
cv::Mat1f pathfind_grid(cv::Size(GRID_W, GRID_H)); 
queue<Msg> message_queue;


Robot robot{-3.75,3.75,0,PI/2};

bool running = true;
State state = State::ManualControl;

float prev_mts_x = 0;
float prev_mts_y = 0;

vector<cv::Point2f> path;

bool telemetry_updated = false;

void update_telemetry(Telemetry* telemetry, tcp::socket* telemetry_socket){
    //get telemetry data
    std::array<char, 1488> buf;
    asio::read(*telemetry_socket, asio::buffer(buf, 1488));
    MtsTelemetryPacket packet;
    memcpy(&packet, buf.data(), sizeof(packet));

    //get actual odometry data from fucked up telemetry
    telemetry->gy = packet.gy;
    telemetry->ds = distance(packet.x,packet.y,prev_mts_x,prev_mts_y) * ENCODER_LINEAR_MULTIPLIER * (packet.vx>0?1:-1);
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

void start_path(queue<Msg>* messages){
    followPath(path, robot, messages);
}


const int wavefront_kernel_size = 2;
const int pathfind_kernel_size = 5;

struct WavefrontPoint {
    int x, y;
    float dist;
    
};

struct CompareWavefrontPoint {
    bool operator()(const WavefrontPoint& a, const WavefrontPoint& b) const {
        return a.dist < b.dist;
    }
};

void wavefront(){
    float wavefront_kernel[wavefront_kernel_size*2+1][wavefront_kernel_size*2+1];
    for(int rx = -wavefront_kernel_size;rx<=wavefront_kernel_size;rx++){
        for(int ry = -wavefront_kernel_size;ry<=wavefront_kernel_size;ry++){
            wavefront_kernel[ry+wavefront_kernel_size][rx+wavefront_kernel_size] = sqrt((float)rx*rx+(float)ry*ry);
        }
    }

    float pathfind_kernel[pathfind_kernel_size*2+1][pathfind_kernel_size*2+1];
    for(int rx = -pathfind_kernel_size;rx<=pathfind_kernel_size;rx++){
        for(int ry = -pathfind_kernel_size;ry<=pathfind_kernel_size;ry++){
            pathfind_kernel[ry+pathfind_kernel_size][rx+pathfind_kernel_size] = sqrt((float)rx*rx+(float)ry*ry)*0.9f;
        }
    }

    float goal_world_x = 0;
    float goal_world_y = 0;
    cv::Point goal = worldToGrid(goal_world_x,goal_world_y);
    cv::Mat1f new_pathfind_grid(cv::Size(GRID_W, GRID_H));

    int iteration = 0;

    while (running){
        new_pathfind_grid.setTo(INFINITY);

        multiset<WavefrontPoint, CompareWavefrontPoint> nextPoints;
        bool calculatedPoints[GRID_H][GRID_W] = {0};

        nextPoints.insert(WavefrontPoint{goal.x,goal.y,0});
        new_pathfind_grid[goal.y][goal.x] = 0;
        calculatedPoints[goal.y][goal.x] = true;

        while(nextPoints.size() > 0){
            WavefrontPoint point = *nextPoints.begin();
            nextPoints.extract(point);
            for(int rx = -wavefront_kernel_size;rx<=wavefront_kernel_size;rx++){
                for(int ry = -wavefront_kernel_size;ry<=wavefront_kernel_size;ry++){
                    int x = point.x+rx;
                    int y = point.y+ry;
                    if(x < 0 || x >= GRID_W || y < 0 || y >= GRID_H) continue;
                    if(cost_grid[y][x] > 0.99f) continue;
                    if(((rx==0 && ry==1) || (rx==0 && ry==-1) || (rx==1 && ry==0) || (rx==-1 && ry==0)) && calculatedPoints[y][x] == false){
                        nextPoints.insert(WavefrontPoint{x,y,new_pathfind_grid[point.y][point.x]});
                        calculatedPoints[y][x] = true;
                    }
                    if(new_pathfind_grid[y][x] > new_pathfind_grid[point.y][point.x] + wavefront_kernel[ry+wavefront_kernel_size][rx+wavefront_kernel_size] + cost_grid[y][x]*20){
                        new_pathfind_grid[y][x] = new_pathfind_grid[point.y][point.x] + wavefront_kernel[ry+wavefront_kernel_size][rx+wavefront_kernel_size] + cost_grid[y][x]*20;
                    }
                }
            }
        }
        new_pathfind_grid.copyTo(pathfind_grid);

        vector<cv::Point2f> newPath;
        cv::Point2f nextPoint(robot.x, robot.y);
        while(true){
            newPath.push_back(nextPoint);
            cv::Point grid_point = worldToGrid(nextPoint);
            int best_x = -1;
            int best_y = -1;
            float min_distance = pathfind_grid[grid_point.y][grid_point.x];
            for(int rx = -pathfind_kernel_size;rx<=pathfind_kernel_size;rx++){
                for(int ry = -pathfind_kernel_size;ry<=pathfind_kernel_size;ry++){
                    int x = grid_point.x+rx;
                    int y = grid_point.y+ry;
                    if(x <= 0 || x >= GRID_W || y <= 0 || y >= GRID_H) continue;
                    if(pathfind_grid[y][x] <= 0) continue;
                    if(pathfind_grid[y][x]+pathfind_kernel[ry+pathfind_kernel_size][rx+pathfind_kernel_size] < min_distance){
                        min_distance = pathfind_grid[y][x]+pathfind_kernel[ry+pathfind_kernel_size][rx+pathfind_kernel_size];
                        best_x = x;
                        best_y = y;
                    }
                }
            }
            nextPoint = gridToWorld(best_x, best_y);
            if(min_distance < 5)break;
        }
        path = newPath;
        
        iteration++;
    }
}

thread path_thread;
thread wavefront_thread;

void draw_loop() {
    bool path_thread_exists = false;

    SetConfigFlags(FLAG_VSYNC_HINT);

    InitWindow(GRID_W, GRID_H, "ArchBTW monitoring");

    Shader shader = LoadShader(NULL, "../grid_shader.fs");

    Color* pixels = new Color[GRID_W * GRID_H];
    Image img = GenImageColor(GRID_W, GRID_H, BLACK);
    img.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    Texture2D blackTexture = LoadTextureFromImage(img);
    UnloadImage(img);

    Texture2D grid_texture = LoadTextureFromImage(Image{grid.data, GRID_W, GRID_H, 1, PIXELFORMAT_UNCOMPRESSED_GRAYSCALE});
    Texture2D cost_grid_texture = LoadTextureFromImage(Image{cost_grid.data, GRID_W, GRID_H, 1, PIXELFORMAT_UNCOMPRESSED_R32});
    Texture2D pathfind_grid_texture = LoadTextureFromImage(Image{pathfind_grid.data, GRID_W, GRID_H, 1, PIXELFORMAT_UNCOMPRESSED_R32});

    while (!WindowShouldClose()){
        if(state == State::ManualControl){
            handle_wasd();
        }
        if(IsKeyPressed(KEY_P)){
            if (!path_thread_exists) {
                path_thread = thread(start_path, &message_queue);
                path_thread_exists = true;
            } 
            else {
                message_queue.push(Msg::STOPFOLOW);
            }
        }
        if(IsKeyPressed(KEY_F)){
            wavefront_thread = thread(wavefront);
        }
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Draw grids
        UpdateTexture(grid_texture, grid.data);
        UpdateTexture(cost_grid_texture, cost_grid.data);
        UpdateTexture(pathfind_grid_texture, pathfind_grid.data);
        BeginShaderMode(shader);
        SetShaderValueTexture(shader, GetShaderLocation(shader, "grid"), grid_texture);
        SetShaderValueTexture(shader, GetShaderLocation(shader, "cost_grid"), cost_grid_texture);
        SetShaderValueTexture(shader, GetShaderLocation(shader, "pathfind_grid"), pathfind_grid_texture);
        float time = GetTime();
        SetShaderValue(shader, GetShaderLocation(shader, "time"), &time, SHADER_UNIFORM_FLOAT);
        DrawTexture(blackTexture, 0, 0, WHITE);
        EndShaderMode();

        //draw path
        for(size_t i = 1;i<path.size();i++){
            cv::Point2f p1 = worldToGrid(path[i-1]);
            cv::Point2f p2 = worldToGrid(path[i]);
            DrawLineEx({p1.x, p1.y}, {p2.x, p2.y}, 3, BLUE);
        }

        //draw robot
        float screen_robot_x = robot.x/CELL_SIZE+GRID_W/2;
        float screen_robot_y = robot.y/CELL_SIZE+GRID_H/2;
        float dir_x = 7*sin(robot.a);
        float dir_y = -7*cos(robot.a);
        DrawCircle(screen_robot_x, screen_robot_y, 7, GREEN);
        DrawLineEx({screen_robot_x, screen_robot_y}, {screen_robot_x+dir_x, screen_robot_y+dir_y}, 3, BLACK);

        // Draw coordinates
        DrawText(TextFormat("X: %.2f", robot.x), 10, 10, 20, GREEN);
        DrawText(TextFormat("Y: %.2f", robot.y), 10, 30, 20, GREEN);
        // Draw FPS
        DrawText(TextFormat("%.02f FPS", 1.0/GetFrameTime()), 10, 50, 20, GREEN);

        EndDrawing();
    }

    running = false;
    // Cleanup
    delete[] pixels;
    UnloadTexture(blackTexture);
    UnloadShader(shader);
    CloseWindow();
}

int main() {
    try{
    //connect to simulation
    asio::io_context io_context;
    string host = "0.0.0.0";
    string port = "5600";
    if(getenv("TEL_HOST") != NULL){
        host = getenv("TEL_HOST");
    }
    if(getenv("TEL_PORT") != NULL){
        port = getenv("TEL_PORT");
    }
    cout << "telemetry host: " << host << ":" << port << endl;
    tcp::acceptor acceptor(io_context, asio::ip::tcp::resolver(io_context).resolve(host, port).begin()->endpoint());
    tcp::socket telemetry_socket(io_context);
    acceptor.accept(telemetry_socket);

    init_movement();

    #ifdef VISUALIZATION
    thread draw_thread(draw_loop);
    #else
    wavefront_thread = thread(wavefront);
    path_thread = thread(start_path, &message_queue);
    #endif

    //add outside walls
    cv::line(grid,cv::Point(23,23),cv::Point(23,427),1,3);
    cv::line(grid,cv::Point(23,427),cv::Point(427,427),1,3);
    cv::line(grid,cv::Point(427,427),cv::Point(427,23),1,3);
    cv::line(grid,cv::Point(427,23),cv::Point(23,23),1,3);

    Telemetry telemetry;

    while (running) {
        //receive telemetry from robot in simulation
        update_telemetry(&telemetry, &telemetry_socket);

        //dead reckoning (The missle knows where it is because....)
        robot.a -= telemetry.gy*DT;
        robot.x += telemetry.ds*sin(robot.a);
        robot.y -= telemetry.ds*cos(robot.a);
        robot.v = telemetry.v;

        //telemetry fully updated
        telemetry_updated = true;

        //copy grid before modifying to fix flickering in render loop
        //maybe it's better to pause rendering when modifying grid instead of copying, but this works for now
        cv::Mat1b gridCopy = grid.clone();

        //obstacle mapping
        ScanPoint scanPoints[360];
        getScanPoints(scanPoints,telemetry,robot);
        
        // //Hough lines
        // cv::Mat1b houghGrid(cv::Size(GRID_W, GRID_H)); 
        // houghGrid.setTo(cv::Scalar(0));
        // for(int i = 0;i<360;i++){
        //     if(scanPoints[i].d < 8){
        //         houghGrid.at<uchar>(worldToGrid(scanPoints[i])) = 255;
        //     }
        // }
        // vector<cv::Vec4i> houghLines;
        // cv::HoughLinesP(houghGrid, houghLines, 2, 0.02, 30, 15, 3);

        // vector<float> angles;

        // cv::Mat3b houghVisualization(cv::Size(GRID_W, GRID_H)); 
        // houghVisualization.setTo(cv::Scalar(0,0,0));
        // for(cv::Vec4i l: houghLines){
        //     float a = atan2(l[3]-l[1],l[2]-l[0]);
        //     while(a > PI/4) a -= PI/2;
        //     while(a < -PI/4) a += PI/2;
        //     if(abs(a) < 0.2f){
        //         angles.push_back(a);
        //     }
        //     cv::line(houghVisualization, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(rand()%256,rand()%256,rand()%256), 1);
        // }
        // if(angles.size() > 0){
        //     float averageAngle = reduce(angles.begin(), angles.end()) / angles.size();
        //     cout << "avg angle: " << averageAngle << endl;
        //     robot.a -= averageAngle;
        // }
        // cv::imshow("Hough Lines Visualization",houghVisualization);
        // cv::waitKey(10);

        // getScanPoints(scanPoints,telemetry,robot);
        
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

        //expand walls for pathfinding
        cv::compare(gridCopy,1,gridCopy,cv::CMP_EQ);
        int dilation_size = 5;
        int dilation_type = cv::MORPH_ELLIPSE;
        cv::Mat element = cv::getStructuringElement( dilation_type,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );
        cv::dilate(gridCopy,gridCopy,element);
        // cv::bitwise_not(gridCopy,gridCopy);
        cv::Mat1b gridCopy2(cv::Size(GRID_W, GRID_H));
        cv::blur(gridCopy,gridCopy2,cv::Size(10,10));
        cv::add(gridCopy,gridCopy2,gridCopy);
        gridCopy.convertTo(cost_grid, CV_32F, 0.0039);
    }

    #ifdef VISUALIZATION
    draw_thread.join();
    #endif
    }catch(std::exception e){
        running = false;
        message_queue.push(Msg::STOPFOLOW);
        path_thread.join();
        wavefront_thread.join();
        cout << "Exception: " << e.what() << endl;
    }
}
