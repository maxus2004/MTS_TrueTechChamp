#include <algorithm>
#include <asio.hpp>
#include <asio/ip/udp.hpp>
#include <cstdint>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <locale>
#include <queue>
#include <thread>
#include <raylib.h>
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>
#include <numeric>
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "utils.h"
#include "params.h"
#include "movement.h"
#include "path.h"

using asio::ip::tcp;
using namespace std;

cv::Mat1b grid(cv::Size(GRID_W, GRID_H)); 
cv::Mat1b pathfind_grid(cv::Size(GRID_W, GRID_H)); 
queue<Msg> message_queue;
cv::Mat distanceGrid(GRID_H, GRID_W, CV_32F);


#ifdef BACKWARDS
Robot robot{12,-1.25,0,PI};
#else
Robot robot{12,-1.25,0,0};
#endif

bool running = true;
State state = State::ManualControl;

float prev_mts_x = 0;
float prev_mts_y = 0;

vector<PathPoint> path;

bool telemetry_updated = false;

void update_telemetry(Telemetry* telemetry, tcp::socket* telemetry_socket){
    //get telemetry data
    std::array<char, 1488> buf;
    asio::read(*telemetry_socket, asio::buffer(buf, 1488));
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
        #ifdef BACKWARDS
        a += PI;
        #endif
        float d = telemetry.distances[i];
        float x = d*sin(a)+robot.x;
        float y = -d*cos(a)+robot.y;
        points[i] = {a,d,x,y};
    }
}

void start_path(queue<Msg>* messages){
    path.push_back({12,-1.25,0});
    path.push_back({11.5,1,1});
    path.push_back({5.5,-3,1});
    path.push_back({3.5,0,1});
    path.push_back({-4,0,1});
    path.push_back({-5,-2,1});
    path.push_back({-7.7,-4,1});
    path.push_back({-7.7,0,0});
    followPath(path,robot, messages);
}

thread path_thread;

dstar::Key getKey(dstar::Cell c, dstar::Cell start, double g, double rhs){
    dstar::Key result;
    result.c = c;
    result.k1 = min(g, rhs) + (abs(c.x - start.x)+abs(c.y - start.y));
    result.k2 = min(g, rhs);
    return result;
}

void pathfinder_loop(cv::Point finish, cv::Point* start){
    /*
    1. Init
        * rhs = g = inf on any cell other then goal
        * in goal rhs = 0 and g = inf
        this makes the goal nonconsistent and so it isput in the "open" queue
    2. Main loop pops the goal from the q, sets g = rhs, that changes the rhs of the cells near and puts them in the queue too
    (Rhs is the minimum g you can find in the adjasent cells + 1, so the adjasent cells to the goal would be 0+1=1)
    3. The process repeaats until no uncelculated cells are left (if there is a possible path from them tothe center)
    4. profit
    */

    const double inf = std::numeric_limits<double>::max();

    // g and rhs values: map[y][x] = {g, rhs}
    std::vector<std::vector<std::pair<double, double>>> map(
        GRID_H, std::vector<std::pair<double, double>>(GRID_W, {inf, inf})
    );

    // Priority queue for open list
    std::priority_queue<
        dstar::Key,
        std::vector<dstar::Key>,
        dstar::KeyComparator
    > open_list; 

    // Initialize goal
    map[finish.y][finish.x] = {inf, 0};

    open_list.push(getKey(
        dstar::Cell(finish.x, finish.y),
        dstar::Cell(start->x, start->y),
        map[finish.y][finish.x].first,
        map[finish.y][finish.x].second
    ));

    // Neighbor offsets: 
    const std::vector<std::pair<int,int>> neighbors = {
        {0,-1}, {0,1}, {-1,0}, {1,0}
    };

    while (true) {
        // // Wait for TRIGGERUPDATE
        // if (message_queue.empty() || message_queue.back() != Msg::TRIGGERUPDATE) {
        //     this_thread::yield();
        //     continue;
        // }
        // message_queue.pop();

        // Process the open list
        while (!open_list.empty()) {
            dstar::Cell t = open_list.top().c;
            open_list.pop();

            double g_t = map[t.y][t.x].first;
            double rhs_t = map[t.y][t.x].second;

            if (g_t == rhs_t) continue; // consistent

            if (g_t > rhs_t) { // overconsistent
                map[t.y][t.x].first = rhs_t; // g = rhs

                // Update neighbors
                for (auto& offset : neighbors) {
                    int nx = t.x + offset.first;
                    int ny = t.y + offset.second;

                    if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H) continue;

                    // Tentative rhs for neighbor
                    double cost = 1.0; // assuming uniform grid cost
                    double new_rhs = map[t.y][t.x].first + cost;

                    if (new_rhs < map[ny][nx].second) {
                        map[ny][nx].second = new_rhs;
                        open_list.push(getKey(
                            dstar::Cell(nx, ny),
                            dstar::Cell(start->x, start->y),
                            map[ny][nx].first,
                            map[ny][nx].second
                        ));
                    }
                }
            } else { // underconsistent
                map[t.y][t.x].first = inf;

                // Update neighbors
                for (auto& offset : neighbors) {
                    int nx = t.x + offset.first;
                    int ny = t.y + offset.second;

                    if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H) continue;

                    double cost = 1.0;
                    double new_rhs = map[t.y][t.x].first + cost;

                    if (map[ny][nx].second == new_rhs) {
                        // recompute rhs of neighbor
                        double min_rhs = inf;
                        for (auto& n2 : neighbors) {
                            int nnx = nx + n2.first;
                            int nny = ny + n2.second;
                            if (nnx < 0 || nnx >= GRID_W || nny < 0 || nny >= GRID_H) continue;
                            min_rhs = std::min(min_rhs, map[nny][nnx].first + cost);
                        }
                        map[ny][nx].second = min_rhs;
                        open_list.push(getKey(
                            dstar::Cell(nx, ny),
                            dstar::Cell(start->x, start->y),
                            map[ny][nx].first,
                            map[ny][nx].second
                        ));
                    }
                }
            }
        }


        // Handle new walls

    }
}

void draw_loop() {
    bool path_thread_exists = false;

    InitWindow(GRID_W, GRID_H, "ArchBTW monitoring");

    Shader shader = LoadShader(NULL, "../grid_shader.fs");

    Color* pixels = new Color[GRID_W * GRID_H];
    Image img = GenImageColor(GRID_W, GRID_H, BLACK);
    img.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    Texture2D gridTex = LoadTextureFromImage(img);
    UnloadImage(img);

    cv::Mat pixelsMat(GRID_H, GRID_W, CV_8UC4, (void*)pixels);

    
    Image imgColor = GenImageColor(GRID_W, GRID_H, BLACK);
    imgColor.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    Texture2D colorTex = LoadTextureFromImage(imgColor);
    UnloadImage(imgColor);

    
    cv::Point goal; 
    PathPoint t;
    t.x = -7.7;
    t.y = 0;
    goal = worldToGrid(t);

    for (int y = 0; y < GRID_H; y++) {
        float dy = float(y - goal.y);
        for (int x = 0; x < GRID_W; x++) {
            float dx = float(x - goal.x);
            distanceGrid.at<float>(y, x) = sqrt(dx*dx + dy*dy);
        }
    }
    double minVal, maxVal;
    cv::minMaxLoc(distanceGrid, &minVal, &maxVal);
    distanceGrid = (distanceGrid - minVal) / (maxVal - minVal);


    cv::Mat colorGrid(GRID_H, GRID_W, CV_8UC3);
    uchar* colorData = colorGrid.data;
    for (int y = 0; y < GRID_H; y++) {
        const float* distRow = distanceGrid.ptr<float>(y);
        for (int x = 0; x < GRID_W; x++) {
            float val = distRow[x];
            int idx = (y * GRID_W + x) * 3;
            colorData[idx + 0] = (uchar)(val * 255);
            colorData[idx + 1] = 0;                         
            colorData[idx + 2] = (uchar)((1.0f - val) * 255); 
        }
    }


    cv::Mat colorGridRGBA;
    cv::cvtColor(colorGrid, colorGridRGBA, cv::COLOR_BGR2BGRA);
    UpdateTexture(colorTex, colorGridRGBA.data);


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
        BeginDrawing();
        ClearBackground(RAYWHITE);

        cv::mixChannels(grid,pixelsMat,{0,0});
        cv::mixChannels(pathfind_grid,pixelsMat,{0,1});

        UpdateTexture(gridTex, pixels);

        // Draw grids
        BeginShaderMode(shader);
        SetShaderValueTexture(shader, GetShaderLocation(shader, "uColorMap"), colorTex);
        DrawTexture(gridTex, 0, 0, WHITE);
        EndShaderMode();

        //draw path
        for(int i = 1;i<path.size();i++){
            cv::Point2f p1 = worldToGrid(path[i-1]);
            cv::Point2f p2 = worldToGrid(path[i]);
            DrawLineEx({p1.x, p1.y}, {p2.x, p2.y}, 3, BLUE);
        }

        // Draw goal
        DrawCircle(goal.x, goal.y, 5, MAGENTA);
        

        //draw robot
        float screen_robot_x = robot.x/CELL_SIZE+GRID_W/2;
        float screen_robot_y = robot.y/CELL_SIZE+GRID_H/2;
        float dir_x = 10*sin(robot.a);
        float dir_y = -10*cos(robot.a);
        DrawCircle(screen_robot_x, screen_robot_y, 10, GREEN);
        DrawLineEx({screen_robot_x, screen_robot_y}, {screen_robot_x+dir_x, screen_robot_y+dir_y}, 3, BLACK);

        // Draw coordinates
        DrawText(TextFormat("X: %.2f", robot.x), 10, 10, 20, RED);
        DrawText(TextFormat("Y: %.2f", robot.y), 10, 30, 20, RED);
        // Draw FPS
        DrawText(TextFormat("%.02f FPS", 1.0/GetFrameTime()), 10, 50, 20, RED);


        EndDrawing();
    }

    running = false;
    // Cleanup
    delete[] pixels;
    UnloadTexture(gridTex);
    UnloadTexture(colorTex);
    UnloadShader(shader);
    CloseWindow();
}

int main() {
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
    path_thread = thread(start_path, &message_queue);
    #endif

    PathPoint st, ft;
    st.x = 12;
    st.y = -1.25;
    ft.x = 7.7;
    ft.y = 0;
    cv::Point s = worldToGrid(st);
    cv::Point f = worldToGrid(ft);

    // void pathfinder_loop(cv::Point finish, cv::Point* start, cv::Mat* graphics_grid, vector<PathPoint>* optimal_path )
    thread dstar(pathfinder_loop, f, &s);

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
        int dilation_size = 10;
        int dilation_type = cv::MORPH_ELLIPSE;
        cv::Mat element = cv::getStructuringElement( dilation_type,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );
        cv::dilate(gridCopy,pathfind_grid,element);
    }

    #ifdef VISUALIZATION
    draw_thread.join();
    #endif
    dstar.join();
}
